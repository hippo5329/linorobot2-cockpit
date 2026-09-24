"""A gate that passes on a node which never started is worse than no gate.

slam_toolbox is a lifecycle node and creates its publishers in on_configure(),
so /map appears in `ros2 topic list` -- and has a publisher -- from the moment
it configures, whether or not it ever activates. A pico2 release test printed
"✅ /map is active" against a slam_toolbox that had logged "Configuring",
selected its Ceres solver and then stopped. Nav2 failed thirty seconds later
with planner_server unable to transform base_link to map, because the map frame
had never existed. The run blamed Nav2 for SLAM's silence.
"""
import os
import sys
import types

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO_ROOT, "scripts"))


def _result(returncode=0, stdout=""):
    return types.SimpleNamespace(returncode=returncode, stdout=stdout, stderr="")


def test_a_listed_but_silent_topic_does_not_satisfy_require_message(monkeypatch):
    import one_click_pipeline as ocp

    seen = []

    def stub_run_ros(cmd, timeout=60, distro="jazzy"):
        seen.append(cmd)
        # The node is configured: the topic is listed and has a publisher, but
        # `echo --once` never yields a message.
        if "topic echo" in cmd:
            return _result(returncode=124, stdout="")
        if "topic info" in cmd:
            return _result(stdout="Publisher count: 1\n")
        return _result(stdout="/map\n")

    monkeypatch.setattr(ocp, "run_ros", stub_run_ros)
    assert ocp.wait_for_topic("/map", timeout_sec=2, distro="jazzy",
                              require_message="info.width") is False
    assert any("topic echo" in c for c in seen), "the gate never tried to read a message"


def test_a_published_map_satisfies_require_message(monkeypatch):
    import one_click_pipeline as ocp

    monkeypatch.setattr(ocp, "run_ros",
                        lambda cmd, timeout=60, distro="jazzy": _result(stdout="192\n"))
    assert ocp.wait_for_topic("/map", timeout_sec=5, distro="jazzy",
                              require_message="info.width") is True


def test_the_slam_step_asks_for_a_message_not_a_name():
    import re
    text = open(os.path.join(REPO_ROOT, "scripts", "one_click_pipeline.py")).read()
    m = re.search(r'wait_for_topic\("/map".*?\)', text, re.S)
    assert m, "the /map gate is gone"
    assert "require_message" in m.group(0), (
        "the /map gate is back to checking the topic NAME, which a lifecycle "
        "node satisfies by configuring and never activating"
    )


def test_the_launcher_itself_recovers_a_missed_activation():
    """The pipeline's self-heal only helps a run that goes through the pipeline.

    `ros2 launch linorobot2_cockpit slam.launch.py` is also how a person starts
    SLAM, and that path hit the same lost transition_event with nothing to
    recover it. The guard belongs in the launcher, where both paths get it.
    """
    text = open(os.path.join(REPO_ROOT, "launchers", "slam.launch.py")).read()
    # The node path is parameterised now (it carries the robot's namespace for
    # multi-robot topic_prefix), so the guard activates "$NODE" rather than a
    # hard-coded /slam_toolbox -- but the default node is still /slam_toolbox.
    assert 'ros2 lifecycle set --no-daemon "$NODE" activate' in text, (
        "slam.launch.py no longer activates a slam_toolbox that its own launch "
        "file left in 'inactive'"
    )
    assert 'activation_guard("/slam_toolbox")' in text or 'def activation_guard(node="/slam_toolbox")' in text, (
        "the default (no-namespace) activation target is no longer /slam_toolbox"
    )
    assert 'ros2 lifecycle get --no-daemon "$NODE"' in text, (
        "the guard must read the state first -- an unconditional activate hides "
        "whether the race happened at all"
    )
    assert "TimerAction" in text, "the guard has to run after the node has had time to configure"


def test_the_guard_is_off_when_autostart_is():
    """autostart=false means the caller drives the lifecycle. Do not fight it."""
    text = open(os.path.join(REPO_ROOT, "launchers", "slam.launch.py")).read()
    assert 'autostart.lower() in ("true", "1", "yes")' in text, (
        "the activation guard must be conditional on autostart"
    )


def test_neither_recovery_asks_the_ros2_daemon():
    """The CLI daemon caches the graph and can be wrong for a long time.

    `ros2 lifecycle get /slam_toolbox` answered "Node not found" for 36 s
    straight, inside a container where slam_toolbox was already active and
    `--no-daemon` answered "active [3]" immediately. A guard that believes the
    cache reports a healthy run as broken, and a recovery that believes it
    gives up on a node that is sitting right there.
    """
    for rel in ("launchers/slam.launch.py", "scripts/one_click_pipeline.py"):
        text = open(os.path.join(REPO_ROOT, rel)).read()
        for line in text.splitlines():
            if "ros2 lifecycle" in line and "slam_toolbox" in line:
                assert "--no-daemon" in line, (
                    f"{rel}: `{line.strip()}` goes through the ros2 daemon"
                )
