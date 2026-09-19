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

    def fake_run_ros(cmd, timeout=60, distro="jazzy"):
        seen.append(cmd)
        # The node is configured: the topic is listed and has a publisher, but
        # `echo --once` never yields a message.
        if "topic echo" in cmd:
            return _result(returncode=124, stdout="")
        if "topic info" in cmd:
            return _result(stdout="Publisher count: 1\n")
        return _result(stdout="/map\n")

    monkeypatch.setattr(ocp, "run_ros", fake_run_ros)
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
