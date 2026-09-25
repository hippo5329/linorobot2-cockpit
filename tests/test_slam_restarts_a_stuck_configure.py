"""A slam_toolbox stuck inside its configure is restarted, once; one stuck in `inactive` is activated.

Gate 20260926-disp8, Yahboom skid_steer jazzy: every topic green at rate, the
drive test 8/8, then slam_toolbox logged "Configuring" and the Ceres solver
lines and never answered a lifecycle query again. The pipeline asked it to
activate -- the repair for the OTHER way it stops short -- and Nav2 failed
behind it with no map frame. The node's own answer tells the two apart.
"""
import os
import subprocess
import sys

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO_ROOT, "scripts"))

import one_click_pipeline as ocp  # noqa: E402


class Graph:
    """The pipeline's view of ROS, scripted per SLAM process."""

    def __init__(self, monkeypatch, tmp_path, maps, states):
        self.maps = list(maps)        # per launch: does /map arrive (first wait, after activate)
        self.states = list(states)    # per launch: what `lifecycle get` answers
        self.launched, self.stopped, self.activated = [], [], 0
        monkeypatch.setattr(ocp, "LOG_DIR", str(tmp_path))
        monkeypatch.setattr(ocp, "launch_bg", self.launch_bg)
        monkeypatch.setattr(ocp, "wait_for_topic", self.wait_for_topic)
        monkeypatch.setattr(ocp, "run_ros", self.run_ros)
        monkeypatch.setattr(ocp, "_stop_group_and_wait", lambda proc: self.stopped.append(proc))

    def launch_bg(self, cmd, log_tag="launch", distro="jazzy"):
        self.launched.append(log_tag)
        return object()

    def wait_for_topic(self, topic, **kw):
        return self.maps[len(self.launched) - 1].pop(0)

    def run_ros(self, cmd, timeout=30, distro="jazzy"):
        n = len(self.launched) - 1
        if "lifecycle set" in cmd:
            self.activated += 1
            return subprocess.CompletedProcess(cmd, 0, "Transitioning successful", "")
        s = self.states[n]
        return subprocess.CompletedProcess(cmd, 0 if s else 1, s, "")


def run(g):
    bg, stack = [], []
    return ocp.start_slam("ros2 launch x", "jazzy", bg, stack), bg, stack


def test_a_node_that_answers_nothing_is_restarted_once(monkeypatch, tmp_path):
    g = Graph(monkeypatch, tmp_path, maps=[[False], [True]], states=["", "active [3]"])
    failures, bg, stack = run(g)
    assert failures == []
    assert g.launched == ["slam", "slam2"], "each attempt keeps its own log"
    assert len(g.stopped) == 1 and g.activated == 0
    assert len(bg) == 1 and [t for t, _ in stack] == ["slam"], "the stopped node must leave the books"


def test_an_inactive_node_is_activated_not_restarted(monkeypatch, tmp_path):
    g = Graph(monkeypatch, tmp_path, maps=[[False, True]], states=["inactive [2]"])
    failures, _, _ = run(g)
    assert failures == [] and g.activated == 1
    assert g.launched == ["slam"] and g.stopped == []


def test_an_active_node_with_no_map_is_reported_not_hidden(monkeypatch, tmp_path):
    """A restart would turn a real mapping fault into a pass."""
    g = Graph(monkeypatch, tmp_path, maps=[[False]], states=["active [3]"])
    failures, _, _ = run(g)
    assert failures == ["SLAM: no map was published"]
    assert g.launched == ["slam"] and g.stopped == []


def test_the_restart_happens_once(monkeypatch, tmp_path):
    g = Graph(monkeypatch, tmp_path, maps=[[False], [False]], states=["", ""])
    failures, _, _ = run(g)
    assert failures == ["SLAM: no map was published"]
    assert g.launched == ["slam", "slam2"] and len(g.stopped) == 1


def test_the_report_names_a_configure_that_never_finished(tmp_path):
    log = tmp_path / "slam.log"
    log.write_text("[slam_toolbox]: Configuring\n[slam_toolbox]: Using solver plugin solver_plugins::CeresSolver\n")
    out = ocp._slam_complaints(str(log), lifecycle="")
    assert "never finished configuring" in out and "'nothing'" in out, out
    assert "stuck in `inactive`" not in out, out
    assert "stuck in `inactive`" in ocp._slam_complaints(str(log), lifecycle="inactive [2]")
