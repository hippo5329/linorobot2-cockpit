"""The Nav2 goal test must fail when the base does not move.

Its pass condition used to be "the planner produced a path around the wall AND
the controller published N cmd_vel messages". Both are measured host-side --
the plan comes from the planner, the count from the controller's own
publications -- so an unplugged board satisfies both, and so does a board whose
firmware subscribes to a /cmd_vel type nobody publishes. The odom delta WAS
computed; it was interpolated into the success string and compared to nothing.

That is how all four -lyrical prebuilt images in rc-20260919 passed a hardware
release test while being unable to receive a velocity command at all.

These tests drive run_test's decision logic through a stand-in node, so they run
without a ROS graph. They are about the RULE, not about rclpy.
"""
import os
import sys
import types

import pytest

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCRIPTS = os.path.join(REPO_ROOT, "scripts")


def _load_module():
    """Import test_nav2_goal with the ROS 2 imports stubbed out."""
    stubs = {
        "rclpy": ["init", "shutdown", "spin_once"],
        "rclpy.action": ["ActionClient"],
        "rclpy.node": ["Node"],
        "rclpy.qos": ["QoSProfile", "ReliabilityPolicy", "HistoryPolicy"],
        "action_msgs.msg": ["GoalStatus"],
        "geometry_msgs.msg": ["PoseStamped", "Twist", "TwistStamped"],
        "nav_msgs.msg": ["Path", "Odometry"],
        "nav2_msgs.action": ["NavigateToPose"],
        "action_msgs": [], "geometry_msgs": [], "nav_msgs": [], "nav2_msgs": [],
    }
    saved = {name: sys.modules.get(name) for name in stubs}
    for name, attrs in stubs.items():
        mod = types.ModuleType(name)
        for a in attrs:
            setattr(mod, a, type(a, (object,), {}))
        sys.modules[name] = mod
    sys.path.insert(0, SCRIPTS)
    try:
        sys.modules.pop("test_nav2_goal", None)
        import test_nav2_goal
        return test_nav2_goal
    finally:
        for name, mod in saved.items():
            if mod is None:
                sys.modules.pop(name, None)
            else:
                sys.modules[name] = mod


MOD = _load_module()


class FakeNode:
    """What run_test reads off the tester, and nothing else."""
    def __init__(self, dist=0.0, yaw=0.0, cmds=40, planned=True, completed=False):
        self.odom_max_dist, self.odom_max_yaw = dist, yaw
        self.cmd_vel_count, self.cmd_vel_stamped_count = cmds, cmds
        self.path_avoids_wall, self.goal_completed = planned, completed
        self.goal_accepted = True
        self.cmd_vel_type = "twist_stamped"
        self.destroyed = False

    def send_goal(self):
        return True

    def destroy_node(self):
        self.destroyed = True


def _run(monkeypatch, node, **kw):
    monkeypatch.setattr(MOD, "Nav2GoalTester", lambda **_: node)
    monkeypatch.setattr(MOD.rclpy, "init", lambda *a, **k: None, raising=False)
    monkeypatch.setattr(MOD.rclpy, "shutdown", lambda *a, **k: None, raising=False)
    monkeypatch.setattr(MOD.rclpy, "spin_once", lambda *a, **k: None, raising=False)
    kw.setdefault("timeout", 0.3)
    return MOD.run_test(**kw)


def test_planning_and_commanding_is_not_enough_without_motion(monkeypatch):
    """The exact shape of the shipped bug: nav2 planned around the wall and pushed
    40 TwistStamped commands, and the base -- listening for plain Twist -- never
    moved a millimetre."""
    assert _run(monkeypatch, FakeNode(dist=0.0, yaw=0.0, cmds=40, planned=True)) is False


def test_motion_makes_it_pass(monkeypatch):
    assert _run(monkeypatch, FakeNode(dist=0.31, yaw=0.0, cmds=40, planned=True)) is True


def test_turning_in_place_counts_as_moving(monkeypatch):
    """A differential base pointed at a goal turns first and translates almost
    nothing; requiring metres alone would fail a perfectly good robot."""
    assert _run(monkeypatch, FakeNode(dist=0.001, yaw=0.9, cmds=40, planned=True)) is True


def test_a_completed_goal_still_has_to_have_moved(monkeypatch):
    """The goal_completed branch is a separate return; it needs the same rule or it
    becomes the way around it."""
    assert _run(monkeypatch, FakeNode(dist=0.0, yaw=0.0, cmds=40, completed=True)) is False


def test_the_timeout_fallback_still_has_to_have_moved(monkeypatch):
    """The most permissive branch -- 'goal accepted and a path exists' -- was the one
    that passed on planning alone."""
    node = FakeNode(dist=0.0, yaw=0.0, cmds=40, planned=True)
    node.goal_completed = False
    assert _run(monkeypatch, node, min_cmds=10**6) is False


def test_no_require_motion_is_still_available_for_a_boardless_bringup(monkeypatch):
    assert _run(monkeypatch, FakeNode(dist=0.0, yaw=0.0, cmds=40, planned=True),
                require_motion=False) is True


def test_the_failure_names_the_cmd_vel_contract(monkeypatch, capsys):
    """A release test that fails without saying why costs more than it saves."""
    _run(monkeypatch, FakeNode(dist=0.0, yaw=0.0, cmds=40, planned=True))
    out = capsys.readouterr().out
    assert "USE_STAMPED_CMD_VEL" in out and "NEVER MOVED" in out.upper()


def test_few_commands_points_at_the_controller_instead(monkeypatch, capsys):
    node = FakeNode(dist=0.0, yaw=0.0, cmds=1, planned=True)
    _run(monkeypatch, node, min_cmds=10**6)
    out = capsys.readouterr().out
    assert "controller" in out and "USE_STAMPED_CMD_VEL" not in out


def test_the_loop_waits_for_a_base_that_starts_moving_late(monkeypatch):
    """A differential base spends the first moment accelerating and rotating, so odom
    is still at zero on the first pass through the loop while the controller is
    already several commands in.

    If the motion rule is applied only at the exit and not in the loop condition,
    that first pass returns a verdict immediately and fails a robot that was about
    to move. The rule has to gate CONTINUING, not just reporting.
    """
    node = FakeNode(dist=0.0, yaw=0.0, cmds=40, planned=True)
    ticks = {"n": 0}

    def spin(*_a, **_k):
        ticks["n"] += 1
        if ticks["n"] >= 3:
            node.odom_max_dist = 0.25

    monkeypatch.setattr(MOD.rclpy, "spin_once", spin, raising=False)
    monkeypatch.setattr(MOD, "Nav2GoalTester", lambda **_: node)
    monkeypatch.setattr(MOD.rclpy, "init", lambda *a, **k: None, raising=False)
    monkeypatch.setattr(MOD.rclpy, "shutdown", lambda *a, **k: None, raising=False)
    assert MOD.run_test(timeout=5.0) is True
    assert ticks["n"] >= 3, "returned before the base had a chance to move"
