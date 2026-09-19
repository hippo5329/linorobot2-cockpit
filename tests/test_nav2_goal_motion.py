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
    """What run_test reads off the tester, and nothing else.

    Defaults model a controller commanding 0.20 m/s / 0.60 rad/s -- nav2's usual
    order of magnitude on this robot -- against a base reporting the at-rest noise
    a real bench run sampled: 0.006 m/s and 0.036 rad/s.
    """
    def __init__(self, odom_lin=0.006, odom_ang=0.036, dist=0.0, yaw=0.0,
                 cmds=40, planned=True, completed=False,
                 cmd_lin=0.20, cmd_ang=0.60):
        self.odom_peak_lin, self.odom_peak_ang = odom_lin, odom_ang
        self.cmd_peak_lin, self.cmd_peak_ang = cmd_lin, cmd_ang
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
    assert _run(monkeypatch, FakeNode()) is False


def test_motion_makes_it_pass(monkeypatch):
    assert _run(monkeypatch, FakeNode(odom_lin=0.18, odom_ang=0.0, dist=0.31)) is True


def test_turning_in_place_counts_as_moving(monkeypatch):
    """A differential base pointed at a goal turns first and translates almost
    nothing; requiring metres alone would fail a perfectly good robot."""
    assert _run(monkeypatch, FakeNode(odom_lin=0.0, odom_ang=0.55, dist=0.001, yaw=0.9)) is True


def test_a_completed_goal_still_has_to_have_moved(monkeypatch):
    """The goal_completed branch is a separate return; it needs the same rule or it
    becomes the way around it."""
    assert _run(monkeypatch, FakeNode(completed=True)) is False


def test_the_timeout_fallback_still_has_to_have_moved(monkeypatch):
    """The most permissive branch -- 'goal accepted and a path exists' -- was the one
    that passed on planning alone."""
    node = FakeNode()
    node.goal_completed = False
    assert _run(monkeypatch, node, min_cmds=10**6) is False


def test_no_require_motion_is_still_available_for_a_boardless_bringup(monkeypatch):
    assert _run(monkeypatch, FakeNode(), require_motion=False) is True


def test_the_failure_names_the_cmd_vel_contract(monkeypatch, capsys):
    """A release test that fails without saying why costs more than it saves."""
    _run(monkeypatch, FakeNode())
    out = capsys.readouterr().out
    assert "USE_STAMPED_CMD_VEL" in out and "NEVER MOVED" in out.upper()


def test_few_commands_points_at_the_controller_instead(monkeypatch, capsys):
    node = FakeNode(cmds=1)
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
    node = FakeNode()
    ticks = {"n": 0}

    def spin(*_a, **_k):
        ticks["n"] += 1
        if ticks["n"] >= 3:
            node.odom_peak_lin = 0.18

    monkeypatch.setattr(MOD.rclpy, "spin_once", spin, raising=False)
    monkeypatch.setattr(MOD, "Nav2GoalTester", lambda **_: node)
    monkeypatch.setattr(MOD.rclpy, "init", lambda *a, **k: None, raising=False)
    monkeypatch.setattr(MOD.rclpy, "shutdown", lambda *a, **k: None, raising=False)
    assert MOD.run_test(timeout=5.0) is True
    assert ticks["n"] >= 3, "returned before the base had a chance to move"


def test_odometry_drift_does_not_count_as_responding(monkeypatch):
    """The reason the rule is on velocity and not displacement.

    A fake-mode board standing still reported vel_lin=0.006 m/s and
    vel_ang=0.036 rad/s on the bench. Integrated over a 25-second goal window
    that is 0.15 m of "travel" and 0.9 rad of "rotation" -- past any displacement
    threshold small enough to be worth setting, and past the 0.02 m / 0.05 rad
    this check first used. Velocity does not accumulate, so the floor stays a
    floor however long the run is.
    """
    drifting = FakeNode(odom_lin=0.006, odom_ang=0.036, dist=0.15, yaw=0.9)
    assert _run(monkeypatch, drifting) is False


def test_a_base_that_tracks_its_setpoint_badly_still_passes(monkeypatch):
    """0.16 rad/s against a 0.60 rad/s command is poor tracking and unambiguous
    receipt. A geared-down, loaded or detuned robot must not fail a release gate
    for it: the question here is whether the command arrived, not how well it was
    followed."""
    assert _run(monkeypatch, FakeNode(odom_lin=0.0, odom_ang=0.16)) is True


def test_even_a_badly_undertracking_base_passes(monkeypatch):
    """The explicit statement that this is not a performance test. 0.12 rad/s
    against a 2.0 rad/s command is 6% -- terrible tracking, and still proof the
    board is receiving /cmd_vel, which is all this gate claims to check."""
    assert _run(monkeypatch, FakeNode(odom_lin=0.0, odom_ang=0.12, cmd_ang=2.0)) is True


def test_a_tiny_command_does_not_lower_the_bar(monkeypatch):
    """The floor is absolute. A controller asking for only 0.01 rad/s cannot make
    a noise-floor reading count as a response."""
    assert _run(monkeypatch, FakeNode(odom_lin=0.006, odom_ang=0.036,
                                      cmd_lin=0.004, cmd_ang=0.01)) is False
