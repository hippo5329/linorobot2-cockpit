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
                 cmd_lin=0.20, cmd_ang=0.60,
                 goal_status=6, goal_error_code=0, goal_error_msg="",
                 goal_rejected=False, goal_dist=float("inf")):
        self.odom_peak_lin, self.odom_peak_ang = odom_lin, odom_ang
        self.cmd_peak_lin, self.cmd_peak_ang = cmd_lin, cmd_ang
        self.odom_max_dist, self.odom_max_yaw = dist, yaw
        self.cmd_vel_count, self.cmd_vel_stamped_count = cmds, cmds
        self.path_avoids_wall, self.goal_completed = planned, completed
        self.goal_accepted = True
        self.cmd_vel_type = "twist_stamped"
        # How the goal ended, and why. 6 is ABORTED -- the common case for a run
        # that times out; a completed run is SUCCEEDED.
        self.goal_status = 4 if completed else goal_status
        self.goal_error_code, self.goal_error_msg = goal_error_code, goal_error_msg
        self.goal_rejected = goal_rejected
        self.distance_remaining = float("nan")
        # Displacement from the goal: inf unless a test says otherwise, so a
        # double that says nothing about position cannot accidentally "arrive".
        self.goal_dist_now = self.goal_dist_min = goal_dist
        self.goal_dist_start = 3.0
        self.destroyed = False

        self.leg_id = 0
        self.leg_start_xy = (0.0, 0.0)
        self.leg_max_dist = dist
        self.legs = []                      # (x, y) of every goal begun

    def cancel_current_goal(self):
        self.cancelled = getattr(self, "cancelled", 0) + 1
        return True

    def begin_leg(self, goal_x, goal_y):
        """A perfect simulated robot: every leg is reached at the same quality."""
        self.leg_id += 1
        self.legs.append((goal_x, goal_y))
        self.goal_x, self.goal_y = goal_x, goal_y
        self.goal_dist_start = 3.0

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


def test_the_path_verified_pass_needs_a_real_traverse(monkeypatch):
    """The success branch must not fire on the in-place rotation at the start.

    It exited as soon as the plan was verified, five commands had gone out and
    moved() was true -- which on a quick machine is the rotation before the robot
    has driven any of the path. Measured on the Yahboom: 5 cmd_vel peaking at
    0.025 m/s, a green line with the robot standing still, while the same board
    over Wi-Fi did 161 commands at 0.400 m/s. The wildly varying counts across
    boards (5, 20, 41, 61, 120, 161) were that race, not the boards.
    """
    rotating = FakeNode(odom_lin=0.0, odom_ang=0.55, dist=0.002, yaw=0.9, cmds=40)
    assert _run(monkeypatch, rotating, timeout=0.3) is True   # the timeout branch still judges by moved()
    # ...but it must not have SHORT-CIRCUITED: a driving base returns in-window.
    driving = FakeNode(odom_lin=0.22, odom_ang=0.1, dist=0.8, yaw=0.2, cmds=40)
    assert _run(monkeypatch, driving, timeout=30.0) is True


def test_a_rotating_base_does_not_end_the_window_early(monkeypatch):
    """A long window plus a base that only spins must run to the timeout.

    If the path-verified branch still accepted rotation, this would return almost
    at once; with the traverse requirement it has to wait, which is what gives a
    robot still turning to face its path the chance to actually drive it.
    """
    import time as _t
    node = FakeNode(odom_lin=0.0, odom_ang=0.55, dist=0.002, yaw=0.9, cmds=40)
    t0 = _t.time()
    _run(monkeypatch, node, timeout=1.0)
    assert _t.time() - t0 >= 0.9, "the success branch short-circuited on rotation again"


def test_min_traverse_is_reachable_from_the_command_line():
    src = open(os.path.join(REPO_ROOT, "scripts", "test_nav2_goal.py")).read()
    assert '"--min-traverse"' in src and "min_traverse=args.min_traverse" in src


def test_a_rejected_goal_fails_and_is_not_a_timeout(monkeypatch):
    """bt_navigator saying no is not the same as never answering.

    Both used to leave goal_accepted False and let the run time out, so a stack
    that refused the pose read exactly like one that was not there.
    """
    rejected = FakeNode(goal_rejected=True, planned=False, cmds=0)
    assert _run(monkeypatch, rejected, timeout=0.3) is False


def test_require_goal_fails_a_verified_plan_that_never_arrived(monkeypatch):
    """--require-goal verifies the goal, not the plan.

    The default contract accepts a plan around the wall plus a responding base.
    When the goal itself is what must be verified, a plan is not a substitute:
    the Yahboom planned around the wall on every leg and reached the goal on
    none of them.
    """
    planned_only = FakeNode(odom_lin=0.25, dist=1.2, planned=True, completed=False,
                            goal_status=6, goal_error_code=105,
                            goal_error_msg="Failed to make progress")
    assert _run(monkeypatch, planned_only, timeout=0.3) is True                     # default
    assert _run(monkeypatch, planned_only, timeout=0.3, require_goal=True) is False  # verified


def test_a_reached_goal_passes_under_require_goal(monkeypatch):
    reached = FakeNode(odom_lin=0.25, dist=2.4, completed=True)
    assert _run(monkeypatch, reached, timeout=30.0, require_goal=True) is True


def test_the_error_code_and_message_reach_the_output(monkeypatch, capsys):
    """An abort without its error_code is a dead end for whoever reads the log.
    105 / 'Failed to make progress' and 102 / 'no valid path' are different
    faults with different fixes; the gate saw both and printed neither.
    """
    aborted = FakeNode(odom_lin=0.25, dist=1.2, goal_status=6, goal_error_code=105,
                       goal_error_msg="Failed to make progress")
    _run(monkeypatch, aborted, timeout=0.3, require_goal=True)
    out = capsys.readouterr().out
    assert "ABORTED" in out and "error_code=105" in out and "Failed to make progress" in out


def test_the_goal_is_judged_by_displacement_not_by_the_status(monkeypatch):
    """A status is not a position.

    A controller can stop short and report SUCCEEDED, and can equally be aborted
    by a behaviour-tree timeout while sitting on top of the goal. Aborting 0.2 m
    short and aborting 2.8 m short are both ABORTED and are not the same result.
    """
    nearly = FakeNode(odom_lin=0.25, dist=2.8, goal_status=6, goal_dist=0.21,
                      goal_error_code=105, goal_error_msg="Failed to make progress")
    assert _run(monkeypatch, nearly, timeout=30.0, require_goal=True) is True
    nowhere = FakeNode(odom_lin=0.25, dist=0.4, goal_status=6, goal_dist=2.83,
                       goal_error_code=105, goal_error_msg="Failed to make progress")
    assert _run(monkeypatch, nowhere, timeout=0.3, require_goal=True) is False


def test_the_gap_and_the_error_both_reach_the_output(monkeypatch, capsys):
    aborted = FakeNode(odom_lin=0.25, dist=0.4, goal_status=6, goal_dist=2.83,
                       goal_error_code=105, goal_error_msg="Failed to make progress")
    _run(monkeypatch, aborted, timeout=0.3, require_goal=True)
    out = capsys.readouterr().out
    assert "ABORTED" in out, out
    assert "error_code=105" in out and "Failed to make progress" in out, out
    assert "2.830 m from the goal" in out and "from 3.000 m at the start" in out, out


def test_a_tighter_tolerance_is_honoured(monkeypatch):
    close = FakeNode(odom_lin=0.25, dist=2.8, goal_status=6, goal_dist=0.21)
    assert _run(monkeypatch, close, timeout=30.0, require_goal=True) is True
    assert _run(monkeypatch, close, timeout=0.3, require_goal=True,
                goal_tolerance=0.10) is False


def test_an_arrival_that_started_on_the_goal_is_rejected(monkeypatch, capsys):
    """The simulated pose survives a run, so a repeated goal can begin under the robot.

    Measured on an RP2350: started 0.484 m from a goal with a 0.30 m tolerance,
    closed 0.185 m in 24 commands, and was reported REACHED. That is the gate
    passing without the robot going anywhere -- the same trap as ok_no_traverse,
    one layer up.
    """
    class OnTop(FakeNode):
        def __init__(self, **kw):
            super().__init__(**kw)
            self.goal_dist_start = 0.484
    node = OnTop(odom_lin=0.12, dist=0.227, goal_dist=0.299)
    assert _run(monkeypatch, node, timeout=0.3, require_goal=True) is False
    out = capsys.readouterr().out
    assert "VACUOUS" in out and "0.484" in out, out


def test_a_real_gap_still_passes(monkeypatch):
    class FarEnough(FakeNode):
        def __init__(self, **kw):
            super().__init__(**kw)
            self.goal_dist_start = 3.0
    node = FarEnough(odom_lin=0.25, dist=2.9, goal_dist=0.12, completed=True)
    assert _run(monkeypatch, node, timeout=30.0, require_goal=True) is True


def test_a_goal_behind_the_wall_is_a_pass_when_the_plan_shows_the_detour(monkeypatch):
    reached = FakeNode(odom_lin=0.25, dist=3.1, goal_dist=0.2, planned=True)
    assert _run(monkeypatch, reached, timeout=0.3, require_goal=True,
                goal_x=3.0, goal_y=0.0) is True


def test_an_arrival_the_gate_could_not_watch_passes_and_says_so(monkeypatch, capsys):
    """On a loaded host the tester misses /plan messages and bursts of /odom
    alike. The wall is solid -- a board commanded into it stops 0.30 m short and
    stays -- and the start gap proves the robot began on the near side, so an
    arrival IS a route around it. The gate says the route is unproven instead of
    failing a leg for what the instrument could not watch."""
    reached = FakeNode(odom_lin=0.25, dist=3.1, goal_dist=0.2, planned=False)
    assert _run(monkeypatch, reached, timeout=0.3, require_goal=True, round_trips=1,
                goal_x=3.0, goal_y=0.0) is True
    assert "unproven" in capsys.readouterr().out


def test_a_near_side_goal_does_not_pretend_to_test_the_wall(monkeypatch):
    """A goal that never needed the detour is judged on arrival alone."""
    reached = FakeNode(odom_lin=0.25, dist=1.1, goal_dist=0.2, planned=False)
    assert _run(monkeypatch, reached, timeout=0.3, require_goal=True,
                goal_x=0.5, goal_y=1.0) is True


def test_round_trips_drive_out_and_home_that_many_times(monkeypatch, capsys):
    """'run back and forth goal 4 times each run': out to (3, 0) behind the wall,
    home to (0, 0), four times -- eight legs, every one of which must arrive."""
    node = FakeNode(odom_lin=0.25, dist=3.1, goal_dist=0.2, planned=True, goal_status=4)
    assert _run(monkeypatch, node, timeout=0.3, round_trips=4,
                goal_x=3.0, goal_y=0.0) is True
    assert node.legs == [(3.0, 0.0), (0.0, 0.0)] * 4
    # A goal Nav2 has already closed (status 4) needs no cancel: the gate asks
    # only when the goal is still open. "another navigator is processing,
    # rejecting request" ended a run at leg 3 of 8 before this.
    assert getattr(node, "cancelled", 0) == 0
    out = capsys.readouterr().out
    assert "leg 8/8 -> (0.00, 0.00): reached" in out
    assert "NAV2 GOAL REACHED 8/8 legs: 4 round trip(s)" in out


def test_a_round_trip_fails_on_the_first_leg_that_does_not_arrive(monkeypatch, capsys):
    class WedgesOnTheWayHome(FakeNode):
        def begin_leg(self, gx, gy):
            super().begin_leg(gx, gy)
            if self.leg_id == 2:            # the first return leg never gets there
                self.goal_dist_min = self.goal_dist_now = 1.7
                self.goal_completed, self.goal_status = False, 6
                self.goal_error_code, self.goal_error_msg = 105, "Failed to make progress"
    node = WedgesOnTheWayHome(odom_lin=0.25, dist=3.1, goal_dist=0.2, planned=True, goal_status=4)
    assert _run(monkeypatch, node, timeout=0.3, round_trips=4) is False
    out = capsys.readouterr().out
    assert "leg 1/8 -> (3.00, 0.00): reached" in out
    assert "NAV2 LEG 2/8 NOT REACHED" in out and "Failed to make progress" in out
    assert len(node.legs) == 2, "one failed leg ends the run"


def test_the_return_leg_is_judged_on_the_wall_too(monkeypatch, capsys):
    """Home from (3, 0) crosses the wall just as the outbound leg did, so a
    measured crossing through its span fails the return leg as well."""
    class ThroughOnTheWayHome(FakeNode):
        def begin_leg(self, gx, gy):
            super().begin_leg(gx, gy)
            self.leg_start_xy = (3.0, 0.0) if gx == 0.0 else (0.0, 0.0)
            self.path_avoids_wall = gx != 0.0
            self.wall_cross_y = [] if gx != 0.0 else [(-0.31, 0.01)]
    node = ThroughOnTheWayHome(odom_lin=0.25, dist=3.1, goal_dist=0.2, goal_status=4)
    assert _run(monkeypatch, node, timeout=0.3, round_trips=1) is False
    assert "LEG 2/2 DROVE INTO THE WALL" in capsys.readouterr().out


def test_round_trips_zero_is_the_classic_one_way_goal(monkeypatch):
    reached = FakeNode(odom_lin=0.25, dist=2.4, completed=True)
    assert _run(monkeypatch, reached, timeout=30.0, require_goal=True, round_trips=0) is True
    assert reached.legs == []


def test_a_leg_driven_around_the_wall_passes_without_the_plan(monkeypatch, capsys):
    """The /plan is what Nav2 intended; where the robot crossed x=2 is what it did.
    A tester that subscribes after the first plans are published would otherwise
    fail a leg the robot demonstrably drove around (GenDrv, 2026-09-22)."""
    node = FakeNode(odom_lin=0.25, dist=3.1, goal_dist=0.2, planned=False, goal_status=4)
    node.wall_cross_y = [(1.72, 0.01)]
    assert _run(monkeypatch, node, timeout=0.3, round_trips=0, require_goal=True,
                goal_x=3.0, goal_y=0.0) is True


def test_driving_through_the_wall_is_not_an_arrival(monkeypatch, capsys):
    """The simulated robot is pushed off the wall segment, so a crossing inside
    the wall's span means the room failed, not the navigation."""
    node = FakeNode(odom_lin=0.25, dist=3.1, goal_dist=0.2, planned=True, goal_status=4)
    node.wall_cross_y = [(0.04, 0.01)]
    assert _run(monkeypatch, node, timeout=0.3, round_trips=1, goal_x=3.0, goal_y=0.0) is False
    assert "DROVE INTO THE WALL" in capsys.readouterr().out


def test_a_crossing_within_its_own_error_bar_of_the_end_claims_nothing(monkeypatch, capsys):
    """The gap between the two /odom samples bracketing the crossing IS the
    error bar on it. A detour round the end at y = -1.6 came back as -1.44 with
    the samples 0.30 m apart: inside the wall by the number, round the end by
    the physics. Neither answer is measured, so the leg is not failed for it."""
    node = FakeNode(odom_lin=0.25, dist=3.1, goal_dist=0.2, planned=False, goal_status=4)
    node.wall_cross_y = [(-1.44, 0.30)]
    assert _run(monkeypatch, node, timeout=0.3, require_goal=True, round_trips=1,
                goal_x=3.0, goal_y=0.0) is True
    out = capsys.readouterr().out
    assert "DROVE INTO THE WALL" not in out
    assert "unproven" in out, out[-200:]


def test_the_goal_distance_is_measured_in_the_frame_the_goal_was_sent_in():
    """A leg "reached" its goal 2.829 m away: the robot had hit the wall, the
    fake wheels slipped, the EKF's odom walked on, and SLAM absorbed the
    difference into map->odom. Nav2 was right that it had arrived; the gate was
    comparing an odom-frame pose with a map-frame goal. The pose now comes from
    map -> base_link, with the frame named in the verdict so the two can never
    be silently mixed again."""
    src = open(os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                            "scripts", "test_nav2_goal.py")).read()
    assert 'goal_msg.pose.header.frame_id = "map"' in src
    assert "def world_xy(self, msg: Odometry)" in src
    assert "lookup_transform(self.goal_frame, self.base_frame" in src
    body = src[src.index("def _odom_cb"):src.index("def send_goal")]
    assert "wx, wy = self.world_xy(msg)" in body
    assert "self.goal_dist_now = math.hypot(self.goal_x - wx, self.goal_y - wy)" in body
    assert "gap = math.hypot(wx - prev[0], wy - prev[1])" in body, "the wall crossing too"
    assert 'frame = "map" if getattr(node, "tf_ok", False) else "odom' in src


def test_clipping_the_corner_is_not_driving_through_the_wall(monkeypatch, capsys):
    """A disc robot rounds a corner wide, and the clamp that keeps it a radius
    clear of the wall puts the crossing just inside the span: y = -1.40 against
    a span of 1.5 was reported as driving through a solid wall."""
    node = FakeNode(odom_lin=0.25, dist=3.1, goal_dist=0.2, planned=True, goal_status=4)
    node.wall_cross_y = [(-1.40, 0.01)]
    assert _run(monkeypatch, node, timeout=0.3, round_trips=1, goal_x=3.0, goal_y=0.0) is True
    assert "DROVE INTO THE WALL" not in capsys.readouterr().out


def test_the_base_going_round_is_not_failed_for_the_estimate_cutting_through(monkeypatch, capsys):
    """The firmware clamps the SIMULATED pose, so /odom/unfiltered is where the
    room's physics live. This EKF fuses velocities only -- no position to correct
    against -- so the filtered pose drifts and can cut a corner the robot never
    cut. Measured on the GenDrv 2026-09-22: the filtered pose crossed x=2.0 at
    y=-0.63, apparently straight through the middle, while the base's own
    odometry had rounded the end. Failing that leg blamed the firmware for the
    estimator, and it was the last red leg holding up a cut."""
    node = FakeNode(odom_lin=0.25, dist=3.1, goal_dist=0.2, planned=False, goal_status=4)
    node.wall_cross_y = [(-0.63, 0.01)]        # the estimate: through the middle
    node.raw_cross_y = [(-1.62, 0.01)]         # the base: round the end
    node.raw_max_x = 2.31
    assert _run(monkeypatch, node, timeout=0.3, require_goal=True, round_trips=1,
                goal_x=3.0, goal_y=0.0) is True
    out = capsys.readouterr().out
    assert "DROVE INTO THE WALL" not in out
    assert "the BASE went round the wall" in out and "the estimate did" in out


def test_the_base_itself_going_through_still_fails_and_says_whose_pose(monkeypatch, capsys):
    """When the unfiltered pose is the one inside the wall, the room failed."""
    node = FakeNode(odom_lin=0.25, dist=3.1, goal_dist=0.2, planned=True, goal_status=4)
    node.wall_cross_y = [(0.04, 0.01)]
    node.raw_cross_y = [(0.04, 0.01)]
    node.raw_max_x = 2.6
    assert _run(monkeypatch, node, timeout=0.3, round_trips=1, goal_x=3.0, goal_y=0.0) is False
    out = capsys.readouterr().out
    assert "DROVE INTO THE WALL" in out
    assert "the base's own odometry crossed" in out


def test_without_unfiltered_odom_the_filtered_pose_is_all_there_is(monkeypatch, capsys):
    """A bringup that never published /odom/unfiltered still gets a verdict, and
    the transcript says the answer rests on the filtered pose."""
    node = FakeNode(odom_lin=0.25, dist=3.1, goal_dist=0.2, planned=True, goal_status=4)
    node.wall_cross_y = [(0.04, 0.01)]
    node.raw_cross_y = []
    node.raw_max_x = float("-inf")
    assert _run(monkeypatch, node, timeout=0.3, round_trips=1, goal_x=3.0, goal_y=0.0) is False
    assert "no /odom/unfiltered" in capsys.readouterr().out


def test_an_instant_abort_with_no_plan_is_the_stack_still_settling(monkeypatch, capsys):
    """Nav2's lifecycle reports "active" once every node has configured, which
    is before the costmaps have a scan to build a plan from. Measured on the
    GenDrv 2026-09-22 (jazzy): the first goal ABORTED after 0 s with 0.000 m
    traversed and planned_around_wall=False, on a stack whose /scan was 8.8 Hz
    and whose /map was publishing. That is startup, not navigation, so it is
    retried once per run -- and a real abort takes time and shows movement, so
    the retry costs nothing when the failure is genuine."""
    class AbortsOnceThenDrives(FakeNode):
        def __init__(self, **kw):
            super().__init__(**kw)
            self._sends = 0
        def send_goal(self):
            self._sends += 1
            if self._sends == 1:
                self.goal_status = 6          # ABORTED, having done nothing
                self.leg_max_dist = 0.0
                self.path_avoids_wall = False
                self.goal_dist_now = self.goal_dist_min = 2.997
            else:
                self.goal_status = 4
                self.goal_dist_now = self.goal_dist_min = 0.1
                self.leg_max_dist = 3.0
                self.path_avoids_wall = True
            return True
    node = AbortsOnceThenDrives(odom_lin=0.25, dist=3.1, goal_dist=0.1, goal_status=6)
    assert _run(monkeypatch, node, timeout=0.3, require_goal=True, round_trips=1,
                goal_x=3.0, goal_y=0.0) is True
    assert "the stack was still settling" in capsys.readouterr().out


def test_the_startup_retry_happens_once_per_run_not_once_per_leg(monkeypatch):
    """Otherwise a stack that aborts everything retries eight times and the run
    takes an hour to say what it knew after the first leg."""
    src = open(os.path.join(SCRIPTS, "test_nav2_goal.py")).read()
    assert "retried_startup = False" in src
    assert src.index("retried_startup = False") < src.index("for i, (gx, gy) in enumerate(legs, 1):")
    assert "and not retried_startup" in src


def test_a_succeeded_status_cannot_outvote_the_distance(monkeypatch, capsys):
    """Measured on the Yahboom 2026-09-22: leg 2/8 reported "reached in 2 s,
    closest 2.830 m" against a 0.40 m tolerance, because Nav2 called the goal
    SUCCEEDED while the robot sat where leg 1 had left it. Leg 3 then began
    0.292 m from its own goal. One leg's false arrival is the next leg's missing
    journey, so the distance decides and the status only explains."""
    node = FakeNode(odom_lin=0.25, dist=3.1, goal_dist=2.830, completed=True)
    assert _run(monkeypatch, node, timeout=0.3, require_goal=True, round_trips=0,
                goal_x=0.0, goal_y=0.0, goal_tolerance=0.40) is False
    assert "NOT REACHED" in capsys.readouterr().out


def test_the_status_is_used_when_there_is_no_pose_at_all(monkeypatch):
    """With nothing on /odom there is nothing better than the status, and the
    run still has to reach a verdict rather than hang."""
    node = FakeNode(odom_lin=0.25, dist=3.1, goal_dist=float("inf"), completed=True)
    node.goal_dist_min = float("inf")
    assert _run(monkeypatch, node, timeout=0.3, require_goal=True, round_trips=0,
                goal_x=0.0, goal_y=0.0, goal_tolerance=0.40) is True


def test_an_open_goal_is_cancelled_on_every_leg_including_the_last(monkeypatch, capsys):
    """A goal Nav2 has not closed is cancelled at the end of EVERY leg.

    The next goal need not come from this process. A soak that left its final
    goal running had the next round's goal rejected by bt_navigator, and the
    next, until Nav2 let go -- 38 of 60 rounds red on 2026-09-23, every burst
    preceded by a round that printed "has not closed the goal".
    """
    monkeypatch.setattr(MOD, "GOAL_CLOSE_SEC", 0.4)
    monkeypatch.setattr(MOD, "GOAL_CLOSE_EXIT_SEC", 0.4)
    node = FakeNode(odom_lin=0.25, dist=3.1, goal_dist=0.2, planned=True, goal_status=-1)
    node.goal_completed = True          # the robot arrived; Nav2 has not said so
    assert _run(monkeypatch, node, timeout=0.3, round_trips=1,
                goal_x=3.0, goal_y=0.0) is True
    # Two legs, and the run's own exit: the last leg is not exempt.
    assert node.cancelled >= 2
    out = capsys.readouterr().out
    assert "has not closed the goal" in out
    assert "may be rejected" in out
