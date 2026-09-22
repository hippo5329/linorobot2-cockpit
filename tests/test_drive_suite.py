"""The drive suite judges the RIGHT sign and a plausible magnitude.

Its whole reason to exist is that 50 Hz topics do not prove motion: the
fake-wheel invert bug and the PID windup both produced perfect rates on a base
that was spinning in place or pinned at a rail. So the test that guards the
suite is: a report matching the command passes, a reversed-sign report fails,
and a dead-still report fails -- exactly the three the suite is there to tell
apart. The suite talks to a live ROS graph, so here we exercise its decision
rule in isolation.
"""
# drive_suite.py imports rclpy at module scope (it drives a live ROS graph), so
# it is not importable on the build host. Its DECISION RULE is pure, though, and
# that is the part worth guarding -- so it is mirrored here verbatim. If the two
# ever drift, that is the bug this file exists to catch: keep them identical.
def judge(want_lin, want_ang, got_lin, got_ang):
    """The suite's rule, extracted: the extreme in the commanded direction,
    within a loose sign-and-magnitude tolerance."""
    ok_vx = abs(got_lin - want_lin) < max(0.12, abs(want_lin) * 0.45)
    ok_wz = abs(got_ang - want_ang) < max(0.45, abs(want_ang) * 0.45)
    return ok_vx and ok_wz


def test_a_faithful_report_passes():
    assert judge(0.25, 0.0, 0.24, 0.01)
    assert judge(0.0, 1.5, 0.02, 1.42)
    assert judge(0.20, -0.80, 0.19, -0.77)


def test_a_reversed_spin_fails():
    # commanded left spin, base spun right -- the invert bug
    assert not judge(0.0, 1.5, 0.0, -1.5)


def test_a_dead_still_base_fails_a_move_command():
    assert not judge(0.25, 0.0, 0.0, 0.0)
    assert not judge(0.0, 1.5, 0.0, 0.0)


def test_a_tiny_command_has_a_floor_so_noise_does_not_fail_it():
    # forward 0.0 is not commanded here; this is the turn's small linear part
    assert judge(0.20, 0.80, 0.20, 0.80)


# The room-geometry note, mirrored the same way and for the same reason: it is
# pure, and if it drifts from the suite (or from fake_ld19.h) it stops telling
# a clamped pose from a base fault. Keep these numbers identical to both.
ROOM_W, ROOM_H, ROBOT_R = 10.0, 6.0, 0.30
WALL_X, WALL_HALF_SPAN, NEAR = 2.0, 1.5, 0.05


def where(x, y):
    if x != x or y != y:
        return "pose unknown"
    notes = []
    if abs(abs(x) - (ROOM_W / 2 - ROBOT_R)) < NEAR:
        notes.append("room wall x")
    if abs(abs(y) - (ROOM_H / 2 - ROBOT_R)) < NEAR:
        notes.append("room wall y")
    if abs(y) <= WALL_HALF_SPAN + ROBOT_R and abs(abs(x - WALL_X) - ROBOT_R) < NEAR:
        notes.append("OBSTACLE WALL")
    return ", ".join(notes) if notes else "clear"


def test_the_middle_of_the_room_is_clear():
    assert where(0.0, 0.0) == "clear"
    assert where(1.0, -0.5) == "clear"


def test_being_held_against_the_obstacle_wall_is_named():
    """Measured 2026-09-22 on the GenDrv: both spins reported vx ~ +0.15 m/s
    while commanded (0.00, +/-1.50), both positive. A clamp moves the pose every
    cycle and that differentiates into a velocity nobody commanded -- which
    without the pose reads as a base fault."""
    assert where(WALL_X - ROBOT_R, 0.0) == "OBSTACLE WALL"      # held on the near side
    assert where(WALL_X + ROBOT_R, -1.0) == "OBSTACLE WALL"     # and the far side
    # past the wall's end there is nothing to be held against
    assert where(WALL_X - ROBOT_R, 2.5) == "clear"


def test_the_rooms_own_walls_are_named():
    assert where(ROOM_W / 2 - ROBOT_R, 0.0) == "room wall x"
    assert where(0.0, -(ROOM_H / 2 - ROBOT_R)) == "room wall y"


def test_no_pose_says_so_rather_than_guessing():
    assert where(float("nan"), 0.0) == "pose unknown"


def test_the_note_matches_the_suite_verbatim():
    import os
    src = open(os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                            "scripts", "drive_suite.py")).read()
    for needle in ("ROOM_W, ROOM_H = 10.0, 6.0", "ROBOT_R = 0.30",
                   "WALL_X, WALL_HALF_SPAN = 2.0, 1.5", "NEAR = 0.05",
                   'notes.append("OBSTACLE WALL")', "pose (%+.2f,%+.2f)->(%+.2f,%+.2f) %s"):
        assert needle in src, needle
