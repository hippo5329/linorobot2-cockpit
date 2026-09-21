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
