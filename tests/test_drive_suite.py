"""The drive suite judges the RIGHT sign and a plausible magnitude.

Its whole reason to exist is that 50 Hz topics do not prove motion: the
simulated-wheel invert bug and the PID windup both produced perfect rates on a base
that was spinning in place or pinned at a rail. So the test that guards the
suite is: a report matching the command passes, a reversed-sign report fails,
and a dead-still report fails -- exactly the three the suite is there to tell
apart. The suite talks to a live ROS graph, so here we exercise its decision
rule in isolation.
"""
# drive_suite.py imports rclpy at module scope (it drives a live ROS graph), so
# it is not importable on the build host. Its pure functions are lifted out with
# ast by _from_suite() below -- they used to be copied here, and a copy is how
# this file went green through `_statistic` changing meaning. The three TOLERANCE
# lines are still a copy, because they are expressions inside a closure rather
# than a function; test_the_tolerances_match_the_suite_verbatim holds them.
def judge(want_lin, want_ang, got_lin, got_ang, want_lat=0.0, got_lat=0.0):
    """The suite's rule, copied: a loose sign-and-magnitude tolerance around the
    command. vy is judged by the same rule as vx -- a sideways command is a
    command like any other."""
    ok_vx = abs(got_lin - want_lin) < max(0.12, abs(want_lin) * 0.45)
    ok_vy = abs(got_lat - want_lat) < max(0.12, abs(want_lat) * 0.45)
    ok_wz = abs(got_ang - want_ang) < max(0.45, abs(want_ang) * 0.45)
    return ok_vx and ok_vy and ok_wz


def strafe_expectation(base_type, strafe_speed=0.20):
    """What the sideways pair must produce on this drivetrain: mirrored from
    the suite's `want = STRAFE_SPEED if MECANUM else 0.0`."""
    return strafe_speed if base_type == "mecanum" else 0.0


def _suite_src():
    import os
    return open(os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                             "scripts", "drive_suite.py")).read()


def _from_suite(*names):
    """The suite's own functions, lifted out by ast.

    They used to be MIRRORED here, and the mirror is why this file went green
    through a change of meaning: `_statistic` stopped being the peak in the
    commanded direction and became the median, and the test that exists to keep
    the copy honest -- "matches the suite verbatim" -- was checking a signature
    line and a `return sum(...)/len(...)` that both survived the rewrite. A copy
    cannot be kept in step by checking some of it. drive_suite imports rclpy at
    module scope, so the module will not import on a desk; ast will.
    """
    import ast
    tree = ast.parse(_suite_src())
    want = {n: None for n in names}
    for node in tree.body:
        if isinstance(node, ast.FunctionDef) and node.name in want:
            want[node.name] = node
    missing = [n for n, v in want.items() if v is None]
    assert not missing, f"{missing} are gone from drive_suite.py"
    ns = {}
    exec(compile(ast.Module([want[n] for n in names], []), "drive_suite.py", "exec"), ns)
    return tuple(ns[n] for n in names)


statistic, peak = _from_suite("_statistic", "_peak")


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
# pure, and if it drifts from the suite (or from sim_ld19.h) it stops telling
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


def test_a_commanded_speed_is_judged_by_the_speed_it_HELD():
    """The median, not the extreme.

    The extreme was judged against a SYMMETRIC tolerance, and that pair cannot
    be satisfied by a base with a transient: a peak is always at least as far
    from the command as the sustained speed, so the more a base overshoots on
    the way up, the worse it scores on a statistic picked for being generous.
    """
    assert statistic([0.10, 0.24, 0.26], 0.25) == 0.24
    assert statistic([-0.10, -0.24, -0.26], -0.25) == -0.24


def test_one_overshoot_on_the_way_up_does_not_fail_the_manoeuvre():
    """2026-09-25, GenDrv mecanum jazzy: `vy -0.464 (want -0.20)` marked BAD
    while the pose moved 0.92 m in the 5 s command -- 0.184 m/s, the commanded
    speed almost exactly. The Yahboom, same drivetrain and firmware, reported
    -0.225 and passed. One sample decided a gate leg."""
    strafe = [-0.464, -0.21, -0.20, -0.19, -0.20, -0.21, -0.20]
    assert judge(0.0, 0.0, 0.0, 0.0, statistic(strafe, -0.20), -0.20)
    # and the old rule is what failed it
    assert not judge(0.0, 0.0, 0.0, 0.0, peak(strafe, -0.20), -0.20)


def test_a_base_that_really_holds_the_wrong_speed_still_fails():
    """The looser statistic must not become a blind one: a base running at
    more than twice the commanded strafe for the whole window is a fault, and
    the median says so."""
    runaway = [-0.46, -0.47, -0.45, -0.46, -0.48]
    assert not judge(0.0, 0.0, 0.0, 0.0, statistic(runaway, -0.20), -0.20)


def test_the_peak_is_reported_so_the_two_can_be_told_apart():
    """"Held the wrong speed" and "overshot once" call for opposite responses,
    so a line carrying only the judged number cannot be read."""
    src = _suite_src()
    assert "pk %+.3f" in src, "the peak is no longer printed beside the judged value"
    assert "pk_vx, pk_vy, pk_wz = _peak(" in src


def test_a_commanded_zero_is_judged_by_the_mean_not_the_worst_sample():
    """Measured on the GenDrv 2026-09-22: a 1.5 rad/s spin reported vx peaks of
    +0.15 m/s while the pose moved 0.05 m in 5 s -- standing still. Judged by
    max(), both spins failed; judged by the mean, they pass, which is what the
    pose says happened. A zero command has no direction to peak in, so an
    extremum answers a different question from the one being asked."""
    spin = [0.15, -0.14, 0.12, -0.13, 0.01]        # noise about zero
    assert abs(statistic(spin, 0.0)) < 0.05
    assert judge(0.0, 1.5, statistic(spin, 0.0), 1.52)
    # and the old rule is what failed it
    assert not judge(0.0, 1.5, max(spin), 1.52)


def test_a_base_that_really_creeps_forward_while_spinning_still_fails():
    """The check must keep catching the thing it exists for: a base that drives
    forward when told to spin has a mean that is not zero."""
    creep = [0.18, 0.17, 0.19, 0.16, 0.18]
    assert statistic(creep, 0.0) > 0.12
    assert not judge(0.0, 1.5, statistic(creep, 0.0), 1.52)


def test_the_suite_judges_the_sustained_value_and_not_the_peak():
    """`_statistic` is exercised for real above; what a source check still has
    to say is WHICH number reaches the verdict."""
    src = _suite_src()
    for needle in ("got_vx = _statistic(vx, lin)", "got_vy = _statistic(vy, lat)",
                   "got_wz = _statistic(wz, ang)",
                   "want = STRAFE_SPEED if MECANUM else 0.0", "STRAFE_SPEED = 0.20"):
        assert needle in src, needle
    for judged in ("ok_vx = abs(got_vx", "ok_vy = abs(got_vy", "ok_wz = abs(got_wz"):
        assert judged in src, f"{judged} -- the verdict no longer reads the sustained value"
    assert "abs(pk_" not in src, "the peak is being judged again"


# --- the sideways pair -------------------------------------------------------
# Strafing is the only thing a mecanum base does that 2wd and skid cannot, so a
# drivetrain axis without it tests nothing: a mecanum leg would go green with a
# dead vy channel. The command is identical on every base; only the right answer
# differs, which is what makes the same pair worth running three times.

def test_a_mecanum_base_must_strafe_at_the_commanded_speed():
    want = strafe_expectation("mecanum")
    assert want == 0.20
    assert judge(0.0, 0.0, 0.0, 0.0, want_lat=want, got_lat=0.19)
    assert judge(0.0, 0.0, 0.0, 0.0, want_lat=-want, got_lat=-0.21)


def test_a_mecanum_base_with_a_dead_vy_channel_fails():
    """The whole reason the pair exists: without it this base passes 6/6."""
    assert not judge(0.0, 0.0, 0.0, 0.0, want_lat=0.20, got_lat=0.00)


def test_a_mecanum_base_that_turns_instead_of_strafing_fails():
    """A wrong inverse-kinematics branch yaws the base instead of translating."""
    assert not judge(0.0, 0.0, 0.0, 1.10, want_lat=0.20, got_lat=0.00)


def test_a_differential_base_must_do_nothing_when_told_to_strafe():
    for base in ("2wd", "4wd", "skid_steer"):
        assert strafe_expectation(base) == 0.0
    # commanded sideways, base stayed put and reported nothing: correct.
    assert judge(0.0, 0.0, 0.0, 0.0, want_lat=0.0, got_lat=0.0)


def test_a_differential_base_that_reports_vy_fails():
    """A 2wd base cannot move sideways, so vy on /odom is a broken odometry
    model -- it will be fused, and nothing else in the suite would catch it."""
    assert not judge(0.0, 0.0, 0.0, 0.0, want_lat=0.0, got_lat=0.18)


def test_the_strafe_pair_runs_on_every_drivetrain():
    """Same command, three different right answers -- so the pair is not
    conditioned on the base type, only its expectation is."""
    import os
    src = open(os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                            "scripts", "drive_suite.py")).read()
    assert 'run("strafe left",  0.00, 0.00, lat=+want)' in src
    assert 'run("strafe right", 0.00, 0.00, lat=-want)' in src
    # every base runs them
    assert "STRAFES = True" in src


def test_the_tolerances_match_the_suite_verbatim():
    """`judge` above is a copy of three expressions. Copies drift, and the last
    one drifted silently for the whole of a release gate, so the copy is only
    allowed to exist while something checks it character for character."""
    src = _suite_src()
    for needle in ("ok_vx = abs(got_vx - lin) < max(0.12, abs(lin) * 0.45)",
                   "ok_vy = abs(got_vy - lat) < max(0.12, abs(lat) * 0.45)",
                   "ok_wz = abs(got_wz - ang) < max(0.45, abs(ang) * 0.45)"):
        assert needle in src, f"the suite's tolerance changed: {needle}"
