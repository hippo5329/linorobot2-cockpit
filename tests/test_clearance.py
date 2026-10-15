"""The 1-Click's room check on a real robot (clearance.py, scan_match.py, test_acc_run.py).

Geometry only -- no ROS: the step a cramped robot inches, the quadrants it reports, the
matcher that finds the start again, and how test_acc's syslog is read. Plus where step
4.8 sits in the pipeline: after the topic gate, before SLAM, real robots only.
"""
import argparse
import math
import os
import re
import sys

import numpy as np
import pytest

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO, "scripts"))

import clearance  # noqa: E402
import scan_match  # noqa: E402
import test_acc_run  # noqa: E402


def wall(x0, y0, x1, y1, step=0.01):
    n = max(2, int(math.hypot(x1 - x0, y1 - y0) / step))
    return np.c_[np.linspace(x0, x1, n), np.linspace(y0, y1, n)]


def room(half=1.5):
    return np.vstack([wall(-half, -half, half, -half), wall(half, -half, half, half),
                      wall(half, half, -half, half), wall(-half, half, -half, -half)])


def test_quadrants_and_nearest():
    P = np.vstack([room(), wall(0.30, -0.2, 0.30, 0.2)])        # a box face 0.30 m ahead
    q = clearance.quadrants(P)
    assert q["front"] == pytest.approx(0.30, abs=0.01)
    assert q["back"] == pytest.approx(1.5, abs=0.01)
    assert clearance.nearest(P) == pytest.approx(0.30, abs=0.01)


def test_step_goes_away_from_the_near_wall():
    P = np.vstack([room(), wall(0.30, -0.5, 0.30, 0.5)])
    h, after = clearance.best_step(P, radius=0.20)
    assert abs(scan_match.wrap(h - math.pi)) < math.radians(1)    # straight back
    assert after == pytest.approx(0.35, abs=0.01)
    turn, direction = clearance.turn_for(h)
    assert turn == pytest.approx(0.0, abs=1e-9) and direction == -1.0  # backs up, no turn


def test_step_from_a_corner_goes_diagonally_out():
    P = np.vstack([room(), wall(0.25, -1.0, 0.25, 1.0), wall(-1.0, 0.25, 1.0, 0.25)])
    h, _ = clearance.best_step(P, radius=0.15)
    assert abs(scan_match.wrap(h - math.radians(-135))) <= math.radians(15)


def test_no_step_when_boxed_in():
    # a ring of 0.22 m all round: every step brings something nearer
    a = np.linspace(0, 2 * math.pi, 720, endpoint=False)
    P = np.c_[0.22 * np.cos(a), 0.22 * np.sin(a)]
    assert clearance.best_step(P, radius=0.20) is None


def test_no_turn_in_place_when_too_close_to_spin():
    # a wall 0.30 m to the left: the best step is to the right, but turning to face it
    # would sweep the circle; only straight steps may be taken
    P = np.vstack([room(), wall(-0.5, 0.30, 0.5, 0.30)])
    step = clearance.best_step(P, radius=0.20, spin_ok=False)
    if step is not None:
        turn, _ = clearance.turn_for(step[0])
        assert turn == pytest.approx(0.0, abs=1e-9)


def test_path_must_keep_the_circle_clear():
    # an opening straight back is blocked by a post the centre would pass 0.10 m from
    P = np.vstack([room(), wall(0.30, -0.5, 0.30, 0.5), np.array([[-0.04, 0.10]])])
    h, _ = clearance.best_step(P, radius=0.20)
    d = np.array([clearance.STEP_M * math.cos(h), clearance.STEP_M * math.sin(h)])
    assert clearance.segment_clearance(P, d) >= min(0.20, clearance.nearest(P)) - 1e-9


def test_locate_finds_a_known_move():
    R = np.vstack([room(), wall(0.4, 0.4, 0.8, 0.4), wall(-0.9, -0.2, -0.9, 0.6)])
    th, tx, ty = math.radians(20), 0.15, -0.10
    c, s = math.cos(th), math.sin(th)
    # the same room seen from a robot at (tx, ty, th): p_robot = Rot(-th)(p_room - t)
    S = (R - np.array([tx, ty])) @ np.array([[c, -s], [s, c]])
    got = scan_match.locate(S, R)
    assert got[0] == pytest.approx(th, abs=math.radians(1))
    assert got[1] == pytest.approx(tx, abs=0.01) and got[2] == pytest.approx(ty, abs=0.01)
    assert got[4] >= 150


def test_scan_points_drop_masked_beams():
    scan = {"angle_min": 0.0, "angle_increment": math.pi / 2, "range_min": 0.05, "range_max": 12.0,
            "ranges": [1.0, float("nan"), 0.10, float("inf")]}
    P = scan_match.scan_points(scan)
    assert P.shape == (1, 2) and P[0, 0] == pytest.approx(1.0)


def test_test_acc_end_lines():
    assert test_acc_run.ended(["IDENT done", "[test_acc] done -- write app=base to leave"])
    assert test_acc_run.ended(["[test_acc] STOPPED: lino-stop -- motors off; write app=base to leave"])
    assert test_acc_run.ended(["test_acc refused: sim_wheel=1, nothing to measure"])
    assert not test_acc_run.ended(["MAX PWM  255.0 -255.0", "IDENT gains kp=0.6"])
    got = test_acc_run.results(["MAX VEL   0.71   0.00 m/s   0.00 rad/s", "noise", "IDENT plant wheel=1"])
    assert got == ["MAX VEL   0.71   0.00 m/s   0.00 rad/s", "IDENT plant wheel=1"]


def _pipeline():
    return open(os.path.join(REPO, "scripts", "one_click_pipeline.py"), encoding="utf-8").read()


def test_step_4_8_sits_before_slam_and_is_on_by_default_only_on_real_robots():
    src = _pipeline()
    i_topics = src.index("Topic verification passed.")
    assert "args.clearance = CLEARANCE_REAL_M if is_real else 0.0" in src
    i_gate = src.index('if scan_from == "lidar" and has_lidar and args.clearance > 0:')
    i_clear = src.index("if not clearance_step(args, params, controller_cfg, args.clearance):")
    i_slam = src.index("# Step 5: SLAM.")
    assert i_topics < i_gate < i_clear < i_slam


def test_topics_only_never_inches():
    src = _pipeline()
    body = src[src.index("def clearance_step("):src.index("def test_acc_phase(")]
    assert 'verb = "inch" if (moves and args.inch) else "check"' in body
    assert "moves = not args.topics_only" in body


def test_test_acc_runs_after_the_initial_map_and_before_the_goals():
    src = _pipeline()
    i_cons = src.index("drivetrain_report.conservative_params(cons)")
    i_start = src.index("nav2_ok, nav2_detail, nav2_log = start_nav2(nav2_cmd(nav2_cfg)")
    i_measure = src.index("nav2_ok, nav2_detail, nav2_log = measure_and_restart(")
    i_goal = src.index("'test_nav2_goal.py'")
    assert i_cons < i_start < i_measure < i_goal
    # with exploration, the explored map is the initial map: measured after it
    i_explore = src.index("ex_res = watch_exploration(")
    assert src.index("measure_and_restart(", i_explore) > i_explore


TRANSCRIPT = """IDENT gains kp=0.6000 ki=0.8000 kd=0.5000 pwm_max=1023 rate_hz=50
IDENT robot base=0 wheels=2 max_rpm=180 ratio=0.800 wheel_d=0.0542
IDENT deadzone wheel=1 pwm=120 duty=0.117
IDENT plant wheel=1 steady_rpm=175.0 tau_ms=40 K=0.17107 peak_rpm=178.0
IDENT plant wheel=2 steady_rpm=171.0 tau_ms=44 K=0.16716 peak_rpm=174.0
IDENT done
MAX VEL   0.49   0.00 m/s   0.00 rad/s
time to 0.9x max vel   0.10 sec
[test_acc] done -- write app=base to leave
"""


def gendrv_reference():
    import yaml
    path = os.path.join(REPO, "config", "reference", "gendrv_config.yaml")
    if not os.path.exists(path):
        pytest.skip("no gendrv reference config")
    return yaml.safe_load(open(path, encoding="utf-8"))


def test_tune_from_test_acc_records_the_measurement():
    import drivetrain_report as dr
    params = gendrv_reference()
    changed = dr.tune_from_test_acc(params, TRANSCRIPT)
    assert changed is not None
    step = params["kinematics"]["step_response"]
    assert step["source"] == "test_acc IDENT, median wheel"
    assert step["max_vel"] > 0 and step["t_to_90"] > 0
    assert dr.tune_from_test_acc(gendrv_reference(), "nothing here") is None


def test_measured_speed_bounds_the_nav2_limits():
    import copy
    import drivetrain_report as dr
    base = gendrv_reference()
    slow = copy.deepcopy(base)
    slow.setdefault("kinematics", {})["step_response"] = {"max_vel": 0.10, "t_to_90": 0.1}
    fast = copy.deepcopy(base)
    fast.setdefault("kinematics", {})["step_response"] = {"max_vel": 0.40, "t_to_90": 0.1}
    key = "nav2.velocity_smoother.ros__parameters.max_velocity"
    assert dr.derived_limits(slow)[key][0] < dr.derived_limits(fast)[key][0]


def test_conservative_halves_what_the_config_sets():
    import drivetrain_report as dr
    params = {"nav2": {"velocity_smoother": {"ros__parameters": {
        "max_velocity": [0.22, 0.0, 1.13], "max_accel": [0.8, 0.0, 1.4]}},
        "controller_server": {"ros__parameters": {"FollowPath": {"desired_linear_vel": 0.18}}}}}
    changed = dr.conservative_params(params)
    vs = params["nav2"]["velocity_smoother"]["ros__parameters"]
    assert vs["max_velocity"] == [0.11, 0.0, 0.565]
    assert vs["max_accel"] == [0.4, 0.0, 0.7]
    assert params["nav2"]["controller_server"]["ros__parameters"]["FollowPath"]["desired_linear_vel"] == 0.09
    assert len(changed) == 3          # keys it does not set are left to the template


# ---------------------------------------------------------------- the home snapshot's frame
# 2026-10-14, a UNO Q: /scan was up 6 s before the base_link -> laser TF, the 5 s wait gave up,
# and the start snapshot was saved in the scan frame. After test_acc the drive home matched
# base_link points against it and refused a weak fix, 0.5 m and 60 deg from the start.

class _R:
    def __init__(self, tf_ok, laser=(0.095, 0.0, 0.0)):
        self.tf_ok, self.laser, self.frame = tf_ok, laser, "laser"

    def wait_odom(self, timeout=5.0):
        return (1.0, 2.0, 0.5)


def test_a_snapshot_without_the_tf_is_refused(tmp_path):
    import numpy as np
    P = np.array([[1.0, 0.0], [0.0, 1.0]])
    assert not clearance.save(str(tmp_path / "s.json"), P, _R(False))
    assert not (tmp_path / "s.json").exists()
    assert clearance.save(str(tmp_path / "s.json"), P, _R(True))
    import json
    assert json.load(open(tmp_path / "s.json"))["laser"] == [0.095, 0.0, 0.0]
    assert json.load(open(tmp_path / "s.json"))["odom"] == [1.0, 2.0, 0.5]


def test_the_odometry_since_the_snapshot_is_in_the_snapshot_frame():
    import math
    # facing +y at (1, 2); now 0.2 m further along +y and turned 15 deg more: 0.2 m ahead
    th, x, y = clearance.odom_since((1.0, 2.0, math.pi / 2),
                                    (1.0, 2.2, math.pi / 2 + math.radians(15)))
    assert abs(x - 0.2) < 1e-9 and abs(y) < 1e-9 and abs(math.degrees(th) - 15) < 1e-9
    # moved 0.3 m to its right
    th, x, y = clearance.odom_since((0.0, 0.0, math.pi / 2), (0.3, 0.0, math.pi / 2))
    assert abs(x) < 1e-9 and abs(y + 0.3) < 1e-9


def test_an_old_snapshot_in_the_scan_frame_is_re_expressed_in_base_link():
    import math
    import numpy as np
    in_laser = np.array([[1.0, 0.0], [0.0, 2.0]])
    got = clearance.snapshot_points({"points": in_laser.tolist()}, (0.095, 0.0, 0.0))
    assert np.allclose(got, [[1.095, 0.0], [0.095, 2.0]])
    turned = clearance.snapshot_points({"points": in_laser.tolist(), "laser": [0.0, 0.0, 0.0]},
                                       (0.0, 0.1, math.pi / 2))
    assert np.allclose(turned, [[0.0, 1.1], [-2.0, 0.1]])


def test_a_snapshot_with_the_same_mount_is_used_as_saved():
    import numpy as np
    P = [[1.0, 0.5], [-0.3, 2.0]]
    got = clearance.snapshot_points({"points": P, "laser": [0.095, 0.0, 0.0]}, (0.095, 0.0, 0.0))
    assert np.allclose(got, P)


def test_the_tf_wait_outlasts_a_slow_robot_computer():
    assert clearance.TF_WAIT_TRIES * 0.2 >= 20.0
    src = open(os.path.join(REPO, "scripts", "clearance.py")).read()
    assert "laser_pose(self.node, self.frame or \"laser\", tries=TF_WAIT_TRIES)" in src


# ---------------------------------------------------------------- the first fix's gate
# A UNO Q robot's masked LiDAR saw 288 points; after test_acc it matched 133 of them at rms
# 0.015 and the fixed 150 refused it. The minimum follows the snapshot now -- and, because a
# tracked robot's fixes put it in two wrong places with 127-189 pairs the same day, a fix is
# also refused when a different pose fits nearly as well.

def _room(walls, step=0.03):
    pts = []
    for (x0, y0), (x1, y1) in walls:
        n = max(2, int(math.hypot(x1 - x0, y1 - y0) / step))
        pts += [(x0 + (x1 - x0) * i / n, y0 + (y1 - y0) * i / n) for i in range(n + 1)]
    return np.asarray(pts, float)


RECT = [((-2, -1.5), (2, -1.5)), ((2, -1.5), (2, 1.5)), ((2, 1.5), (-2, 1.5)), ((-2, 1.5), (-2, -1.5))]


def test_the_pair_minimum_follows_the_snapshot():
    assert clearance.home_min_pairs(1000) == 150
    assert clearance.home_min_pairs(288) == 115          # 133 pairs now pass
    assert clearance.home_min_pairs(150) == 100          # never under the floor


def test_a_symmetric_room_is_refused_as_ambiguous():
    home = _room(RECT)
    cands = scan_match.locate_candidates(home.copy(), home)
    why = clearance.first_fix_fault(cands, len(home))
    assert "ambiguous" in why, why


def test_an_asymmetric_room_is_driven_on():
    walls = RECT + [((0.8, -1.5), (0.8, -0.9)), ((0.8, -0.9), (1.4, -0.9)), ((-2, 0.6), (-1.3, 1.5))]
    home = _room(walls)
    c, s = math.cos(0.2), math.sin(0.2)
    moved = (home - [0.3, -0.2]) @ np.array([[c, -s], [s, c]])      # the robot 0.36 m off, turned
    cands = scan_match.locate_candidates(moved, home)
    assert clearance.first_fix_fault(cands, len(home)) == "", cands[:3]


def test_a_weak_fix_names_what_it_needed():
    cands = [(90, (0.0, 0.0, 0.0, 0.015, 90))]
    assert "need 115 of a 288-point snapshot" in clearance.first_fix_fault(cands, 288)


# The drive home on a mecanum base with a masked LiDAR (2026-10-15): it turned to face home,
# the masked rear swung across the snapshot, the match fell under 60 pairs mid-turn and the
# run ended LOST 0.22 m from home.

def test_a_holonomic_base_slides_home_without_turning():
    # 0.22 m to the right of home, facing as it did: home is straight to its left
    vx, vy, wz = clearance.home_step((math.radians(1.0), 0.006, -0.217), 0.217, holonomic=True)
    assert vy > 0.1 and abs(vx) < 0.01 and wz == 0.0
    # turned 90 deg left, home behind its right flank: it slides there and turns slowly back
    vx, vy, wz = clearance.home_step((math.pi / 2, 0.0, -0.2), 0.2, holonomic=True)
    assert vx > 0.1 and abs(vy) < 0.01 and wz == -0.3


def test_a_diff_drive_base_still_turns_to_face_home_first():
    vx, vy, wz = clearance.home_step((0.0, 0.006, -0.217), 0.217, holonomic=False)
    assert vx == 0.0 and vy == 0.0 and abs(wz) >= 0.2


def test_the_corridor_follows_the_direction_of_travel():
    P = np.array([[0.0, 0.25]])                       # a box 0.25 m to the left
    assert clearance.path_blocked(P, 0.0, 0.1, 0.20) == pytest.approx(0.25)
    assert clearance.path_blocked(P, 0.1, 0.0, 0.20) == 0.0
    assert clearance.path_blocked(P, 0.0, 0.0, 0.20) == 0.0


class _StillRobot:
    """Scans from a list (the last one repeats); commands are recorded, nothing moves."""

    def __init__(self, scans):
        self.scans, self.scan, self.sent = list(scans), None, []

    def claim_cmd_vel(self):
        return True

    def next_scan(self, timeout=1.0):
        self.scan = self.scans.pop(0) if len(self.scans) > 1 else self.scans[0]
        return self.scan

    def points(self, n=5, timeout=15.0):             # standing still: the scan just read
        return self.scan

    def mount(self):
        return (0.0, 0.0, 0.0)

    def send(self, vx, wz, vy=0.0):
        self.sent.append((vx, vy, wz))

    def stop(self):
        self.sent.append((0.0, 0.0, 0.0))

    def spin(self, secs):
        pass

    def wait_odom(self, timeout=5.0):
        return None


class _TurningRobot:
    """/odom follows the last command 0.1 s late; enough of a robot for odom_turn."""

    def __init__(self):
        self.odom, self.cmds, self.wz, self.t = (0.0, 0.0, 0.0), [], 0.0, 0.0

    def wait_odom(self, timeout=5.0):
        return self.odom

    def spin(self, secs):
        self.t += secs
        if len(self.cmds) > 5:                     # 0.1 s of commands still in flight
            self.wz = self.cmds[-6]
        x, y, yaw = self.odom
        self.odom = (x, y, scan_match.wrap(yaw + self.wz * secs))

    def send(self, vx, wz, vy=0.0):
        self.cmds.append(wz)

    def stop(self):
        self.cmds += [0.0] * 6
        for _ in range(6):
            self.spin(0.02)


def test_the_last_turn_is_closed_on_odom_and_lands_within_a_few_degrees():
    r = _TurningRobot()
    assert clearance.odom_turn(r, math.radians(-9.0))
    assert abs(math.degrees(r.odom[2]) + 9.0) < 3.0, math.degrees(r.odom[2])
    assert max(abs(w) for w in r.cmds) <= clearance.ODOM_TURN_WZ


def test_no_odom_falls_back_to_the_lidar_closed_turn():
    assert clearance.odom_turn(_StillRobot([np.zeros((1, 2))]), 0.2) is False


def _home_run(monkeypatch, tmp_path, scans, home):
    import argparse
    monkeypatch.setattr(clearance.scan_match, "scan_points", lambda s, laser: s)
    monkeypatch.setattr(clearance, "snapshot_points", lambda snap, laser: home)
    snap = tmp_path / "snap.json"
    snap.write_text("{}")
    a = argparse.Namespace(snap=str(snap), timeout=1.5, tol=0.05, tol_deg=3.0, radius=0.20,
                           holonomic=True, trace=False)
    return clearance.cmd_home(a, _StillRobot(scans))


WALLS = RECT + [((0.8, -1.5), (0.8, -0.9)), ((0.8, -0.9), (1.4, -0.9)), ((-2, 0.6), (-1.3, 1.5))]


def test_a_weak_match_stops_and_looks_again_instead_of_giving_up(monkeypatch, tmp_path, capsys):
    home = _room(WALLS)
    c, s = math.cos(0.2), math.sin(0.2)
    good = (home - [0.3, -0.2]) @ np.array([[c, -s], [s, c]])   # 0.36 m off, turned
    junk = np.random.default_rng(1).uniform(-5, 5, (40, 2))     # what a blurred turn matches
    _home_run(monkeypatch, tmp_path, [good] + [junk] * clearance.TRACK_BAD_SCANS + [good], home)
    out = capsys.readouterr().out
    assert "relocate 1:" in out and "LOST" not in out, out


def test_a_robot_that_cannot_be_found_again_is_lost_with_the_reason(monkeypatch, tmp_path, capsys):
    home = _room(WALLS)
    junk = np.random.default_rng(2).uniform(-5, 5, (40, 2))
    c, s = math.cos(0.2), math.sin(0.2)
    good = (home - [0.3, -0.2]) @ np.array([[c, -s], [s, c]])
    assert _home_run(monkeypatch, tmp_path, [good] + [junk], home) == 1
    out = capsys.readouterr().out
    assert "HOME_FAILED" in out and "relocate 1:" in out and "LOST (the first fix is weak" in out, out


def test_both_home_drives_pass_holonomic_for_a_mecanum_base_only():
    import one_click_pipeline as ocp
    assert ocp.holonomic_flag({"kinematics": {"base_type": "mecanum"}}) == " --holonomic"
    assert ocp.holonomic_flag({"kinematics": {"base_type": "2wd"}}) == ""
    assert ocp.holonomic_flag({}) == ""
    for f in ("one_click_pipeline.py", "robot_bringup.py"):
        src = open(os.path.join(REPO, "scripts", f)).read()
        assert "holonomic_flag(" in src.split("clearance.py')} home --snap", 1)[1][:200], f


# ---- a LiDAR with a blind sector looks around (MC200: its rear 130..225 deg is the mast) ----

BLIND = [(129.0, 97.0)]


def test_blind_sectors_wrap_and_parse():
    assert clearance.parse_blind(["129:97", "350:20"]) == [(129.0, 97.0), (350.0, 20.0)]
    m = clearance.in_blind([128.0, 130.0, 225.0, 227.0, 355.0, 5.0, 11.0], [(129, 97), (350, 20)])
    assert m.tolist() == [False, True, True, False, True, True, False]
    with pytest.raises(ValueError):
        clearance.parse_blind(["10:0"])


def test_an_unseen_bearing_is_named_and_not_judged(capsys):
    P = wall(1.2, -1.0, 1.2, 1.0)                       # only the front wall; behind is blind
    assert clearance.report(P, 1.0, unseen=BLIND)
    assert "unseen: 129..226 deg" in capsys.readouterr().out


def test_no_step_heads_into_a_blind_sector():
    P = wall(0.30, -1.0, 0.30, 1.0)                     # a wall just ahead: backing off is best
    h, _ = clearance.best_step(P, 0.20)
    assert abs(math.degrees(scan_match.wrap(h))) > 150.0
    step = clearance.best_step(P, 0.20, unseen=BLIND)
    assert step is None or not clearance.in_blind([math.degrees(step[0]) % 360], BLIND)[0]


def test_a_carried_look_keeps_the_old_points_only_where_it_is_blind():
    old = np.array([[-0.5, 0.0], [0.5, 0.0]])            # one behind, one ahead
    new = np.array([[0.6, 0.0]])
    out = clearance.carry(old, (0.0, 0.1, 0.0), new, BLIND)   # moved 0.1 m forward
    assert out.tolist() == [[0.6, 0.0], [-0.6, 0.0]]
    assert clearance.carry(old, (0.0, 0.1, 0.0), new, []) is new


class _LookRobot:
    """Turns in place instantly; each look sees the room from its heading, minus the blind
    sector and the chassis -- and, from `intruder_after` turns on, a box just behind."""

    topic, laser = "/scan", (0.0, 0.0, 0.0)

    def __init__(self, world, blind=BLIND, odom_err_deg=0.0, intruder_after=None):
        self.world, self.blind, self.yaw, self.turns = world, blind, 0.0, []
        self.err, self.intruder_after = math.radians(odom_err_deg), intruder_after
        self.x = self.y = 0.0
        self.slides = []

    def wait_odom(self, timeout=5.0):
        return (self.x, self.y, scan_match.wrap(self.yaw * (1.0 + self.err)))

    def mount(self):
        return self.laser

    def claim_cmd_vel(self):
        return True

    def slide(self, dist, heading, radius, timeout=None, guard=True):
        h = self.yaw + heading
        self.x += dist * math.cos(h)
        self.y += dist * math.sin(h)
        self.slides.append((round(dist, 3), round(math.degrees(scan_match.wrap(heading))), guard))
        return True

    def turn_by(self, angle, timeout=15.0, guard=0.0):
        if guard and self.intruder_after is not None and len(self.turns) >= self.intruder_after:
            return False
        self.turns.append(math.degrees(angle))
        self.yaw = scan_match.wrap(self.yaw + angle)
        return True

    def spin(self, secs):
        pass

    def stop(self):
        pass

    def points(self, n=5, timeout=15.0):
        P = clearance.into_frame(self.world, (self.yaw, self.x, self.y))
        keep = ~clearance.in_blind(clearance.bearings(P), self.blind)
        return P[keep & (np.hypot(P[:, 0], P[:, 1]) >= scan_match.BODY_M)]


L_ROOM = [((-1.6, -1.2), (2.2, -1.2)), ((2.2, -1.2), (2.2, 0.4)), ((2.2, 0.4), (1.4, 1.3)),
          ((1.4, 1.3), (-1.6, 1.3)), ((-1.6, 1.3), (-1.6, -1.2))]


def test_a_blind_lidar_sees_the_whole_circle_and_turns_back(capsys):
    world = np.vstack([_room(L_ROOM), wall(-0.45, -0.3, -0.45, 0.3)])   # a box behind, 0.45 m
    r = _LookRobot(world)
    P0 = r.points()
    assert clearance.nearest(P0) > 0.9                  # the first look misses the box
    P = clearance.look_around(r, P0, BLIND, 0.20, 5)
    assert P is not None and abs(clearance.nearest(P) - 0.45) < 0.03
    assert sum(r.turns[:-1]) >= 97.0 and len(r.turns) <= 6
    assert abs(math.degrees(r.yaw)) < 1.0               # back at the start heading
    assert not clearance.report(P, 1.0)
    assert "deg still unseen" in capsys.readouterr().out


def test_the_merge_follows_the_match_not_a_drifting_odom():
    world = np.vstack([_room(L_ROOM), wall(-0.8, -0.3, -0.8, 0.3)])
    r = _LookRobot(world, odom_err_deg=8.0)             # /odom reads 8 % more turn than happened
    P = clearance.look_around(r, r.points(), BLIND, 0.20, 5)
    box = P[(P[:, 0] < -0.5) & (P[:, 0] > -1.2) & (np.abs(P[:, 1]) < 0.3)]
    assert len(box) and np.abs(box[:, 0] + 0.8).max() < 0.03


def test_no_look_around_with_something_inside_the_turning_circle(capsys):
    world = np.vstack([_room(L_ROOM), wall(0.21, -0.1, 0.21, 0.1)])
    r = _LookRobot(world)
    assert clearance.look_around(r, r.points(), BLIND, 0.20, 5) is None
    assert r.turns == [] and "LOOK-AROUND REFUSED" in capsys.readouterr().out


def test_a_turn_stopped_by_the_guard_turns_back_and_refuses():
    r = _LookRobot(_room(L_ROOM), intruder_after=2)
    assert clearance.look_around(r, r.points(), BLIND, 0.20, 5) is None
    assert np.allclose(r.turns[:2], [30.0, 30.0]) and abs(math.degrees(r.yaw)) < 1.0


def test_a_whole_circle_lidar_never_turns():
    r = _LookRobot(_room(L_ROOM), blind=[])
    P = clearance.look_around(r, r.points(), [], 0.20, 5)
    assert P is not None and r.turns == []


def test_the_callers_pass_the_mask_as_blind_and_look_only_when_the_run_moves():
    import one_click_pipeline as ocp
    cfg = {"lidar": {"mask": {"sectors": [[130, 225]]}}}
    assert ocp.blind_flags(cfg) == " --blind 129:97 --look-around"
    assert ocp.blind_flags(cfg, look=False) == " --blind 129:97"
    assert ocp.blind_flags({"lidar": {}}) == ""
    src = open(os.path.join(REPO, "scripts", "one_click_pipeline.py")).read()
    assert "blind_flags(controller_cfg, look=moves)" in src
    src = open(os.path.join(REPO, "scripts", "robot_bringup.py")).read()
    assert "blind_flags(robot.tgt)" in src.split("clearance.py')} check --need", 1)[1][:200]


# ---- before the turn: onto ground already seen free ----

LASER = (0.095, 0.0)


def _seen(world, blind=BLIND):
    P = world - np.array(LASER)
    keep = ~clearance.in_blind(clearance.bearings(P), blind)
    P = world[keep]
    return P[np.hypot(P[:, 0], P[:, 1]) >= scan_match.BODY_M]


def test_a_blind_robot_moves_forward_until_its_turning_circle_is_on_seen_ground():
    P = _seen(_room(L_ROOM))
    h, d = clearance.open_spot(P, BLIND, 0.33, False, 0.8, LASER)
    assert h == 0.0 and 0.30 <= d <= 0.45                # just past its own circle, no further
    ranges = clearance.free_ranges(P, BLIND, LASER)
    assert clearance.known_free(clearance.disk((d, 0.0), 0.33), ranges, LASER).all()
    assert not clearance.known_free(clearance.disk((0.0, 0.0), 0.33), ranges, LASER).all()


def test_no_open_ground_ahead_is_refused_for_a_base_that_must_turn():
    world = np.vstack([_room(L_ROOM), wall(0.65, -1.0, 0.65, 1.0)])     # a wall 0.65 m ahead
    assert clearance.open_spot(_seen(world), BLIND, 0.33, False, 0.8, LASER) is None


def test_a_mecanum_base_slides_aslant_where_a_turning_base_is_refused():
    # a wall ahead and to the right: straight on is too short, 15 deg left is not; a plain
    # sideways slide would sweep the edge of the blind sector, so it is never chosen
    P = _seen(np.vstack([_room(L_ROOM), wall(0.65, -1.0, 0.65, -0.2)]))
    assert clearance.open_spot(P, BLIND, 0.33, False, 0.8, LASER) is None
    h, d = clearance.open_spot(P, BLIND, 0.33, True, 0.8, LASER)
    assert 0.0 < math.degrees(h) <= 45.0 and d <= 0.5
    P = _seen(np.vstack([_room(L_ROOM), wall(0.65, -1.0, 0.65, 0.2)]))
    assert clearance.open_spot(P, BLIND, 0.33, True, 0.8, LASER) is None


def test_a_dark_bearing_is_known_free_only_a_metre_out():
    P = _seen(_room([((3.0, -3.0), (3.0, 3.0))]))        # only a far wall ahead
    r = clearance.free_ranges(P, BLIND, LASER)
    assert r[90] == pytest.approx(clearance.KNOWN_NO_RETURN_M) and r[180] == 0.0


def test_a_whole_circle_lidar_turns_where_it_stands():
    assert clearance.open_spot(_seen(_room(L_ROOM), []), [], 0.33, False, 0.8, LASER) == (0.0, 0.0)


def test_the_callers_pass_holonomic_to_the_look_around():
    src = open(os.path.join(REPO, "scripts", "one_click_pipeline.py")).read()
    assert "blind_flags(controller_cfg, look=moves)}{holonomic_flag(params)}" in src
    src = open(os.path.join(REPO, "scripts", "robot_bringup.py")).read()
    assert "holonomic_flag(robot.params)" in src.split("clearance.py')} check --need", 1)[1][:300]


def test_the_look_ends_where_the_robot_was_put_and_a_second_does_not_creep():
    # MC200, 2026-10-15: each look moved it 0.35 m forward and left it there; the clearance
    # step and test_acc's own look walked it onto boxes 0.51 m ahead, and test_acc found no
    # room. The move to open ground is retraced over the ground it swept, unguarded (it backs
    # into the blind sector), and the merged circle comes back into the start frame.
    world = np.vstack([_room(L_ROOM), wall(0.88, -0.3, 0.88, 0.3), wall(-0.6, -0.3, -0.6, 0.3)])
    r = _LookRobot(world)
    a = argparse.Namespace(blind=["129:97"], scans=5, look_around=True, radius=0.30,
                           holonomic=True, look_travel=0.8)
    for _ in range(2):
        P, unseen, odom0, rc = clearance.first_look(a, r)
        assert rc == 0 and unseen == () and P is not None
        assert math.hypot(r.x, r.y) < 1e-9 and abs(math.degrees(r.yaw)) < 1.0
        ahead = P[(np.abs(P[:, 1]) < 0.25) & (P[:, 0] > 0)]
        behind = P[(np.abs(P[:, 1]) < 0.25) & (P[:, 0] < 0)]
        assert abs(ahead[:, 0].min() - 0.88) < 0.03     # the boxes, from where it stands
        assert abs(behind[:, 0].max() + 0.60) < 0.03    # the box the first look could not see
        assert np.allclose(odom0[:2], (0.0, 0.0))
    out, back = r.slides[0], r.slides[1]
    assert out[2] and not back[2] and back[0] == out[0]
    assert abs(scan_match.wrap(math.radians(back[1] - out[1] - 180))) < 1e-6


def test_a_look_that_fails_after_the_move_still_comes_back(capsys):
    # MC200, 2026-10-15: test_acc's look moved 0.35 m, then /scan_raw stopped; the robot was
    # left out there. The way back needs no scan: it retraces ground it has just swept.
    world = np.vstack([_room(L_ROOM), wall(0.88, -0.3, 0.88, 0.3)])
    r = _LookRobot(world)
    seen = r.points
    calls = []

    def points(n=5, timeout=15.0):
        calls.append(n)
        return seen() if len(calls) == 1 else None
    r.points = points
    a = argparse.Namespace(blind=["129:97"], scans=5, look_around=True, radius=0.30,
                           holonomic=True, look_travel=0.8)
    P, _, _, rc = clearance.first_look(a, r)
    assert P is None and rc == 2 and "no scan" in capsys.readouterr().out
    assert len(r.slides) == 2 and not r.slides[1][2]
    assert math.hypot(r.x, r.y) < 1e-9 and r.turns == []



def test_test_acc_s_count_of_its_move_gives_the_first_fix_another_start():
    # MC200, 2026-10-15: test_acc turned it 45-60 deg; from home's spot every 30 deg the fix
    # was weak or had a rival, and its odometry had restarted with the board. test_acc's own
    # HOME line, even 20 deg off, starts the match where it settles alone.
    world = np.vstack([_room(L_ROOM), wall(1.23, -0.3, 1.23, 0.3)])
    r = _LookRobot(world)
    a = argparse.Namespace(blind=["129:97"], scans=5, look_around=True, radius=0.30,
                           holonomic=True, look_travel=0.8)
    home, _, _, _ = clearance.first_look(a, r)
    r.yaw, r.x, r.y = math.radians(58.2), 0.33, -0.157
    S = r.points()
    alone = scan_match.locate_candidates(S, home)
    assert "ambiguous" in clearance.first_fix_fault(alone, len(home))
    both = sorted(alone + scan_match.locate_candidates(S, home, guess=(math.radians(38.2), 0.33, -0.157)),
                  key=lambda c: -c[0])
    assert clearance.first_fix_fault(both, len(home)) == ""
    th, x, y = both[0][1][:3]
    assert abs(math.degrees(th) - 58.2) < 5 and math.hypot(x - 0.33, y + 0.157) < 0.05


def test_home_guesses_read_test_acc_s_last_home_line(tmp_path):
    import one_click_pipeline as ocp
    f = tmp_path / "t.txt"
    f.write_text("HOME ident x=0.079 y=-0.004 yaw_imu=16.5 yaw_wheels=0.9 net_m=0,0,0,0\n"
                 "noise\nHOME runs x=0.198 y=-0.284 yaw_imu=45.5 yaw_wheels=23.3 net_m=0,0,0,0\n")
    assert ocp.home_guesses(str(f)) == " --guess 45.5:0.198:-0.284 --guess 23.3:0.198:-0.284"
    f.write_text("HOME runs x=0.1 y=0.2 yaw_imu=5.0 yaw_wheels=5.0 net_m=0,0,0,0\n")
    assert ocp.home_guesses(str(f)) == " --guess 5.0:0.1:0.2"
    f.write_text("MAX VEL 0.7\n")
    assert ocp.home_guesses(str(f)) == "" and ocp.home_guesses(str(tmp_path / "none")) == ""
    for src in ("one_click_pipeline.py", "robot_bringup.py"):
        text = open(os.path.join(os.path.dirname(__file__), "..", "scripts", src)).read()
        assert "home_guesses(transcript)" in text, src
