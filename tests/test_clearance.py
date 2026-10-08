"""The 1-Click's room check on a real robot (clearance.py, scan_match.py, test_acc_run.py).

Geometry only -- no ROS: the step a cramped robot inches, the quadrants it reports, the
matcher that finds the start again, and how test_acc's syslog is read. Plus where step
4.8 sits in the pipeline: after the topic gate, before SLAM, real robots only.
"""
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
    i_measure = src.index("nav2_ok, nav2_detail, nav2_log = measure_and_restart()")
    i_goal = src.index("'test_nav2_goal.py'")
    assert i_cons < i_start < i_measure < i_goal
    # with exploration, the explored map is the initial map: measured after it
    i_explore = src.index("explore_watch.py")
    assert src.index("measure_and_restart()", i_explore) > i_explore


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
