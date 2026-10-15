"""The room a real robot is asked for follows the robot: its size (the Nav2 radius, or its
footprint's circle) and, for test_acc's full-power runs, its top speed."""
import os
import sys

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO_ROOT, "scripts"))

import gen_firmware_header as g  # noqa: E402
import one_click_pipeline as ocp  # noqa: E402


def _params(radius=None, footprint=None, rpm=177.0, wheel_d=0.075):
    rp = {}
    if radius is not None:
        rp["robot_radius"] = radius
    if footprint is not None:
        rp["footprint"] = footprint
    return {"kinematics": {"base_type": "mecanum", "wheel_diameter": wheel_d, "motor_max_rpm": rpm,
                           "max_rpm": rpm, "lr_wheels_distance": 0.2, "fr_wheels_distance": 0.2},
            "nav2": {"local_costmap": {"local_costmap": {"ros__parameters": rp}},
                     "global_costmap": {"global_costmap": {"ros__parameters": rp}}}}


def test_a_footprint_sets_the_radius_by_its_farthest_corner():
    assert abs(g.footprint_radius("[[0.3, 0.2], [0.3, -0.2], [-0.3, -0.2], [-0.3, 0.2]]") - 0.3606) < 1e-3
    assert g.footprint_radius([[0.1, 0.0], [0.0, 0.25]]) == 0.25
    assert g.footprint_radius(None) is None and g.footprint_radius("[]") is None
    # the footprint wins over robot_radius, as in Nav2
    r_fp = g.nav2_robot_radius(_params(radius=0.1, footprint="[[0.3, 0.2], [-0.3, -0.2]]"))
    assert r_fp > 0.36 and r_fp > g.nav2_robot_radius(_params(radius=0.1))


def test_the_clearance_grows_with_a_big_robot_and_never_falls_under_0_40():
    assert ocp.clearance_real(_params(radius=0.10)) == ocp.CLEARANCE_REAL_M
    big = ocp.clearance_real(_params(radius=0.35))
    assert big >= g.nav2_robot_radius(_params(radius=0.35)) + ocp.CLEARANCE_EDGE_M - 1e-9


def test_test_acc_asks_a_fast_robot_for_more_than_a_metre():
    slow = ocp.test_acc_room(_params(radius=0.15, rpm=60.0, wheel_d=0.06), 1.0)
    fast = ocp.test_acc_room(_params(radius=0.30, rpm=300.0, wheel_d=0.09), 1.0)
    assert slow == 1.0
    # 300 rpm on 0.09 m wheels: 1.41 m/s, 1.1 s of it, plus the radius
    assert fast > 1.8
    # the floor given (the command line's --test-acc-clearance) still holds
    assert ocp.test_acc_room(_params(radius=0.15, rpm=60.0, wheel_d=0.06), 1.4) == 1.4


def test_a_measured_slow_step_lowers_the_acceleration_limit_and_never_raises_it():
    import copy
    import yaml as _y
    import drivetrain_report as dr
    base = _y.safe_load(open(os.path.join(REPO_ROOT, "config", "reference", "unoq_config.yaml")))
    acc = "nav2.velocity_smoother.ros__parameters.max_accel"
    model = dr.derived_limits(copy.deepcopy(base)).get(acc)
    slow = copy.deepcopy(base)
    slow["kinematics"]["step_response"] = {"max_vel": 0.5, "t_to_90": 1.5}    # 0.3 m/s2: a heavy robot
    got = dr.derived_limits(slow).get(acc)
    assert got and model and got[0] < model[0] and got[0] <= 0.3
    quick = copy.deepcopy(base)
    quick["kinematics"]["step_response"] = {"max_vel": 0.7, "t_to_90": 0.05}  # 12.6 m/s2
    assert dr.derived_limits(quick).get(acc)[0] <= model[0] + 1e-9


def test_a_mecanum_strafes_slower_than_it_drives():
    import drivetrain_report as dr
    import yaml as _y
    p = _y.safe_load(open(os.path.join(REPO_ROOT, "config", "reference", "unoq_config.yaml")))
    assert p["kinematics"]["base_type"] == "mecanum"
    v, vy, _ = dr.derived_limits(p)["nav2.velocity_smoother.ros__parameters.max_velocity"]
    assert 0 < vy < v and abs(vy - round(v * dr.MECANUM_STRAFE_FRAC, 2)) <= 0.011
