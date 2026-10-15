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
