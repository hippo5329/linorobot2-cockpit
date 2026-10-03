"""While SLAM builds the map from a narrow scan, every yaw-rate source is capped.

A masked LiDAR (210 deg) mapping at 1.0-1.25 rad/s stepped SLAM's heading 13
then 28 deg, odometry right to 0.3 deg, and the robot never got home (gate,
2026-10-03). The cap applies to the velocity smoother (which MPPI's wz_max is
derived from), the rotation shim's turn-to-heading rate and the Spin behaviour --
and to nothing on a 360 deg LiDAR or on a run that localises on a saved map.
"""
import copy
import os
import sys

import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "scripts"))

import mapping_limits  # noqa: E402

CAP = mapping_limits.NARROW_MAPPING_WZ


def _gendrv():
    with open(os.path.join(ROOT, "config", "reference", "gendrv_config.yaml")) as fh:
        return yaml.safe_load(fh)


def _with_scan(params, controller):
    p = copy.deepcopy(params)
    p["base_controller"] = controller
    return p


MASKED = {"lidar": {"model": "LD19", "mask": {"sectors": [[105, 255]]}}}
CAMERA = {"lidar": {"model": "NONE"}, "sensors": {"use_sim_depth": True}}
FULL = {"lidar": {"model": "LD19"}}


def _limits(nav2):
    vs = nav2["velocity_smoother"]["ros__parameters"]
    fp = nav2["controller_server"]["ros__parameters"]["FollowPath"]
    bs = nav2["behavior_server"]["ros__parameters"]
    return vs["max_velocity"][2], vs["min_velocity"][2], fp.get("rotate_to_heading_angular_vel"), bs["max_rotational_vel"]


def test_a_narrow_scan_mapping_turns_no_faster_than_the_cap():
    base = _gendrv()
    for scan in (MASKED, CAMERA):
        params = _with_scan(base, scan)
        nav2 = copy.deepcopy(params["nav2"])
        assert mapping_limits.cap_mapping_turn_rate(nav2, params, map_file="") == CAP
        wz_max, wz_min, shim, spin = _limits(nav2)
        assert wz_max == CAP and wz_min == -CAP
        assert shim <= CAP and spin <= CAP


def test_a_full_lidar_or_a_saved_map_keeps_its_limits():
    base = _gendrv()
    before = _limits(base["nav2"])
    assert before[0] > CAP, "the template already turns slower than the cap -- this test no longer tests anything"
    for scan, map_file in ((FULL, ""), (MASKED, "/maps/saved.yaml"), (CAMERA, "/maps/saved.yaml")):
        params = _with_scan(base, scan)
        nav2 = copy.deepcopy(params["nav2"])
        assert mapping_limits.cap_mapping_turn_rate(nav2, params, map_file=map_file) is None
        assert _limits(nav2) == before


def test_the_cap_comes_before_mppi_derives_wz_max():
    src = open(os.path.join(ROOT, "launchers", "nav2.launch.py")).read()
    assert src.index("mapping_limits.cap_mapping_turn_rate(") < src.index("uses_mppi = mecanum_controller(")
