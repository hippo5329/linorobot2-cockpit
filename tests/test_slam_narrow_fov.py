"""A narrow scan takes slam.narrow_fov_overrides; a 360 deg LiDAR keeps slam_toolbox's defaults.

Replayed on recorded runs (2026-10-03), the same parameters were right for one
and wrong for the other: held hard to the odometry, a depth camera's map stopped
rotating (3-4 m / 30-39 deg of pose error down to 0.2 m / 2.5 deg), while on the
1280 m2 warehouse a LiDAR lost 1.9 m and two live maps came out rotated 1 deg.
"""
import math
import os
import re
import sys

import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "scripts"))

import depth_camera  # noqa: E402

REFS = ("gendrv", "pico2_mecanum", "xrp", "yb_eet01")
STRONG = {"angle_variance_penalty": 0.02, "minimum_angle_penalty": 0.3,
          "distance_variance_penalty": 0.1, "minimum_distance_penalty": 0.3,
          "loop_match_minimum_response_coarse": 0.6, "loop_match_minimum_response_fine": 0.7,
          "loop_match_minimum_chain_size": 15,
          "loop_search_maximum_distance": 2.0,
          "coarse_search_angle_offset": 0.175,
          "correlation_search_space_dimension": 0.3,
          "use_response_expansion": False}
# slam_toolbox's own defaults (mapper_params_online_async.yaml), which a full LiDAR keeps
UPSTREAM = {"angle_variance_penalty": 1.0, "minimum_angle_penalty": 0.9,
            "distance_variance_penalty": 0.5, "minimum_distance_penalty": 0.5,
            "loop_match_minimum_response_coarse": 0.35, "loop_match_minimum_response_fine": 0.45,
            "loop_match_minimum_chain_size": 10,
            "coarse_search_angle_offset": 0.349}


def _ref(name):
    with open(os.path.join(ROOT, "config", "reference", f"{name}_config.yaml")) as fh:
        return yaml.safe_load(fh)


def _with(controller, slam=None):
    return {"base_controller": controller, "slam": slam if slam is not None else {"narrow_fov_overrides": dict(STRONG)}}


def test_scan_fov_by_source():
    assert depth_camera.scan_fov_deg({"lidar": {"model": "LD19"}}) == 360.0
    assert depth_camera.scan_fov_deg({"lidar": {"model": "LD19", "mask": {"sectors": [[105, 255]]}}}) == 210.0
    assert round(depth_camera.scan_fov_deg({"lidar": {"model": "NONE"}, "sensors": {"use_sim_depth": True}})) == 87
    assert depth_camera.scan_fov_deg({"lidar": {"model": "NONE"}}) is None


def test_overrides_apply_to_a_camera_and_a_masked_lidar_only():
    camera = {"lidar": {"model": "NONE"}, "sensors": {"use_sim_depth": True}}
    masked = {"lidar": {"model": "LD19", "mask": {"sectors": [[105, 255]]}}}
    full = {"lidar": {"model": "LD19"}}
    assert depth_camera.narrow_slam_overrides(_with(camera)) == STRONG
    assert depth_camera.narrow_slam_overrides(_with(masked)) == STRONG
    assert depth_camera.narrow_slam_overrides(_with(full)) == {}
    # no block, no overrides -- whatever the sensor
    assert depth_camera.narrow_slam_overrides(_with(camera, slam={})) == {}


def test_every_reference_carries_the_block_and_keeps_upstream_values():
    for name in REFS:
        slam = _ref(name)["slam"]
        assert slam["narrow_fov_overrides"] == STRONG, name
        rp = slam["slam_toolbox"]["ros__parameters"]
        assert {k: rp[k] for k in UPSTREAM} == UPSTREAM, name


def test_the_launcher_never_passes_the_block_to_the_node():
    src = open(os.path.join(ROOT, "launchers", "slam.launch.py")).read()
    pop = src.index('slam_data.pop("narrow_fov_overrides", None)')
    wrap = src.index('"ros__parameters" not in slam_data.get("slam_toolbox", {})')
    assert pop < wrap, "the block must leave slam_data before it is wrapped for the node"
    assert re.search(r"rp\.update\(overrides\)", src)


def test_a_loop_closure_cannot_reach_the_next_room():
    """loop_search_space_dimension bounds how far one closure moves the map; the rooms
    world repeats every ~2.9 m, so the window (full width) must stay under twice that."""
    for name in REFS + ("makerspet_mini",):
        with open(os.path.join(ROOT, "config", "reference", f"{name}_config.yaml")) as fh:
            rp = yaml.safe_load(fh)["slam"]["slam_toolbox"]["ros__parameters"]
        assert rp["loop_search_space_dimension"] / 2 < 2.9 / 2 + 0.1, name


def test_a_narrow_scan_searches_half_the_angle():
    """rc65: a 90 deg view matched best at the coarse window's edge and the map turned
    in steps of one window (20.8, 41.2, 61.4 deg) until a room was sealed. Half the
    window bounds a false turn; a 360 deg LiDAR keeps slam_toolbox's 0.349 rad."""
    for name in REFS + ("makerspet_mini", "unoq"):
        slam = _ref(name)["slam"]
        narrow = slam["narrow_fov_overrides"]["coarse_search_angle_offset"]
        full = slam["slam_toolbox"]["ros__parameters"]["coarse_search_angle_offset"]
        assert full == 0.349, name
        assert narrow < full and math.degrees(narrow) <= 10.1, name
        # the coarse step still divides the window into more than a few candidates
        assert narrow / slam["slam_toolbox"]["ros__parameters"]["coarse_angle_resolution"] >= 4, name


def test_a_narrow_scan_searches_a_smaller_window():
    """rc-20261013.2, explore_camera_mecanum (87 deg camera): during explore_lite's look-around
    spin the robot stood still while SLAM's pose jumped +0.24 m, then +0.27 m -- each the edge
    of correlation_search_space_dimension 0.5 (+-0.25 m). The walls drawn at the slipped pose
    sealed the room and the return home failed. rc-20261013.4 did it again: 2.18 m. Like the
    angle window (rc65), a narrow scan gets a smaller one; a 360 deg LiDAR keeps 0.5."""
    for name in REFS + ("makerspet_mini", "unoq"):
        slam = _ref(name)["slam"]
        narrow = slam["narrow_fov_overrides"]["correlation_search_space_dimension"]
        full = slam["slam_toolbox"]["ros__parameters"]["correlation_search_space_dimension"]
        assert full == 0.5, name
        assert narrow / 2 <= 0.15 < full / 2, name
        # still several steps of the fine grid wide, or the matcher has nothing to choose from
        step = slam["slam_toolbox"]["ros__parameters"]["correlation_search_space_resolution"]
        assert narrow / step >= 20, name


def test_a_narrow_scan_never_widens_its_angle_search():
    """r136 (d7a3e26): each large error of a narrow scan's map was ONE map->odom step of
    30, 50 or 70 deg -- 10 deg plus karto's response expansion, which re-runs a match that
    scored zero at +20, +40, +60 deg (ScanMatcher::MatchScan). An 87 deg camera (or the
    58 deg Astra) turning to a wall the running buffer has not seen scores zero, and the
    widened search takes the wrong wall: explore_camera_mecanum lyrical -70 deg (never got
    home), astra_mecanum lyrical +30, astra_skid_steer jazzy -51. With loop closing off the
    jumps stayed (trial, Astra 30/26/50 deg), so it was never the closures. Without the
    expansion a zero match keeps the odometry's pose. A 360 deg LiDAR keeps the expansion."""
    for name in REFS + ("makerspet_mini", "unoq"):
        slam = _ref(name)["slam"]
        assert slam["narrow_fov_overrides"]["use_response_expansion"] is False, name
        assert slam["slam_toolbox"]["ros__parameters"]["use_response_expansion"] is True, name
