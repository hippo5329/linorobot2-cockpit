"""Frontier exploration runs on the package's own parameters, with only this robot's written over.

explore_lite's vendor launch file defaults use_sim_time to TRUE, which on a real
clock never lets a timer fire; the base frame and namespace are the robot's.
"""
import os
import sys

import yaml

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO_ROOT, "scripts"))


def _explore_params():
    src = open(os.path.join(REPO_ROOT, "launchers", "explore.launch.py")).read()
    a = src.index("def explore_params(")
    b = src.index("\ndef ", a + 10)
    import cockpit_paths
    ns = {"os": os, "yaml": yaml, "cockpit_paths": cockpit_paths}
    exec(src[a:b], ns)
    return ns["explore_params"]


def _share(tmp_path):
    (tmp_path / "config").mkdir()
    (tmp_path / "config" / "params_costmap.yaml").write_text(yaml.safe_dump({"explore_node": {"ros__parameters": {
        "robot_base_frame": "base_link", "return_to_init": True, "costmap_topic": "/global_costmap/costmap",
        "planner_frequency": 0.15, "min_frontier_size": 0.75}}}))
    return str(tmp_path)


def test_the_vendor_values_stand_and_the_clock_is_real(tmp_path):
    rp = _explore_params()({}, _share(tmp_path))
    assert rp["planner_frequency"] == 0.15 and rp["return_to_init"] is True
    assert rp["use_sim_time"] is False
    assert rp["costmap_topic"] == "global_costmap/costmap", "the costmap, relative for a namespace"


def test_the_last_doorway_is_a_frontier(tmp_path):
    """The camera's last view of the east door was a 5-cell frontier (0.25 m); it must count,
    and a 2-cell scrap must not."""
    rp = _explore_params()({}, _share(tmp_path))
    assert 2 * 0.05 < rp["min_frontier_size"] <= 5 * 0.05 - 0.04


def test_the_robot_decides_its_frame_and_tuning(tmp_path):
    params = {"base_controller": {"topic_prefix": "r1"}, "slam": {"base_frame": "base_link"},
              "explore": {"ros__parameters": {"min_frontier_size": 0.4}}}
    rp = _explore_params()(params, _share(tmp_path))
    assert rp["robot_base_frame"] == "r1/base_link"
    assert rp["min_frontier_size"] == 0.4


def test_the_pipeline_explores_instead_of_the_fixed_goal():
    pipe = open(os.path.join(REPO_ROOT, "scripts", "one_click_pipeline.py")).read()
    assert "ros2 launch linorobot2_cockpit explore.launch.py" in pipe
    assert "explore_watch.py" in pipe and '"--explore"' in pipe
    watch = open(os.path.join(REPO_ROOT, "scripts", "explore_watch.py")).read()
    assert '"returned_to_origin"' in watch and "TRANSIENT_LOCAL" in watch
    assert "test -x /opt/lino_ws/lib/explore_lite/explore" in open(os.path.join(REPO_ROOT, "docker", "Dockerfile")).read()


def test_explore_is_default_unless_disabled_manually():
    pipe = open(os.path.join(REPO_ROOT, "scripts", "one_click_pipeline.py")).read()
    assert '"--no-explore"' in pipe
    assert "args.explore = not args.require_goal" in pipe
    html = open(os.path.join(REPO_ROOT, "web", "frontend", "index.html")).read()
    assert 'id="cockpit-explore" checked' in html
    assert 'id="cockpit-pipeline-world"' in html


def test_multi_room_exploration_world():
    import depth_camera
    params = {"base_controller": {"simulation": {"world": "rooms"}}}
    walls = depth_camera.sim_walls(params)
    assert len(walls) == len(depth_camera.MULTI_ROOM_WALLS)
    assert depth_camera.sim_room(params)["wall_obstacle"] is False


def test_feed_map_to_simulator():
    pipe = open(os.path.join(REPO_ROOT, "scripts", "one_click_pipeline.py")).read()
    assert '"--world-map"' in pipe
    assert 'world_map:={args.world_map}' in pipe
    bringup = open(os.path.join(REPO_ROOT, "launchers", "bringup.launch.py")).read()
    assert 'world_map_arg' in bringup




def _watch():
    import importlib.util
    spec = importlib.util.spec_from_file_location("explore_watch", os.path.join(REPO_ROOT, "scripts", "explore_watch.py"))
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def test_battery_stop_is_the_median_under_load():
    """A lone dip under a step's load is not the floor; the median of ten readings is (a hexapod stopped
    at 7.0 V read 6.45 V on its way home: the stop must leave the walk home its margin)."""
    w = _watch()
    assert not w.battery_low([7.4] * 10, 0.0)          # off
    assert not w.battery_low([7.1, 7.1], 7.2)          # too few readings to judge
    assert not w.battery_low([7.4] * 7 + [6.9] * 3, 7.2)
    assert w.battery_low([7.4] * 3 + [7.1] * 7, 7.2)
    assert not w.battery_low([7.0] * 20 + [7.3] * 10, 7.2)  # only the last ten count


def test_a_robot_pinned_in_place_is_stalled():
    """Under 5 cm and 15 deg on the map in the window: a hexapod held against a chair leg (2026-11-09).
    Turning in place (a survey, a recovery spin) and walking are not stalls."""
    import math
    w = _watch()
    still = [(t, 1.0 + 0.001 * (t % 2), 2.0, 0.1) for t in range(0, 100)]
    assert w.stalled(still, 99, 90)
    assert not w.stalled(still, 99, 0)                 # off
    assert not w.stalled(still[:50], 49, 90)           # not watched long enough
    walking = [(t, 0.01 * t, 0.0, 0.0) for t in range(100)]
    assert not w.stalled(walking, 99, 90)
    turning = [(t, 1.0, 2.0, math.radians(2 * t)) for t in range(100)]
    assert not w.stalled(turning, 99, 90)
    # it moved early, then stuck for the last 90 s
    assert w.stalled([(t, min(t, 10) * 0.1, 0.0, 0.0) for t in range(110)], 109, 90)


def test_the_pipeline_drives_home_after_a_battery_or_stall_stop():
    """explore_watch's exits 6 (battery floor) and 7 (stalled) end like 4 (room explored): explore_lite
    stopped, Nav2 drives home; a stall stays a failure even when the robot gets home."""
    pipe = open(os.path.join(REPO_ROOT, "scripts", "one_click_pipeline.py")).read()
    assert "if ended in (4, 6, 7):" in pipe
    assert '"--explore-stop-volts"' in pipe and '"--explore-stall-s"' in pipe
    assert "--stall-s {stall_s}" in pipe and "explore_stop_volts" in pipe
    assert "(120.0 if is_real else 0.0)" in pipe   # real robots only: the simulation legs are unchanged
