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


