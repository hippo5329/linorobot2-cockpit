"""Navigating on a saved map localises with AMCL; the map is not handed to a file that ignores it.

The cockpit offered "navigate on a saved map", and nav2.launch.py passed map:=
to nav2_bringup's navigation_launch.py -- which starts neither a map server nor
AMCL on either distro. No map frame, every goal failed. It now starts
map_server, amcl and lifecycle_manager_localization, the shape of
nav2_bringup's own localization_launch.py, with AMCL's parameters taken from the
INSTALLED nav2_bringup (upstream Nav2 is the reference) and only the robot's
frame, motion model and initial pose overridden.
"""
import os
import types

import yaml

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def read(*p):
    return open(os.path.join(REPO_ROOT, *p)).read()


def _launcher_fn(name):
    """One function lifted out of nav2.launch.py (which imports `launch`, absent here)."""
    src = read("launchers", "nav2.launch.py")
    a = src.index(f"def {name}(")
    b = src.index("\ndef ", a + 10)
    ns = {"os": os, "yaml": yaml, "FindPackageShare": None}
    exec(src[a:b], ns)
    return ns[name]


def _bringup(tmp_path, amcl):
    share = tmp_path / "nav2_bringup"
    (share / "params").mkdir(parents=True)
    (share / "params" / "nav2_params.yaml").write_text(yaml.safe_dump({"amcl": {"ros__parameters": amcl}}))
    return str(share)


def test_amcl_starts_from_the_installed_nav2_defaults(tmp_path):
    amcl_params = _launcher_fn("amcl_params")
    share = _bringup(tmp_path, {"base_frame_id": "base_footprint", "update_min_d": 0.25,
                                "random_seed": -1, "robot_model_type": "nav2_amcl::DifferentialMotionModel"})
    p = amcl_params({"kinematics": {"base_type": "2wd"}}, "/maps/room.yaml", (0.0, 0.0, 0.0), share)
    rp = p["amcl"]["ros__parameters"]
    assert rp["update_min_d"] == 0.25 and rp["random_seed"] == -1, "Nav2's own values stand"
    assert rp["base_frame_id"] == "base_link", "this tree is odom -> base_link"
    assert rp["set_initial_pose"] is True and rp["initial_pose"]["x"] == 0.0
    assert p["map_server"]["ros__parameters"]["yaml_filename"] == "/maps/room.yaml"


def test_a_mecanum_base_uses_the_omni_model(tmp_path):
    amcl_params = _launcher_fn("amcl_params")
    p = amcl_params({"kinematics": {"base_type": "mecanum"}}, "m.yaml", (1.0, 2.0, 0.5), _bringup(tmp_path, {}))
    rp = p["amcl"]["ros__parameters"]
    assert rp["robot_model_type"] == "nav2_amcl::OmniMotionModel"
    assert rp["initial_pose"] == {"x": 1.0, "y": 2.0, "z": 0.0, "yaw": 0.5}


def test_the_robots_own_amcl_block_wins(tmp_path):
    amcl_params = _launcher_fn("amcl_params")
    params = {"nav2": {"amcl": {"ros__parameters": {"max_particles": 800}}}}
    p = amcl_params(params, "m.yaml", (0, 0, 0), _bringup(tmp_path, {"max_particles": 2000}))
    assert p["amcl"]["ros__parameters"]["max_particles"] == 800


def test_the_map_is_served_not_passed_to_navigation_launch():
    src = read("launchers", "nav2.launch.py")
    assert 'launch_args["map"] = map_file' not in src
    assert '"node_names": ["map_server", "amcl"]' in src
    assert "actions += localization_actions(" in src


def test_the_pipeline_runs_no_slam_on_a_saved_map():
    pipe = read("scripts", "one_click_pipeline.py")
    i = pipe.index("elif has_lidar and args.map:")
    assert i < pipe.index('print(f"\\n[5/6] [SLAM] Launching SLAM Toolbox'), "the map branch must come first"
    assert '+ (f" map:={os.path.abspath(args.map)}" if args.map else "")' in pipe
