"""A depth camera as the scan source, or a second obstacle source beside a LiDAR.

Upstream linorobot2's depth cameras (realsense, zed, zedm, zed2, zed2i, oakd,
oakdlite, oakdpro), and a simulated one -- there is no depth camera on the
bench -- that sees the same room as the simulated LD19. The Sim MCU defaults to
its LD19; the camera can replace it or join it.
"""
import math
import os
import sys

import pytest
import yaml

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO_ROOT, "scripts"))
sys.path.insert(0, os.path.join(REPO_ROOT, "launchers"))

import depth_camera as dc  # noqa: E402
import gen_robot_description as grd  # noqa: E402


def read(*p):
    return open(os.path.join(REPO_ROOT, *p)).read()


# --- who makes /scan ----------------------------------------------------------

SIM_LD19 = {"name": "sim", "lidar": {"model": "ld19"}, "sensors": {"use_sim_ld19": True}}
SIM_CAMERA = {"name": "sim", "lidar": {"model": "ld19"}, "sensors": {"use_sim_ld19": False, "use_sim_depth": True}}
SIM_BOTH = {"name": "sim", "lidar": {"model": "ld19"}, "sensors": {"use_sim_ld19": True, "use_sim_depth": True}}


@pytest.mark.parametrize("controller,source,role", [
    (SIM_LD19, "lidar", None),                     # the Sim MCU's default: its LD19
    (SIM_CAMERA, "depth", "scan"),                 # LD19 off: the camera is /scan
    (SIM_BOTH, "lidar", "obstacles"),              # both: SLAM on the LD19, the camera for Nav2
    ({"name": "pico2", "lidar": {"model": "none"}, "depth_camera": {"model": "realsense"}}, "depth", "scan"),
    ({"name": "pico2", "lidar": {"model": "ld19"}, "depth_camera": {"model": "oakd"}}, "lidar", "obstacles"),
    ({"name": "pico2", "lidar": {"model": "none"}}, None, None),
])
def test_one_rule_for_who_makes_scan(controller, source, role):
    assert dc.scan_source(controller) == source
    assert dc.camera_role(controller) == role


def test_the_sim_mcus_lidar_is_its_simulation():
    """Its lidar block names the LD19 it SIMULATES; with that off, there is no LiDAR."""
    assert not dc.lidar_fitted(SIM_CAMERA)
    real = dict(SIM_CAMERA, name="pico2")
    assert dc.lidar_fitted(real), "on a real board a named model is a real LiDAR"


def test_every_upstream_camera_is_supported():
    upstream = {"realsense", "zed", "zedm", "zed2", "zed2i", "oakd", "oakdlite", "oakdpro"}
    assert set(dc.DEPTH_MODELS) == upstream
    with pytest.raises(ValueError):
        dc.depth_model({"depth_camera": {"model": "kinect"}})


# --- the drivers ---------------------------------------------------------------

def test_depthai_launch_file_follows_the_distro():
    """2.x (jazzy) ships camera.launch.py, 3.x (lyrical) driver.launch.py."""
    assert dc.driver_launch("oakd", "jazzy", "camera_link")[1] == "launch/camera.launch.py"
    assert dc.driver_launch("oakdpro", "lyrical", "camera_link")[1] == "launch/driver.launch.py"
    args = dc.driver_launch("oakdlite", "jazzy", "camera_link")[2]
    assert args["camera_model"] == "OAK-D-LITE" and args["parent_frame"] == "camera_link"


def test_realsense_topics_are_not_double_prefixed():
    pkg, _, args = dc.driver_launch("realsense", "jazzy", "camera_link")
    assert pkg == "realsense2_camera" and args["camera_namespace"] == "" and args["camera_name"] == "camera"
    assert dc.DEPTH_MODELS["realsense"][3] == "/camera/depth/image_rect_raw"


def test_the_zed_never_publishes_odom_or_map():
    """Its visual odometry would be a second parent for frames the EKF and SLAM own."""
    args = dc.driver_launch("zed2i", "jazzy", "zed_camera_link")[2]
    assert args["publish_tf"] == "false" and args["publish_map_tf"] == "false"


# --- the room it sees ----------------------------------------------------------

def test_the_room_is_the_one_the_firmware_gets():
    params = {"base_controller": {"simulation": {"map_width": 8.0, "wall_x1": 1.0, "wall_x2": 1.0,
                                                 "wall_obstacle": False}}}
    room = dc.sim_room(params)
    assert room["map_width"] == 8.0 and room["map_height"] == 6.0 and room["wall_obstacle"] is False
    assert len(dc.room_segments(room)) == 4
    assert len(dc.room_segments(dc.sim_room({}))) == 5, "the default room has the obstacle wall"


def test_both_simulated_scanners_read_the_configured_room():
    """sim_laser_node hardcoded 10 x 6 m; it now takes the room bringup hands both nodes."""
    laser = read("scripts", "sim_laser_node.py")
    assert "ROOM_MIN_X" not in laser and "dc.room_segments(room)" in laser
    launch = read("launchers", "bringup.launch.py")
    assert launch.count("depth_camera.sim_room(params)") == 2


def test_a_wall_facing_the_camera_is_one_depth_across_the_image():
    """z-depth, not range: every column that hits the obstacle wall 2 m ahead reads 2.0 m."""
    pytest.importorskip("numpy")
    segs = dc.room_segments(dc.sim_room({}))
    row = dc.depth_row(0.0, 0.0, 0.0, segs, dc.column_angles())
    on_wall = [u for u, a in enumerate(dc.column_angles()) if abs(2.0 * math.tan(a)) <= 1.4]
    assert on_wall and all(abs(row[u] - 2.0) < 1e-9 for u in on_wall)
    assert dc.raycast(0.0, 0.0, math.pi, segs, 12.0) == pytest.approx(5.0), "the back wall"


def test_the_noise_is_a_d435s():
    """sigma = z^2 * 0.08 / (f * 0.05) with f = 212 / tan(43.5 deg): 29 mm at 2 m."""
    assert dc.NOISE_K * 4.0 == pytest.approx(0.0287, abs=0.0005)


# --- the URDF --------------------------------------------------------------------

def _params(**controller):
    p = yaml.safe_load(read("config", "reference", "pico2_mecanum_config.yaml"))
    p["base_controller"].update(controller)
    return p


def test_a_camera_link_only_on_a_robot_with_a_camera():
    assert "camera_link" not in grd.build_urdf(_params())
    assert '<child link="camera_link" />' in grd.build_urdf(_params(depth_camera={"model": "realsense"}))
    assert '<child link="zed_camera_link" />' in grd.build_urdf(_params(depth_camera={"model": "zed2"}))
    sim = _params(sensors={"use_sim_depth": True})
    assert '<child link="camera_link" />' in grd.build_urdf(sim), "the simulated camera needs no model"


# --- simulation mode, and Nav2 -----------------------------------------------------

def test_sim_mode_keeps_the_ld19_off_on_a_camera_robot(tmp_path):
    import mcu_env
    cam = tmp_path / "cam.yaml"
    cam.write_text(yaml.safe_dump({"base_controller": {"name": "pico2", "lidar": {"model": "none"},
                                                       "depth_camera": {"model": "realsense"}}}))
    env = {}
    mcu_env.apply_sensor_mode(env, "sim", str(cam))
    assert env["sim_ld19"] == "0"
    both = tmp_path / "both.yaml"
    both.write_text(yaml.safe_dump({"base_controller": {"name": "pico2", "lidar": {"model": "ld19"},
                                                        "depth_camera": {"model": "realsense"}}}))
    env = {}
    mcu_env.apply_sensor_mode(env, "sim", str(both))
    assert env["sim_ld19"] == "1"


def _nav2_launch():
    """add_camera_obstacle_source itself, lifted out of nav2.launch.py: the module
    imports `launch`, which the test venv does not have, and a skipped test would
    leave this unrun everywhere but the image."""
    import types
    src = read("launchers", "nav2.launch.py")
    a = src.index("def add_camera_obstacle_source(")
    b = src.index("\ndef ", a + 10)
    ns = {}
    exec(src[a:b], ns)
    return types.SimpleNamespace(add_camera_obstacle_source=ns["add_camera_obstacle_source"])


def test_beside_a_lidar_the_camera_is_a_second_obstacle_source():
    nav2 = _nav2_launch()
    p = yaml.safe_load(read("config", "reference", "gendrv_config.yaml"))
    data = p["nav2"]
    p["base_controller"]["depth_camera"] = {"model": "realsense"}
    changed = nav2.add_camera_obstacle_source(data, p)
    assert changed == ["local_costmap.voxel_layer", "global_costmap.obstacle_layer"]
    layer = data["local_costmap"]["local_costmap"]["ros__parameters"]["voxel_layer"]
    assert layer["observation_sources"].split() == ["scan", "camera"]
    assert layer["camera"]["topic"] == "/camera/scan" and layer["camera"]["obstacle_min_range"] == 0.45


def test_as_the_scan_it_adds_nothing_to_nav2():
    nav2 = _nav2_launch()
    p = yaml.safe_load(read("config", "reference", "gendrv_config.yaml"))
    p["base_controller"]["lidar"] = {"model": "none"}
    p["base_controller"]["sensors"]["use_sim_ld19"] = False
    p["base_controller"]["depth_camera"] = {"model": "realsense"}
    assert nav2.add_camera_obstacle_source(p["nav2"], p) == []


def test_the_pipeline_and_bringup_share_the_rule():
    pipe = read("scripts", "one_click_pipeline.py")
    assert "scan_from = depth_camera.scan_source(controller_cfg)" in pipe
    assert '" sim_depth:=true" if sim_depth' in pipe
    launch = read("launchers", "bringup.launch.py")
    assert 'robot_has_lidar = scan_from == "lidar"' in launch
    assert "robot_has_lidar = bool(lidar_cfg)" not in launch
