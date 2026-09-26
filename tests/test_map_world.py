"""A saved map as the simulated world: the loop explore -> save -> simulate in it -> navigate on it.

World "map" (base_controller.simulation.world_map) makes the host's simulated
laser and depth camera raycast an occupancy map (depth_camera.GridWorld) -- a
map a SLAM or exploration run saved. No board can hold one, so the board's
LD19 emulator is switched off and its own box widened so its sonar and
collision stay quiet; the host laser is the /scan.
"""
import math
import os
import sys

import pytest
import yaml

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO_ROOT, "scripts"))

import depth_camera as dc  # noqa: E402
import mcu_env  # noqa: E402

np = pytest.importorskip("numpy")


def write_map(tmp_path, w=40, h=20, res=0.1, origin=(-2.0, -1.0), wall_x=1.0):
    """A 4 x 2 m map, free, with a wall column at x = wall_x (map frame)."""
    img = np.full((h, w), 254, dtype=np.uint8)
    col = int((wall_x - origin[0]) / res)
    img[:, col] = 0
    (tmp_path / "m.pgm").write_bytes(b"P5\n# made by a test\n%d %d\n255\n" % (w, h) + img.tobytes())
    (tmp_path / "m.yaml").write_text(yaml.safe_dump({
        "image": "m.pgm", "resolution": res, "origin": [origin[0], origin[1], 0.0],
        "negate": 0, "occupied_thresh": 0.65, "free_thresh": 0.25}))
    return str(tmp_path / "m.yaml")


def test_the_grid_is_raycast(tmp_path):
    g = dc.GridWorld(write_map(tmp_path))
    r = g.ranges(0.0, 0.0, [0.0, math.pi / 2, math.pi], 12.0)
    assert r[0] == pytest.approx(1.0, abs=0.1), "the wall 1 m ahead"
    assert math.isinf(r[1]) and math.isinf(r[2]), "past the map's edge is empty"


def test_the_start_places_the_robot_in_the_map(tmp_path):
    g = dc.GridWorld(write_map(tmp_path), start=(0.5, 0.0, math.pi))
    x, y, yaw = g.to_world(0.0, 0.0, 0.0)
    assert (x, y) == (0.5, 0.0) and yaw == pytest.approx(math.pi)
    # facing -x from x = 0.5: the wall is behind; facing +x it is 0.5 m away
    assert g.ranges(x, y, [0.0], 12.0)[0] == pytest.approx(0.5, abs=0.1)


def test_the_camera_sees_the_same_map(tmp_path):
    g = dc.GridWorld(write_map(tmp_path))
    row = dc.depth_row(0.0, 0.0, 0.0, g, dc.column_angles())
    centre = row[len(row) // 2]
    assert centre == pytest.approx(1.0, abs=0.1)


def world(tmp_path, **extra):
    return {"base_controller": {"name": "sim", "simulation": dict(
        {"world": "map", "world_map": write_map(tmp_path)}, **extra)}}


def test_the_board_emulator_is_off_and_its_box_out_of_reach(tmp_path):
    env = mcu_env.hardware_env(world(tmp_path))
    assert env["sim_ld19"] == "0" and env["sim_wall"] in (0, "0")
    assert float(env["sim_map_w"]) >= 100 and float(env["sim_map_h"]) >= 100


def test_a_map_world_needs_a_map(tmp_path):
    with pytest.raises(ValueError, match="world_map"):
        dc.world_map({"base_controller": {"simulation": {"world": "map"}}})
    path, start = dc.world_map(world(tmp_path))
    assert path.endswith("m.yaml") and start == (0.0, 0.0, 0.0)


def test_bringup_puts_the_host_laser_on_the_map():
    launch = open(os.path.join(REPO_ROOT, "launchers", "bringup.launch.py")).read()
    assert "world_map_path, world_start = depth_camera.world_map(params)" in launch
    assert 'if effective_lidar_comm_mode == "udp_server" and not no_board and not world_map_path' in launch
    assert "**world_params," in launch
    for node in ("sim_laser_node.py", "sim_depth_node.py"):
        assert "dc.GridWorld(" in open(os.path.join(REPO_ROOT, "scripts", node)).read()
