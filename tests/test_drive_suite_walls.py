"""The drive suite knows the walls of the robot's own simulated world (2026-10-06).

It mirrored only the default room (the box and the obstacle wall at x = 2.0). In the four-room
world an exploration leg ended at (+0.65, +0.18); its forward manoeuvre was held 0.31 m short of
the interior wall at x = 1.5, the suite called the pose "clear", and failed the base on distance
-- every mecanum exploration leg, rc63 and rc64. drive_suite imports rclpy, so its functions are
lifted with ast (test_drive_suite._where_for) and given the walls set_world() would build.
"""
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "scripts"))
sys.path.insert(0, os.path.join(ROOT, "tests"))

import depth_camera  # noqa: E402
from test_drive_suite import _where_for, _suite_src  # noqa: E402


def segments(cfg):
    room = depth_camera.sim_room(cfg)
    return depth_camera.room_segments(room, depth_camera.sim_walls(cfg)), bool(room["wall_obstacle"])


def test_the_rooms_interior_wall_holds_the_robot():
    cfg = {"base_controller": {"name": "sim", "simulation": {"world": "rooms"}}}
    segs, obstacle = segments(cfg)
    where = _where_for(segs, 0.30, obstacle)
    assert where(1.19, 0.22).startswith("interior wall")     # the rc64 leg's end pose
    assert where(0.65, 0.18) == "clear"                       # where it started
    assert "OBSTACLE WALL" not in where(1.70, 0.0)            # rooms has no test wall


def test_configured_walls_hold_the_robot_too():
    cfg = {"base_controller": {"name": "sim", "simulation": {"walls": [[0.0, -1.0, 0.0, 1.0]]}}}
    segs, obstacle = segments(cfg)
    assert _where_for(segs, 0.30, obstacle)(-0.30, 0.0).startswith("interior wall")


def test_a_map_world_has_no_walls():
    assert _where_for([], 0.30, False)(1.19, 0.22) == "clear"


def test_the_suite_reads_the_world_and_waives_a_held_distance():
    src = _suite_src()
    assert "def set_world(cfg: dict, world: str = None)" in src
    assert 'if depth_camera.sim_world(cfg) == "map":' in src
    assert "gen_firmware_header.nav2_robot_radius(cfg)" in src
    assert 'held = (lin or lat) and not ok_d and where not in ("clear", "pose unknown")' in src
    pipe = open(os.path.join(ROOT, "scripts", "one_click_pipeline.py")).read()
    assert "--config {params_path} --base-type {base_type}" in pipe
