"""The simulated world can have rooms: interior walls from one list, everywhere.

Exploration has to be tested somewhere with rooms to find. The walls are
base_controller.simulation.walls; the firmware's simulated LD19 (and its sonar)
raycasts and collides with them through the env key sim_walls, and the host's
simulated laser and depth camera get the same list, so every sensor sees one
world.
"""
import math
import os
import sys

import pytest

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO_ROOT, "scripts"))

import depth_camera as dc  # noqa: E402
import mcu_env  # noqa: E402


def cfg(walls):
    return {"base_controller": {"name": "sim", "simulation": {"walls": walls, "wall_obstacle": False}}}


def test_walls_reach_the_board_as_one_key():
    env = mcu_env.hardware_env(cfg([[1.5, -3, 1.5, 1.8], [1.5, 0, 3.8, 0]]))
    assert env.get("sim_walls") == "1.5,-3,1.5,1.8;1.5,0,3.8,0"


def test_the_firmware_reads_raycasts_and_collides_with_them():
    fw = open(os.path.join(REPO_ROOT, "firmware", "common", "lib", "lidar", "sim_ld19.h")).read()
    assert 'envGet("sim_walls"' in fw
    assert "for (int i = 0; i < count + walls_n_; i++)" in fw, "raycast"
    assert "pushOffSegment(x, y, w[0], w[1], w[2], w[3]" in fw, "collision"
    assert f"SIM_WALLS_MAX = {dc.SIM_WALLS_MAX};" in fw


def test_the_host_sensors_see_the_same_walls():
    room = dict(dc.ROOM_DEFAULTS, wall_obstacle=False)
    walls = dc.sim_walls(cfg(dc.MULTI_ROOM_WALLS))
    segs = dc.room_segments(room, walls)
    # 1 m off the door's centre, looking west, the middle room's west wall stands 1.5 m away
    assert dc.raycast(0.0, 1.0, math.pi, segs, 10.0) == pytest.approx(1.5)
    # through the door (y = 0) the far west wall, 5 m away
    assert dc.raycast(0.0, 0.0, math.pi, segs, 10.0) == pytest.approx(5.0)
    assert dc.walls_from_flat(dc.walls_flat(walls)) == walls
    launch = open(os.path.join(REPO_ROOT, "launchers", "bringup.launch.py")).read()
    assert launch.count('"walls": depth_camera.walls_flat(depth_camera.sim_walls(params)),') == 2


def test_the_multi_room_doors_fit_the_robot():
    """Every door is at least 1.2 m: 2 x 0.26 m radius plus room to spare."""
    assert dc.MULTI_ROOM_WALLS[1][1] - dc.MULTI_ROOM_WALLS[0][3] >= 1.2
    assert 3.0 - dc.MULTI_ROOM_WALLS[2][3] >= 1.2
    assert 5.0 - dc.MULTI_ROOM_WALLS[3][2] >= 1.2


@pytest.mark.parametrize("bad", [[[1, 2, 3]], "1,2,3,4", [[0, 0, 1, 1]] * 13])
def test_a_bad_wall_list_is_refused(bad):
    with pytest.raises((ValueError, TypeError)):
        dc.sim_walls(cfg(bad))


def test_no_walls_changes_nothing():
    assert dc.sim_walls({}) == [] and dc.walls_flat([]) == [0.0] and dc.walls_from_flat([0.0]) == []
