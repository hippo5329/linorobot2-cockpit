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


def _map_cfg(tmp_path):
    path = tmp_path / "maprobot_config.yaml"
    path.write_text(yaml.safe_dump(world(tmp_path)))
    return str(path)


def test_the_sim_mcus_raycast_is_gridworlds(tmp_path):
    """The Sim MCU's C++ reader (sim_world_map.h) against GridWorld, beam for beam."""
    import shutil
    import subprocess
    cxx = shutil.which("g++") or shutil.which("c++")
    if not cxx:
        pytest.skip("no C++ compiler")
    src = tmp_path / "wm.cpp"
    src.write_text("""#define LINO_HOST 1
#include "sim_world_map.h"
int main(int c, char **v) {
  SimWorldMap m; if (!m.load(v[1], v[2])) { fprintf(stderr, "%s\\n", m.error()); return 1; }
  for (int i = 0; i < 36; i++) printf("%.4f\\n", m.range(0.2f, 0.1f, i * 10 * M_PI / 180, 12.0f));
}
""")
    exe = tmp_path / "wm"
    subprocess.run([cxx, "-O1", "-I", os.path.join(REPO_ROOT, "firmware", "common", "lib", "lidar"),
                    str(src), "-o", str(exe)], check=True)
    m = write_map(tmp_path)
    out = subprocess.run([str(exe), m, "0.5,-0.3,0.4"], capture_output=True, text=True, check=True).stdout
    got = [float(v) for v in out.split()]
    g = dc.GridWorld(m, (0.5, -0.3, 0.4))
    wx, wy, wyaw = g.to_world(0.2, 0.1, 0.0)
    want = g.ranges(wx, wy, [wyaw + i * 10 * math.pi / 180 for i in range(36)], 12.0)
    for a, b in zip(got, want):
        assert a == pytest.approx(min(b, 12.0), abs=1e-3)
    assert any(v < 12.0 for v in got), "the map's wall must be seen"


def test_the_board_emulator_is_off_and_its_box_out_of_reach(tmp_path):
    env = mcu_env.hardware_env(world(tmp_path))
    assert env["sim_ld19"] == "0" and env["sim_wall"] in (0, "0")
    assert float(env["sim_map_w"]) >= 100 and float(env["sim_map_h"]) >= 100


def test_a_map_world_needs_a_map(tmp_path):
    with pytest.raises(ValueError, match="world_map"):
        dc.world_map({"base_controller": {"simulation": {"world": "map"}}})
    path, start = dc.world_map(world(tmp_path))
    assert path.endswith("m.yaml") and start == (0.0, 0.0, 0.0)


def test_the_sim_mcu_raycasts_the_map_and_the_ld_driver_publishes_scan():
    """User, 2026-10-06: "sim mcu should do the raycasts from imported map and let stl driver
    publish /scan". The map goes into the Sim MCU's env; its emulator stays on and streams to
    the udp_server driver. A physical board cannot hold a map and is refused."""
    launch = open(os.path.join(REPO_ROOT, "launchers", "bringup.launch.py")).read()
    assert "world_map_path, world_start = depth_camera.world_map(params)" in launch
    assert 'if effective_lidar_comm_mode == "udp_server" and not use_host_sim_laser' in launch
    assert "world=(world_map_path, world_start)" in launch
    assert "[MAP WORLD] a board cannot hold the saved map" in launch
    assert "**world_params," in launch          # the sim_base path's laser and the camera
    for node in ("sim_laser_node.py", "sim_depth_node.py"):
        assert "dc.GridWorld(" in open(os.path.join(REPO_ROOT, "scripts", node)).read()
    fw = open(os.path.join(REPO_ROOT, "firmware", "common", "lib", "lidar", "sim_ld19.h")).read()
    assert '#include "sim_world_map.h"' in fw and 'envGet("sim_world_map", NULL)' in fw
    assert "min_dist = world_.range(ox, oy, ray_rad" in fw


def test_the_sim_mcu_env_keeps_its_emulator_on_a_map_world(tmp_path):
    import host_firmware
    src = open(os.path.join(REPO_ROOT, "scripts", "host_firmware.py")).read()
    assert 'holds_map=bool(world and world[0])' in src
    assert 'e["sim_world_map"] = os.path.abspath(path)' in src
    cfg = _map_cfg(tmp_path)
    board, host = {"sim_ld19": "1"}, {"sim_ld19": "1"}
    mcu_env.apply_sensor_mode(board, "sim", cfg)
    mcu_env.apply_sensor_mode(host, "sim", cfg, holds_map=True)
    assert board["sim_ld19"] == "0" and host["sim_ld19"] == "1"
    assert host_firmware is not None


def test_the_sonar_raycasts_the_map_too():
    """On the Sim MCU the emulator's sonar cone raycasts the map (sim_ld19.h rangeAheadM
    goes through raycastRangeMm); the host casts it only with no board, or for a
    camera-only robot, whose emulator is off."""
    launch = open(os.path.join(REPO_ROOT, "launchers", "bringup.launch.py")).read()
    assert ("host_sonar = (no_board or (bool(world_map_path) and "
            "(host_fw_bin is None or not robot_has_lidar))) and") in launch
    assert 'name="sim_sonar_node"' in launch and '"topic": "sim_sonar_scan", **world_params' in launch


def test_exploration_never_sends_a_goal_over_a_running_one():
    """Lyrical's Nav2 refuses a goal sent over a running one, even with
    allow_navigator_preemption (which stays at upstream's default, off)."""
    assert 'setdefault("allow_navigator_preemption"' not in \
        open(os.path.join(REPO_ROOT, "launchers", "nav2.launch.py")).read()
    assert "Never replace a goal in progress" in \
        open(os.path.join(REPO_ROOT, "scripts", "prepare_docker_vendor.sh")).read()
