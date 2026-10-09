"""A real robot simulates nothing, anywhere: env, drive suite, Nav2 judge, SLAM nudge.

User, 2026-10-09: "make sure sim wall or other sim things are totally off on real
robots". A real robot's env carried sim_sonar 1 (the default) and the whole
simulated drivetrain model; the Nav2 judge failed a real detour past x=2.0 as
"DROVE INTO THE WALL" -- the simulation's wall; and the drive suite and the SLAM
nudge drove fixed distances with no look at the room.
"""
import math
import os

import pytest

import mcu_env
from odom_check import room_for

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def read(*parts):
    with open(os.path.join(REPO, *parts), encoding="utf-8") as fh:
        return fh.read()


@pytest.mark.parametrize("design", ["gendrv", "unoq", "yb_eet01"])
def test_a_designs_env_switches_every_simulation_off(reference, design):
    env = mcu_env.hardware_env(reference(design))
    sims = {k: v for k, v in env.items() if k.startswith("sim_")}
    assert sims == {k: "0" for k in mcu_env.SIM_SWITCHES}, sims


def test_a_bench_board_that_asks_for_simulation_keeps_it(reference):
    params = reference("gendrv")
    sensors = params["base_controller"].setdefault("sensors", {})
    sensors["use_sim_wheel"] = True          # a real chip on simulated wheels
    env = mcu_env.hardware_env(params)
    assert "sim_mass" in env and "sim_radius" in env


def test_orientation_stddev_reaches_the_env(reference):
    params = reference("gendrv")
    assert "ori_sd" not in mcu_env.hardware_env(params), "unset: the firmware's 0.01 rad"
    params["base_controller"].setdefault("sensors", {})["orientation_stddev"] = 0.5
    assert mcu_env.hardware_env(params)["ori_sd"] == 0.5
    for bad in (0, -1, 4.0, "wide"):
        params["base_controller"]["sensors"]["orientation_stddev"] = bad
        with pytest.raises(ValueError, match="orientation_stddev"):
            mcu_env.hardware_env(params)


def test_the_firmware_squares_ori_sd_into_the_covariance():
    src = read("firmware", "src", "main.cpp")
    assert 'envGet("ori_sd"' in src and "ori_var_mag = (sd > 0.0) ? sd * sd" in src
    assert "publish_mag ? ori_var_mag : 1.0e6" in src


def wall(x, y0=-1.0, y1=1.0, n=41):
    return [(x, y0 + (y1 - y0) * i / (n - 1)) for i in range(n)]


def test_a_straight_move_stops_short_of_the_room():
    secs, note = room_for(wall(5.0), 0.25, 0.0, 0.0, 5.0, 0.16, 0.40)
    assert (secs, note) == (5.0, ""), "1.25 m with 4.84 m clear: the full move"
    secs, note = room_for(wall(1.55), 0.25, 0.0, 0.0, 5.0, 0.16, 0.40)
    assert secs == pytest.approx((1.55 - 0.16 - 0.40) / 0.25) and "shortened" in note
    secs, note = room_for(wall(0.80), 0.25, 0.0, 0.0, 5.0, 0.16, 0.40)
    assert secs is None and "only 0.64 m clear ahead" in note
    secs, _ = room_for(wall(-0.80), -0.25, 0.0, 0.0, 5.0, 0.16, 0.40)
    assert secs is None, "backward looks behind"
    secs, _ = room_for(wall(0.80), -0.25, 0.0, 0.0, 5.0, 0.16, 0.40)
    assert secs == 5.0, "a wall ahead does not stop a move backward"


def test_an_arc_needs_its_whole_disc_and_a_spin_its_footprint():
    r = 0.20 / 0.80
    clear = [(math.cos(a) * 3, math.sin(a) * 3) for a in range(0, 360, 5)]
    assert room_for(clear, 0.20, 0.80, 0.0, 5.0, 0.16, 0.40) == (5.0, "")
    block = clear + [(0.0, 2 * r + 0.1)]          # on the left turn's far side
    assert room_for(block, 0.20, 0.80, 0.0, 5.0, 0.16, 0.40)[0] is None
    assert room_for(block, 0.20, -0.80, 0.0, 5.0, 0.16, 0.40)[0] == 5.0, "the right turn misses it"
    assert room_for(clear + [(0.15, 0.0)], 0.0, 1.5, 0.0, 5.0, 0.16, 0.40)[0] is None
    assert room_for(clear, 0.0, 1.5, 0.0, 5.0, 0.16, 0.40) == (5.0, "")


def test_the_pipeline_runs_real_robots_without_the_simulations_room():
    src = read("scripts", "one_click_pipeline.py")
    assert 'goal_args += " --no-wall"' in src
    assert '--real --guard {CLEARANCE_REAL_M}' in src
    assert '" --guard {CLEARANCE_REAL_M} --radius "' in src or "--guard {CLEARANCE_REAL_M} --radius" in src
    nav = read("scripts", "test_nav2_goal.py")
    assert "if NO_WALL:\n            return False" in nav and "global NO_WALL" in nav
