"""The simulated test_acc must BE test_acc, not a second opinion about it.

scripts/drivetrain_report.py now runs the wheel model instead of only solving
it, on test_acc.cpp's own profile -- and that only means anything if the
transcription is faithful. A simulation that quietly differs is worse than no
simulation, because it produces numbers people will tune a velocity smoother
from and there is no longer a board in the loop to contradict it.

So this file pins the transcription to the two files it transcribes: the timing
and measurement rules to test_acc.cpp, the physics to fake_wheel.h.
"""
import math
import os
import re
import sys

import pytest
import yaml

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO_ROOT, "scripts"))

import drivetrain_report as dr  # noqa: E402

ACC_CPP = os.path.join(REPO_ROOT, "firmware", "src", "tools", "test_acc.cpp")
REF = os.path.join(REPO_ROOT, "config", "reference")


def _cfg(name="gendrv", **sim):
    with open(os.path.join(REF, f"{name}_config.yaml"), encoding="utf-8") as fh:
        params = yaml.safe_load(fh)
    params["base_controller"].setdefault("simulation", {}).update(sim)
    return params


def _d(name="gendrv", **sim):
    return dr.drivetrain(_cfg(name, **sim))


def test_the_timing_matches_the_tool_it_simulates():
    """20 ms ticks and 1 s phases are test_acc's, not ours to pick."""
    src = open(ACC_CPP, encoding="utf-8").read()
    ticks = int(re.search(r"const unsigned ticks = (\d+)", src).group(1))
    run_time = int(re.search(r"const unsigned run_time = (\d+)", src).group(1))
    assert dr.TICK_S == pytest.approx(ticks / 1000.0), "tick length drifted from test_acc.cpp"
    assert dr.PHASE_SAMPLES == run_time // ticks, "phase length drifted from test_acc.cpp"


def test_the_trace_is_four_phases_of_samples():
    run = dr.run_test_acc(_d())
    assert len(run["trace"]) == 4 * dr.PHASE_SAMPLES


def test_the_first_sample_is_taken_before_any_time_passes():
    """record() reads getRPM() and only then delay()s, so sample 0 of the run is
    the robot at rest. Stepping first put every number one tick early, which is
    a 20 ms error in "time to 0.9x max"."""
    run = dr.run_test_acc(_d())
    assert run["trace"][0][0] == 0.0


def test_the_stop_distance_is_the_coast_not_the_drive():
    """dump_record() integrates the SECOND quarter of the buffer -- the coast
    after the first drive phase. Integrating the drive phase instead would report
    how far it travelled, which is a different number that looks plausible."""
    d = _d()
    run = dr.run_test_acc(d)
    coast = run["trace"][dr.PHASE_SAMPLES:2 * dr.PHASE_SAMPLES]
    assert run["stop"] == pytest.approx(sum(t[0] for t in coast) * dr.TICK_S)
    # ...and it must be shorter than the distance covered while driving.
    drive = sum(t[0] for t in run["trace"][:dr.PHASE_SAMPLES]) * dr.TICK_S
    assert 0 < run["stop"] < drive


def test_a_heavier_robot_accelerates_slower_and_stops_further():
    light = dr.run_test_acc(_d(robot_mass=1.5))
    heavy = dr.run_test_acc(_d(robot_mass=15.0))
    assert heavy["max_acc"] < light["max_acc"]
    assert heavy["t_to_90"] > light["t_to_90"]
    assert heavy["stop"] > light["stop"]


def test_a_worse_gearbox_is_slower_in_every_column():
    good = dr.run_test_acc(_d(gear_efficiency=0.9))
    worn = dr.run_test_acc(_d(gear_efficiency=0.4))
    assert worn["max_vel"] < good["max_vel"]
    assert worn["max_acc"] < good["max_acc"]


def test_a_softer_pack_costs_acceleration_but_little_top_speed():
    """The sag is proportional to current, and at terminal speed there is almost
    none: a soft pack is a slow start, not a slow robot."""
    stiff = dr.run_test_acc(_d(battery_sag=0.05))
    soft = dr.run_test_acc(_d(battery_sag=0.6))
    assert soft["max_acc"] < stiff["max_acc"]
    assert soft["max_vel"] > stiff["max_vel"] * 0.85


def test_the_sag_lag_matters_so_tau_is_not_decoration():
    """A held load sags deeper than a brief one. With tau at zero the sag is
    fully developed in the first tick, so the first kick is weaker."""
    instant = dr.run_test_acc(_d(battery_sag=0.5, battery_sag_tau_ms=0))
    lagged = dr.run_test_acc(_d(battery_sag=0.5, battery_sag_tau_ms=1500))
    assert lagged["max_acc"] > instant["max_acc"]


def test_the_pack_is_shared_so_four_wheels_cost_more_than_two():
    """The only coupling between the simulated wheels. Without it a 4WD base
    accelerated exactly like a 2WD one, which is why it was added."""
    two = dr.run_test_acc(_d("gendrv"))                 # differential, 2 driven
    four = dr.run_test_acc(_d("pico2_mecanum"))         # mecanum, 4 driven
    assert four["max_acc"] < two["max_acc"] * 1.0 + 1e-9 or four["max_vel"] < two["max_vel"] * 1.05


def test_rotation_uses_the_bases_own_radius():
    """A mecanum turns on (lr + fr)/2, so the same wheel speeds give it a lower
    yaw rate than a differential base of the same track."""
    diff = dr.run_test_acc(_d("gendrv"), rotate=True)
    mec = dr.run_test_acc(_d("pico2_mecanum"), rotate=True)
    assert mec["max_vel"] < diff["max_vel"]


def test_a_rotation_run_does_not_translate_and_a_straight_run_does_not_spin():
    straight = dr.run_test_acc(_d("pico2_mecanum"), rotate=False)
    spin = dr.run_test_acc(_d("pico2_mecanum"), rotate=True)
    assert straight["max_vel_lin"] > 0.1 and abs(straight["max_vel_ang"]) < 0.05
    assert spin["max_vel_ang"] > 0.1 and abs(spin["max_vel_lin"]) < 0.05


def test_the_measured_speed_sits_below_the_asymptote():
    """test_acc drives for 1 s; the closed form is where the wheel converges
    given longer. The measurement must be the smaller of the two, or one of them
    is wrong."""
    d = _d()
    p = dr.performance(d)
    run = dr.run_test_acc(d)
    assert run["max_vel"] < p["lin_vel"]
    assert run["max_vel"] > p["lin_vel"] * 0.8, "1 s should get most of the way there"


def test_a_stalled_duty_band_still_produces_a_run():
    """FAKE_WHEEL_STALL_DUTY is read from the header; a rename would make every
    wheel stall at full PWM and the report would show a robot that cannot move."""
    d = _d()
    assert 0.0 < d["stall_duty"] < 0.5
    assert dr.run_test_acc(d)["max_vel"] > 0.1
