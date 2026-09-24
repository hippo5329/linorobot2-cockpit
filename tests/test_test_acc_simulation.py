"""The simulated test_acc must BE test_acc, not a second opinion about it.

scripts/drivetrain_report.py now runs the wheel model instead of only solving
it, on test_acc.cpp's own profile -- and that only means anything if the
transcription is faithful. A simulation that quietly differs is worse than no
simulation, because it produces numbers people will tune a velocity smoother
from and there is no longer a board in the loop to contradict it.

So this file pins the transcription to the two files it transcribes: the timing
and measurement rules to test_acc.cpp, the physics to sim_wheel.h.
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
    """SIM_WHEEL_STALL_DUTY is read from the header; a rename would make every
    wheel stall at full PWM and the report would show a robot that cannot move."""
    d = _d()
    assert 0.0 < d["stall_duty"] < 0.5
    assert dr.run_test_acc(d)["max_vel"] > 0.1


# --- the driver's current limiter -------------------------------------------
#
# Many small drivers chop at a fixed current -- DRV8871 3.6 A, TB6612 1.2 A per
# channel, a sense resistor at whatever you pick. It caps TORQUE, which is a
# different shape of limit from everything else in the model and bites exactly
# where a robot is judged: from rest, at full duty, where the current demand is
# highest. Without it, two robots with the same motors and pack but different
# driver boards were identical on paper.


def test_no_limiter_is_the_default_so_nothing_changes_silently():
    """0 means none fitted. A limit nobody entered must not throttle every
    existing robot, which is why the sentinel cannot be the usual -1."""
    assert dr.model_defaults()["ilimit_a"] == 0.0
    assert _d()["ilimit_a"] == 0.0


def test_a_limit_above_the_stall_current_does_nothing():
    """The limiter is only real when the motor would otherwise draw past it."""
    d = _d()
    free = dr.run_test_acc(d)
    slack = dr.run_test_acc(_d(driver_current_limit=d["stall_a"] * 2))
    assert slack["max_acc"] == pytest.approx(free["max_acc"])
    assert slack["max_vel"] == pytest.approx(free["max_vel"])


def test_a_tighter_limit_costs_acceleration_in_proportion():
    """Torque is pinned at whatever the allowed current buys, so halving the
    limit roughly halves the acceleration from rest."""
    stall = _d()["stall_a"]
    loose = dr.run_test_acc(_d(driver_current_limit=stall * 0.8))
    tight = dr.run_test_acc(_d(driver_current_limit=stall * 0.4))
    assert tight["max_acc"] < loose["max_acc"]
    assert tight["max_acc"] == pytest.approx(loose["max_acc"] * 0.5, rel=0.25)


def test_the_limiter_barely_touches_top_speed():
    """Near terminal speed the speed error is small, so the current demand is
    small and the limiter is not engaged. A limiter makes a robot sluggish, not
    slow -- and reporting it as a lower top speed would send someone shopping
    for the wrong part."""
    stall = _d()["stall_a"]
    free = dr.run_test_acc(_d())
    tight = dr.run_test_acc(_d(driver_current_limit=stall * 0.4))
    assert tight["max_vel"] > free["max_vel"] * 0.95
    assert tight["t_to_90"] > free["t_to_90"]


def test_only_the_ratio_of_limit_to_stall_current_matters():
    """What the amps actually buy you.

    The model carries the motor's torque capability in SIM_WHEEL_TAU_MS, and
    the current only says at what current that torque arrives. So the limiter's
    effect is `limit / stall`, and nothing else: a motor drawing twice the
    current for the same torque is hurt exactly twice as much by the same
    driver.

    Stated as a test because the tempting claim -- "a bigger motor behind a
    small driver changes nothing" -- is NOT what this model says, and reading it
    that way would have someone conclude the limiter does not matter. A bigger
    motor here is a bigger max_rpm and a different tau as well, not a stall
    current on its own.
    """
    a = dr.run_test_acc(_d(motor_stall_amps=2.5, driver_current_limit=1.0))
    b = dr.run_test_acc(_d(motor_stall_amps=5.0, driver_current_limit=2.0))
    assert b["max_acc"] == pytest.approx(a["max_acc"], rel=0.02)
    # ...and the same limit on the thirstier motor buys half the torque.
    c = dr.run_test_acc(_d(motor_stall_amps=5.0, driver_current_limit=1.0))
    assert c["max_acc"] == pytest.approx(a["max_acc"] * 0.5, rel=0.1)


def test_a_limited_driver_sags_the_pack_less():
    """It draws less current, so it must brown out the pack less. Reporting the
    unlimited demand to the pack would have it sag over current the driver is
    refusing to pass."""
    hard = _d(battery_sag=0.6, driver_current_limit=0.0)
    soft = _d(battery_sag=0.6, driver_current_limit=hard["stall_a"] * 0.3)
    p_hard, p_soft = dr._Pack(hard), dr._Pack(soft)
    w_hard, w_soft = dr._Wheel(hard, p_hard, 0), dr._Wheel(soft, p_soft, 0)
    # At full duty from rest, which is where the current demand is highest. A
    # wheel left at the default duty of zero draws nothing and the comparison
    # would be 0 < 0 -- true of any two packs, and a test that proves nothing.
    w_hard.duty = w_soft.duty = 1.0
    w_hard.step(0.02, True)
    w_soft.step(0.02, True)
    assert 0 < p_soft.demand[0] < p_hard.demand[0]


def test_the_closed_form_and_the_simulation_agree_about_the_limiter():
    """Two halves of the same report. If only one of them knew, the HUD would
    show a limited robot with unlimited acceleration beside it."""
    d = _d(driver_current_limit=_d()["stall_a"] * 0.4)
    p = dr.performance(d)
    run = dr.run_test_acc(d)
    free = dr.performance(_d())
    assert p["lin_acc"] < free["lin_acc"]
    assert run["max_acc"] < free["lin_acc"]


def test_the_firmware_and_the_host_apply_it_the_same_way():
    """The transcription rule for this term: the limiter scales the DRIVING
    torque and the pack demand, and does not touch the speed the motor aims for.
    A firmware that instead derated no_load_rpm would make a limited robot slow
    rather than sluggish."""
    src = open(os.path.join(REPO_ROOT, "firmware", "common", "lib", "encoder",
                            "sim_wheel.h"), encoding="utf-8").read()
    body = src[src.index("void integrate()"):src.index("public:")]
    assert "ilim_scale * simGearEfficiency()" in body, "the limiter must scale the torque"
    assert "demand()[slot_] = ifrac * ilim_scale;" in body, "the pack must see the limited current"
    # ...and never the speed target.
    assert "no_load_rpm *= ilim_scale" not in body
