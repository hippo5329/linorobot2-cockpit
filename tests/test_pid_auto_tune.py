"""test_acc is a step response, so the wheel loop can be designed rather than guessed.

That was the design intent of the tool: a step into a wheel at rest, sampled
every 20 ms -- which is CONTROL_TIMER, the control loop's own period. From a
step you get the plant; from the plant you get gains that are stable by
construction.

Nothing used it that way. The gains were kp 0.6 / ki 0.8 / kd 0.5 on every
robot, and closed on the identified plant they overshoot 13% and cross the
setpoint three times. They also could not be tested: before the wheel model,
simulation mode's encoders tracked the command instead of responding to PWM, so every
set of gains looked perfect on the bench.

Two things this file holds down:

  - the identification is a first-order fit in the LOOP's units (rpm per PWM
    count, seconds), and the discretisation matches PID::compute() -- which is
    positional and NOT time-normalised, so ki carries the sample period inside
    it. Getting that wrong gives gains that are off by a factor of Ts.
  - a candidate is only offered after the closed loop has been simulated and
    found calm at both a crawl and near full speed. IMC knows nothing about the
    dead zone, the anti-windup clamp, the PWM rail or the Coulomb drag.
"""
import math
import os
import sys

import pytest
import yaml

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO_ROOT, "scripts"))

import drivetrain_report as dr  # noqa: E402

REF = os.path.join(REPO_ROOT, "config", "reference")
MAIN_CPP = os.path.join(REPO_ROOT, "firmware", "src", "main.cpp")
PID_CPP = os.path.join(REPO_ROOT, "firmware", "common", "lib", "pid", "pid.cpp")


def _cfg(name="gendrv", **kine):
    with open(os.path.join(REF, f"{name}_config.yaml"), encoding="utf-8") as fh:
        params = yaml.safe_load(fh)
    params.setdefault("kinematics", {}).update(kine)
    return params


def _d(name="gendrv", **kine):
    return dr.drivetrain(_cfg(name, **kine))


def test_the_sample_period_is_the_control_loops_own():
    """The step is sampled at CONTROL_TIMER, which is what makes the fit usable
    for the loop that runs at that rate. If the two drift apart, ki -- which
    carries Ts inside it -- is silently wrong."""
    src = open(MAIN_CPP, encoding="utf-8").read()
    ms = int(src.split("#define CONTROL_TIMER")[1].split()[0])
    assert dr.TICK_S == pytest.approx(ms / 1000.0)


def test_the_plant_is_identified_in_the_loops_units():
    """rpm per PWM count, not m/s per duty: the PID's error is rpm and its
    output is counts, so a gain in any other units is a number that cannot be
    typed into the config."""
    d = _d()
    plant = dr.identify_plant(d)
    assert plant["pwm_step"] == d["pwm_max"] == 1023.0
    assert plant["gain"] == pytest.approx(plant["steady_rpm"] / 1023.0)
    assert 0.05 < plant["gain"] < 0.5


def test_pwm_bits_change_the_gain_not_the_robot():
    """A 12-bit board reaches the same speed with four times the counts, so its
    plant gain is a quarter. Gains derived without noticing would be 4x wrong."""
    ten = dr.identify_plant(_d(pwm_bits=10))
    twelve = dr.identify_plant(_d(pwm_bits=12))
    assert twelve["steady_rpm"] == pytest.approx(ten["steady_rpm"], rel=0.01)
    assert twelve["gain"] == pytest.approx(ten["gain"] / 4.0, rel=0.01)


def test_tau_is_the_632_percent_crossing():
    d = _d()
    plant = dr.identify_plant(d)
    run = dr.run_test_acc(d)
    rpm = [v * 60.0 / d["circ"] for v, _ in run["trace"][:dr.PHASE_SAMPLES]]
    at_tau = rpm[int(round(plant["tau"] / dr.TICK_S))]
    assert at_tau == pytest.approx(plant["steady_rpm"] * 0.632, rel=0.1)


def test_a_heavier_robot_has_a_longer_time_constant():
    assert dr.identify_plant(_d())["tau"] < \
        dr.identify_plant(dr.drivetrain(_cfg_mass(12.0)))["tau"]


def _cfg_mass(kg):
    params = _cfg()
    params["base_controller"]["simulation"]["robot_mass"] = kg
    return params


def test_the_discretisation_matches_the_firmwares_pid():
    """PID::compute() accumulates a plain sum, so ki = Kc*Ts/Ti rather than
    Kc/Ti. A test on the arithmetic alone would not catch the firmware changing
    shape, so the firmware's own form is asserted too."""
    src = open(PID_CPP, encoding="utf-8").read()
    assert "integral_ += error;" in src, "the integral is no longer a plain sum"
    assert "derivative_ = error - prev_error_;" in src

    d = _d()
    plant = dr.identify_plant(d)
    g = dr.suggest_pid(d, plant, lambda_ratio=2.0)
    kc = plant["tau"] / (plant["gain"] * plant["tau"] * 2.0)
    assert g["kp"] == pytest.approx(kc, rel=0.01)
    assert g["ki"] == pytest.approx(g["kp"] * plant["ts"] / plant["tau"], rel=0.01)


def test_no_derivative_term_is_offered():
    """A first-order plant needs none, and this one differentiates an unfiltered
    50 Hz encoder reading -- on a real robot that is an amplifier for
    quantisation noise."""
    assert dr.suggest_pid(_d())["kd"] == 0.0
    assert dr.auto_tune_pid(_d())["kd"] == 0.0


def test_a_slower_lambda_is_a_gentler_loop():
    d = _d()
    fast = dr.suggest_pid(d, lambda_ratio=1.0)
    slow = dr.suggest_pid(d, lambda_ratio=4.0)
    assert slow["kp"] < fast["kp"]


def test_the_closed_loop_is_simulated_before_the_gains_are_offered():
    """The whole point. IMC does not know about the dead zone, the anti-windup
    clamp, the PWM rail or the Coulomb drag, so the candidate is run."""
    d = _d()
    tuned = dr.auto_tune_pid(d)
    assert tuned is not None
    assert tuned["checked"], "no closed-loop evidence attached"
    for run in tuned["checked"]:
        assert run["overshoot"] <= dr.PID_MAX_OVERSHOOT
        assert run["crossings"] <= dr.PID_MAX_CROSSINGS


def test_it_is_checked_at_a_crawl_as_well_as_at_speed():
    """A wheel loop is not linear: the dead zone and the Coulomb drag dominate
    at low speed, the rail and the pack's sag at high speed."""
    tuned = dr.auto_tune_pid(_d())
    setpoints = sorted(r["setpoint"] for r in tuned["checked"])
    assert len(setpoints) >= 2
    assert setpoints[0] < setpoints[-1] / 2


def test_the_shipped_gains_ring_on_this_plant():
    """The finding that motivates all of it. Stated as a test so that if someone
    later makes them calm, this file says so instead of quietly passing."""
    d = _d()
    shipped = _cfg()["kinematics"]["pid"]
    plant = dr.identify_plant(d)
    run = dr.simulate_closed_loop(d, shipped, setpoint_rpm=plant["steady_rpm"] * 0.9)
    assert run["overshoot"] > dr.PID_MAX_OVERSHOOT or run["crossings"] > 0, \
        "the shipped gains no longer ring -- update this test and the docs"


def test_an_unstable_gain_is_rejected_rather_than_reported_as_tuned():
    d = _d()
    wild = dr.simulate_closed_loop(d, {"kp": 80.0, "ki": 20.0, "kd": 0.0})
    assert wild["overshoot"] > dr.PID_MAX_OVERSHOOT or wild["crossings"] > 0


def test_the_loop_reaches_the_setpoint_it_was_given():
    """A tuned loop with no steady-state error is the minimum bar; the integral
    term exists for exactly that."""
    d = _d()
    tuned = dr.auto_tune_pid(d)
    plant = dr.identify_plant(d)
    run = dr.simulate_closed_loop(d, tuned, setpoint_rpm=plant["steady_rpm"] * 0.5,
                                  seconds=4.0)
    assert abs(run["steady_error"]) < plant["steady_rpm"] * 0.02


def test_the_anti_windup_clamp_is_the_firmwares():
    """It decides how a saturated loop recovers, which is most of what stability
    means for a loop that spends its life against the rail."""
    src = open(os.path.join(REPO_ROOT, "scripts", "drivetrain_report.py"),
               encoding="utf-8").read()
    body = src[src.index("def simulate_closed_loop"):src.index("# What \"stable\"")]
    assert "i_max = pwm_max / abs(ki)" in body
    assert open(PID_CPP, encoding="utf-8").read().count("i_max = limit / (double)fabs(ki_)") == 1


# --- identifying from a REAL robot ------------------------------------------

TRANSCRIPT = """MAX PWM 1023.0 -1023.0
MAX VEL   0.71   0.00 m/s    0.03 rad/s
MAX ACC   3.76   0.00 m/s2   0.21 rad/s2
time to 0.9x max vel   0.42 sec
distance to stop   0.13 m
"""


def test_a_real_transcript_identifies_the_plant():
    got = dr.parse_test_acc(TRANSCRIPT)
    assert got["max_vel"] == 0.71
    assert got["t_to_90"] == 0.42
    assert got["pwm_step"] == 1023.0


def test_the_first_run_is_the_one_used():
    """test_acc halves the PWM every fourth run, so a later block is a step of a
    different size; fitting one plant to two different inputs is meaningless."""
    two_runs = TRANSCRIPT + TRANSCRIPT.replace("1023.0", "511.5").replace("0.71", "0.35")
    got = dr.parse_test_acc(two_runs)
    assert got["max_vel"] == 0.71 and got["pwm_step"] == 1023.0


def test_something_that_is_not_a_transcript_is_refused():
    assert dr.parse_test_acc("MAX VEL   0.71   0.00 m/s") is None, \
        "one of the two numbers is not an identification"


def test_the_measured_plant_uses_the_first_order_relation():
    """0.9 of final is reached at ln(10)*tau for a first-order step -- which is
    why those are the two lines test_acc prints."""
    d = _d()
    plant = dr.plant_from_measurements(d, 0.71, 0.42, 1023.0)
    assert plant["tau"] == pytest.approx(0.42 / math.log(10.0))
    assert plant["steady_rpm"] == pytest.approx(0.71 * 60.0 / d["circ"])
    assert plant["measured"] is True


def test_a_recorded_measurement_beats_the_model():
    """A robot that has been measured must not silently fall back to the model
    the next time somebody saves its config."""
    params = _cfg()
    modelled = dr.config_plant(params)
    assert not modelled.get("measured")
    params["kinematics"]["step_response"] = {"max_vel": 0.71, "t_to_90": 0.42}
    measured = dr.config_plant(params)
    assert measured["measured"] is True
    assert measured["steady_rpm"] != pytest.approx(modelled["steady_rpm"], rel=0.01)


def test_a_half_written_measurement_is_ignored_not_half_used():
    params = _cfg()
    params["kinematics"]["step_response"] = {"max_vel": 0.71}
    assert not dr.config_plant(params).get("measured")


# --- the flag ----------------------------------------------------------------

def test_gains_are_not_rewritten_from_the_model_alone():
    """A Nav2 limit is a bound -- wrong means slow, and the smoother clamps it
    either way. A loop gain is stability, and nothing downstream saves it. No
    motor has ever turned here, so a simulated plant does not get to change one
    unattended; the report still says loudly what it would suggest."""
    params = _cfg()
    assert dr.auto_pid_enabled(params) is False
    assert not any(k.startswith("kinematics.pid.") for k in dr.derived_limits(params))
    # ...while the bounds, which are safe to be wrong about, still are.
    assert dr.auto_limits_enabled(params) is True
    assert any(k.startswith("nav2.") for k in dr.derived_limits(params))


def test_a_measured_robot_does_get_its_gains_written():
    params = _cfg()
    params["kinematics"]["step_response"] = {"max_vel": 0.71, "t_to_90": 0.42}
    assert dr.auto_pid_enabled(params) is True
    assert any(k.startswith("kinematics.pid.") for k in dr.derived_limits(params))


def test_the_flag_overrides_the_default_in_both_directions():
    params = _cfg()
    params["kinematics"]["auto_pid"] = True
    assert dr.auto_pid_enabled(params) is True, "the model's gains, if you ask for them"
    params["kinematics"]["step_response"] = {"max_vel": 0.71, "t_to_90": 0.42}
    params["kinematics"]["auto_pid"] = False
    assert dr.auto_pid_enabled(params) is False, "off even on a measured robot"
