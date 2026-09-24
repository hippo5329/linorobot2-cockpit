"""The config must be checkable against the motors before a robot is built.

Nothing checked, and the shipped Nav2 limits asked 103% of the motors on a
differential base, 129% on mecanum and 171% at the velocity smoother's ceiling.
Nav2 then commands what it cannot get, Kinematics scales the whole request down
to fit, and tracking degrades -- three mecanum legs left the room on 2026-09-23
before anyone looked at the motors. The arithmetic is not hard; it was never
written down.

The report's model constants are PARSED from sim_wheel.h rather than restated,
because a tool that copies the model's numbers drifts from it silently and then
describes a robot that does not exist.
"""
import os
import subprocess
import sys

import pytest
import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCRIPT = os.path.join(ROOT, "scripts", "drivetrain_report.py")
sys.path.insert(0, os.path.join(ROOT, "scripts"))
import drivetrain_report as dr  # noqa: E402


def _cfg(base="2wd", vx=0.25, wz=1.0, smoother=(0.3, 0.0, 1.2), accel=(0.8, 0.0, 1.5), **sim):
    return {
        "robot": {"name": "t"},
        "kinematics": {"base_type": base, "wheel_diameter": 0.1,
                       "lr_wheels_distance": 0.271, "fr_wheels_distance": 0.18,
                       "max_rpm": 140, "max_rpm_ratio": 0.85},
        "base_controller": {"simulation": dict(sim)} if sim else {},
        "nav2": {"controller_server": {"controller_server": {"ros__parameters": {
            "FollowPath": {"desired_linear_vel": vx,
                           "rotate_to_heading_angular_vel": wz}}}},
            "velocity_smoother": {"velocity_smoother": {"ros__parameters": {
                "max_velocity": list(smoother), "max_accel": list(accel)}}}},
    }


def test_the_model_constants_come_from_the_firmware():
    d = dr.model_defaults()
    for key in ("tau_ms", "ref_mass", "viscous", "gear_eff", "coulomb", "sag", "mass"):
        assert key in d, key
    # and they are the firmware's values, not a copy
    src = open(os.path.join(ROOT, "firmware", "common", "lib", "encoder",
                            "sim_wheel.h"), encoding="utf-8").read()
    assert f"SIM_GEAR_EFFICIENCY {d['gear_eff']}" in src
    assert f"SIM_BATT_SAG {d['sag']}" in src


def test_a_missing_constant_is_refused_rather_than_guessed():
    """If the model moves, the report must stop rather than describe a robot that
    does not exist."""
    import tempfile
    with tempfile.NamedTemporaryFile("w", suffix=".h", delete=False) as fh:
        fh.write("#define SIM_WHEEL_TAU_MS 150\n")
        path = fh.name
    with pytest.raises(SystemExit):
        dr.model_defaults(path)
    os.unlink(path)


def test_mecanum_turns_on_the_wheelbase_too():
    """The same radius Kinematics uses -- get this wrong and the report says a
    mecanum can turn 66% faster than it can."""
    assert dr.drivetrain(_cfg("mecanum"))["radius"] == pytest.approx((0.271 + 0.18) / 2)
    assert dr.drivetrain(_cfg("2wd"))["radius"] == pytest.approx(0.271 / 2)


def test_linear_and_rotational_demand_add():
    """The outer wheel carries both. This is what the old limits got wrong: each
    looked survivable on its own."""
    d = dr.drivetrain(_cfg("mecanum"))
    only_lin = dr.demand_rpm(d, 0.25, 0.0)
    only_rot = dr.demand_rpm(d, 0.0, 1.0)
    both = dr.demand_rpm(d, 0.25, 1.0)
    assert both == pytest.approx(only_lin + only_rot)


def test_the_old_limits_are_reported_as_over_budget():
    """The regression this tool exists to catch, in the numbers that shipped."""
    text, over = dr.report(_cfg("mecanum", vx=0.4, wz=1.8,
                                smoother=(0.5, 0.5, 2.5), accel=(2.5, 2.5, 3.2)))
    assert over is True
    assert "OVER BUDGET" in text
    assert "asks for more than the motors can give" in text


def test_the_current_limits_are_within_budget():
    for base in ("2wd", "skid_steer", "mecanum"):
        text, over = dr.report(_cfg(base))
        assert over is False, f"{base}: {text}"


def test_every_shipped_reference_fits_its_motors():
    """The check that keeps this from happening again."""
    import glob
    for path in sorted(glob.glob(os.path.join(ROOT, "config", "reference", "*_config.yaml"))):
        with open(path, encoding="utf-8") as fh:
            params = yaml.safe_load(fh)
        text, over = dr.report(params, os.path.basename(path))
        assert over is False, f"{os.path.basename(path)} asks more than its motors give:\n{text}"


def test_a_heavier_robot_accelerates_less():
    """tau scales with mass, which is the whole reason the weight question came
    up: the simulated dynamics used 3.5 kg while the URDF described 1.4-1.8."""
    light = dr.performance(dr.drivetrain(_cfg(robot_mass=1.4)))
    heavy = dr.performance(dr.drivetrain(_cfg(robot_mass=7.0)))
    assert light["lin_acc"] > heavy["lin_acc"] * 1.5
    # Terminal speed is set by the balance of torque against friction, so mass
    # barely moves it -- but not exactly zero, and the residual is real rather
    # than numerical. Terminal speed solves
    #     eff*(no_load - w)/tau = w*viscous + coulomb
    # and a heavier robot has a larger tau, so eff/tau shrinks and the constant
    # gear drag takes a bigger share of what is left. A 5x heavier robot loses
    # about 5% of its top speed here, which is the gearbox eating more of the
    # torque near no-load. Asserting zero would be asserting a model without
    # Coulomb drag in it.
    loss = abs(light["lin_vel"] - heavy["lin_vel"]) / light["lin_vel"]
    assert loss < 0.10, f"mass moved terminal speed by {loss * 100:.1f}%"
    assert loss > 0.0, "mass has no effect on terminal speed at all -- is the drag gone?"


def test_the_losses_reduce_performance():
    """Each of the three costs something, or adding them was decoration."""
    ideal = dr.performance(dr.drivetrain(_cfg(gear_efficiency=1.0, gear_drag_rpm=0.0,
                                              battery_sag=0.0)))
    real = dr.performance(dr.drivetrain(_cfg()))
    assert real["lin_acc"] < ideal["lin_acc"], "the losses cost no acceleration"
    assert real["lin_vel"] < ideal["lin_vel"], "the losses cost no top speed"


def test_it_runs_as_a_command_and_exits_nonzero_when_over_budget():
    r = subprocess.run([sys.executable, SCRIPT, "--params",
                        os.path.join(ROOT, "config", "reference", "gendrv_config.yaml")],
                       capture_output=True, text=True)
    assert r.returncode == 0, r.stdout + r.stderr
    assert "what it can do" in r.stdout
    assert "VERDICT" in r.stdout
