"""Simulated wheels drive no real wheel: with sim_wheel on, every env written to a board has
every motor and encoder pin at -1, and every motor driver returns at once on a -1 pin."""
import os
import re
import sys

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO_ROOT, "scripts"))

import mcu_env  # noqa: E402

REAL_PINS = {"m1_pwm": "5", "m1_in_a": "6", "m1_in_b": "7", "m1_enc_a": "8", "m1_enc_b": "9",
             "m2_pwm": "10", "m2_in_a": "11", "m2_in_b": "12"}


def test_simulated_wheels_write_every_drivetrain_pin_as_minus_one():
    env = mcu_env.decode(mcu_env.encode(dict(REAL_PINS, sim_wheel="1", imu="sim")))
    for n in range(1, 5):
        for suffix in mcu_env.DRIVETRAIN_PIN_KEYS:
            assert env[f"m{n}_{suffix}"] == "-1", f"m{n}_{suffix}"
    assert env["imu"] == "sim"


def test_real_wheels_keep_their_pins():
    env = mcu_env.decode(mcu_env.encode(dict(REAL_PINS, sim_wheel="0")))
    for k, v in REAL_PINS.items():
        assert env[k] == v
    assert "m3_pwm" not in env


def test_the_caller_dict_is_not_changed():
    src = dict(REAL_PINS, sim_wheel="1")
    mcu_env.encode(src)
    assert src == dict(REAL_PINS, sim_wheel="1")


def test_every_motor_driver_returns_on_a_minus_one_pin():
    src = open(os.path.join(REPO_ROOT, "firmware", "common", "lib", "motor", "default_motor.h")).read()
    for cls in ("Generic2", "Generic1", "BTS7960", "ESC"):
        body = src[src.index(f"class {cls}"):]
        body = body[:body.index("\n};")]
        spin_paths = re.findall(r"void (?:forward|reverse|spin|brake)\([^)]*\)\s*(?:override)?\s*\{([^}]*)", body)
        assert spin_paths, cls
        for path in spin_paths:
            assert "< 0) return;" in path or "spin(" in path, f"{cls}: a drive path without the -1 guard"
