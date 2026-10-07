"""The gendrv design drives its BTS7960 in the BTS7960's pin order.

The class drives two PWMs and "forward" is the PWM on in_a (firmware
default_motor.h), so in_a must be each motor's forward input. Upstream
linorobot2_hardware's gendrv_config.h has, under USE_BTS7960_MOTOR_DRIVER,
MOTOR1_IN_A 17 / IN_B 21 and MOTOR2_IN_A 23 / IN_B 22. The design shipped the
generic 2-pin order (21/17, 22/23) and ran both wheels of a real GenDrv robot
backwards with every invert false (2026-10-07); upstream's order turned them
forward.
"""
import os
import sys

import yaml

HERE = os.path.dirname(__file__)
sys.path.insert(0, os.path.join(HERE, "..", "scripts"))

import mcu_env  # noqa: E402

DESIGN = os.path.join(HERE, "..", "config", "reference", "gendrv_config.yaml")
UPSTREAM = {"motor1": (25, 17, 21), "motor2": (26, 23, 22)}   # pwm, in_a (forward), in_b


def test_the_design_uses_the_bts7960_order():
    bc = yaml.safe_load(open(DESIGN))["base_controller"]
    assert str(bc["driver_type"]).upper() == "BTS7960"
    for motor, (pwm, in_a, in_b) in UPSTREAM.items():
        pins = bc["pins"][motor]
        assert (pins["pwm"], pins["in_a"], pins["in_b"]) == (pwm, in_a, in_b), motor
        assert pins.get("invert") is False, f"{motor}: the order fixes direction, not an invert"


def test_the_board_gets_that_order(tmp_path):
    secrets = tmp_path / "secrets.yaml"
    secrets.write_text('wifi:\n  ssid: "x"\n  password: "x"\n')
    env = mcu_env.env_from_config(DESIGN, str(secrets))
    assert env["motor_driver"] == "bts7960"
    assert (env["m1_in_a"], env["m1_in_b"], env["m2_in_a"], env["m2_in_b"]) == (17, 21, 23, 22)
    assert str(env["m1_inv"]) == "0" and str(env["m2_inv"]) == "0"
