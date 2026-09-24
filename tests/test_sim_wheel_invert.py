"""A simulated wheel must ignore `invert`, exactly as it ignores the pins.

On a real robot the right-hand side is mirrored, so the motor driver inverts
what it drives and the encoder inverts what it reads. The two cancel.

In sim mode there is no motor: the pins are -1, MotorInterface::spin() returns
without touching anything, and nothing applies the motor half of that pair --
but SimEncoder::feed() was still applying the encoder half. Wheel 2 ran
backwards whenever wheel 1 ran forwards, so the simulated robot spun on the
spot instead of driving.

Measured on a bare Pico 2, 2026-09-20:

    commanded (0.20, 0.00) -> odom vx -0.459 m/s, wz -4.869 rad/s
    commanded (0.00, 0.00) -> odom vx -0.6 m/s,   wz -3.7 rad/s
    after the fix:
    commanded (0.20, 0.00) -> odom vx +0.222 m/s, wz +/-0.05 rad/s
    commanded (0.80, 0.00) -> odom vx +0.852 m/s
    commanded (0.00, 1.50) -> odom wz +1.654 rad/s

Nav2 planned a path around the obstacle wall and the base could never follow
it. The zero-wiring promise, broken by a sign.
"""
import os
import re
import sys

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO_ROOT, "scripts"))

SIM = os.path.join(REPO_ROOT, "firmware", "common", "lib", "encoder", "sim_wheel.h")


def _sim_encoder_src():
    src = open(SIM).read()
    i = src.index("class SimEncoder")
    j = src.find("\nclass ", i + 1)
    return src[i: j if j > 0 else len(src)]


def test_the_simulated_wheel_does_not_apply_invert():
    body = _sim_encoder_src()
    assert "invert_" not in body, (
        "SimEncoder keeps an invert flag again -- in sim mode nothing applies "
        "the MOTOR inversion, so applying the encoder's makes wheel 2 run "
        "backwards and the robot spins instead of driving")
    assert "(void)invert;" in body, "invert should be explicitly ignored, and said so"


def test_it_still_ignores_the_pins():
    body = _sim_encoder_src()
    assert "(void)pin1;" in body and "(void)pin2;" in body


def test_the_bare_module_declares_nothing_mirrored():
    """The default is forward, for motors and encoders alike (user rule,
    2026-09-22). A real differential base IS mirrored on one side, and that is
    a measured fact about a chassis, never something a bare module inherits --
    the simulated wheel ignores the flag anyway, so a default of `true` only
    ever bit the first real robot. Nothing in a bare config is inverted."""
    from gen_bare_config import bare_pins
    p = bare_pins("pico")
    for n in range(1, 5):
        assert p[f"motor{n}"]["invert"] is False, f"motor{n}"
        assert p[f"encoder{n}"]["invert"] is False, f"encoder{n}"
