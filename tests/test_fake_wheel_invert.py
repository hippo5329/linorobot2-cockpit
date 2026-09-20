"""A simulated wheel must ignore `invert`, exactly as it ignores the pins.

On a real robot the right-hand side is mirrored, so the motor driver inverts
what it drives and the encoder inverts what it reads. The two cancel.

In fake mode there is no motor: the pins are -1, MotorInterface::spin() returns
without touching anything, and nothing applies the motor half of that pair --
but FakeEncoder::feed() was still applying the encoder half. Wheel 2 ran
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

FAKE = os.path.join(REPO_ROOT, "firmware", "common", "lib", "encoder", "fake_wheel.h")


def _fake_encoder_src():
    src = open(FAKE).read()
    i = src.index("class FakeEncoder")
    j = src.find("\nclass ", i + 1)
    return src[i: j if j > 0 else len(src)]


def test_the_simulated_wheel_does_not_apply_invert():
    body = _fake_encoder_src()
    assert "invert_" not in body, (
        "FakeEncoder keeps an invert flag again -- in fake mode nothing applies "
        "the MOTOR inversion, so applying the encoder's makes wheel 2 run "
        "backwards and the robot spins instead of driving")
    assert "(void)invert;" in body, "invert should be explicitly ignored, and said so"


def test_it_still_ignores_the_pins():
    body = _fake_encoder_src()
    assert "(void)pin1;" in body and "(void)pin2;" in body


def test_the_bare_module_still_declares_the_mirrored_side():
    """The config is not wrong -- a real differential base IS mirrored. The fix
    belongs in the simulation, so the flags must stay."""
    from gen_bare_config import bare_pins
    p = bare_pins()
    assert p["motor2"]["invert"] is True
    assert p["encoder2"]["invert"] is True
    assert p["motor1"]["invert"] is False
    assert p["encoder1"]["invert"] is False
