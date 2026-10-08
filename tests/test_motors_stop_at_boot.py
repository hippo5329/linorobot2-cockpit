"""A real robot's motors are stopped from the first line of setup() (2026-10-08).

User: "For real robots, the motor output must be initialized to stop immediately" --
the tracked test robot's wheels ran at boot. The drivetrain came after the banner, the reset reason,
the board init and an RP2's USB wait; and a BTS7960 raised its enable pin while IN_A/IN_B
were still floating inputs.
"""
import os
import re

ROOT = os.path.join(os.path.dirname(__file__), "..", "firmware")


def _read(*p):
    return open(os.path.join(ROOT, *p)).read()


def test_drivetrain_is_the_first_thing_setup_does():
    main = _read("src", "main.cpp")
    body = main[main.index("void setup()"):]
    body = body[body.index("{") + 1:]
    code = [ln.strip() for ln in body.splitlines() if ln.strip() and not ln.strip().startswith("//")]
    assert code[0] == "initDrivetrain();"
    assert body.count("initDrivetrain();") == 1


def test_every_driver_pulls_its_pins_low_before_output():
    h = _read("common", "lib", "motor", "default_motor.h")
    for cls in ("Generic2", "Generic1", "BTS7960"):
        for ctor in re.findall(r"\n\s+%s\([^)]*\):.*?\n        }\n" % cls, h, re.S):
            assert "pinMode(" not in ctor, cls
            assert "motorPinLow(" in ctor, cls


def test_a_bts7960_is_enabled_last():
    h = _read("common", "lib", "motor", "default_motor.h")
    ctor = re.search(r"BTS7960\(float pwm_frequency, int pwm_bits, bool invert, int enable_pin.*?\n        }\n", h, re.S).group(0)
    assert ctor.rstrip().rstrip("}").rstrip().endswith("enable(enable_pin);")
    assert ctor.index("analogWrite(in_b_pin_, 0)") < ctor.index("enable(enable_pin);")
