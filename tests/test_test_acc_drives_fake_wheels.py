"""test_acc must drive the simulated wheels, or it measures nothing and says so.

FakeEncoder::feed() is what turns a PWM into simulated wheel motion. It was
called from main.cpp's moveBase() and nowhere else, so on a fake-wheel board
test_acc drove nothing -- and still printed a full table:

    MAX VEL   0.00   0.00 m/s    0.03 rad/s
    MAX ACC   0.37   0.00 m/s2   2.91 rad/s2

Zero velocity beside non-zero acceleration is arithmetically impossible from
real motion; it is encoder noise differentiated. Measured on the z13 Pico 2,
2026-09-23. That matters because the project wiki tells people to set the
velocity smoother's max_velocity and max_accel "according to test_acc test
result" -- so a tool that reports noise as data misconfigures the robot.

A real encoder's feed() is a no-op, so one call path serves both and which one
runs stays the env's decision rather than the compiler's.
"""
import os
import re

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ACC = os.path.join(ROOT, "firmware", "src", "tools", "test_acc.cpp")
MAIN = os.path.join(ROOT, "firmware", "src", "main.cpp")


def _src(p):
    with open(p, encoding="utf-8") as fh:
        return fh.read()


def _helper_body():
    s = _src(ACC)
    i = s.index("static void driveAll")
    j = s.index("{", i)
    return s[j:s.index("\n}", j)]


def test_the_tool_feeds_the_simulated_encoders():
    body = _helper_body()
    for n in (1, 2, 3, 4):
        assert f"motor{n}_encoder->feed(pwm{n});" in body, f"wheel {n} is not driven"


def test_every_wheel_that_is_spun_is_also_fed():
    """Four spins, four feeds, same arguments -- a pair that drifts would drive
    three simulated wheels and leave one at zero, which looks like a robot that
    pulls to one side."""
    body = _helper_body()
    assert body.count("->spin(") == 4
    assert body.count("->feed(") == 4
    for n in (1, 2, 3, 4):
        assert f"motor{n}_controller->spin(pwm{n});" in body


def test_the_helper_does_not_call_itself():
    """The first version of this was written by substituting a regex over the
    spin quartets, and the regex matched the helper's OWN body: driveAll called
    driveAll, which compiles and recurses until the stack is gone."""
    body = _helper_body()
    assert "driveAll(" not in body, "driveAll is recursive"


def test_no_spin_site_bypasses_the_helper():
    """Any quartet left spinning directly would move the real motors and leave
    the simulated ones still, which is the bug this file exists for."""
    s = _src(ACC)
    after = s[s.index("\n}", s.index("static void driveAll")):]
    assert "motor1_controller->spin(" not in after, \
        "a spin site bypasses driveAll and will not drive simulated wheels"


def test_main_still_feeds_too():
    """The control loop is the other caller and must stay one."""
    m = _src(MAIN)
    for n in (1, 2, 3, 4):
        assert re.search(rf"motor{n}_encoder\.feed\(pwm{n}\);", m), f"main.cpp lost wheel {n}"
