"""test_acc measures motors. With no motors it refuses and says where to look.

The history here is two mistakes in opposite directions, and the second is the
instructive one.

First, SimEncoder::feed() was called from main.cpp's moveBase() and nowhere
else, so on a sim-wheel board test_acc drove nothing -- and still printed a
full table:

    MAX VEL   0.00   0.00 m/s    0.03 rad/s
    MAX ACC   0.37   0.00 m/s2   2.91 rad/s2

Zero velocity beside non-zero acceleration is arithmetically impossible from
real motion; it is encoder noise differentiated. Measured on a bare Pico 2,
2026-09-23.

The fix was to feed the simulated wheels from this tool too, which worked: the
table filled with plausible numbers. And that was the second mistake, because
those numbers were a measurement OF THE SIMULATOR -- taken on an MCU, over a
serial line, after a flash, and unable to vary the terms that matter (mass, gear
efficiency, pack sag, driver losses) without another one. The model lives in
sim_wheel.h and the host can simply run it: scripts/drivetrain_report.py steps
the same equations on this tool's own 20 ms / 1 s profile.

So the rule is now: with sim wheels, print where the answer comes from and
stop. Which matters for the same reason the first bug did -- the project wiki
tells people to set the velocity smoother's max_velocity and max_accel "according
to test_acc test result", so anything this tool prints will be used to configure
a robot.

main.cpp is the opposite case and must keep feeding: the simulated wheels are
what makes a sim-mode Nav2 run move, and that is the bench's whole gate.
"""
import os
import re

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ACC = os.path.join(ROOT, "firmware", "src", "tools", "test_acc.cpp")
MAIN = os.path.join(ROOT, "firmware", "src", "main.cpp")
REPORT = os.path.join(ROOT, "scripts", "drivetrain_report.py")


def _src(p):
    with open(p, encoding="utf-8") as fh:
        return fh.read()


def _helper_body():
    s = _src(ACC)
    i = s.index("static void driveAll")
    j = s.index("{", i)
    return s[j:s.index("\n}", j)]


def test_the_tool_does_not_drive_the_simulated_wheels():
    assert "->feed(" not in _src(ACC), \
        "test_acc drives the simulator again, and will report it as a measurement"


def test_every_wheel_is_still_spun_through_the_one_helper():
    body = _helper_body()
    assert body.count("->spin(") == 4
    for n in (1, 2, 3, 4):
        assert f"motor{n}_controller->spin(pwm{n});" in body, f"wheel {n} is not spun"


def test_the_helper_does_not_call_itself():
    """The first version of this was written by substituting a regex over the
    spin quartets, and the regex matched the helper's OWN body: driveAll called
    driveAll, which compiles and recurses until the stack is gone."""
    assert "driveAll(" not in _helper_body(), "driveAll is recursive"


def test_no_spin_site_bypasses_the_helper():
    s = _src(ACC)
    after = s[s.index("\n}", s.index("static void driveAll")):]
    assert "motor1_controller->spin(" not in after, "a spin site bypasses driveAll"


def test_a_sim_wheel_board_is_refused_before_any_motor_is_spun():
    """The refusal has to come first. Falling through to the run would drive the
    motor pins of a board whose config says it has none."""
    s = _src(ACC)
    loop = s[s.index("void loop_()"):]
    guard = loop.index("if (sim_wheels)")
    assert guard < loop.index("driveAll("), "the guard runs after the first drive"
    assert "return;" in loop[guard:loop.index("driveAll(")]


def test_the_refusal_names_the_tool_that_answers_instead():
    """A refusal that does not say where to go is a dead end, and the person
    reading it wants a number, not a rule."""
    s = _src(ACC)
    assert "drivetrain_report.py" in s
    assert "--params" in s
    assert os.path.exists(REPORT)


def test_the_env_decides_not_the_build():
    """One image serves a bare module and a wired robot; an #ifdef here would
    put the decision in whichever config generated the header."""
    s = _src(ACC)
    assert "wheelsAreSim()" in s
    acc_guard = s[s.index("sim_wheels = wheelsAreSim();") - 400:]
    assert "#ifdef" not in acc_guard[:acc_guard.index("wheelsAreSim();")]


def test_the_guard_is_set_after_the_env_is_read():
    """wheelsAreSim() calls initMcuEnv() itself, but reading it before the tool
    has probed the bus would order the banner lines confusingly."""
    s = _src(ACC)
    assert s.index("initMcuEnv();") < s.index("sim_wheels = wheelsAreSim();")


def test_main_still_feeds_the_simulated_wheels():
    """The control loop is the remaining caller and must stay one: sim mode is
    how every Nav2 leg on the bench moves."""
    m = _src(MAIN)
    for n in (1, 2, 3, 4):
        assert re.search(rf"motor{n}_encoder\.feed\(pwm{n}\);", m), f"main.cpp lost wheel {n}"
