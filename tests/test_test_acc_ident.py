"""test_acc identifies the robot it is running on, per wheel and with the loop closed.

The open-loop table (MAX VEL / MAX ACC / time to 0.9x) is a step response, and
the host already fits a plant to it. Two things it cannot answer, and both decide
whether the gains are right:

  per wheel   the table averages four wheels into one velocity. Four real wheels
              have different friction, different gearboxes and different wear, and
              one set of gains is then wrong for three of them -- which on a robot
              looks like a base that pulls to one side under acceleration, reads
              like a kinematics error, and is not one.
  closed loop a plant fit says what the motor does when shoved; it does not say
              whether the PID around it is stable. The simulated check cannot
              answer that for a real robot either -- it knows nothing about that
              robot's backlash, encoder quantisation or load.

So the board emits an IDENT block, and this file holds down its shape, its
arithmetic and the two encodings that are easy to get quietly wrong.
"""
import math
import os
import sys

import pytest

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO_ROOT, "scripts"))

import drivetrain_report as dr  # noqa: E402

ACC = os.path.join(REPO_ROOT, "firmware", "src", "tools", "test_acc.cpp")

TRANSCRIPT = """IDENT gains kp=0.6000 ki=0.8000 kd=0.5000 pwm_max=1023 rate_hz=50
IDENT robot base=0 wheels=2 max_rpm=140 ratio=0.850 wheel_d=0.1000
IDENT deadzone wheel=1 pwm=40 duty=0.039
IDENT deadzone wheel=2 pwm=61 duty=0.060
IDENT plant wheel=1 steady_rpm=135.6 tau_ms=182 K=0.13255 peak_rpm=137.0
IDENT plant wheel=2 steady_rpm=118.2 tau_ms=240 K=0.11554 peak_rpm=119.9
IDENT loop wheel=1 sp=23.8 overshoot=0.121 crossings=3 settle_ms=1180 err=0.31 final=23.9
IDENT loop wheel=2 sp=23.8 overshoot=0.201 crossings=5 settle_ms=-1 err=0.88 final=24.6
IDENT done
MAX PWM 1023.0 -1023.0
MAX VEL   0.71   0.00 m/s    0.03 rad/s
time to 0.9x max vel   0.42 sec
"""


def test_the_block_parses_into_per_wheel_facts():
    got = dr.parse_ident(TRANSCRIPT)
    assert got["gains"]["kp"] == 0.6
    assert got["plant"][1]["tau_ms"] == 182
    assert got["plant"][2]["K"] == 0.11554
    assert got["deadzone"][2]["pwm"] == 61
    assert len(got["loop"]) == 2


def test_an_older_firmware_says_so_instead_of_erroring():
    """A board running firmware from before this existed is a fact to report,
    not a crash -- and the two-line fallback still works for it."""
    assert dr.parse_ident("MAX VEL 0.71 m/s\ntime to 0.9x max vel 0.42 sec\n") is None
    assert dr.parse_test_acc(TRANSCRIPT) is not None, "the fallback must still parse"


def test_mismatched_wheels_are_named():
    """The finding the aggregate table cannot make."""
    found = dr.wheels_disagree(dr.parse_ident(TRANSCRIPT))
    assert found, "a 182 ms wheel beside a 240 ms one must be reported"
    assert any(label == "time constant" for _, label, _, _, _ in found)


def test_matching_wheels_are_not_flagged():
    text = TRANSCRIPT.replace("steady_rpm=118.2 tau_ms=240 K=0.11554",
                              "steady_rpm=134.0 tau_ms=180 K=0.13100")
    assert not dr.wheels_disagree(dr.parse_ident(text))


def test_the_tuner_uses_the_median_wheel_not_the_mean():
    """A single seized or miswired wheel drags an average and produces gains
    that suit no wheel at all; the median still describes a real one."""
    text = TRANSCRIPT + ("IDENT plant wheel=3 steady_rpm=2.0 tau_ms=4000 K=0.00195 "
                         "peak_rpm=2.1\n")
    d = dr.drivetrain({"kinematics": {"base_type": "2wd", "wheel_diameter": 0.1,
                                      "lr_wheels_distance": 0.271, "max_rpm": 140,
                                      "counts_per_rev": 4000}})
    plant = dr.plant_from_ident(d, dr.parse_ident(text))
    # The seized wheel must not drag the fit toward itself.
    assert plant["gain"] > 0.10, plant
    assert plant["tau"] < 0.5, plant


def test_never_settled_is_not_reported_as_instant():
    """settle_ms 0 would read as "settled immediately", which is the opposite of
    what it means -- and it is exactly the wheel you most want to notice."""
    lines = dr.ident_report(dr.parse_ident(TRANSCRIPT))
    never = [l for l in lines if "wheel 2 at" in l]
    assert never and "never" in never[0], never
    assert "RINGS" in never[0]
    src = open(ACC, encoding="utf-8").read()
    assert "settle_tick >= ticks ? -1" in src, "the firmware must emit -1, not 0"


def test_the_firmware_emits_every_field_the_parser_reads():
    src = open(ACC, encoding="utf-8").read()
    for line in ("IDENT gains", "IDENT robot", "IDENT deadzone", "IDENT plant",
                 "IDENT loop", "IDENT done"):
        assert line in src, line
    for field in ("kp=", "pwm_max=", "rate_hz=", "steady_rpm=", "tau_ms=", "K=",
                  "overshoot=", "crossings=", "settle_ms=", "err="):
        assert field in src, field


def test_the_loop_pass_starts_from_a_known_state():
    """The integral carries between steps, so the second setpoint would be
    measured on a loop that is already wound -- and its overshoot would belong to
    the previous step. Done through the PID's own zeroing path rather than by
    adding a reset method, so identification starts where real driving starts."""
    src = open(ACC, encoding="utf-8").read()
    body = src[src.index("void loopStep"):src.index("void run()")]
    assert "compute(0.0f" in body, "the loop is not zeroed between setpoints"
    pid_cpp = open(os.path.join(REPO_ROOT, "firmware", "common", "lib", "pid",
                                "pid.cpp"), encoding="utf-8").read()
    assert "setpoint == 0.0f && fabs(error) < 0.5" in pid_cpp, \
        "the firmware's zeroing path moved; test_acc relies on it"


def test_it_identifies_at_three_setpoints():
    """A wheel loop is not linear: stiction and the dead zone dominate at a
    crawl, the PWM rail and the pack's sag near the top."""
    src = open(ACC, encoding="utf-8").read()
    run = src[src.index("void run()"):src.index("}  // namespace ident")]
    assert run.count("loopStep(") == 3, run


def test_it_runs_once_not_once_per_run():
    """The twelve runs below halve the PWM as they go, and a plant fitted across
    different step sizes is not a plant."""
    src = open(ACC, encoding="utf-8").read()
    assert "static bool identified = false;" in src
    assert "if (!identified)" in src


def test_identification_is_skipped_on_a_fake_wheel_board():
    """It must sit AFTER the refusal: there is nothing to identify, and the
    host models it better than a serial line can report it."""
    src = open(ACC, encoding="utf-8").read()
    loop = src[src.index("void loop_()"):]
    assert loop.index("if (fake_wheels)") < loop.index("ident::run()")


def test_no_trace_is_stored_for_the_closed_loop_pass():
    """Everything is accumulated online. A per-wheel trace at the control rate is
    several kilobytes on a board whose whole static segment is 124580 bytes."""
    src = open(ACC, encoding="utf-8").read()
    body = src[src.index("void loopStep"):src.index("void run()")]
    assert "[" not in body.split("for (unsigned t")[1].split("}")[0], \
        "the closed-loop pass appears to index an array"
