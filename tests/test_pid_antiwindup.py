"""The PID must not wind up, and must let go when it is told to stand still.

`integral_` was a pure accumulator with no bound, and the one line that ever
cleared it was guarded by `setpoint == 0 && error == 0` -- exact float
equality on a noisy measurement, so in practice it never fired.

Found on the bench 2026-09-20 with an instrumented build, on a GenDrv board
commanded to hold still:

    DBG cmd=0.00,0.00 req=0.0,0.0 rpm=-139.5,138.4 pwm=-1023,1023 sim=1
    DBG cmd=0.25,0.00 req=31.4,31.4 rpm=-138.4,139.6 pwm=-1023,1023 sim=1

Both wheels pinned at opposite rails at the 140 rpm maximum while the request
was zero, and asking for 31.4 rpm changed nothing -- the integral term alone
was already past the rail. The robot spun on the spot; the drive suite scored
0/6 and Nav2 could not move the base. With the fix the same board scores 6/6.

These are behavioural tests: they compile the real pid.cpp against a stub
Arduino.h and run the loop, so they fail if the clamp is weakened rather than
merely reworded.
"""
import ctypes
import os
import shutil
import subprocess
import sys

import pytest

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PID_DIR = os.path.join(REPO_ROOT, "firmware", "common", "lib", "pid")

STUB = """
#pragma once
#include <math.h>
#include <stdint.h>
template <class T, class A, class B>
T constrain(T v, A lo, B hi) { return v < (T)lo ? (T)lo : (v > (T)hi ? (T)hi : v); }
"""

SHIM = """
#include "pid.h"
extern "C" {
void *pid_new(float mn, float mx, float kp, float ki, float kd) {
    return (void *)new PID(mn, mx, kp, ki, kd);
}
double pid_compute(void *p, float setpoint, float measured) {
    return ((PID *)p)->compute(setpoint, measured);
}
}
"""

MAX_PWM = 1023.0
MAX_RPM = 140.0


@pytest.fixture(scope="module")
def pid_lib(tmp_path_factory):
    cxx = shutil.which("g++") or shutil.which("clang++")
    if not cxx:
        pytest.skip("no host C++ compiler")
    d = tmp_path_factory.mktemp("pid")
    open(os.path.join(d, "Arduino.h"), "w").write(STUB)
    open(os.path.join(d, "shim.cpp"), "w").write(SHIM)
    so = os.path.join(d, "libpid.so")
    subprocess.run(
        [cxx, "-shared", "-fPIC", "-O0", "-o", so,
         os.path.join(d, "shim.cpp"), os.path.join(PID_DIR, "pid.cpp"),
         "-I", str(d), "-I", PID_DIR],
        check=True, capture_output=True)
    lib = ctypes.CDLL(so)
    lib.pid_new.restype = ctypes.c_void_p
    lib.pid_new.argtypes = [ctypes.c_float] * 5
    lib.pid_compute.restype = ctypes.c_double
    lib.pid_compute.argtypes = [ctypes.c_void_p, ctypes.c_float, ctypes.c_float]
    return lib


class Wheel:
    """A first-order motor that cannot exceed MAX_RPM -- the plant that made
    the real loop saturate."""

    def __init__(self, lib, kp=0.6, ki=0.3, kd=0.1):
        self.lib = lib
        self.pid = lib.pid_new(-MAX_PWM, MAX_PWM, kp, ki, kd)
        self.rpm = 0.0

    def step(self, setpoint, noise=0.0):
        pwm = self.lib.pid_compute(self.pid, setpoint, self.rpm + noise)
        target = MAX_RPM * pwm / MAX_PWM
        self.rpm += (target - self.rpm) * 0.08
        return pwm


def test_an_unreachable_setpoint_does_not_pin_the_output_forever(pid_lib):
    """Ask for more than the motor can give, for a long time, then ask for
    zero. The loop must come off the rail at once and actually stop the wheel."""
    w = Wheel(pid_lib)
    for _ in range(4000):
        w.step(10 * MAX_RPM)
    assert w.step(10 * MAX_RPM) == pytest.approx(MAX_PWM), "should be saturated"

    outputs = [w.step(0.0) for _ in range(120)]
    assert outputs[0] < MAX_PWM, "still on the +rail one tick after cmd=0"
    assert min(outputs) < 0, "never even tried to brake"
    assert abs(w.rpm) < 1.0, (
        "wheel still turning at %.1f rpm four seconds after being told to "
        "stop -- the integral kept a debt it cannot repay" % w.rpm)
    assert abs(outputs[-1]) < MAX_PWM * 0.05, (
        "%.0f PWM into a stopped wheel" % outputs[-1])


def test_it_lets_go_when_told_to_stand_still_on_a_noisy_measurement(pid_lib):
    """The old reset needed `error == 0` exactly. Real encoders never give
    that, so the tolerance is the point of the test."""
    w = Wheel(pid_lib)
    for _ in range(2000):
        w.step(120.0)
    w.rpm = 0.0
    noise = [0.21, -0.17, 0.09, -0.3, 0.14, -0.05]
    out = [w.step(0.0, noise[i % len(noise)]) for i in range(20)]
    assert abs(out[-1]) < 5.0, (
        "%.1f PWM while standing still with sub-rpm noise -- the integral did "
        "not reset" % out[-1])


def test_the_integral_alone_cannot_exceed_the_actuator(pid_lib):
    """Saturate hard, then reverse the request. The wheel must turn round
    within a few ticks, not after unwinding minutes of accumulation."""
    w = Wheel(pid_lib)
    for _ in range(8000):
        w.step(10 * MAX_RPM)
    flipped = None
    for i in range(1, 60):
        if w.step(-MAX_RPM) < 0:
            flipped = i
            break
    assert flipped is not None and flipped < 40, (
        "took %s ticks to reverse" % flipped)


def test_a_zero_gain_integral_is_not_a_division_by_zero(pid_lib):
    """The clamp divides by ki. With no integral term at all it must be
    skipped, not evaluated -- a NaN here would be silently constrained into a
    plausible-looking PWM."""
    w = Wheel(pid_lib, ki=0.0)
    out = 0.0
    for _ in range(500):
        out = w.step(10 * MAX_RPM)
    assert out == out, "NaN out of the clamp"
    assert 0.0 < out <= MAX_PWM
    assert w.rpm == w.rpm and w.rpm > 0.0
