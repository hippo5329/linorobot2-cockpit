"""The turn radius is not the same for all three base types, and it was.

Upstream computes the tangential wheel speed as `angular_z * (lr / 2)` for a
differential drive, a skid steer AND a mecanum, in two places -- once to
command the wheels and once to read them back as odometry.

For a differential drive that is correct: the centre of rotation is the middle
of the drive axle.

For a MECANUM it is wrong. The standard term is (lx + ly): the rollers put the
WHEELBASE into the yaw equation alongside the track. Ignoring it under-commands
every turn by (1 + fr/lr) -- on the shipped pico2_mecanum reference, lr 0.271
and fr 0.18, a factor of 1.66. Ask that robot for 1.0 rad/s and it gives 0.60.

For a SKID STEER the ideal model happens to match the differential one -- the
four wheels are at +/-lr/2 and the wheelbase does not enter it -- but four
driven wheels cannot pivot without sliding sideways, and the scrub makes the
real turn slower than the ideal. That is a property of tyres and floor, not of
geometry, so it is a per-robot measurement (kinematics.angular_scale) rather
than a derived number.

Why the bench never caught it: the same radius converts the command into wheel
RPM and the RPM back into odometry, so in sim mode the error cancels exactly.
Every leg of the matrix reports odom agreeing with cmd_vel to three decimals on
a mecanum base that would, on a real floor, turn at 60% of the commanded rate.
These tests compile the real kinematics.cpp and drive it, which is the only
place that check can live.
"""
import ctypes
import math
import os
import shutil
import subprocess

import pytest

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
KIN_DIR = os.path.join(REPO_ROOT, "firmware", "common", "lib", "kinematics")

STUB = """
#pragma once
#include <math.h>
#include <stdint.h>
#ifndef PI
#define PI 3.1415926535897932384626433832795
#endif
template <class T, class A, class B>
T constrain(T v, A lo, B hi) { return v < (T)lo ? (T)lo : (v > (T)hi ? (T)hi : v); }
"""

SHIM = """
#include "kinematics.h"
extern "C" {
void *kin_new(int base, int max_rpm, float ratio, float op_v, float max_v,
              float wheel_d, float lr, float fr, float scale) {
    return (void *)new Kinematics((Kinematics::base)base, max_rpm, ratio, op_v,
                                  max_v, wheel_d, lr, fr, scale);
}
float kin_radius(void *k) { return ((Kinematics *)k)->getRotationRadius(); }
void kin_rpm(void *k, float vx, float vy, float wz, float *out) {
    Kinematics::rpm r = ((Kinematics *)k)->getRPM(vx, vy, wz);
    out[0] = r.motor1; out[1] = r.motor2; out[2] = r.motor3; out[3] = r.motor4;
}
void kin_vel(void *k, float m1, float m2, float m3, float m4, float *out) {
    Kinematics::velocities v = ((Kinematics *)k)->getVelocities(m1, m2, m3, m4);
    out[0] = v.linear_x; out[1] = v.linear_y; out[2] = v.angular_z;
}
}
"""

DIFFERENTIAL, SKID_STEER, MECANUM = 0, 1, 2

# The default chassis every reference config shares.
WHEEL_D = 0.1
LR = 0.271
FR = 0.18
MAX_RPM = 140


@pytest.fixture(scope="module")
def kin_lib(tmp_path_factory):
    cxx = shutil.which("g++") or shutil.which("clang++")
    if not cxx:
        pytest.skip("no host C++ compiler")
    d = tmp_path_factory.mktemp("kin")
    open(os.path.join(d, "Arduino.h"), "w").write(STUB)
    open(os.path.join(d, "shim.cpp"), "w").write(SHIM)
    so = os.path.join(d, "libkin.so")
    subprocess.run(
        [cxx, "-shared", "-fPIC", "-O0", "-Wall", "-Werror", "-o", so,
         os.path.join(d, "shim.cpp"), os.path.join(KIN_DIR, "kinematics.cpp"),
         "-I", str(d), "-I", KIN_DIR],
        check=True, capture_output=True)
    lib = ctypes.CDLL(so)
    lib.kin_new.restype = ctypes.c_void_p
    lib.kin_new.argtypes = [ctypes.c_int, ctypes.c_int] + [ctypes.c_float] * 7
    lib.kin_radius.restype = ctypes.c_float
    lib.kin_radius.argtypes = [ctypes.c_void_p]
    lib.kin_rpm.argtypes = [ctypes.c_void_p, ctypes.c_float, ctypes.c_float,
                            ctypes.c_float, ctypes.POINTER(ctypes.c_float * 4)]
    lib.kin_vel.argtypes = [ctypes.c_void_p] + [ctypes.c_float] * 4 + \
        [ctypes.POINTER(ctypes.c_float * 3)]
    return lib


class Base:
    """A base whose wheels do exactly what they are told: the ideal plant.

    Deliberately ideal. The question these tests ask is whether the firmware's
    own two halves agree with the physics, not whether a motor tracks; an ideal
    plant is the only one where a disagreement can only be the maths.
    """

    def __init__(self, lib, base, lr=LR, fr=FR, scale=1.0):
        self.lib = lib
        self.k = lib.kin_new(base, MAX_RPM, 1.0, 12.0, 12.0, WHEEL_D, lr, fr, scale)

    @property
    def radius(self):
        return self.lib.kin_radius(self.k)

    def rpm(self, vx, vy=0.0, wz=0.0):
        out = (ctypes.c_float * 4)()
        self.lib.kin_rpm(self.k, vx, vy, wz, ctypes.byref(out))
        return list(out)

    def vel(self, rpms):
        out = (ctypes.c_float * 3)()
        self.lib.kin_vel(self.k, *rpms, ctypes.byref(out))
        return list(out)

    def round_trip(self, vx, vy=0.0, wz=0.0):
        """Command -> wheels -> measured motion, the way sim mode does it."""
        return self.vel(self.rpm(vx, vy, wz))


def _wheel_speeds(rpms):
    """RPM -> m/s at the rim."""
    return [r / 60.0 * math.pi * WHEEL_D for r in rpms]


# --- the radius itself -------------------------------------------------------

def test_a_differential_drive_turns_on_its_axle(kin_lib):
    assert Base(kin_lib, DIFFERENTIAL).radius == pytest.approx(LR / 2)


def test_a_mecanum_turns_on_the_track_AND_the_wheelbase(kin_lib):
    """(lx + ly), the textbook term -- not ly alone."""
    assert Base(kin_lib, MECANUM).radius == pytest.approx((LR + FR) / 2)


def test_the_mecanum_wheelbase_is_not_silently_dropped(kin_lib):
    """The regression, stated as a number: the shipped mecanum reference.

    If this ever reads 0.1355 again, a mecanum robot asked for 1 rad/s turns at
    0.6 and its odometry agrees with it.
    """
    m = Base(kin_lib, MECANUM)
    assert m.radius > Base(kin_lib, DIFFERENTIAL).radius
    assert m.radius == pytest.approx(0.2255, abs=1e-4)


def test_a_skid_steer_is_geometrically_a_differential_drive(kin_lib):
    """Uncalibrated, it must behave exactly as before: no invented constant."""
    assert Base(kin_lib, SKID_STEER).radius == pytest.approx(LR / 2)


def test_the_scrub_measurement_widens_the_skid_track(kin_lib):
    """angular_scale is how a measured chassis says "I turn slower than my tape
    measure predicts"."""
    assert Base(kin_lib, SKID_STEER, scale=1.3).radius == pytest.approx(LR / 2 * 1.3)
    # and it must not touch the other two: they do not scrub
    assert Base(kin_lib, DIFFERENTIAL, scale=1.3).radius == pytest.approx(LR / 2)
    assert Base(kin_lib, MECANUM, scale=1.3).radius == pytest.approx((LR + FR) / 2)


# --- what the wheels are actually told ---------------------------------------

def test_a_mecanum_asked_to_spin_drives_its_wheels_at_the_right_speed(kin_lib):
    """The physical statement: at w rad/s a wheel at radius R runs at w*R."""
    m = Base(kin_lib, MECANUM)
    wz = 1.0
    speeds = _wheel_speeds(m.rpm(0.0, 0.0, wz))
    assert abs(speeds[0]) == pytest.approx(wz * (LR + FR) / 2, rel=1e-3)
    # the old, wrong value -- 0.1355 m/s -- must not come back
    assert abs(speeds[0]) > wz * LR / 2 * 1.5


def test_the_two_halves_cannot_disagree(kin_lib):
    """Command -> wheels -> odometry is the identity for every base type.

    This is the property sim mode DOES have, and the reason sim mode cannot
    see the bug above: it held just as firmly when the radius was wrong. It is
    here so that fixing one half without the other fails loudly.
    """
    for base in (DIFFERENTIAL, SKID_STEER, MECANUM):
        b = Base(kin_lib, base, scale=1.25 if base == SKID_STEER else 1.0)
        vx, vy, wz = 0.15, (0.1 if base == MECANUM else 0.0), 0.5
        out = b.round_trip(vx, vy, wz)
        assert out[0] == pytest.approx(vx, rel=2e-3), base
        assert out[1] == pytest.approx(vy, rel=2e-3, abs=1e-6), base
        assert out[2] == pytest.approx(wz, rel=2e-3), base


# --- the saturation rule -----------------------------------------------------

def test_an_over_fast_diagonal_turn_keeps_its_direction(kin_lib):
    """The mecanum case the two old guards both missed.

    Each required one component to be EXACTLY zero -- `angular_z == 0` or
    `linear_y == 0` -- so a base doing all three at once was never scaled, and
    the per-motor constrain() clipped whichever wheels were over the rail.
    Clipping changes the RATIO between the wheels, and for a mecanum the ratio
    is the direction: the robot curves off the commanded heading instead of
    tracking it more slowly.
    """
    m = Base(kin_lib, MECANUM)
    vx, vy, wz = 3.0, 2.0, 2.5          # far beyond 140 rpm
    got = m.round_trip(vx, vy, wz)
    # slower, but the same motion: every component scaled by one factor
    scale = got[0] / vx
    assert 0.0 < scale < 1.0, got
    assert got[1] == pytest.approx(vy * scale, rel=5e-3), got
    assert got[2] == pytest.approx(wz * scale, rel=5e-3), got


def test_saturation_actually_binds_at_the_motor_limit(kin_lib):
    """Scaled to fit, not scaled to nothing: the busiest wheel sits on the rail."""
    m = Base(kin_lib, MECANUM)
    rpms = m.rpm(3.0, 2.0, 2.5)
    assert max(abs(r) for r in rpms) == pytest.approx(MAX_RPM, rel=1e-3)


def test_a_differential_drive_saturates_exactly_as_it_used_to(kin_lib):
    """The general rule must subsume the old `|x| + |tan|` branch, not replace
    its behaviour: 2wd is what every green bench leg was driven with."""
    d = Base(kin_lib, DIFFERENTIAL)
    vx, wz = 2.0, 1.0
    rpms = d.rpm(vx, 0.0, wz)
    assert max(abs(r) for r in rpms) == pytest.approx(MAX_RPM, rel=1e-3)
    got = d.round_trip(vx, 0.0, wz)
    assert got[2] / got[0] == pytest.approx(wz / vx, rel=5e-3), "turn radius changed"


def test_a_tiny_angular_z_no_longer_defeats_the_scaler(kin_lib):
    """`angular_z == 0` was float equality on a controller output. Nav2 sends
    1e-4 rad/s, and that was enough to skip scaling entirely."""
    m = Base(kin_lib, MECANUM)
    rpms = m.rpm(3.0, 2.0, 1e-4)
    assert max(abs(r) for r in rpms) == pytest.approx(MAX_RPM, rel=1e-3), rpms


def test_a_reachable_command_is_left_alone(kin_lib):
    """Scaling a base that is nowhere near its limit would be its own bug."""
    m = Base(kin_lib, MECANUM)
    got = m.round_trip(0.1, 0.05, 0.2)
    assert got[0] == pytest.approx(0.1, rel=2e-3)
    assert got[1] == pytest.approx(0.05, rel=2e-3)
    assert got[2] == pytest.approx(0.2, rel=2e-3)


def test_a_stopped_base_stays_stopped(kin_lib):
    for base in (DIFFERENTIAL, SKID_STEER, MECANUM):
        assert Base(kin_lib, base).rpm(0.0, 0.0, 0.0) == [0.0, 0.0, 0.0, 0.0]
