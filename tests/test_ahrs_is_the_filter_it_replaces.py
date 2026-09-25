"""The board's AHRS must BE imu_filter_madgwick, not a second opinion about it.

Moving the fusion onto the board removes a real fragility: imu/data was the rate
of MATCHED imu/data_raw + imu/mag pairs through an ApproximateTime synchroniser
five deep, and both topics cross a best-effort micro-ROS session. The bench
measured what that costs asymmetrically -- /odom held 33 Hz on two slowed legs
while /imu/data fell to 10, because /odom needs no partner.

But the move is only worth making if it changes WHERE the filter runs and not
WHAT it computes: a bench result from before and after has to be comparable, and
a firmware that quietly fuses differently would invalidate every heading number
this project has recorded.

So the first test here compiles BOTH -- firmware/common/lib/imu/ahrs.h and
imu_tools' own ImuFilter -- drives them with the same sequences, and requires the
quaternions to agree. The rest state the physics, so they hold even where the
reference is not checked out.
"""
import ctypes
import math
import os
import subprocess
import textwrap

import pytest

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MAIN = os.path.join(REPO_ROOT, "firmware", "src", "main.cpp")
AHRS_H = os.path.join(REPO_ROOT, "firmware", "common", "lib", "imu", "ahrs.h")
REF_DIR = os.path.join(os.path.dirname(REPO_ROOT), "imu_tools", "imu_filter_madgwick")
REF_CPP = os.path.join(REF_DIR, "src", "imu_filter.cpp")

SHIM = r"""
#include "ahrs.h"
extern "C" {
void *ah_new(float gain, float zeta) {
    AHRS *a = new AHRS(); a->setGain(gain); a->setZeta(zeta); return (void *)a;
}
void ah_update(void *h, float gx, float gy, float gz, float ax, float ay,
               float az, float mx, float my, float mz, float dt)
{ ((AHRS *)h)->update(gx, gy, gz, ax, ay, az, mx, my, mz, dt); }
void ah_update_imu(void *h, float gx, float gy, float gz, float ax, float ay,
                   float az, float dt)
{ ((AHRS *)h)->updateIMU(gx, gy, gz, ax, ay, az, dt); }
void ah_quat(void *h, double *out) {
    ((AHRS *)h)->quaternion(out[0], out[1], out[2], out[3]);
}
void ah_gravity(void *h, float g, float *out) {
    ((AHRS *)h)->gravity(out[0], out[1], out[2], g);
}
void ah_gravity_from(double qx, double qy, double qz, double qw, float g,
                     float *out)
{ AHRS::gravityFrom(qx, qy, qz, qw, g, out[0], out[1], out[2]); }
}
"""

REF_SHIM = r"""
#include "imu_filter_madgwick/imu_filter.h"
extern "C" {
void *rf_new(double gain, double zeta) {
    ImuFilter *f = new ImuFilter();
    f->setAlgorithmGain(gain);
    f->setDriftBiasGain(zeta);
    f->setWorldFrame(WorldFrame::ENU);
    return (void *)f;
}
void rf_update(void *h, float gx, float gy, float gz, float ax, float ay,
               float az, float mx, float my, float mz, float dt)
{ ((ImuFilter *)h)->madgwickAHRSupdate(gx, gy, gz, ax, ay, az, mx, my, mz, dt); }
void rf_update_imu(void *h, float gx, float gy, float gz, float ax, float ay,
                   float az, float dt)
{ ((ImuFilter *)h)->madgwickAHRSupdateIMU(gx, gy, gz, ax, ay, az, dt); }
void rf_quat(void *h, double *out) {
    ((ImuFilter *)h)->getOrientation(out[3], out[0], out[1], out[2]);
}
}
"""


def _build(tmp, name, source, includes, extra_sources=()):
    src = os.path.join(tmp, name + ".cpp")
    with open(src, "w", encoding="utf-8") as fh:
        fh.write(textwrap.dedent(source))
    so = os.path.join(tmp, name + ".so")
    cmd = ["g++", "-std=c++17", "-O2", "-shared", "-fPIC", "-o", so, src]
    cmd += list(extra_sources)
    for inc in includes:
        cmd += ["-I", inc]
    res = subprocess.run(cmd, capture_output=True, text=True)
    return so, res


@pytest.fixture(scope="module")
def ours(tmp_path_factory):
    """The SUBJECT. A build failure here is a failure, never a skip."""
    tmp = str(tmp_path_factory.mktemp("ahrs"))
    so, res = _build(tmp, "ours", SHIM, [os.path.dirname(AHRS_H)])
    if res.returncode != 0:
        pytest.fail(f"ahrs.h does not build: {res.stderr[:1500]}")
    lib = ctypes.CDLL(so)
    lib.ah_new.restype = ctypes.c_void_p
    lib.ah_new.argtypes = [ctypes.c_float, ctypes.c_float]
    lib.ah_update.argtypes = [ctypes.c_void_p] + [ctypes.c_float] * 10
    lib.ah_update_imu.argtypes = [ctypes.c_void_p] + [ctypes.c_float] * 7
    lib.ah_quat.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_double)]
    lib.ah_gravity.argtypes = [ctypes.c_void_p, ctypes.c_float,
                               ctypes.POINTER(ctypes.c_float)]
    lib.ah_gravity_from.argtypes = [ctypes.c_double] * 4 + [
        ctypes.c_float, ctypes.POINTER(ctypes.c_float)]
    return lib


@pytest.fixture(scope="module")
def reference(tmp_path_factory):
    """The REFERENCE, which is another repo and may simply not be here. That is
    the one honest skip in this file -- it is not the code under test."""
    if not os.path.exists(REF_CPP):
        pytest.skip(f"imu_tools is not checked out at {REF_DIR}")
    tmp = str(tmp_path_factory.mktemp("ref"))
    so, res = _build(tmp, "ref", REF_SHIM,
                     [os.path.join(REF_DIR, "include")], [REF_CPP])
    if res.returncode != 0:
        pytest.skip(f"the reference filter does not build here: {res.stderr[:400]}")
    lib = ctypes.CDLL(so)
    lib.rf_new.restype = ctypes.c_void_p
    lib.rf_new.argtypes = [ctypes.c_double, ctypes.c_double]
    lib.rf_update.argtypes = [ctypes.c_void_p] + [ctypes.c_float] * 10
    lib.rf_update_imu.argtypes = [ctypes.c_void_p] + [ctypes.c_float] * 7
    lib.rf_quat.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_double)]
    return lib


def _quat(lib, handle, getter):
    buf = (ctypes.c_double * 4)()
    getter(handle, buf)
    return [buf[i] for i in range(4)]      # x, y, z, w


# A level robot facing along +X in an ENU world, with a field 50 uT north and
# 40 uT down. Then a yaw rate. Nothing here is tuned to either implementation.
LEVEL_ACCEL = (0.0, 0.0, 9.81)
FIELD = (0.0, 50.0, -40.0)


def _sequence(spin_rate, n=300, dt=0.02):
    return [((0.0, 0.0, spin_rate), LEVEL_ACCEL, FIELD, dt) for _ in range(n)]


@pytest.mark.parametrize("spin", [0.0, 0.5, -1.2])
def test_the_board_filter_agrees_with_the_node_it_replaces(ours, reference, spin):
    a = ours.ah_new(0.1, 0.0)
    r = reference.rf_new(0.1, 0.0)
    for gyro, accel, mag, dt in _sequence(spin):
        ours.ah_update(a, *gyro, *accel, *mag, dt)
        reference.rf_update(r, *gyro, *accel, *mag, dt)
    got = _quat(ours, a, ours.ah_quat)
    want = _quat(reference, r, reference.rf_quat)
    # Quaternions are double-cover: q and -q are the same rotation.
    if sum(g * w for g, w in zip(got, want)) < 0:
        want = [-v for v in want]
    for name, g, w in zip("xyzw", got, want):
        assert g == pytest.approx(w, abs=2e-3), (
            f"spin {spin}: q{name} {g:.6f} against the reference's {w:.6f} -- "
            "the board filter is not the filter it replaces")


def test_the_six_axis_path_agrees_too(ours, reference):
    a = ours.ah_new(0.1, 0.0)
    r = reference.rf_new(0.1, 0.0)
    for _ in range(300):
        ours.ah_update_imu(a, 0.0, 0.0, 0.3, *LEVEL_ACCEL, 0.02)
        reference.rf_update_imu(r, 0.0, 0.0, 0.3, *LEVEL_ACCEL, 0.02)
    got = _quat(ours, a, ours.ah_quat)
    want = _quat(reference, r, reference.rf_quat)
    if sum(g * w for g, w in zip(got, want)) < 0:
        want = [-v for v in want]
    for name, g, w in zip("xyzw", got, want):
        assert g == pytest.approx(w, abs=2e-3), f"q{name} {g:.6f} vs {w:.6f}"


def test_a_missing_magnetometer_falls_back_rather_than_producing_nan(ours):
    """An all-zero or non-finite field is what a failed I2C read looks like. The
    reference guards it because a NaN quaternion poisons every consumer."""
    a = ours.ah_new(0.1, 0.0)
    for _ in range(100):
        ours.ah_update(a, 0.0, 0.0, 0.2, *LEVEL_ACCEL, 0.0, 0.0, 0.0, 0.02)
    q = _quat(ours, a, ours.ah_quat)
    assert all(math.isfinite(v) for v in q), f"quaternion went non-finite: {q}"
    assert abs(sum(v * v for v in q) - 1.0) < 1e-3, f"not a unit quaternion: {q}"


def test_a_level_robot_reports_gravity_on_z_and_nothing_on_x_or_y(ours):
    """What the caller subtracts to publish a specific force that means
    acceleration -- the ax/ay the EKF fuses."""
    a = ours.ah_new(0.1, 0.0)
    for _ in range(400):
        ours.ah_update(a, 0.0, 0.0, 0.0, *LEVEL_ACCEL, *FIELD, 0.02)
    buf = (ctypes.c_float * 3)()
    ours.ah_gravity(a, 9.81, buf)
    gx, gy, gz = buf[0], buf[1], buf[2]
    assert abs(gx) < 0.05, f"gravity leaked {gx:.3f} into x on a level robot"
    assert abs(gy) < 0.05, f"gravity leaked {gy:.3f} into y on a level robot"
    assert gz == pytest.approx(9.81, abs=0.05), f"gravity on z is {gz:.3f}"


def test_a_tilted_robot_puts_gravity_where_the_tilt_points(ours):
    """The whole reason the EKF needs this: a 5 degree pitch is 0.85 m/s2 of
    constant fake acceleration in ax if nobody removes it."""
    a = ours.ah_new(0.1, 0.0)
    pitch = math.radians(5.0)
    accel = (-9.81 * math.sin(pitch), 0.0, 9.81 * math.cos(pitch))
    for _ in range(600):
        ours.ah_update(a, 0.0, 0.0, 0.0, *accel, *FIELD, 0.02)
    buf = (ctypes.c_float * 3)()
    ours.ah_gravity(a, 9.81, buf)
    assert buf[0] == pytest.approx(accel[0], abs=0.15), (
        f"the estimate puts gravity at x={buf[0]:.3f} where the reading says "
        f"{accel[0]:.3f}")


# ---------------------------------------------------------------------------
# Removing gravity, and the part that nearly did not get it.
#
# The filter stands aside for a BNO085, which fuses on-chip -- and that part
# reports RAW specific force, so gravity would have stayed in the ax and ay the
# EKF fuses, on exactly the one part whose orientation is best. Meanwhile
# bringup.launch.py tells the EKF not to remove gravity, because the board
# normally has. One part, quietly feeding a constant 9.8 into a fused
# acceleration. Caught by reading the diff, so it gets a test.
def test_the_two_gravity_entry_points_are_the_same_arithmetic(ours):
    """gravity() reads the filter's own quaternion; gravityFrom() takes any
    quaternion, so a chip-fused part is served by the same code. Two spellings of
    one formula is how they drift apart."""
    a = ours.ah_new(0.1, 0.0)
    pitch = math.radians(7.0)
    accel = (-9.81 * math.sin(pitch), 0.0, 9.81 * math.cos(pitch))
    for _ in range(600):
        ours.ah_update(a, 0.0, 0.0, 0.0, *accel, *FIELD, 0.02)

    member = (ctypes.c_float * 3)()
    ours.ah_gravity(a, 9.80665, member)
    q = _quat(ours, a, ours.ah_quat)
    static = (ctypes.c_float * 3)()
    ours.ah_gravity_from(q[0], q[1], q[2], q[3], 9.80665, static)
    for i, axis in enumerate("xyz"):
        assert member[i] == pytest.approx(static[i], abs=1e-4), (
            f"g{axis}: member {member[i]:.6f} against static {static[i]:.6f}")


def test_an_identity_orientation_puts_all_of_gravity_on_z(ours):
    """What the firmware subtracts before the estimate has converged, and what a
    driver that never fills in a quaternion leaves behind."""
    out = (ctypes.c_float * 3)()
    ours.ah_gravity_from(0.0, 0.0, 0.0, 1.0, 9.80665, out)
    assert out[0] == pytest.approx(0.0, abs=1e-6)
    assert out[1] == pytest.approx(0.0, abs=1e-6)
    assert out[2] == pytest.approx(9.80665, abs=1e-6)


def _fusion_branch(src):
    """The body of `if (!imu || !imu->hasFusedOrientation())`, by brace matching
    rather than by looking for a string that happens to follow it."""
    i = src.index("if (!imu || !imu->hasFusedOrientation())")
    open_brace = src.index("{", i)
    depth = 0
    for j in range(open_brace, len(src)):
        if src[j] == "{":
            depth += 1
        elif src[j] == "}":
            depth -= 1
            if depth == 0:
                return src[open_brace:j + 1]
    raise AssertionError("the fusion branch is not brace-balanced")


def test_gravity_is_removed_outside_the_branch_that_can_be_skipped():
    """The bug this catches: with the subtraction inside, a chip-fused part gets
    no removal at all, and there is nothing in any log to say so."""
    src = open(MAIN, encoding="utf-8").read()
    branch = _fusion_branch(src)
    assert "gravityFrom" not in branch, (
        "gravity is removed inside the branch that a chip-fused part skips, so "
        "a BNO085 would publish specific force as acceleration")
    assert "AHRS::gravityFrom(" in src, "nothing removes gravity at all"


def test_the_filter_is_seeded_by_a_flag_not_by_the_rollover_guard():
    """sim_wheel.h can lean on `dt > 1 s` to catch its first call, because the
    wheels are fed from boot while micros() is still small. Publishing starts
    only after the agent handshake, so here that guard would let an unseeded
    first interval -- micros() itself -- through as real rotation."""
    src = open(MAIN, encoding="utf-8").read()
    branch = _fusion_branch(src)
    assert "ahrs_seeded" in branch, "the first interval is not seeded explicitly"
    assert branch.index("ahrs_seeded") < branch.index("1000000UL"), \
        "the seeding check comes after the interval is already being judged"


def test_the_real_path_does_not_borrow_a_simulation_constant():
    """It used SIM_IMU_GRAVITY -- a constant from sim_wheel.h -- for the gravity
    it removes on a real robot. ahrs.h owns AHRS_GRAVITY instead."""
    src = open(MAIN, encoding="utf-8").read()
    branch_and_after = src[src.index("if (!imu || !imu->hasFusedOrientation())"):]
    branch_and_after = branch_and_after[:branch_and_after.index("diagTime(DIAGT_SENSORS")]
    assert "SIM_IMU_GRAVITY" not in branch_and_after, \
        "the real fusion path depends on a simulation constant"
    assert "AHRS_GRAVITY" in branch_and_after
    ahrs = open(AHRS_H, encoding="utf-8").read()
    assert "#define AHRS_GRAVITY 9.80665f" in ahrs, \
        "standard gravity is no longer stated where the filter can see it"
