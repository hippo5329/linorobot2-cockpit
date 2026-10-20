"""Static friction is higher than running friction in the simulated wheel.

On the floor a tracked robot's tracks broke away at 13-20 % duty (lino-ident ramp,
2026-10-19) while Nav2's 0.3 rad/s heading turn asks for about 8 %: the real
tracks stick-slipped through every slow turn. The model had one 4 % band for
both starting and running, so a simulated robot turned smoothly at any rate and
nothing that depends on the difference -- the PID's dead-zone compensation, a
controller that limits its command by the MEASURED speed -- could be seen in
simulation. `simulation.breakaway_duty` (env sim_breakaway) is the stopped
wheel's threshold, `simulation.stall_duty` (sim_stall_duty) the turning one's.

Compiled from the real header, like test_sim_wheel_gearbox_and_sag.py.
"""
import ctypes
import os
import shutil
import subprocess

import pytest

from test_sim_wheel_gearbox_and_sag import ENC, ROOT, SHIM, STUB

STEP_US = 20000          # the firmware's control period


def _build(tmp_path_factory, name, defines):
    cxx = shutil.which("g++") or shutil.which("clang++")
    if not cxx:
        pytest.skip("no host C++ compiler")
    d = tmp_path_factory.mktemp(name)
    open(os.path.join(d, "Arduino.h"), "w").write(STUB)
    open(os.path.join(d, "encoder_interface.h"), "w").write(
        open(os.path.join(ENC, "encoder_interface.h")).read())
    open(os.path.join(d, "mcu_env.h"), "w").write(
        "#pragma once\n"
        "static inline float envFloat(const char *, float d) { return d; }\n"
        "static inline bool envFlag(const char *, bool d) { return d; }\n"
        "static inline const char *envGet(const char *, const char *d) { return d; }\n"
        "static inline const char *envPrefixed(const char *n) { return n; }\n"
        "static inline int envInt(const char *, int d) { return d; }\n"
        "static inline unsigned envU16(const char *, unsigned d) { return d; }\n"
        "static inline unsigned long envU32(const char *, unsigned long d) { return d; }\n"
        "static inline void initMcuEnv() {}\n"
        "static inline void envFloatVec(const char *, float *, int) {}\n")
    for sub in ("micro_ros_utilities", "sensor_msgs/msg", "geometry_msgs/msg"):
        os.makedirs(os.path.join(d, sub), exist_ok=True)
    open(os.path.join(d, "micro_ros_utilities", "string_utilities.h"), "w").write(
        "#pragma once\n#include <string.h>\n"
        "typedef struct { char *data; size_t size; size_t capacity; } rosidl_runtime_c__String;\n"
        "static inline rosidl_runtime_c__String micro_ros_string_utilities_set("
        "rosidl_runtime_c__String s, const char *) { return s; }\n")
    open(os.path.join(d, "geometry_msgs", "msg", "vector3.h"), "w").write(
        "#pragma once\ntypedef struct { double x, y, z; } geometry_msgs__msg__Vector3;\n")
    open(os.path.join(d, "sensor_msgs", "msg", "imu.h"), "w").write(
        "#pragma once\n#include <geometry_msgs/msg/vector3.h>\n"
        "#include <micro_ros_utilities/string_utilities.h>\n"
        "typedef struct { int sec; unsigned nanosec; } __t;\n"
        "typedef struct { __t stamp; rosidl_runtime_c__String frame_id; } __h;\n"
        "typedef struct { double x, y, z, w; } __q;\n"
        "typedef struct { __h header; __q orientation; double orientation_covariance[9];\n"
        "  geometry_msgs__msg__Vector3 angular_velocity; double angular_velocity_covariance[9];\n"
        "  geometry_msgs__msg__Vector3 linear_acceleration; double linear_acceleration_covariance[9];\n"
        "} sensor_msgs__msg__Imu;\n")
    open(os.path.join(d, "sensor_msgs", "msg", "magnetic_field.h"), "w").write(
        "#pragma once\n#include <sensor_msgs/msg/imu.h>\n"
        "typedef struct { __h header; geometry_msgs__msg__Vector3 magnetic_field;\n"
        "  double magnetic_field_covariance[9]; } sensor_msgs__msg__MagneticField;\n")
    for hdr in ("imu_interface.h", "mag_interface.h"):
        src = os.path.join(ROOT, "firmware", "common", "lib", "imu", hdr)
        open(os.path.join(d, hdr), "w").write(open(src).read())
    open(os.path.join(d, "shim.cpp"), "w").write(SHIM)
    so = os.path.join(d, "libfw.so")
    res = subprocess.run(
        # -fno-gnu-unique: the header's shared pack lives in function-local statics
        # of inline functions, which GCC makes process-wide unique -- without it
        # this library's wheels load the other sim-wheel tests' pack.
        [cxx, "-shared", "-fPIC", "-O0", "-std=gnu++11", "-fno-gnu-unique", *defines, "-o", so,
         os.path.join(d, "shim.cpp"), "-I", str(d), "-I", ENC],
        capture_output=True, text=True)
    # A subject that will not compile is a failure, not a skip (see the gearbox test).
    assert res.returncode == 0, res.stderr[-2000:]
    lib = ctypes.CDLL(so)
    lib.enc_new.restype = ctypes.c_void_p
    lib.enc_feed.argtypes = [ctypes.c_void_p, ctypes.c_int]
    lib.enc_rpm.argtypes = [ctypes.c_void_p]
    lib.enc_rpm.restype = ctypes.c_float
    lib.tick.argtypes = [ctypes.c_ulong]
    return lib


@pytest.fixture(scope="module")
def sticky(tmp_path_factory):
    # A tracked robot: breakaway measured (median 140 of 1023); the running floor is an
    # estimate (Nav2's 8 % turn sat stuck), not yet measured by a down-ramp.
    return _build(tmp_path_factory, "sticky",
                  ["-DSIM_WHEEL_BREAKAWAY_DUTY=0.137", "-DSIM_WHEEL_STALL_DUTY=0.08"])


@pytest.fixture(scope="module")
def plain(tmp_path_factory):
    return _build(tmp_path_factory, "plain", [])


def _hold(lib, enc, pwm, seconds):
    rpms = []
    for _ in range(int(seconds * 1e6 / STEP_US)):
        lib.tick(STEP_US)
        lib.enc_feed(enc, pwm)
        rpms.append(lib.enc_rpm(enc))
    return rpms


def test_default_model_is_unchanged(plain):
    # breakaway defaults to the stall duty: 8 % from rest moves, as it always did
    enc = plain.enc_new(450)
    assert max(_hold(plain, enc, 82, 2.0)) > 5.0


def test_a_stopped_wheel_needs_the_breakaway(sticky):
    enc = sticky.enc_new(450)
    assert max(_hold(sticky, enc, 82, 2.0)) == 0.0      # 8 %: Nav2's slow turn
    assert max(_hold(sticky, enc, 150, 2.0)) > 5.0      # 14.7 %: past the breakaway


def test_a_turning_wheel_keeps_going_below_the_breakaway(sticky):
    enc = sticky.enc_new(450)
    _hold(sticky, enc, 150, 2.0)
    running = _hold(sticky, enc, 110, 2.0)
    assert min(running[-10:]) > 2.0, "10.8 % keeps a turning wheel turning"
    # ...until it drops under the running floor and stops; then 10.8 % is not enough again
    _hold(sticky, enc, 20, 2.0)
    assert max(_hold(sticky, enc, 110, 2.0)) == 0.0


def test_a_slow_request_from_rest_sticks_then_jumps(sticky):
    # The firmware's PID, as configured on that robot (kp 0.961, ki 0.48 per tick),
    # asked for 3 rpm from rest: nothing moves while it winds up to the breakaway,
    # then the wheel jumps well past the request. That delay and that jump are what
    # a slow Nav2 heading turn meets on every start; the plain model had neither.
    enc = sticky.enc_new(450)
    integral, first_move, peak = 0.0, None, 0.0
    for i in range(int(5.0 * 1e6 / STEP_US)):
        sticky.tick(STEP_US)
        rpm = sticky.enc_rpm(enc)
        err = 3.0 - rpm
        integral = max(-2131.0, min(2131.0, integral + err))
        pwm = int(max(-1023, min(1023, 0.961 * err + 0.48 * integral)))
        sticky.enc_feed(enc, pwm)
        if first_move is None and abs(rpm) > 0.5:
            first_move = i * STEP_US / 1e6
        peak = max(peak, rpm)
    assert first_move is not None and first_move > 1.0, f"moved after {first_move} s"
    assert peak > 2 * 3.0, f"peak {peak:.1f} rpm for a 3 rpm request"


def test_the_python_model_matches(sticky):
    # sim_base_node (the board-less stack) and drivetrain_report run
    # drivetrain_report._Wheel: the same breakaway, from the same config keys.
    import sys
    sys.path.insert(0, os.path.join(ROOT, "scripts"))
    import drivetrain_report as dr
    params = {"kinematics": {"base_type": "2wd", "max_rpm": 100, "wheel_diameter": 0.1,
                             "lr_wheels_distance": 0.2, "pwm_bits": 10},
              "base_controller": {"simulation": {"breakaway_duty": 0.137, "stall_duty": 0.08}}}
    d = dr.drivetrain(params)
    assert (d["stall_duty"], d["breakaway_duty"]) == (0.08, 0.137)
    w = dr._Wheel(d, dr._Pack(d), 0)
    for duty, moves in ((82 / 1023, False), (150 / 1023, True), (110 / 1023, True)):
        w.duty = duty
        for _ in range(100):
            w.step(STEP_US / 1e6, True)
        assert (abs(w.rpm) > 0.5) == moves, (duty, w.rpm)
    # ...and the PID's compensation: 84 of 1023 asked forward becomes ~ 130 + 84 * 0.87
    assert abs(dr.pid_deadzone(84.0, 3.0, 130.0, 1023.0) - (130 + 84 * 893 / 1023)) < 1e-9
    assert dr.pid_deadzone(-84.0, 3.0, 130.0, 1023.0) == -84.0     # braking passes through
    assert dr.pid_deadzone(84.0, 0.0, 130.0, 1023.0) == 84.0       # a stop passes through
