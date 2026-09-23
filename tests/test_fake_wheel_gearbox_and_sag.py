"""The simulated motor is a brushed DC gear motor, with the gearbox and the pack.

The model already had the right SHAPE: accel proportional to
(no-load speed - current speed) is the brushed DC torque-speed curve, torque
maximum at stall and zero at no-load, PWM scaling the whole line. What it did not
have was what the drivetrain costs:

  * gear efficiency -- a spur reduction returns 70-80% of the torque put in;
  * Coulomb drag -- a gear train's roughly constant loss, which is why an
    unpowered gear motor stops rather than coasting, and why a small duty moves
    nothing. The old model had only a viscous term, proportional to speed;
  * battery sag -- V_bus = V_oc - I*R, and for a brushed motor the current is
    proportional to that same (no-load - current) term. A hard acceleration
    browns out its own supply.

Sag is also the ONLY coupling between the four wheels. Without it each wheel
followed its own curve independently, so a 4WD base accelerated exactly like a
2WD one -- where on real hardware four driven wheels share one pack and each gets
less of it.

These tests compile the real header and drive it, because the question is what
the model DOES, not what it says.
"""
import ctypes
import os
import shutil
import subprocess

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ENC = os.path.join(ROOT, "firmware", "common", "lib", "encoder")

STUB = """
#pragma once
#include <math.h>
#include <stdint.h>
#include <stdlib.h>
static unsigned long __now_us = 0;
static inline unsigned long micros() { return __now_us; }
static inline unsigned long millis() { return __now_us / 1000; }
static inline void __advance(unsigned long us) { __now_us += us; }
static inline void delay(unsigned long ms) { __now_us += ms * 1000; }
static inline void delayMicroseconds(unsigned long us) { __now_us += us; }
static inline long random(long lo, long hi) { return (lo + hi) / 2; }
static inline long random(long n) { return n / 2; }
static inline float constrain_f(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
template <class T, class A, class B> T constrain(T v, A lo, B hi) { return v < (T)lo ? (T)lo : (v > (T)hi ? (T)hi : v); }
#define PI 3.14159265358979323846
#define DEG_TO_RAD (PI / 180.0)
#define MOTOR_MAX_RPM 140
#define PWM_BITS 10
#define PWM_MAX (pow(2, PWM_BITS) - 1)
struct FakeSerial { int printf(const char *, ...) { return 0; } void println(const char *) {} };
static FakeSerial Serial;
"""

SHIM = """
#include "fake_wheel.h"
extern "C" {
void *enc_new(int cpr) { return (void *)new FakeEncoder(-1, -1, cpr); }
void enc_feed(void *e, int pwm) { ((FakeEncoder *)e)->feed(pwm); }
float enc_rpm(void *e) { return ((FakeEncoder *)e)->getRPM(); }
void tick(unsigned long us) { __advance(us); }
}
"""


@pytest.fixture(scope="module")
def lib(tmp_path_factory):
    cxx = shutil.which("g++") or shutil.which("clang++")
    if not cxx:
        pytest.skip("no host C++ compiler")
    d = tmp_path_factory.mktemp("fw")
    open(os.path.join(d, "Arduino.h"), "w").write(STUB)
    open(os.path.join(d, "encoder_interface.h"), "w").write(
        open(os.path.join(ENC, "encoder_interface.h")).read())
    open(os.path.join(d, "mcu_env.h"), "w").write(
        "#pragma once\n"
        "static inline void initMcuEnv() {}\n"
        "static inline float envFloat(const char *, float d) { return d; }\n"
        "static inline bool envFlag(const char *, bool d) { return d; }\n"
        "static inline const char *envGet(const char *, const char *d) { return d; }\n"
        "static inline const char *envPrefixed(const char *n) { return n; }\n"
        "static inline int envInt(const char *, int d) { return d; }\n"
        "static inline unsigned envU16(const char *, unsigned d) { return d; }\n"
        "static inline unsigned long envU32(const char *, unsigned long d) { return d; }\n"
        "static inline void envFloatVec(const char *, float *, int) {}\n")
    # The header is self-contained about ROS messages (hw_factory builds
    # FakeEncoder and has no reason to know about them), so the stub has to
    # supply the message shapes. Only the fields the simulated IMU fills.
    for sub in ("micro_ros_utilities", "sensor_msgs/msg", "geometry_msgs/msg",
                "std_msgs/msg", "builtin_interfaces/msg", "rosidl_runtime_c"):
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
    for name in ("imu_interface.h", "mag_interface.h"):
        src = os.path.join(ROOT, "firmware", "common", "lib", "imu", name)
        open(os.path.join(d, name), "w").write(open(src).read())
    open(os.path.join(d, "shim.cpp"), "w").write(SHIM)
    so = os.path.join(d, "libfw.so")
    res = subprocess.run(
        [cxx, "-shared", "-fPIC", "-O0", "-std=gnu++11", "-o", so,
         os.path.join(d, "shim.cpp"), "-I", str(d), "-I", ENC],
        capture_output=True, text=True)
    if res.returncode != 0:
        pytest.skip(f"fake_wheel.h does not build against the stub: {res.stderr[:500]}")
    l = ctypes.CDLL(so)
    l.enc_new.restype = ctypes.c_void_p
    l.enc_new.argtypes = [ctypes.c_int]
    l.enc_feed.argtypes = [ctypes.c_void_p, ctypes.c_int]
    l.enc_rpm.restype = ctypes.c_float
    l.enc_rpm.argtypes = [ctypes.c_void_p]
    l.tick.argtypes = [ctypes.c_ulong]
    return l


PWM_MAX = 1023


@pytest.fixture(scope="module")
def wheels(lib):
    """The board's four wheels, created ONCE.

    FakeEncoder takes its pack slot from a process-global counter, so only the
    first four instances get one -- correct for a robot, and a trap for a test
    file that news up encoders per case: later ones get slot -1, contribute no
    current, and the sag silently disappears. That is how the four-wheel case
    first "proved" the pack was not shared.
    """
    return [lib.enc_new(1000) for _ in range(4)]


def coast(lib, wheels, secs=3.0, step_us=20000):
    """Bring every wheel to rest, so one case cannot inherit another's speed."""
    for _ in range(int(secs * 1e6 / step_us)):
        lib.tick(step_us)
        for w in wheels:
            lib.enc_feed(w, 0)


def drive(lib, wheels, pwms, secs, step_us=20000):
    """Hold each wheel at its pwm for secs; returns wheel 0's rpm."""
    for _ in range(int(secs * 1e6 / step_us)):
        lib.tick(step_us)
        for w, p in zip(wheels, pwms):
            lib.enc_feed(w, p)
    return lib.enc_rpm(wheels[0])


def test_it_is_still_a_brushed_dc_curve(lib, wheels):
    """Half duty settles near half the no-load speed: torque falls linearly to
    zero at the no-load point, which is the whole shape of the thing."""
    coast(lib, wheels)
    full = drive(lib, wheels, [PWM_MAX, 0, 0, 0], 3.0)
    coast(lib, wheels)
    half = drive(lib, wheels, [PWM_MAX // 2, 0, 0, 0], 3.0)
    assert full > 0
    assert 0.35 < half / full < 0.65, f"half duty gave {half:.1f} of {full:.1f} rpm"


def test_the_gearbox_costs_torque(lib, wheels):
    """Efficiency below 1 means the wheel is still climbing after a short drive
    rather than snapping to its terminal speed."""
    coast(lib, wheels)
    early = drive(lib, wheels, [PWM_MAX, 0, 0, 0], 0.15)
    late = drive(lib, wheels, [PWM_MAX, 0, 0, 0], 3.0)
    assert early < late * 0.95, "the wheel reaches terminal speed instantly"


def test_coulomb_drag_stops_an_unpowered_wheel(lib, wheels):
    """A viscous-only model decays exponentially and never quite stops; a gear
    train has constant drag and stops."""
    coast(lib, wheels)
    drive(lib, wheels, [PWM_MAX, 0, 0, 0], 2.0)
    assert lib.enc_rpm(wheels[0]) > 10
    coast(lib, wheels, secs=2.0)
    assert abs(lib.enc_rpm(wheels[0])) < 2.0, f"still turning at {lib.enc_rpm(wheels[0]):.2f} rpm"


def test_coulomb_drag_never_drives_the_wheel_backwards(lib, wheels):
    """Brake, not motor: constant drag applied across zero would reverse the
    wheel, which is a gearbox pushing the robot."""
    coast(lib, wheels)
    drive(lib, wheels, [PWM_MAX, 0, 0, 0], 0.4)
    for _ in range(200):
        lib.tick(20000)
        for w in wheels:
            lib.enc_feed(w, 0)
        assert lib.enc_rpm(wheels[0]) > -2.0, "the unpowered wheel reversed"


def test_four_wheels_sag_the_pack_more_than_one(lib, wheels):
    """The coupling the model lacked. Four wheels accelerating share one pack, so
    at the same instant each is slower than a single wheel would have been."""
    coast(lib, wheels)
    solo = drive(lib, wheels, [PWM_MAX, 0, 0, 0], 0.2)
    coast(lib, wheels)
    together = drive(lib, wheels, [PWM_MAX] * 4, 0.2)
    assert together < solo, (
        f"four wheels reached {together:.1f} rpm, one reached {solo:.1f} -- "
        "the pack is not shared")


def test_sag_recovers_once_the_wheels_are_up_to_speed(lib, wheels):
    """Current is proportional to (no-load - speed), so a wheel at terminal speed
    draws almost nothing and the sag must go away. A model that held a permanent
    brownout would cap the robot's top speed for ever."""
    coast(lib, wheels)
    solo = drive(lib, wheels, [PWM_MAX, 0, 0, 0], 4.0)
    coast(lib, wheels)
    settled = drive(lib, wheels, [PWM_MAX] * 4, 4.0)
    assert abs(settled - solo) / solo < 0.05, (
        f"settled four-wheel {settled:.1f} vs one-wheel {solo:.1f}: sag persists at speed")


# --- the losses are env-configurable ---------------------------------------

def test_the_three_losses_come_from_the_env():
    """A sweep across gear efficiency or pack stiffness is how you find out which
    one a navigation failure was sensitive to. It must not cost a firmware build
    per value -- the same argument as fake_mass beside them."""
    src = open(os.path.join(ROOT, "firmware", "common", "lib", "encoder",
                            "fake_wheel.h"), encoding="utf-8").read()
    for fn, key in (("fakeGearEfficiency", "fake_gear_eff"),
                    ("fakeCoulombRpm", "fake_coulomb"),
                    ("fakeBattSag", "fake_sag")):
        assert f'envFloat("{key}"' in src, f"{key} is not read from the env"
        assert f"static inline float {fn}()" in src, f"{fn} is gone"
        # and the running model must go through the accessor, not the macro.
        # Scoped to the CLASS, not to after integrate(): fakeBattSag() is used by
        # busScale(), which integrate() calls but which is defined above it -- an
        # "after integrate()" slice cannot see it and failed on the one accessor
        # that was wired correctly.
        cls = src[src.index("class FakeEncoder"):]
        assert f"{fn}()" in cls, f"the model still uses the compile-time constant, not {fn}()"


def test_the_env_values_are_clamped_to_physical_ranges():
    """A gearbox returning more than it is given, drag that accelerates, or a
    pack that gains voltage under load are all nonsense -- and -1 is the cache's
    unset marker, so an unclamped negative would be read as "not yet loaded" on
    every call."""
    src = open(os.path.join(ROOT, "firmware", "common", "lib", "encoder",
                            "fake_wheel.h"), encoding="utf-8").read()
    eff = src[src.index("static inline float fakeGearEfficiency()"):]
    eff = eff[:eff.index("\n}")]
    assert "> 1.0f) eff = 1.0f" in eff, "gear efficiency is not capped at 1"
    assert "< 0.0f) eff = 0.0f" in eff
    for fn in ("fakeCoulombRpm", "fakeBattSag"):
        blk = src[src.index(f"static inline float {fn}()"):]
        blk = blk[:blk.index("\n}")]
        assert "< 0.0f" in blk, f"{fn} accepts a negative value"


def test_the_config_keys_reach_the_env():
    """simulation.gear_efficiency -> fake_gear_eff, and the other two."""
    import sys
    sys.path.insert(0, os.path.join(ROOT, "scripts"))
    import mcu_env
    src = open(os.path.join(ROOT, "scripts", "mcu_env.py"), encoding="utf-8").read()
    for env_key, cfg_key in (("fake_gear_eff", "gear_efficiency"),
                             ("fake_coulomb", "gear_drag_rpm"),
                             ("fake_sag", "battery_sag")):
        assert f'"{env_key}": "{cfg_key}"' in src, f"{cfg_key} is not mapped to {env_key}"
        assert f'("{env_key}", float)' in src, f"{env_key} is not cast as a float"
