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
// The board config supplies these; the stub must follow the header. It did not,
// and `MOTOR_OPERATING_VOLTAGE` -- added with the voltage derating -- made
// sim_wheel.h stop COMPILING here. Every case in this file then reported
// "skipped", so the simulated motor had no local coverage at all while the model
// was being changed. See the fixture: a subject that will not build is a
// failure, not a skip.
#define MOTOR_OPERATING_VOLTAGE 12.0
#define MOTOR_POWER_MAX_VOLTAGE 12.0
#define PWM_BITS 10
#define PWM_MAX (pow(2, PWM_BITS) - 1)
struct StubSerial { int printf(const char *, ...) { return 0; } void println(const char *) {} };
static StubSerial Serial;
"""

SHIM = """
#include "sim_wheel.h"
extern "C" {
void *enc_new(int cpr) { return (void *)new SimEncoder(-1, -1, cpr); }
void enc_feed(void *e, int pwm) { ((SimEncoder *)e)->feed(pwm); }
float enc_rpm(void *e) { return ((SimEncoder *)e)->getRPM(); }
void tick(unsigned long us) { __advance(us); }
int enc_read(void *e) { return ((SimEncoder *)e)->read(); }
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
    # SimEncoder and has no reason to know about them), so the stub has to
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
        # FAIL, not skip. A missing compiler is the environment's business and
        # skips above; a subject that will not COMPILE is the finding, and
        # reporting it as "skipped" is how six cases in this file sat out every
        # run from the moment MOTOR_OPERATING_VOLTAGE entered the header. The
        # simulated motor then had no local coverage at all, through a stretch in
        # which the model was repeatedly changed -- and the defect that was
        # waiting in it sent a robot 30 m out of a 6 m room. Same lesson as
        # backend-tests-skip-without-fastapi: a green run that skipped the
        # subject is not a green run.
        pytest.fail(f"sim_wheel.h does not build against the stub: {res.stderr[:1500]}")
    l = ctypes.CDLL(so)
    l.enc_new.restype = ctypes.c_void_p
    l.enc_new.argtypes = [ctypes.c_int]
    l.enc_feed.argtypes = [ctypes.c_void_p, ctypes.c_int]
    l.enc_rpm.restype = ctypes.c_float
    l.enc_rpm.argtypes = [ctypes.c_void_p]
    l.tick.argtypes = [ctypes.c_ulong]
    l.enc_read.restype = ctypes.c_int
    l.enc_read.argtypes = [ctypes.c_void_p]
    return l


PWM_MAX = 1023


@pytest.fixture(scope="module")
def wheels(lib):
    """The board's four wheels, created ONCE.

    SimEncoder takes its pack slot from a process-global counter, so only the
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
    per value -- the same argument as sim_mass beside them."""
    src = open(os.path.join(ROOT, "firmware", "common", "lib", "encoder",
                            "sim_wheel.h"), encoding="utf-8").read()
    for fn, key in (("simGearEfficiency", "sim_gear_eff"),
                    ("simCoulombRpm", "sim_coulomb"),
                    ("simBattSag", "sim_sag")):
        assert f'envFloat("{key}"' in src, f"{key} is not read from the env"
        assert f"static inline float {fn}()" in src, f"{fn} is gone"
        # and the running model must go through the accessor, not the macro.
        # Scoped to the CLASS, not to after integrate(): simBattSag() is used by
        # busScale(), which integrate() calls but which is defined above it -- an
        # "after integrate()" slice cannot see it and failed on the one accessor
        # that was wired correctly.
        cls = src[src.index("class SimEncoder"):]
        assert f"{fn}()" in cls, f"the model still uses the compile-time constant, not {fn}()"


def test_the_env_values_are_clamped_to_physical_ranges():
    """A gearbox returning more than it is given, drag that accelerates, or a
    pack that gains voltage under load are all nonsense -- and -1 is the cache's
    unset marker, so an unclamped negative would be read as "not yet loaded" on
    every call."""
    src = open(os.path.join(ROOT, "firmware", "common", "lib", "encoder",
                            "sim_wheel.h"), encoding="utf-8").read()
    eff = src[src.index("static inline float simGearEfficiency()"):]
    eff = eff[:eff.index("\n}")]
    assert "> 1.0f) eff = 1.0f" in eff, "gear efficiency is not capped at 1"
    assert "< 0.0f) eff = 0.0f" in eff
    for fn in ("simCoulombRpm", "simBattSag"):
        blk = src[src.index(f"static inline float {fn}()"):]
        blk = blk[:blk.index("\n}")]
        assert "< 0.0f" in blk, f"{fn} accepts a negative value"


def test_the_config_keys_reach_the_env():
    """simulation.gear_efficiency -> sim_gear_eff, and the other two."""
    import sys
    sys.path.insert(0, os.path.join(ROOT, "scripts"))
    import mcu_env
    src = open(os.path.join(ROOT, "scripts", "mcu_env.py"), encoding="utf-8").read()
    for env_key, cfg_key in (("sim_gear_eff", "gear_efficiency"),
                             ("sim_coulomb", "gear_drag_rpm"),
                             ("sim_sag", "battery_sag")):
        assert f'"{env_key}": "{cfg_key}"' in src, f"{cfg_key} is not mapped to {env_key}"
        assert f'("{env_key}", float)' in src, f"{env_key} is not cast as a float"


# --- what a stalled loop does to the model ----------------------------------
# `integrate()` is an explicit Euler step of d(rpm)/dt = (no_load - rpm)/tau.
# tau is SIM_WHEEL_TAU_MS = 150 ms at the reference mass, so the step is free of
# overshoot only while dt < tau and stable only while dt < 2*tau. The rollover
# guard admits dt up to 1 SECOND -- a gain of dt/tau = 6.7 -- and the wheel speed
# is never bounded by the motor's own no-load speed, so one long loop interval
# takes the simulated wheel to a speed the motor cannot reach and the encoder
# integrates it faithfully.
#
# Why it matters beyond tidiness: RAN AWAY verdicts are an RP2-only event on this
# bench -- 21 in 286 RP2 leg attempts against 0 in 434 ESP32 ones -- and one of
# them reported 30.8 m from the goal after 7 s, which at the 0.25 m/s cap is
# distance that cannot be driven in the time. That is an integration jump.
#
# The battery-sag integrator in this same header already clamps its own gain to
# 1.0. Only the wheel was left unbounded.

def _peak_and_ticks(lib, step_us, secs=2.0, pwm=PWM_MAX, cpr=1000):
    """Hold full duty for `secs` of MODEL time, sampled in `step_us` slices."""
    ws = [lib.enc_new(cpr) for _ in range(4)]
    peak = 0.0
    for _ in range(int(secs * 1e6 / step_us)):
        lib.tick(step_us)
        for w in ws:
            lib.enc_feed(w, pwm)
        peak = max(peak, abs(lib.enc_rpm(ws[0])))
    return peak, lib.enc_read(ws[0])


def test_a_stalled_loop_cannot_take_the_wheel_past_its_own_motor(lib):
    """400 ms between calls is a loop hiccup, not a different motor."""
    peak, _ = _peak_and_ticks(lib, 400_000)
    assert peak <= 140 * 1.10, (
        f"a 400 ms loop interval drove the simulated wheel to {peak:.0f} rpm, "
        "past MOTOR_MAX_RPM=140 -- the encoder reports it as real motion")


def test_a_stalled_loop_never_reports_MORE_distance_than_it_travelled(lib):
    """The asymmetry is the point, so the bound is one-sided.

    Before the slicing, two seconds of full duty read 3830 ticks at 20 ms steps
    and 8872 at 900 ms -- 2.3x the distance, which odometry takes as ground
    truth and which is how a robot ends up 30 m outside a 6 m room. After it,
    the coarse figure is at or below the fine one.

    It is not exactly equal, and the residual is worth naming rather than
    hiding behind a symmetric tolerance: the pack sag and the current limiter
    are evaluated ONCE per call, from the speed at entry, so a long call spends
    its whole interval at the brown-out the first instant implied and
    UNDER-reports (3059 against 3828 at 400 ms). Under-reporting during a stall
    is the conservative direction -- it cannot invent motion -- so it is left
    as it is rather than risk changing how the four wheels share the pack.
    """
    _, fine = _peak_and_ticks(lib, 20_000)
    _, coarse = _peak_and_ticks(lib, 400_000)
    assert fine > 0
    assert coarse <= fine * 1.05, (
        f"a stalled loop reported {coarse} ticks where 50 Hz reported {fine} -- "
        "distance the robot did not travel")
    assert coarse > fine * 0.5, (
        f"{coarse} against {fine}: the model has stopped advancing, not just damped")


def test_the_model_is_already_resolved_at_the_normal_loop_rate(lib):
    """The guard above must not be bought by changing what the model does when
    nothing is wrong: halving an ordinary 20 ms step changes almost nothing."""
    _, at20 = _peak_and_ticks(lib, 20_000)
    _, at10 = _peak_and_ticks(lib, 10_000)
    assert abs(at20 - at10) / at10 < 0.05, f"{at10} vs {at20} ticks"


def test_a_subject_that_will_not_build_fails_rather_than_skipping():
    """The guard on the guard. Written as a source check because the alternative
    is breaking the stub on purpose inside a live fixture."""
    src = open(os.path.abspath(__file__), encoding="utf-8").read()
    build = src[src.index("res = subprocess.run("):]
    build = build[:build.index("l = ctypes.CDLL")]
    assert "pytest.fail(" in build, \
        "a header that does not compile is being reported as a skip again"
    assert "pytest.skip" not in build, "the build failure branch skips"
