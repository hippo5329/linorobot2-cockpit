"""/imu/data is the only IMU topic, and the gate proves the chip from it.

There used to be two. The board published /imu/data_raw (specific force, gravity
in it) and imu_filter_madgwick turned it into /imu/data (gravity removed), and
the accelerometer-is-alive check -- |accel| within 5..30, because a dead chip
publishing zeros passes any rate check -- belonged to the raw topic alone. Asked
of the filtered one it was a coin toss: on 2026-09-23 the same MPU6050 read
|accel|=10.31 and passed on jazzy, 0.57 and failed on lyrical.

Since 2026-09-25 the board fuses its own orientation (ahrs.h) and publishes
/imu/data with gravity already subtracted; there is no raw topic. So the check
rebuilds the specific force by adding back exactly what the board took out --
AHRS::gravityFrom() applied to the quaternion in the same message -- and asks
the old question of that. Not of the residual: the residual is only small once
the filter has converged, and an upside-down part (the GenDrv's QMI8658 reads
z at -10.15) starts 180 degrees from the filter's level seed.

All of it holds with or without a magnetometer, which a real robot may not have.
"""
import math
import os
import sys
import types

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCRIPTS = os.path.join(REPO_ROOT, "scripts")
SRC = os.path.join(SCRIPTS, "verify_topics.py")


def _load_module():
    """Import verify_topics with the ROS 2 imports stubbed out."""
    stubs = {
        "rclpy": ["init", "shutdown", "spin_once", "ok"],
        "rclpy.node": ["Node"],
        "rclpy.qos": ["QoSProfile", "ReliabilityPolicy", "HistoryPolicy",
                      "DurabilityPolicy"],
        "sensor_msgs.msg": ["Imu", "LaserScan", "MagneticField", "FluidPressure",
                            "Temperature", "BatteryState"],
        "nav_msgs.msg": ["Odometry"],
        "sensor_msgs": [], "nav_msgs": [],
    }
    saved = {name: sys.modules.get(name) for name in stubs}
    for name, attrs in stubs.items():
        mod = types.ModuleType(name)
        for a in attrs:
            setattr(mod, a, type(a, (object,), {}))
        sys.modules[name] = mod
    sys.path.insert(0, SCRIPTS)
    try:
        sys.modules.pop("verify_topics", None)
        import verify_topics
        return verify_topics
    finally:
        for name, mod in saved.items():
            if mod is None:
                sys.modules.pop(name, None)
            else:
                sys.modules[name] = mod


MOD = _load_module()


class V:
    def __init__(self, x=0.0, y=0.0, z=0.0, w=0.0):
        self.x, self.y, self.z, self.w = x, y, z, w


class SimImu:
    def __init__(self, accel=(0.0, 0.0, -9.81), gyro=(0.0, 0.0, 0.0),
                 quat=(0.0, 0.0, 0.0, 1.0)):
        self.linear_acceleration = V(*accel)
        self.angular_velocity = V(*gyro)
        self.orientation = V(*quat)


def _read(p):
    with open(p, encoding="utf-8") as fh:
        return fh.read()


def _published(raw, quat):
    """What the board publishes: the chip's specific force minus the gravity
    its quaternion implies (main.cpp, `linear_acceleration.x -= ggx` etc.)."""
    g = MOD._gravity_from(V(*quat))
    return SimImu(accel=tuple(r - c for r, c in zip(raw, g)), quat=quat)


LEVEL = (0.0, 0.0, 0.0, 1.0)
INVERTED = (1.0, 0.0, 0.0, 0.0)          # rolled 180 degrees about x


def test_there_is_no_raw_topic_any_more():
    src = _read(SRC)
    assert "/imu/data_raw" not in src, "the gate still listens for a topic nothing publishes"
    assert "/imu/data_raw" not in MOD.RANGE_CHECKS
    assert MOD.RANGE_CHECKS["/imu/data"] is MOD._imu_range


def test_the_gravity_the_gate_adds_back_is_the_gravity_the_board_took_out():
    """Same formula and same constant as AHRS::gravityFrom(), or the rebuilt
    specific force is off by whatever they disagree on."""
    ahrs = _read(os.path.join(REPO_ROOT, "firmware", "common", "lib", "imu", "ahrs.h"))
    assert f"#define AHRS_GRAVITY {MOD.AHRS_GRAVITY}f" in ahrs
    for line in ("gx = g * 2.0f * (x * z - w * y);",
                 "gy = g * 2.0f * (w * x + y * z);",
                 "gz = g * (w * w - x * x - y * y + z * z);"):
        assert line in ahrs, f"ahrs.h changed its gravity: {line}"
    assert MOD._gravity_from(V(*LEVEL)) == (0.0, 0.0, MOD.AHRS_GRAVITY)
    gx, gy, gz = MOD._gravity_from(V(*INVERTED))
    assert (gx, gy) == (0.0, 0.0) and gz == -MOD.AHRS_GRAVITY


def test_a_live_level_chip_passes():
    ok, note = MOD._imu_range(_published((0.05, -0.02, 9.79), LEVEL))
    assert ok and "9.7" in note


def test_an_upside_down_chip_passes_before_the_filter_has_turned_over():
    """The GenDrv's recorded QMI8658 reading, published while the estimate is
    still at its level seed: residual ~2 g, and a healthy chip."""
    raw = (-0.92, 1.62, -10.15)
    unconverged = _published(raw, LEVEL)
    a = unconverged.linear_acceleration
    assert math.sqrt(a.x ** 2 + a.y ** 2 + a.z ** 2) > 19.0   # what a residual check would see
    assert MOD._imu_range(unconverged)[0]
    assert MOD._imu_range(_published(raw, INVERTED))[0]


def test_a_dead_accelerometer_fails_whatever_the_estimate():
    for quat in (LEVEL, INVERTED):
        ok, note = MOD._imu_range(_published((0.0, 0.0, 0.0), quat))
        assert not ok and "outside 5..30" in note


def test_a_filter_with_no_estimate_fails():
    ok, note = MOD._imu_range(SimImu(accel=(0.0, 0.0, 0.0), quat=(0.0, 0.0, 0.0, 0.0)))
    assert not ok and "norm" in note
    assert not MOD._imu_range(SimImu(accel=(float("nan"), 0.0, 0.0), quat=LEVEL))[0]


def test_imu_data_may_be_required():
    """Naming it in --require is what turns its value check on, so --require
    must not reject it as a topic the gate does not know."""
    assert 'known = set(verifier.optional_samples) | {"/imu/data"}' in _read(SRC)


def test_a_real_imu_requires_imu_data_with_or_without_a_magnetometer():
    """sensor_topics() is what the pipeline turns into --require."""
    sys.path.insert(0, SCRIPTS)
    import one_click_pipeline as ocp
    assert ocp.SENSOR_TOPICS["imu"] == ["/imu/data"]
    nine = ocp.sensor_topics({"sensors": {"imu": "ICM20948", "mag": "AK09916"}})
    assert "/imu/data" in nine and "/imu/mag" in nine
    six = ocp.sensor_topics({"sensors": {"imu": "MPU6050", "mag": "NONE"}})
    assert "/imu/data" in six and "/imu/mag" not in six
    # a synthesised IMU is not a chip to check gravity on
    assert "/imu/data" not in ocp.sensor_topics({"sensors": {"imu": "MPU6050", "use_sim_imu": True}})
    assert "/imu/data" not in ocp.sensor_topics({"sensors": {"imu": "NONE"}})
