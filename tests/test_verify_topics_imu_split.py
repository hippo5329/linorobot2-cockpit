"""/imu/data and /imu/data_raw are two topics, and the gate must not confuse them.

Both used to be subscribed with the SAME callback, which filed every message it
got under "/imu/data". So the sample the range check judged was whichever of the
two arrived first, and the two do not say the same thing: bringup.launch.py runs
imu_filter_madgwick with remove_gravity_vector: True, so /imu/data carries
gravity-free acceleration (~0.5 m/s^2 on a still board) while /imu/data_raw
carries specific force (~9.81).

The check asked both for 5..30 m/s^2. On 2026-09-23 the same MPU6050 on the same
bench, ten minutes apart, read |accel|=10.31 and PASSED on jazzy and |accel|=0.57
and FAILED on lyrical -- a coin toss on the one gate that looks at a real chip.
The rate row was double-counted from the same mistake, and only read ~50 Hz
because madgwick answers within the callback's 5 ms de-duplication window.
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


class FakeImu:
    def __init__(self, accel=(0.0, 0.0, -9.81), gyro=(0.0, 0.0, 0.0),
                 quat=(0.0, 0.0, 0.0, 1.0)):
        self.linear_acceleration = V(*accel)
        self.angular_velocity = V(*gyro)
        self.orientation = V(*quat)


def _read(p):
    with open(p, encoding="utf-8") as fh:
        return fh.read()


def test_the_raw_topic_has_its_own_callback():
    src = _read(SRC)
    # one callback per topic: /imu/data_raw must not be routed into _imu_cb,
    # which is what fills the rate-gated "/imu/data" row.
    assert 'self.create_subscription(Imu, "/imu/data_raw", self._imu_cb' not in src
    assert 'self.create_subscription(Imu, "/imu/data_raw", self._imu_raw_cb' in src
    assert 'self.optional_samples["/imu/data_raw"] = msg' in src


def test_gravity_is_asked_of_the_raw_topic_and_not_the_filtered_one():
    assert MOD.RANGE_CHECKS["/imu/data_raw"] is MOD._imu_raw_range
    assert MOD.RANGE_CHECKS["/imu/data"] is MOD._imu_filtered_range

    still = FakeImu(accel=(0.0, 0.0, -9.81))
    ok, note = MOD._imu_raw_range(still)
    assert ok and "9.81" in note

    dead = FakeImu(accel=(0.0, 0.0, 0.0))
    ok, note = MOD._imu_raw_range(dead)
    assert not ok and "outside 5..30" in note


def test_a_gravity_free_filtered_sample_is_not_a_failure():
    """The exact lyrical reading that failed the gate: it is correct output."""
    filtered = FakeImu(accel=(-0.03, 0.08, -0.56),
                       quat=(0.999, 0.0, 0.0, -0.025))
    n = math.sqrt(sum(c * c for c in (0.999, 0.0, 0.0, -0.025)))
    filtered.orientation = V(0.999 / n, 0.0, 0.0, -0.025 / n)
    ok, _ = MOD._imu_filtered_range(filtered)
    assert ok
    # and the same sample would have failed the old, misapplied check
    assert not MOD._imu_raw_range(filtered)[0]


def test_the_filtered_row_still_catches_a_filter_with_no_estimate():
    ok, note = MOD._imu_filtered_range(FakeImu(quat=(0.0, 0.0, 0.0, 0.0)))
    assert not ok and "norm" in note
    nan = FakeImu(accel=(float("nan"), 0.0, 0.0))
    assert not MOD._imu_filtered_range(nan)[0]


def test_a_real_imu_makes_the_raw_topic_required():
    """sensor_topics() is what the pipeline turns into --require."""
    sys.path.insert(0, SCRIPTS)
    import one_click_pipeline as ocp
    assert ocp.SENSOR_TOPICS["imu"] == ["/imu/data_raw"]
    cfg = {"sensors": {"imu": "MPU6050", "mag": "AK09918"}}
    assert "/imu/data_raw" in ocp.sensor_topics(cfg)
    # a board whose IMU is synthesised is not a chip to check gravity on
    cfg_fake = {"sensors": {"imu": "MPU6050", "use_fake_imu": True}}
    assert "/imu/data_raw" not in ocp.sensor_topics(cfg_fake)
    cfg_none = {"sensors": {"imu": "NONE"}}
    assert "/imu/data_raw" not in ocp.sensor_topics(cfg_none)
