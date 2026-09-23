"""An IMU message must carry a valid quaternion, even when nothing computes one.

sensor_msgs/Imu.orientation is zero-initialised, which is (0,0,0,0). That is not
"no rotation" -- it is not a rotation: the norm is 0, so any consumer that
normalises divides by zero, and RViz rejects the message.

It mattered little while this message was only /imu/data_raw, because madgwick
ignores the orientation of its input. It matters now: under the two-configuration
rule a robot with no magnetometer has the BASE publish /imu/data, and that is the
topic consumers read.

Measured on the z13 Pico 2 with an LSM6DSOX, 2026-09-23, over a bare micro-ROS
agent -- the run that was otherwise clean:

    PUBLISHERS imu/data      1
    FUSED_RATE               48.92
    FUSED_ACC_MEAN           9.920      <- agrees with test_sensors over serial
    FUSED_QNORM_MEAN         0.0000     <- this
    FUSED_DISTINCT           1

Identity (w = 1) is the honest value: the driver does not know the orientation,
and identity is what "no correction" looks like to a consumer. The EKF does not
fuse it either way -- with no magnetometer bringup.launch.py clears
imu0_config[5].
"""
import os
import re

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
IFACE = os.path.join(ROOT, "firmware", "common", "lib", "imu", "imu_interface.h")
DRIVERS = os.path.join(ROOT, "firmware", "common", "lib", "imu", "default_imu.h")


def _read(p):
    with open(p, encoding="utf-8") as fh:
        return fh.read()


def _init_body():
    s = _read(IFACE)
    start = s.index("bool init()")
    return s[start:s.index("\n        }", start)]


def test_the_message_starts_as_a_valid_identity_rotation():
    body = _init_body()
    assert "imu_msg_.orientation.w = 1.0;" in body, \
        "the quaternion is left at (0,0,0,0), which is not a rotation"
    for axis in "xyz":
        assert f"imu_msg_.orientation.{axis} = 0.0;" in body, axis


def test_it_is_set_before_the_sensor_starts():
    """So that a driver which fails to start still leaves a valid message behind,
    and one that computes orientation overwrites it rather than racing it."""
    body = _init_body()
    assert body.index("orientation.w = 1.0") < body.index("startSensor()")


def test_a_driver_with_real_fusion_still_overwrites_all_four():
    """The BNO085 has on-chip fusion; identity must not survive into its output."""
    s = _read(DRIVERS)
    blk = s[s.index("class BNO085IMU"):]
    for axis in ("x", "y", "z", "w"):
        assert re.search(rf"imu_msg_\.orientation\.{axis}\s*=\s*bno085_\.getQuat", blk), axis


def test_no_driver_leaves_a_zero_quaternion_on_purpose():
    """A driver that sets three components and forgets w would publish a norm-0
    quaternion again, which is the exact failure this file records."""
    s = _read(DRIVERS)
    sets_w = len(re.findall(r"imu_msg_\.orientation\.w\s*=", s))
    sets_x = len(re.findall(r"imu_msg_\.orientation\.x\s*=", s))
    assert sets_w == sets_x, "a driver writes the vector part without the scalar part"
