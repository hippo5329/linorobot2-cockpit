"""The BNO085's rates are published in the units the chip reports.

Measured on the MC200 (a BNO085 on an Arduino UNO Q), 2026-10-12, by speed_check
against the LiDAR: /imu/data angular_velocity.z was 0.014x the true rate --
0.006 rad/s at a real 0.58 rad/s spin, 0.019 at 1.13. The driver multiplied the
gyro by DEG_TO_RAD, but the SparkFun library's getGyroX() is
qToFloat(rawGyroX, gyro_Q1 = 9): the SH-2 calibrated gyroscope report, which is
ALREADY rad/s. 1/57.3 = 0.0175.

The accelerometer is the SH-2 calibrated accelerometer report, Q8 m/s^2,
returned unscaled by getAccelX(): it must reach the message unscaled too.
"""
import os
import re

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
IMU_H = os.path.join(ROOT, "firmware", "common", "lib", "imu", "default_imu.h")


def _bno085_block():
    with open(IMU_H, encoding="utf-8") as fh:
        src = fh.read()
    start = src.index("class BNO085IMU")
    nxt = src.find("\nclass ", start + 10)
    return src[start:nxt if nxt != -1 else len(src)]


def _method(blk, name):
    body = blk[blk.index(f"{name}() override"):]
    return body[:body.index("\n        }\n")]


def test_the_gyro_is_not_converted_from_degrees():
    body = _method(_bno085_block(), "readGyroscope")
    code = "\n".join(l.split("//")[0] for l in body.splitlines())   # comments may name it
    assert "DEG_TO_RAD" not in code, "the BNO085 gyro is rad/s already; DEG_TO_RAD shrinks it 57.3x"
    for axis in "XYZ":
        assert re.search(rf"gyro_\.{axis.lower()}\s*=\s*\w+\.getGyro{axis}\(\);", code), axis


def test_the_accelerometer_is_not_rescaled():
    body = _method(_bno085_block(), "readAccelerometer")
    for axis in "XYZ":
        assert re.search(rf"accel_\.{axis.lower()}\s*=\s*\w+\.getAccel{axis}\(\);", body), axis
