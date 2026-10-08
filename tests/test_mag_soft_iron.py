"""The magnetometer's soft iron reaches the firmware (2026-10-08).

User: "do 1,2,3 all" -- a slow turn showed the TS100's horizontal field squashed 1.19:1
(motors and pack): +-7 deg of heading twice per turn that a bias cannot remove.
"""
import os
import sys

ROOT = os.path.join(os.path.dirname(__file__), "..")
sys.path.insert(0, os.path.join(ROOT, "scripts"))
import mcu_env  # noqa: E402

BASE = {"base_controller": {"imu_tuning": {}}}


def _env(soft):
    cfg = {"base_controller": {"imu_tuning": {"mag_soft_iron": soft}}}
    return mcu_env.hardware_env(cfg)


def test_four_numbers_or_a_two_by_two_reach_the_env():
    assert _env([0.93, 0.05, 0.05, 1.11])["mag_soft"] == "0.93,0.05,0.05,1.11"
    assert _env([[0.93, 0.05], [0.05, 1.11]])["mag_soft"] == "0.93,0.05,0.05,1.11"


def test_the_identity_or_a_wrong_shape_writes_nothing():
    for bad in ([1, 0, 0, 1], [1.0, 0.0], [[1, 2, 3]], "x", None):
        assert "mag_soft" not in _env(bad), bad


def test_the_firmware_applies_it_after_the_bias():
    src = open(os.path.join(ROOT, "firmware", "src", "main.cpp")).read()
    i_bias = src.index("mag_msg->magnetic_field.z -= mag_bias[2];")
    i_soft = src.index('envFloatVec("mag_soft", s, 4)')
    i_fuse = src.index("---- fuse, now that gyro")
    assert i_bias < i_soft < i_fuse


def test_test_acc_forwards_the_lidar():
    src = open(os.path.join(ROOT, "firmware", "src", "tools", "test_acc.cpp")).read()
    assert "initLidar();" in src
