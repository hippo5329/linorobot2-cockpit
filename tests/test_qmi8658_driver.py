"""The QMI8658 driver is the compact in-header one, not the QST reference copy.

These pin the properties that the rewrite exists for. A future edit that
brings back a second bus transaction per sample, a boot-time on-chip
calibration, or a getData() override that the base pointer never reaches
fails here before it reaches a board.
"""
import os
import re

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
IMU_DIR = os.path.join(ROOT, "firmware", "common", "lib", "imu")
DEFAULT_IMU = os.path.join(IMU_DIR, "default_imu.h")


def _class_body(name):
    with open(DEFAULT_IMU) as fh:
        src = fh.read()
    start = src.index(f"class {name} : public IMUInterface")
    end = src.index("\n};", start)
    return src[start:end]


def test_the_qst_reference_driver_is_gone():
    for stale in ("QMI8658.h", "QMI8658.cpp", "QMI8658reg.h"):
        assert not os.path.exists(os.path.join(IMU_DIR, stale)), stale
    with open(DEFAULT_IMU) as fh:
        assert '#include "QMI8658.h"' not in fh.read()


def test_one_burst_per_sample_covers_status_through_gz():
    body = _class_body("QMI8658IMU")
    # 0x2D (STATUSINT) .. 0x40 (GZ_H) inclusive is 20 bytes.
    assert "REG_STATUSINT = 0x2D" in body
    assert "BURST_FIRST = REG_STATUSINT" in body
    assert "BURST_LEN   = 20" in body
    # readGyroscope() is the transaction, readAccelerometer() is the cache --
    # the order IMUInterface::getData() calls them in.
    gyro = body[body.index("readGyroscope() override"):]
    gyro = gyro[:gyro.index("}")]
    assert "sample();" in gyro
    accel = body[body.index("readAccelerometer() override"):]
    accel = accel[:accel.index("}")]
    assert "sample()" not in accel and "Wire" not in accel


def test_the_driver_does_not_override_getdata_or_touch_the_bus_clock():
    body = _class_body("QMI8658IMU")
    assert "Imu getData(" not in body, "IMUInterface::getData() is not virtual; an override is dead code"
    assert "setClock" not in body, "board_init owns the bus clock (env i2c_clock)"


def test_data_ready_enables_both_int_pins_and_sync_sample():
    body = _class_body("QMI8658IMU")
    fn = body[body.index("enableDataReadyInterrupt() override"):]
    fn = fn[:fn.index("return")]
    assert "CTRL1_INT1_EN | CTRL1_INT2_EN" in fn
    assert "CTRL7_SYNC" in fn
    assert "DRDY_DIS" in fn  # the comment says why INT2 stays live


def test_int_pins_are_high_impedance_until_asked():
    body = _class_body("QMI8658IMU")
    start = body[body.index("startSensor() override"):]
    start = start[:start.index("readGyroscope()")]
    assert "ctrl1_ = CTRL1_ADDR_AI | CTRL1_BE;" in start
    assert "INT1_EN" not in start and "INT2_EN" not in start


def test_no_on_chip_calibration_at_boot():
    body = _class_body("QMI8658IMU")
    assert "0xA2" not in body and "On_Demand_Cali" not in body and "delay(2200)" not in body
    # The soft reset is the only long wait, and it is short.
    waits = [int(x) for x in re.findall(r"delay\((\d+)\)", body)]
    assert waits and max(waits) <= 100, waits


def test_both_addresses_are_tried_and_a_nack_cannot_look_like_who_am_i():
    body = _class_body("QMI8658IMU")
    assert "{ 0x6B, 0x6A }" in body
    assert "readReg(REG_WHO_AM_I) == 0x05" in body
    read = body[body.index("uint8_t readReg(uint8_t reg)"):]
    read = read[:read.index("bool readBlock")]
    assert read.count("return 0xFF;") == 2, "a NACK must read as 0xFF, never 0x05"


def test_scales_match_the_configured_ranges():
    body = _class_body("QMI8658IMU")
    assert "CTRL2_8G_224HZ      = 0x25" in body and "ACCEL_LSB_PER_G   = 4096.0f" in body
    assert "CTRL3_1024DPS_224HZ = 0x65" in body and "GYRO_LSB_PER_DPS  = 32.0f" in body
    assert "g_to_accel_" in body and "DEG_TO_RAD_F" in body
