"""The LSM6DSOX drives INT1, which is the pin the bench board brings out.

Fitted to the z13 Pico 2 on 2026-09-23, same wiring as the parts before it:
I2C0 SDA GP0 / SCL GP1, and INT1 to GPIO 2 (user).

LSM6DSOXIMU inherited enableDataReadyInterrupt() -> false, so with the line
wired the firmware attached an ISR, waited a second for an edge the chip was
never told to produce, and fell back to polling. Four drivers were in that
state (GY85, MPU9250, LSM6DSOX, BNO085); this is the second of them done.

The part is 6-axis: accelerometer and gyroscope, no magnetometer. A run on this
board publishes no /imu/mag unless a separate magnetometer is on the bus, and
that is correct rather than a fault.
"""
import os
import re

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
IMU_H = os.path.join(ROOT, "firmware", "common", "lib", "imu", "default_imu.h")


def _cls():
    src = open(IMU_H, encoding="utf-8").read()
    start = src.index("class LSM6DSOXIMU")
    return src[start:src.index("\nclass ", start + 10)]


def _fn():
    cls = _cls()
    fn = cls[cls.index("bool enableDataReadyInterrupt"):]
    return fn[:fn.index("\n        }")]


def test_the_driver_can_ask_the_chip_for_data_ready():
    assert "enableDataReadyInterrupt" in _cls(), "LSM6DSOX still cannot drive its INT pin"


def test_it_routes_to_int1_not_int2():
    """INT1 is the pin that is wired. INT2 would need CTRL4_C.INT2_on_INT1 to
    reach the same wire, and routing there gives a line that never fires."""
    fn = _fn()
    assert "REG_INT1_CTRL" in fn
    assert "INT2" not in fn.replace("INT2_on_INT1", ""), "the driver touches INT2"


def test_the_data_ready_is_pulsed_not_level():
    """DRDY_PULSED (COUNTER_BDR_REG1 bit 7).

    Level mode holds the line asserted until the data is read, and this driver
    reads gyro and accel in SEPARATE transactions without clearing the
    condition between them -- so there is one rising edge and then nothing.
    getData() reads that as a line that fired once and died and the staleness
    ceiling turns the feature back into polling, silently. The ICM-20948 had
    the identical trap in INT1_LATCH_INT_EN.
    """
    fn = _fn()
    assert "REG_COUNTER_BDR1" in fn
    assert "0x80" in fn, "DRDY_PULSED is never set"


def test_the_pin_mode_write_preserves_the_configuration_bits():
    """CTRL3_C carries BDU and IF_INC, both set by startSensor(). Clearing
    IF_INC would break every multi-byte read the driver makes, so the write has
    to be read-modify-write, not a constant."""
    fn = _fn()
    assert "readReg(REG_CTRL3_C)" in fn, "CTRL3_C is written without reading it"
    assert "~0x30" in fn, "H_LACTIVE and PP_OD are not both forced"


def test_the_enable_is_read_back():
    """An I2C write that never landed looks exactly like a chip that will not
    drive the line, and only one of those is worth checking the wiring for."""
    fn = _fn()
    assert re.search(r"return \(readReg\(REG_INT1_CTRL\)", fn), "the enable is assumed, not verified"


def test_every_imu_that_claims_an_interrupt_pin_can_configure_one():
    """The inventory, so the gap is visible rather than discovered on a bench.

    A driver without the override attaches an ISR and polls a second later.
    That is honest but only if someone knows it before wiring a pin.
    """
    src = open(IMU_H, encoding="utf-8").read()
    classes = re.findall(r"class (\w+IMU)\s*:\s*public IMUInterface", src)
    with_drdy = set()
    for name in classes:
        start = src.index(f"class {name}")
        nxt = src.find("\nclass ", start + 10)
        body = src[start:nxt if nxt != -1 else len(src)]
        if "enableDataReadyInterrupt" in body:
            with_drdy.add(name)
    # Fake has no chip to ask; the rest are real parts.
    assert "LSM6DSOXIMU" in with_drdy
    assert {"MPU6050IMU", "QMI8658IMU", "ICM42670IMU", "ICM20948IMU"} <= with_drdy
    missing = {c for c in classes if c not in with_drdy} - {"FakeIMU"}
    # Named, not asserted empty: closing these is per-part register work and
    # this test should record the state, not block on it.
    assert missing <= {"GY85IMU", "MPU9250IMU", "BNO085IMU"}, \
        f"an IMU lost its data-ready support: {missing}"
