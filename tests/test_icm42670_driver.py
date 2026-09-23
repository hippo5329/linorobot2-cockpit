"""The ICM-42670-P driver: the part the Yahboom YB-EET01 V2.0 actually carries."""
import os
import re

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FW = os.path.join(ROOT, "firmware")


def _read(p):
    with open(p, encoding="utf-8") as fh:
        return fh.read()


def _body():
    src = _read(os.path.join(FW, "common", "lib", "imu", "default_imu.h"))
    start = src.index("class ICM42670IMU : public IMUInterface")
    return src[start:src.index("\n};", start)]


def test_identity_and_addresses():
    b = _body()
    assert "REG_WHO_AM_I          = 0x75" in b and "== 0x67" in b
    assert "{ 0x68, 0x69 }" in b


def test_one_burst_from_temp_through_gz_in_the_chips_declared_byte_order():
    b = _body()
    assert "REG_TEMP_DATA1        = 0x09" in b and "BURST_LEN = 14" in b
    # TDK's driver reads INTF_CONFIG0.SENSOR_DATA_ENDIAN rather than assuming
    # the reset default; so does this one, and one decoder honours it.
    assert "REG_INTF_CONFIG0      = 0x35" in b and "DATA_BIG_ENDIAN       = 0x10" in b
    assert "big_endian_ = (readReg(REG_INTF_CONFIG0) & DATA_BIG_ENDIAN) != 0" in b
    assert b.count("rd16(&b[") == 7 and "be16" not in b and "le16" not in b
    gyro = b[b.index("readGyroscope() override"):]
    assert "sample();" in gyro[:gyro.index("}")]


def test_who_am_i_is_retried_because_the_first_transaction_after_the_scan_was_empty():
    b = _body()
    fn = b[b.index("bool findChip()"):]
    fn = fn[:fn.index("\n        }\n")]
    assert "WHO_TRIES" in fn and "delay(1);" in fn
    assert "WHO_TRIES             = 3" in b
    assert "in 3 tries" in b


def test_init_follows_tdks_order_pure_i2c_reset_pure_i2c_reset_done():
    b = _body()
    fn = b[b.index("bool startSensor() override"):]
    order = [fn.index(k) for k in ("findChip()", "pureI2C();", "REG_SIGNAL_PATH_RESET, SOFT_RESET",
                                   "silent after soft reset", "RESET_DONE", "REG_INTF_CONFIG0",
                                   "REG_PWR_MGMT0, PWR_LN_BOTH")]
    assert order == sorted(order)
    assert fn.count("pureI2C();") == 2, "once before the reset, once after it restores the defaults"
    pure = b[b.index("void pureI2C()"):]
    assert "REG_BLK_SEL_W, 0" in pure and "REG_BLK_SEL_R, 0" in pure and "~I3C_EN_BITS" in pure
    assert "I3C_EN_BITS           = 0x0C" in b and "REG_INTF_CONFIG1      = 0x36" in b
    assert "REG_INT_STATUS        = 0x3A" in b and "RESET_DONE            = 0x10" in b
    assert "Wire.begin();" not in b, "initBoard() owns the bus"


def test_the_bus_scan_stops_at_the_last_assignable_address():
    probe = _read(os.path.join(FW, "common", "lib", "i2c_probe", "i2c_probe.cpp"))
    assert "for (uint8_t addr = 0x08; addr <= 0x77; addr++)" in probe
    assert "0x7E" in probe and "I3C broadcast" in probe


def test_scales_match_the_ranges():
    b = _body()
    assert "GYRO_1000DPS_200HZ    = 0x28" in b and "GYRO_LSB_PER_DPS = 32.768f" in b
    assert "ACCEL_8G_200HZ        = 0x28" in b and "ACCEL_LSB_PER_G  = 4096.0f" in b


def test_registered_everywhere_a_driver_must_be():
    assert '{"icm42670", makeIMU<ICM42670IMU>}' in _read(os.path.join(FW, "common", "lib", "sensor_factory", "sensor_factory.cpp"))
    probe = _read(os.path.join(FW, "common", "lib", "i2c_probe", "i2c_probe.cpp"))
    assert 'if (who == 0x67) {' in probe and '"ICM42670", "icm42670"' in probe
    assert '"ICM42670"' in _read(os.path.join(ROOT, "scripts", "mcu_env.py"))
    assert 'value="ICM42670"' in _read(os.path.join(ROOT, "web", "frontend", "index.html"))


def test_the_bootsel_touch_is_not_claimed_when_the_tty_is_gone():
    """flash_mcu must not report a touch it never sent.

    Inside a container the USB reset that precedes the touch takes the tty away
    (a container runtime attaches a device node once). pyserial then failed to open it,
    the stty fallback failed silently on the same missing node, and the pulse
    still returned True -- so the flasher printed "the board answered the
    1200-baud touch and did NOT reach BOOTSEL" about a board that had heard
    nothing. From the host the same board reached BOOTSEL in 0.3 s.
    """
    src = _read(os.path.join(ROOT, "scripts", "flash_mcu.py"))
    reset = src[src.index("def usb_reset_target"):src.index("def pulse_1200_baud")]
    assert "did not come back openable within" in reset and reset.rstrip().endswith("return False"), \
        "usb_reset_target must say when the tty never came back, not return True"
    # And "came back" means OPENS: 0.1 s after the reset the old node still
    # exists but opens with ENXIO, then it vanishes, then the new one appears.
    assert "os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)" in reset
    assert "gone = True" in reset, "wait for the old node to go before trusting a new one"
    pulse = src[src.index("def pulse_1200_baud"):src.index("# USB interface classes")]
    assert "is gone after the usb reset; cannot send the 1200-baud touch" in pulse
    assert "touched = (r.returncode == 0)" in pulse, "a failed stty is not a touch"
    assert "s.dtr = False" in pulse, "drop DTR explicitly; the core wants 1200 baud AND DTR low"
