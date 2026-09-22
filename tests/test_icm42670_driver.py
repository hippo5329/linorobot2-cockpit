"""The ICM-42670-P driver: the part the Yahboom YB-EET01 V2.0 actually carries."""
import os

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


def test_data_ready_goes_to_both_int_pins_as_a_push_pull_pulse():
    b = _body()
    fn = b[b.index("enableDataReadyInterrupt() override"):]
    fn = fn[:fn.index("return")]
    assert "REG_INT_CONFIG, INT_PP_HIGH_PULSE_BOTH" in fn
    assert "REG_INT_SOURCE0, UI_DRDY_EN" in fn and "REG_INT_SOURCE3, UI_DRDY_EN" in fn
    assert "INT_PP_HIGH_PULSE_BOTH = 0x1B" in b and "UI_DRDY_EN            = 0x08" in b


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


def test_the_interrupt_edge_count_is_reported_once():
    m = _read(os.path.join(FW, "src", "main.cpp"))
    assert "fired %lu times in %.1f s = %.1f Hz" in m, \
        "the edge report must print the window it measured, not a nominal one"
    # The window is elapsed time since the attach, not a constant: the publish
    # path this runs in only starts when the agent connects.
    assert "const uint32_t int_ms    = millis() - imu->intAttachedMs();" in m
    h = _read(os.path.join(FW, "common", "lib", "imu", "imu_interface.h"))
    assert "volatile uint32_t int_edges_" in h and "uint32_t intEdges() const" in h
    c = _read(os.path.join(FW, "common", "lib", "imu", "imu_interface.cpp"))
    assert "instance_->int_edges_++;" in c
