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


def test_one_big_endian_burst_from_temp_through_gz():
    b = _body()
    assert "REG_TEMP_DATA1        = 0x09" in b and "BURST_LEN = 14" in b
    assert "be16" in b and "le16" not in b
    gyro = b[b.index("readGyroscope() override"):]
    assert "sample();" in gyro[:gyro.index("}")]


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
    assert "fired %lu times in the first 5 s" in m
    h = _read(os.path.join(FW, "common", "lib", "imu", "imu_interface.h"))
    assert "volatile uint32_t int_edges_" in h and "uint32_t intEdges() const" in h
    c = _read(os.path.join(FW, "common", "lib", "imu", "imu_interface.cpp"))
    assert "instance_->int_edges_++;" in c
