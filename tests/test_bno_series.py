"""The BNO series: one chip, two roles (firmware/common/lib/imu/bno_chips.h).

MC200 (an UNO Q with a BNO085 on Qwiic) needed its magnetometer calibrated, and
the firmware had never read a BNO085's field: the driver enabled the game
rotation vector, gyro and accelerometer only, and there was no BNO magnetometer
driver at all. Now one chip object per part serves the IMU and the magnetometer
roles. These tests compile bno_chips.h against a stub SparkFun BNO080 and a
stub I2C bus and drive it:

  * either start order runs begin() once, and each role only adds its reports;
  * with the field in use the chip stops being the orientation source, sends no
    game rotation vector, and its dynamic MAGNETOMETER calibration is switched
    off (accel and gyro left on) -- again after every chip reset;
  * without the field nothing changes: the chip fuses, no calibration command;
  * the BNO055 is put in AMG mode with SI units, from the datasheet's sequence.

Plus the contract around it: the factory rows, the probe's entries (a BNO08x is
an IMU only, so AUTO never switches its heading source; a BNO055 offers its
field like the ICM-20948), the covariances, and the gyro read in rad/s.
"""
import os
import re
import shutil
import subprocess
import sys
import tempfile

import pytest
import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
IMU_DIR = os.path.join(ROOT, "firmware", "common", "lib", "imu")
CHIPS = os.path.join(IMU_DIR, "bno_chips.h")
DEFAULT_IMU = os.path.join(IMU_DIR, "default_imu.h")
DEFAULT_MAG = os.path.join(IMU_DIR, "default_mag.h")
FACTORY = os.path.join(ROOT, "firmware", "common", "lib", "sensor_factory", "sensor_factory.cpp")
PROBE = os.path.join(ROOT, "firmware", "common", "lib", "i2c_probe", "i2c_probe.cpp")
sys.path.insert(0, os.path.join(ROOT, "scripts"))


def _read(p):
    with open(p, encoding="utf-8") as fh:
        return fh.read()


def _block(src, start):
    i = src.index(start)
    j = src.find("\nclass ", i + 10)
    return src[i:j if j != -1 else len(src)]


# --- the chip, compiled and driven --------------------------------------------

STUB_ARDUINO = r"""
#pragma once
#include <stdint.h>
#include <stdio.h>
#include <string.h>
typedef bool boolean;
static unsigned long g_ms = 0;
static inline unsigned long millis() { return g_ms; }
static inline void delay(unsigned long ms) { g_ms += ms; }
"""

# A register-file I2C bus: the BNO055 at 0x28, a BNO08x that only ACKs at 0x4B.
STUB_WIRE = r"""
#pragma once
#include <Arduino.h>
struct StubWire {
    uint8_t regs[256] = {0};
    uint8_t addr = 0, reg = 0, nwr = 0, rdptr = 0;
    bool have_reg = false;
    char log[2048] = {0};
    void beginTransmission(int a) { addr = (uint8_t)a; nwr = 0; have_reg = false; }
    void write(uint8_t v) {
        if (nwr++ == 0) { reg = v; rdptr = v; have_reg = true; return; }
        if (addr == 0x28) {
            regs[reg] = v;
            char e[24]; snprintf(e, sizeof e, "W%02X=%02X;", reg, v); strncat(log, e, sizeof log - strlen(log) - 1);
            reg++;
        }
    }
    int endTransmission(bool = true) { return (addr == 0x28 || addr == 0x4B) ? 0 : 2; }
    int requestFrom(int a, int n) { return (a == 0x28) ? n : 0; }
    int read() { return regs[rdptr++]; }
};
static StubWire Wire;
"""

STUB_SYSLOG = "#pragma once\n#define LOG_INFO 6\n#define LOG_ERR 3\nstatic inline void syslog(int, const char *, ...) {}\n"

# Records every call the chip makes on the library.
STUB_BNO080 = r"""
#pragma once
#include <Arduino.h>
#include <Wire.h>
#define SENSOR_REPORTID_ACCELEROMETER 0x01
#define SENSOR_REPORTID_GYROSCOPE 0x02
#define SENSOR_REPORTID_MAGNETIC_FIELD 0x03
#define SENSOR_REPORTID_GAME_ROTATION_VECTOR 0x08
#define COMMAND_ME_CALIBRATE 7
struct TwoWire {};
static char g_calls[4096];
static void callLog(const char *s) { strncat(g_calls, s, sizeof g_calls - strlen(g_calls) - 1); strncat(g_calls, ";", 2); }
class BNO080 {
public:
    uint8_t shtpData[32] = {0};
    bool reset_flag = false;
    uint16_t queue[8] = {0}; int qn = 0;
    template <typename W> bool begin(uint8_t a, W &) { char e[24]; snprintf(e, sizeof e, "begin%02X", a); callLog(e); return true; }
    bool hasReset() { bool r = reset_flag; reset_flag = false; return r; }
    void enableGameRotationVector(uint16_t ms) { callLog(ms ? "grv" : "grvoff"); }
    void enableGyro(uint16_t) { callLog("gyro"); }
    void enableAccelerometer(uint16_t) { callLog("accel"); }
    void enableMagnetometer(uint16_t) { callLog("mag"); }
    void sendCommand(uint8_t c) {
        char e[48]; snprintf(e, sizeof e, "cmd%d:%d,%d,%d", c, shtpData[3], shtpData[4], shtpData[5]); callLog(e);
    }
    uint16_t getReadings() { return qn ? queue[--qn] : 0; }
};
"""

DRIVER = r"""
#include "bno_chips.h"
static void reset_log() { g_calls[0] = 0; }
int main() {
    // 1. the magnetometer first, then the IMU
    {
        Bno08xChip c;
        reset_log();
        printf("magfirst_start=%d\n", c.wantMag());
        printf("magfirst_after_mag=%s\n", g_calls);
        reset_log();
        printf("magfirst_imu_start=%d\n", c.wantImu());
        printf("magfirst_after_imu=%s\n", g_calls);
        printf("magfirst_chip_fuses=%d\n", c.chipFusesOrientation());
        // a chip reset re-arms everything, the calibration setting included
        reset_log();
        c.dev().reset_flag = true;
        c.dev().queue[0] = SENSOR_REPORTID_MAGNETIC_FIELD; c.dev().qn = 1;
        c.poll();
        printf("magfirst_after_reset=%s\n", g_calls);
        printf("magfirst_have_mag=%d have_imu=%d\n", c.haveMag(), c.haveImu());
    }
    // 2. the IMU only: exactly the old behaviour
    {
        Bno08xChip c;
        reset_log();
        c.wantImu();
        printf("imuonly=%s\n", g_calls);
        printf("imuonly_chip_fuses=%d\n", c.chipFusesOrientation());
    }
    // 3. the IMU first, then the magnetometer
    {
        Bno08xChip c;
        reset_log();
        c.wantImu(); c.wantMag();
        printf("imufirst=%s\n", g_calls);
        printf("imufirst_chip_fuses=%d\n", c.chipFusesOrientation());
    }
    // 4. the BNO055
    {
        Wire.regs[0x00] = 0xA0;
        printf("bno055_start=%d\n", bno055Chip().start());
        printf("bno055_writes=%s\n", Wire.log);
        printf("bno055_again=%d\n", bno055Chip().start());
        printf("bno055_writes_after=%s\n", Wire.log);
    }
    return 0;
}
"""


@pytest.fixture(scope="module")
def run():
    cxx = shutil.which("g++") or shutil.which("clang++")
    if not cxx:
        pytest.skip("no host C++ compiler")
    d = tempfile.mkdtemp(prefix="bno_")
    for name, text in (("Arduino.h", STUB_ARDUINO), ("Wire.h", STUB_WIRE),
                       ("syslog.h", STUB_SYSLOG),
                       ("SparkFun_BNO080_Arduino_Library.h", STUB_BNO080),
                       ("main.cpp", DRIVER)):
        with open(os.path.join(d, name), "w") as fh:
            fh.write(text)
    exe = os.path.join(d, "bno")
    res = subprocess.run([cxx, "-std=c++17", "-O0", "-o", exe, os.path.join(d, "main.cpp"),
                          "-I", d, "-I", IMU_DIR], capture_output=True, text=True)
    assert res.returncode == 0, res.stderr[:2000]
    out = subprocess.run([exe], capture_output=True, text=True, check=True).stdout
    vals = {}
    for line in out.splitlines():
        k, _, v = line.partition("=")
        vals[k] = v
    return vals


def test_the_magnetometer_starting_first_starts_the_chip_once(run):
    assert run["magfirst_start"] == "1"
    assert run["magfirst_after_mag"].count("begin") == 1
    assert run["magfirst_imu_start"] == "1"
    assert "begin" not in run["magfirst_after_imu"], "the second role ran begin() again"


def test_with_the_field_the_chip_is_not_the_orientation_source(run):
    assert run["magfirst_chip_fuses"] == "0"
    assert run["imufirst_chip_fuses"] == "0"
    for key in ("magfirst_after_imu", "imufirst"):
        last = run[key].rsplit("grv", 1)[1]
        assert last.startswith("off;"), \
            f"{key}: the game rotation vector is left on while the board fuses the field"


def test_the_chips_magnetometer_calibration_is_off_and_the_rest_left_on(run):
    # COMMAND_ME_CALIBRATE (7) with P0 accel 1, P1 gyro 1, P2 mag 0
    assert "mag;cmd7:1,1,0" in run["magfirst_after_mag"]
    assert "cmd7:1,1,0" in run["imufirst"]


def test_a_chip_reset_re_arms_the_calibration_setting_too(run):
    after = run["magfirst_after_reset"]
    assert "mag" in after and "cmd7:1,1,0" in after
    assert "gyro" in after and "accel" in after


def test_without_the_field_nothing_changes(run):
    assert run["imuonly"].count("begin") == 1
    for report in ("grv", "gyro", "accel"):
        assert report in run["imuonly"], report
    assert "cmd" not in run["imuonly"], "the chip's calibration was touched without a magnetometer"
    assert "mag" not in run["imuonly"]
    assert run["imuonly_chip_fuses"] == "1"


def test_both_role_sets_are_one_begin_in_either_order(run):
    assert run["imufirst"].count("begin") == 1


def test_the_bno055_goes_to_amg_mode_with_si_units(run):
    assert run["bno055_start"] == "1"
    w = run["bno055_writes"]
    # CONFIG first (settings only change there), then units, then AMG last
    assert w.index("W3D=00") < w.index("W3B=02") < w.index("W3D=07"), w
    assert w.endswith("W3D=07;")
    assert run["bno055_again"] == "1"
    assert run["bno055_writes_after"] == w, "a second role reconfigured a running BNO055"


# --- the drivers on top of it ----------------------------------------------------

def test_the_bno085_gyro_is_read_in_rad_per_s():
    """The SH-2 calibrated gyroscope report is rad/s (Q9); multiplying it by
    DEG_TO_RAD made /imu/data's angular velocity 57.3 times too small."""
    blk = _block(_read(DEFAULT_IMU), "class BNO085IMU")
    assert "DEG_TO_RAD" not in blk.split("readGyroscope() override", 1)[1].split("logImuDataUnavailable", 1)[0]
    assert re.search(r"gyro_\.z\s*=\s*d\.getGyroZ\(\);", blk)


def test_the_bno085_magnetometer_reports_tesla_from_the_same_chip():
    blk = _block(_read(DEFAULT_MAG), "class BNO085MAG")
    assert "bno08xChip().wantMag()" in blk
    assert "getMagX() * 1e-6" in blk


def test_the_bno055_scales_follow_the_datasheet():
    src = _read(CHIPS)
    assert "ACC_LSB_PER_MS2 = 100.0" in src
    assert "GYR_LSB_PER_RPS = 900.0" in src
    assert "MAG_LSB_PER_UT = 16.0" in src
    assert "NOT RUN ON SILICON" in src, "the BNO055 driver no longer says it is untested"


def test_every_bno_name_is_in_the_factory():
    src = _read(FACTORY)
    for name in ("bno085", "bno080", "bno086"):
        assert re.search(rf'\{{"{name}",\s*makeIMU<BNO085IMU>\}}', src), name
        assert re.search(rf'\{{"{name}",\s*makeMAG<BNO085MAG>\}}', src), name
    assert re.search(r'\{"bno055",\s*makeIMU<BNO055IMU>\}', src)
    assert re.search(r'\{"bno055",\s*makeMAG<BNO055MAG>\}', src)


def test_the_probe_offers_a_bno08x_as_an_imu_only():
    """Its field changes where the heading comes from; a config names that."""
    src = _read(PROBE)
    blk = src[src.index("addr == 0x4A || addr == 0x4B"):]
    blk = blk[:blk.index("return;")]
    assert '"imu", "BNO085", "bno085"' in blk
    assert '"mag"' not in blk


def test_the_probe_drives_a_bno055_and_offers_its_field():
    src = _read(PROBE)
    blk = src[src.index("addr == 0x28 || addr == 0x29"):]
    blk = blk[:blk.index("return;")]
    assert '"imu", "BNO055", "bno055"' in blk
    assert '"mag", "BNO055", "bno055"' in blk


def _env(imu, mag):
    import mcu_env
    from gen_bare_config import bare_config
    cfg = bare_config("esp32")
    cfg["base_controller"]["sensors"]["imu"] = imu
    cfg["base_controller"]["sensors"]["mag"] = mag
    d = tempfile.mkdtemp()
    secrets = os.path.join(d, "secrets.yaml")
    with open(secrets, "w") as fh:
        fh.write("{}\n")
    params = os.path.join(d, "robot.yaml")
    with open(params, "w") as fh:
        fh.write(yaml.safe_dump(cfg))
    return mcu_env.env_from_config(params, secrets)


@pytest.mark.parametrize("part", ["BNO085", "BNO080", "BNO086", "BNO055"])
def test_a_named_bno_field_brings_its_variance(part):
    env = _env(part, part)
    assert env["mag_cov"] == "3.6e-13", env.get("mag_cov")
    assert "accel_cov" in env and "gyro_cov" in env


def test_the_bno08x_aliases_share_the_bno085s_figures():
    a, b = _env("BNO085", "NONE"), _env("BNO086", "NONE")
    for key in ("accel_cov", "gyro_cov", "ori_cov"):
        assert a[key] == b[key], key


def test_a_bno055_has_no_orientation_variance_of_its_own():
    """AMG mode: the board fuses it, so the chip claims no orientation."""
    env = _env("BNO055", "NONE")
    assert "ori_cov" not in env
    assert env["accel_cov"] == _env("BNO085", "NONE")["accel_cov"]


def test_one_part_is_one_row_in_the_tables():
    """A second name for the same silicon must not count twice in the median
    the simulated IMU is sized to."""
    src = _read(os.path.join(ROOT, "scripts", "mcu_env.py"))
    for alias in ("BNO080", "BNO086", "BNO055"):
        assert not re.search(rf'"{alias}":\s*\{{', src), f"{alias} is a row of its own"


def test_the_bno055_scales_match_the_units_it_selects():
    """UNIT_SEL decides what the gyro register means: 16 LSB per deg/s by
    default, 900 LSB per rad/s with bit 1 set. A scale that does not match the
    bit is the BNO085's 57.3x error again (tests/test_bno085_gyro_units.py).
    The accelerometer is m/s^2 with bit 0 clear (100 LSB per m/s^2); the
    magnetometer is always 16 LSB per uT."""
    src = _read(CHIPS)
    m = re.search(r"w8\(REG_UNIT_SEL,\s*0x([0-9A-Fa-f]{2})\)", src)
    assert m, "the BNO055 no longer selects its units"
    unit_sel = int(m.group(1), 16)
    gyro_rad = bool(unit_sel & 0x02)
    accel_ms2 = not (unit_sel & 0x01)
    want_gyro = "GYR_LSB_PER_RPS = 900.0" if gyro_rad else "GYR_LSB_PER_DPS = 16.0"
    assert want_gyro in src, f"UNIT_SEL 0x{unit_sel:02X} needs {want_gyro}"
    assert accel_ms2 and "ACC_LSB_PER_MS2 = 100.0" in src
    blk = _block(_read(DEFAULT_IMU), "class BNO055IMU")
    assert "DEG_TO_RAD" not in blk and "GYR_LSB_PER_RPS" in blk
