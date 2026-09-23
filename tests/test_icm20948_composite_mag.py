"""A 9-axis part is one chip, and every IMU may own a data-ready line.

Both of these were found on the bench Pico 2 on 2026-09-23, after its MPU6050
was replaced with an ICM-20948 on the same wiring (I2C0 GP0/GP1, INT on GPIO 2).

1. THE COMPOSITE. i2c_detect reported three devices:

       [0x0C] mag AK09918  driver ak09918      <- the ICM-20948's own
       [0x68] imu ICM20948 driver icm20948
       [0x68] mag AK09916  driver icm20948

   The first is not a separate chip. ICM20948IMU::startSensor() sets
   INT_PIN_CFG BYPASS_EN to reach its AK09918, which wires it onto the MAIN bus
   at 0x0C and leaves it there until something clears the bit -- so the probe's
   own comment ("never ACKs a scan of the main bus") is true from a power-on
   reset and false for the rest of the board's life. i2cProbeFind takes the
   first match, so the magnetometer was selected as a standalone part at the
   lower address. It worked, but only because IMU init runs before MAG init and
   turns the bypass on; reorder those and the mag goes silent with the bus
   still ACKing and nothing to explain it.

(This file also covered the ICM-20948's data-ready interrupt. That whole path
was removed on 2026-09-24 -- an ISR that cannot read the bus buys a timestamp
whose pairing with the eventually-read sample is uncertain -- so those tests
went with it. The composite fold is independent of it and stands.)
"""
import ctypes
import os
import shutil
import subprocess

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FW = os.path.join(ROOT, "firmware", "common", "lib")
PROBE_DIR = os.path.join(FW, "i2c_probe")
IMU_DIR = os.path.join(FW, "imu")  # noqa: F401  (kept: the fold tests read it)


def _read(p):
    with open(p, encoding="utf-8") as fh:
        return fh.read()


# --- the composite fold, compiled and driven -------------------------------

STUB_ARDUINO = """
#pragma once
#include <stdint.h>
#include <stdio.h>
#include <string.h>
struct FakeWire {
    void begin() {}
    void beginTransmission(int) {}
    void write(uint8_t) {}
    int  endTransmission(bool = true) { return 1; }
    int  requestFrom(int, int) { return 0; }
    int  available() { return 0; }
    int  read() { return 0xFF; }
};
static FakeWire Wire;
struct FakeSerial { int printf(const char *, ...) { return 0; } void println(const char *) {} };
static FakeSerial Serial;
static inline unsigned long millis() { return 0; }
static inline void delay(unsigned long) {}
"""

SHIM = """
#include "i2c_probe.h"
#include <string.h>
extern "C" {
// A device table is built here rather than scanned: the bus state that
// produces the duplicate (bypass latched by a previous boot) cannot be
// arranged from a test.
int fold(int with_imu, char *out_driver, char *out_model, int n) {
    I2CDevice devs[3];
    int c = 0;
    devs[c++] = I2CDevice{0x0C, "mag", "AK09918", "ak09918", "USE_AK09918_MAG", "standalone"};
    if (with_imu) {
        devs[c++] = I2CDevice{0x68, "imu", "ICM20948", "icm20948", "USE_ICM20948_IMU", "imu"};
        devs[c++] = I2CDevice{0x68, "mag", "AK09916", "icm20948", "USE_ICM20948_MAG", "inside"};
    }
    i2cProbeFoldComposites(devs, c);
    strncpy(out_driver, devs[0].driver, n - 1);
    strncpy(out_model, devs[0].model, n - 1);
    // what a caller would actually select
    const I2CDevice *m = i2cProbeFind(devs, c, "mag");
    return m ? (strcmp(m->driver, "icm20948") == 0 ? 1 : 0) : -1;
}
}
"""


@pytest.fixture(scope="module")
def fold_lib(tmp_path_factory):
    cxx = shutil.which("g++") or shutil.which("clang++")
    if not cxx:
        pytest.skip("no host C++ compiler")
    d = tmp_path_factory.mktemp("probe")
    open(os.path.join(d, "Arduino.h"), "w").write(STUB_ARDUINO)
    open(os.path.join(d, "Wire.h"), "w").write("#pragma once\n#include <Arduino.h>\n")
    open(os.path.join(d, "lino_console.h"), "w").write("#pragma once\n")
    open(os.path.join(d, "syslog.h"), "w").write(
        "#pragma once\n#define LOG_INFO 6\n#define LOG_WARNING 4\n"
        "static inline void syslog(int, const char *, ...) {}\n")
    open(os.path.join(d, "shim.cpp"), "w").write(SHIM)
    so = os.path.join(d, "libprobe.so")
    res = subprocess.run(
        [cxx, "-shared", "-fPIC", "-O0", "-o", so,
         os.path.join(d, "shim.cpp"), os.path.join(PROBE_DIR, "i2c_probe.cpp"),
         "-I", str(d), "-I", PROBE_DIR],
        capture_output=True, text=True)
    if res.returncode != 0:
        pytest.skip(f"i2c_probe does not build against the stub: {res.stderr[:400]}")
    lib = ctypes.CDLL(so)
    lib.fold.restype = ctypes.c_int
    lib.fold.argtypes = [ctypes.c_int, ctypes.c_char_p, ctypes.c_char_p, ctypes.c_int]
    return lib


def _fold(lib, with_imu):
    drv = ctypes.create_string_buffer(64)
    mdl = ctypes.create_string_buffer(64)
    picked = lib.fold(1 if with_imu else 0, drv, mdl, 64)
    return drv.value.decode(), mdl.value.decode(), picked


def test_the_satellite_mag_is_folded_into_the_part_that_owns_it(fold_lib):
    driver, model, picked = _fold(fold_lib, with_imu=True)
    assert driver == "icm20948", f"0x0C still looks like a standalone {model}"
    assert picked == 1, "the selected magnetometer is not the ICM-20948's"


def test_a_real_standalone_ak09918_is_left_alone(fold_lib):
    """The fold must key on the composite being PRESENT, not on the address.
    An AK09918 on a board with no ICM-20948 is its own chip."""
    driver, _model, picked = _fold(fold_lib, with_imu=False)
    assert driver == "ak09918", "a standalone magnetometer was reattributed"
    assert picked == 0


def test_the_fold_runs_inside_the_probe_not_only_in_the_test():
    src = _read(os.path.join(PROBE_DIR, "i2c_probe.cpp"))
    scan = src[src.index("int i2cProbe(I2CDevice *out"):]
    assert "i2cProbeFoldComposites(out, sink.count);" in scan[:900], \
        "i2cProbe returns an unfolded list, so only the test sees the fix"


# --- the interrupt ---------------------------------------------------------