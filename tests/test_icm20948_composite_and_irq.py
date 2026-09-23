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

2. THE INTERRUPT. ICM20948IMU inherited enableDataReadyInterrupt() -> false, so
   with INT wired the firmware attached an ISR, waited a second for an edge the
   chip was never told to produce, and polled: "data-ready pin 2 never fired in
   1 s". And the ISR itself reached ONE static instance, which made data-ready a
   single-sensor feature -- a second line would have silently marked the wrong
   sensor's sample fresh.
"""
import ctypes
import os
import shutil
import subprocess

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FW = os.path.join(ROOT, "firmware", "common", "lib")
PROBE_DIR = os.path.join(FW, "i2c_probe")
IMU_DIR = os.path.join(FW, "imu")


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

def test_the_icm20948_can_ask_its_chip_for_data_ready():
    body = _read(os.path.join(IMU_DIR, "default_imu.h"))
    start = body.index("class ICM20948IMU")
    cls = body[start:body.index("\nclass ", start + 10)]
    assert "enableDataReadyInterrupt" in cls, "the ICM-20948 still cannot drive INT1"
    assert "0x11" in cls, "INT_ENABLE_1 is never written"
    assert "0x01" in cls, "RAW_DATA_0_RDY_EN is never set"


def test_enabling_the_interrupt_does_not_switch_the_magnetometer_off():
    """INT_PIN_CFG carries BYPASS_EN, which is how the AK09918 is reachable at
    all. Writing the pin's shape must preserve it -- a bare w8(0x0F, ...) here
    would turn the magnetometer off to turn the interrupt on."""
    body = _read(os.path.join(IMU_DIR, "default_imu.h"))
    start = body.index("class ICM20948IMU")
    cls = body[start:body.index("\nclass ", start + 10)]
    fn = cls[cls.index("bool enableDataReadyInterrupt"):]
    fn = fn[:fn.index("\n        }")]
    assert "r8(0x0F)" in fn, "INT_PIN_CFG is written without reading it first"
    assert "~0xF0" in fn, "the pin's mode bits are not all forced"


def test_the_interrupt_is_pulsed_not_latched():
    """INT1_LATCH_INT_EN (INT_PIN_CFG bit 5) must be CLEARED, not inherited.

    Latched, the line goes high on the first sample and stays high, because
    nothing in this driver reads INT_STATUS (0x1A) to clear it. That is exactly
    one rising edge and then silence -- which getData() sees as a line that
    fired once and died, and the staleness ceiling then papers over by reading
    the bus anyway. A data-ready feature that silently degrades to polling is
    the fault this whole change exists to remove.

    It is clear today only because startSensor() writes 0x0F = 0x02 first.
    Depending on that ordering is the same fragility as depending on it for the
    magnetometer bypass, which is already a known trap on this part.
    """
    body = _read(os.path.join(IMU_DIR, "default_imu.h"))
    start = body.index("class ICM20948IMU")
    cls = body[start:body.index("\nclass ", start + 10)]
    fn = cls[cls.index("bool enableDataReadyInterrupt"):]
    fn = fn[:fn.index("\n        }")]
    # ~0xF0 clears ACTL(7), OPEN(6), LATCH(5) and ANYRD_2CLEAR(4); bit 1
    # BYPASS_EN survives.
    assert "~0xF0" in fn
    assert "~0xC0" not in fn, "only ACTL and OPEN are forced; LATCH is left to chance"


def test_data_ready_is_multiplexed_not_a_singleton():
    """One static instance made data-ready single-sensor: a second line would
    overwrite the first and mark the WRONG sensor's sample fresh."""
    hdr = _read(os.path.join(IMU_DIR, "imu_interface.h"))
    cpp = _read(os.path.join(IMU_DIR, "imu_interface.cpp"))
    assert "static IMUInterface *instance_;" not in hdr, "the singleton is back"
    assert "sources_[IMU_INT_MAX_SOURCES]" in hdr
    assert "int_slot_" in hdr
    # one trampoline per slot, because attachInterrupt takes no argument on
    # the AVR and RP2 cores
    for n in range(4):
        assert f"isrSlot{n}" in cpp, f"no trampoline for slot {n}"


def test_running_out_of_slots_is_refused_out_loud():
    """Silently polling would look identical to a line that never fires, and
    this is a wiring fact the person holding the board can act on."""
    hdr = _read(os.path.join(IMU_DIR, "imu_interface.h"))
    fn = hdr[hdr.index("void attachDataReady(int pin)"):]
    fn = fn[:fn.index("\n        int intPin()")]
    assert "no interrupt slot free" in fn
    assert "int_pin_ = -1;" in fn, "a sensor with no slot must fall back to polling"


def test_reattaching_the_same_sensor_does_not_consume_a_second_slot():
    hdr = _read(os.path.join(IMU_DIR, "imu_interface.h"))
    fn = hdr[hdr.index("void attachDataReady(int pin)"):]
    fn = fn[:fn.index("\n        int intPin()")]
    assert "sources_[i] == this" in fn, "a re-attach exhausts the table"
