"""The Oradar MS200 is read by the LD driver, through its own product type.

Oradar's manual (PD-P2117008) gives the MS200 the LD19's packet -- 0x54 header,
12 points of (distance mm, intensity), 0.01 deg start/end angles, a ms timestamp,
the same CRC-8 table, 230400 8N1 -- with two differences the LD19 type would get
wrong:

* the second byte is "top three bits reserved, low five = 12 points", where the
  LD parser accepts only 0x2C: a unit that sends 0x0C would never produce a scan;
* intensities 0-15 are reserved codes (0 invalid, 2 high reflectivity, 3 low SNR),
  not echoes, so those points are no return.

It also sends a serial-number frame (0x55 0xAA ...) at power-up, which must be
skipped. scripts/prepare_docker_vendor.sh adds LDType::MS_200; this compiles the
staged driver and pushes whole revolutions through parse, filter and assembly,
as test_ldlidar_scan_assembly does for the LD19.
"""
import os
import shutil
import subprocess

import pytest

import sys
REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO_ROOT, "scripts"))
import lidar_drivers  # noqa: E402

DRIVER = os.path.join(REPO_ROOT, "docker", "vendor", "ldlidar_stl_ros2", "ldlidar_driver")

HARNESS = r"""
#include "lipkg.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <chrono>
#include <vector>
using namespace ldlidar;

static uint64_t now_ns() {
    return (uint64_t)std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
}
static uint8_t crc8(const uint8_t *d, int len) {   // the manual's table: poly 0x4D
    static uint8_t tab[256]; static bool built = false;
    if (!built) {
        for (int i = 0; i < 256; i++) {
            uint8_t c = (uint8_t)i;
            for (int b = 0; b < 8; b++) c = (c & 0x80) ? (uint8_t)((c << 1) ^ 0x4D) : (uint8_t)(c << 1);
            tab[i] = c;
        }
        built = true;
    }
    uint8_t c = 0;
    for (int i = 0; i < len; i++) c = tab[(c ^ d[i]) & 0xFF];
    return c;
}
static void put16(uint8_t *p, uint16_t v) { p[0] = v & 0xFF; p[1] = v >> 8; }

static const int PER_PACK = 12, PER_REV = 456;
static const float DEG = 360.0f / PER_REV;

// argv: type (ms200|ld19), second byte (hex), reserved_every (0 = none), sn (0/1)
int main(int argc, char **argv) {
    const bool ms = strcmp(argv[1], "ms200") == 0;
    const uint8_t verlen = (uint8_t)strtol(argv[2], nullptr, 16);
    const int reserved_every = atoi(argv[3]);
    const bool sn = atoi(argv[4]) != 0;
    LiPkg pkg;
    pkg.ClearDataProcessStatus();
    pkg.RegisterTimestampGetFunctional(now_ns);
    pkg.SetProductType(ms ? LDType::MS_200 : LDType::LD_19);
    if (sn) {   // the power-up serial-number frame, manual table 5-1
        const uint8_t f[] = {0x55, 0xAA, 0x01, 0x0A, 'C','F','3','P','5','2','5','0','0','2', 0x00, 0xF2, 0x31};
        pkg.CommReadCallback((const char *)f, sizeof f);
    }
    int idx = 0, frames = 0, smallest = 1 << 30, nan_pts = 0, total = 0;
    uint32_t ms_clock = 0;
    for (int n = 0; n < 38 * 6; n++) {
        uint8_t p[47];
        p[0] = 0x54; p[1] = verlen;
        put16(&p[2], 3600);
        put16(&p[4], (uint16_t)(fmodf(idx * DEG, 360.0f) * 100.0f));
        for (int i = 0; i < PER_PACK; i++) {
            put16(&p[6 + i * 3], 2000);           // a 2 m circle: continuous, no gaps
            p[8 + i * 3] = (reserved_every && (idx + i) % reserved_every == 0) ? 3 : 200;
        }
        put16(&p[42], (uint16_t)(fmodf((idx + PER_PACK - 1) * DEG, 360.0f) * 100.0f));
        put16(&p[44], (uint16_t)(ms_clock % 30000));
        p[46] = crc8(p, 46);
        idx = (idx + PER_PACK) % PER_REV; ms_clock += 3;
        pkg.CommReadCallback((const char *)p, sizeof p);
        Points2D out;
        if (pkg.GetLaserScanData(out)) {
            frames++;
            if ((int)out.size() < smallest) smallest = (int)out.size();
            for (auto &pt : out) { total++; if (pt.distance == 0 && pt.intensity == 0) nan_pts++; }
        }
    }
    printf("%d %d %d %d\n", frames, frames ? smallest : 0, nan_pts, total);
    return 0;
}
"""


@pytest.fixture(scope="module")
def harness(tmp_path_factory):
    if not os.path.isdir(DRIVER):
        pytest.skip("ldlidar_stl_ros2 is not staged (run scripts/prepare_docker_vendor.sh)")
    cxx = shutil.which("g++") or shutil.which("clang++")
    if not cxx:
        pytest.skip("no host C++ compiler")
    src_dt = open(os.path.join(DRIVER, "include", "core", "ldlidar_datatype.h")).read()
    if "MS_200" not in src_dt:
        pytest.fail("the staged driver has no LDType::MS_200 -- re-run scripts/prepare_docker_vendor.sh")
    d = tmp_path_factory.mktemp("ms200")
    src = os.path.join(d, "harness.cpp")
    open(src, "w").write(HARNESS)
    exe = os.path.join(d, "harness")
    cmd = [cxx, "-O0", "-w", "-o", exe, src,
           os.path.join(DRIVER, "src", "dataprocess", "lipkg.cpp"),
           os.path.join(DRIVER, "src", "filter", "tofbf.cpp"),
           os.path.join(DRIVER, "src", "logger", "log_module.cpp")]
    for inc in ("dataprocess", "core", "filter", "logger"):
        cmd += ["-I", os.path.join(DRIVER, "include", inc)]
    res = subprocess.run(cmd, capture_output=True, text=True)
    if res.returncode != 0:
        pytest.skip("cannot build the vendored driver here: %s" % res.stderr.strip().splitlines()[-1:])
    return exe


def _run(exe, kind, verlen, reserved_every=0, sn=False):
    out = subprocess.run([exe, kind, verlen, str(reserved_every), "1" if sn else "0"],
                         capture_output=True, text=True, check=True).stdout
    return tuple(int(x) for x in out.split())


@pytest.mark.parametrize("verlen", ["0C", "2C"])
def test_an_ms200_scan_is_published_whatever_its_reserved_bits(harness, verlen):
    frames, smallest, nan_pts, _ = _run(harness, "ms200", verlen, sn=True)
    assert frames >= 3 and smallest == 456, (verlen, frames, smallest)
    assert nan_pts == 0


def test_the_ms200_reserved_intensities_are_no_return(harness):
    frames, _, nan_pts, total = _run(harness, "ms200", "0C", reserved_every=4)
    assert frames >= 3
    assert nan_pts == total // 4, (nan_pts, total)


def test_a_wrong_point_count_is_still_refused(harness):
    assert _run(harness, "ms200", "0D")[0] == 0


def test_the_ld19_is_untouched(harness):
    assert _run(harness, "ld19", "0C")[0] == 0           # still 0x2C only
    frames, smallest, _, _ = _run(harness, "ld19", "2C")
    assert frames >= 3 and smallest == 456


def test_ms200_is_an_ld_model_with_its_own_product():
    assert lidar_drivers.family("ms200") == "ldlidar"
    assert lidar_drivers.ld_product("ms200") == ("LDLiDAR_MS200", 456)
    assert "ms200" not in lidar_drivers.LIDAR_INIT          # it ranges at power-up
    vendor = open(os.path.join(REPO_ROOT, "scripts", "prepare_docker_vendor.sh")).read()
    assert 'product_name == "LDLiDAR_MS200"' in vendor and "LDType::MS_200" in vendor
    sys.path.insert(0, os.path.join(REPO_ROOT, "web", "backend"))
    try:
        import system_utils
    except Exception:                                        # backend deps absent: source check
        src = open(os.path.join(REPO_ROOT, "web", "backend", "system_utils.py")).read()
        assert '"code": "ms200"' in src and '"product": "LDLiDAR_MS200"' in src
    else:
        codes = {m["code"]: m for m in system_utils.LASER_SENSORS["ldlidar"]["models"]}
        assert codes["ms200"]["product"] == "LDLiDAR_MS200"
