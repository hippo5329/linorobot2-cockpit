"""A scan with no gaps in it must still reach /scan.

`Tofbf::NearFilter` groups a revolution by continuity, then "connects 0 and 359
degrees" by moving the last group onto the front one and erasing it. When the
whole revolution is ONE group -- which it is whenever the surface is continuous
all the way round -- `front()` and `back()` are the same vector: it is inserted
into itself (undefined behaviour, the iterators are invalidated by the
reallocation) and then erased as a duplicate. The filter returns nothing.

Downstream that is silent. `LiPkg::AssemblePacket` publishes only
`if (tmp.size() > 0)` and erases the revolution from its buffer only inside
that same branch, so the points pile up until the overrun bail throws them
away, and `demo.cpp` prints nothing for the DATA_WAIT that results. The node
logs "ldlidar communication is normal", advertises /scan, and never publishes.

Our fake LD19 room is geometrically perfect, so every revolution is one group.
Measured 2026-09-20 by driving the driver from a harness with our exact bytes:

    in=456 pending=456 groups=1 -> WRAP MERGE fires -> groups=0 -> filtered=0

Only the first frame survived, because the driver drops the very first packet
to seed its timestamp, which leaves an arc that does not close. A real LD19 is
saved by its own noise, not by anything in the code.

`scripts/prepare_docker_vendor.sh` patches the staged copy. This test compiles
the staged driver and runs a continuous scan through it, so the patch cannot be
lost in a re-vendor without the suite saying so.
"""
import os
import shutil
import subprocess
import sys

import pytest

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DRIVER = os.path.join(REPO_ROOT, "docker", "vendor", "ldlidar_stl_ros2",
                      "ldlidar_driver")

HARNESS = r"""
#include "lipkg.h"
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <chrono>
#include <vector>
using namespace ldlidar;

static uint64_t now_ns() {
    return (uint64_t)std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
}

// firmware/common/lib/lidar/fake_ld19.h, transcribed.
static const uint16_t POINTS_PER_PACK = 12;
static const uint16_t POINTS_PER_REV  = 456;
static const float    DEG_PER_POINT   = 360.0f / 456.0f;
static const uint16_t SPEED_DPS       = 3600;

static uint8_t crc8(const uint8_t *d, int len) {
    static uint8_t tab[256]; static bool built = false;
    if (!built) {
        for (int i = 0; i < 256; i++) {
            uint8_t c = (uint8_t)i;
            for (int b = 0; b < 8; b++)
                c = (c & 0x80) ? (uint8_t)((c << 1) ^ 0x4D) : (uint8_t)(c << 1);
            tab[i] = c;
        }
        built = true;
    }
    uint8_t c = 0;
    for (int i = 0; i < len; i++) c = tab[(c ^ d[i]) & 0xFF];
    return c;
}
static void put16(uint8_t *p, uint16_t v) { p[0] = v & 0xFF; p[1] = v >> 8; }

static uint16_t point_idx = 0;
static uint32_t fake_ms = 0;

// A square room: continuous all the way round, which is the whole point.
static uint16_t range_mm(float deg) {
    float r = deg * (float)M_PI / 180.0f, best = 1e9f, half = 2.0f;
    float c = cosf(r), s = sinf(r);
    if (fabsf(c) > 1e-6f) { float t = half / fabsf(c); if (t < best) best = t; }
    if (fabsf(s) > 1e-6f) { float t = half / fabsf(s); if (t < best) best = t; }
    return (uint16_t)(best * 1000.0f);
}

static void build_pack(uint8_t *pkt) {
    pkt[0] = 0x54; pkt[1] = 0x2C;
    put16(&pkt[2], SPEED_DPS);
    float start_deg = point_idx * DEG_PER_POINT;
    float end_deg   = (point_idx + POINTS_PER_PACK - 1) * DEG_PER_POINT;
    put16(&pkt[4], (uint16_t)(fmodf(start_deg, 360.0f) * 100.0f));
    for (int i = 0; i < POINTS_PER_PACK; i++) {
        float deg = (point_idx + i) * DEG_PER_POINT;
        uint16_t dist = range_mm(deg);
        uint8_t inten = (dist == 0) ? 0 : (dist < 1500) ? 220 : (dist < 4000) ? 180 : 120;
        put16(&pkt[6 + i * 3], dist);
        pkt[6 + i * 3 + 2] = inten;
    }
    put16(&pkt[42], (uint16_t)(fmodf(end_deg, 360.0f) * 100.0f));
    put16(&pkt[44], (uint16_t)(fake_ms % 30000));
    pkt[46] = crc8(pkt, 46);
    point_idx += POINTS_PER_PACK;
    if (point_idx >= POINTS_PER_REV) point_idx -= POINTS_PER_REV;
    fake_ms += 3;
}

int main(int argc, char **argv) {
    int per_callback = (argc > 1) ? atoi(argv[1]) : 1;
    int total_packs  = (argc > 2) ? atoi(argv[2]) : 38 * 6;
    LiPkg pkg;
    pkg.ClearDataProcessStatus();
    pkg.RegisterTimestampGetFunctional(now_ns);
    pkg.SetProductType(LDType::LD_19);
    std::vector<uint8_t> chunk;
    int frames = 0, smallest = 1 << 30;
    for (int n = 0; n < total_packs; n++) {
        uint8_t pkt[47];
        build_pack(pkt);
        chunk.insert(chunk.end(), pkt, pkt + 47);
        if ((n + 1) % per_callback == 0) {
            pkg.CommReadCallback((const char *)chunk.data(), chunk.size());
            chunk.clear();
            Points2D out;
            if (pkg.GetLaserScanData(out)) {
                frames++;
                if ((int)out.size() < smallest) smallest = (int)out.size();
            }
        }
    }
    printf("%d %d\n", frames, frames ? smallest : 0);
    return 0;
}
"""

POINTS_PER_REV = 456
PACKS_PER_REV = 38


@pytest.fixture(scope="module")
def harness(tmp_path_factory):
    if not os.path.isdir(DRIVER):
        pytest.skip("ldlidar_stl_ros2 is not staged "
                    "(run scripts/prepare_docker_vendor.sh)")
    cxx = shutil.which("g++") or shutil.which("clang++")
    if not cxx:
        pytest.skip("no host C++ compiler")
    d = tmp_path_factory.mktemp("ld19")
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
        pytest.skip("cannot build the vendored driver here: %s"
                    % res.stderr.strip().splitlines()[-1:])
    return exe


def _run(harness, per_callback, revolutions=6):
    out = subprocess.run([harness, str(per_callback),
                          str(PACKS_PER_REV * revolutions)],
                         capture_output=True, text=True, check=True).stdout
    frames, smallest = (int(x) for x in out.split())
    return frames, smallest


@pytest.mark.parametrize("per_callback, what", [
    (1, "one packet at a time, as a UART delivers them"),
    (30, "30 packets a datagram, as the UDP sink batches them"),
    (PACKS_PER_REV, "a whole revolution in one chunk"),
])
def test_a_gapless_scan_is_published(harness, per_callback, what):
    frames, smallest = _run(harness, per_callback)
    assert frames >= 3, (
        "only %d scans out of 6 revolutions (%s) -- the wrap merge is eating "
        "the single-group revolution again" % (frames, what))
    assert smallest == POINTS_PER_REV, (
        "a published scan had %d points instead of %d (%s): a partial arc, "
        "which is what survives when the full revolution is discarded"
        % (smallest, POINTS_PER_REV, what))


def test_the_patch_is_in_the_staged_source(harness):
    """The fix lives in scripts/prepare_docker_vendor.sh, so a re-vendor that
    silently stops applying it has to fail here rather than on the bench."""
    src = open(os.path.join(DRIVER, "src", "filter", "tofbf.cpp")).read()
    assert "group.size() > 1 &&" in src, (
        "the wrap-merge guard is missing from the staged driver -- did "
        "prepare_docker_vendor.sh fail to patch it?")
