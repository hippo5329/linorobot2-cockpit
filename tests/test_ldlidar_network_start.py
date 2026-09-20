"""A LiDAR reached over the network must actually deliver scans.

`LDLidarDriver::Start()` has two overloads. The serial one ends with

    is_start_flag_ = true;
    SetIsOkStatus(true);

and the network one -- TCP client, TCP server, UDP client, UDP server -- ended
with a bare `return true;`, setting neither. `GetLaserScanData()` then takes
its `if (!is_start_flag_) return LidarStatus::STOP;` branch on every call, and
`demo.cpp`'s switch has no case for STOP: it falls into `default: break`. The
node publishes nothing, for ever, and says nothing about it.

Everything upstream looks healthy, which is what made it expensive to find.
"ldlidar node start is success" comes from Start()'s return value;
"ldlidar communication is normal" comes from a different flag the parser sets
when it accepts a packet -- so the socket is open, the datagrams arrive, the
CRC passes, and full 456-point revolutions are assembled and then never
collected. Instrumenting the node's loop on the GenDrv bench, 2026-09-20:

    DBG loops=400 normal=0 wait=0 timeout=0 other=399 freq=0.00 pts=0

`other` is STOP. After the patch, on the same board and the same stream:

    DBG loops=400 normal=330 wait=69 timeout=0 other=0 freq=10.00 pts=456
    /scan  average rate: 8.309

`scripts/prepare_docker_vendor.sh` patches the staged copy. This test starts
the real driver as a UDP server on loopback, feeds it LD19 packets from a
client, and demands NORMAL -- so a re-vendor that drops the patch fails here
rather than on a robot with no /scan.
"""
import os
import shutil
import subprocess

import pytest

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PKG = os.path.join(REPO_ROOT, "docker", "vendor", "ldlidar_stl_ros2")
DRIVER = os.path.join(PKG, "ldlidar_driver")

HARNESS = r"""
#include "ldlidar_driver.h"
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cmath>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <thread>
using namespace ldlidar;

static uint64_t now_ns() {
    return (uint64_t)std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
}

static const uint16_t POINTS_PER_PACK = 12, POINTS_PER_REV = 456, SPEED_DPS = 3600;
static const float DEG_PER_POINT = 360.0f / 456.0f;

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

static uint16_t point_idx = 0, fake_ms = 0;
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
    put16(&pkt[44], fake_ms);
    pkt[46] = crc8(pkt, 46);
    point_idx += POINTS_PER_PACK;
    if (point_idx >= POINTS_PER_REV) point_idx -= POINTS_PER_REV;
    fake_ms = (uint16_t)((fake_ms + 3) % 30000);
}

int main(int argc, char **argv) {
    const char *port = (argc > 1) ? argv[1] : "18899";
    LDLidarDriver drv;
    drv.RegisterGetTimestampFunctional(now_ns);

    // Start() blocks until a client is heard, so the feeder runs alongside it.
    volatile bool stop = false;
    std::thread feeder([&]() {
        int fd = socket(AF_INET, SOCK_DGRAM, 0);
        sockaddr_in to{};
        to.sin_family = AF_INET;
        to.sin_port = htons((uint16_t)atoi(port));
        to.sin_addr.s_addr = inet_addr("127.0.0.1");
        while (!stop) {
            uint8_t dg[47 * 30];
            for (int i = 0; i < 30; i++) build_pack(dg + i * 47);
            sendto(fd, dg, sizeof(dg), 0, (sockaddr *)&to, sizeof(to));
            std::this_thread::sleep_for(std::chrono::milliseconds(25));
        }
        close(fd);
    });

    bool started = drv.Start(LDType::LD_19, "0.0.0.0", port, COMM_UDP_SERVER_MODE);
    int normal = 0, other = 0;
    double freq = -1.0;
    size_t pts = 0;
    if (started) {
        auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(6);
        while (std::chrono::steady_clock::now() < deadline && normal < 3) {
            Points2D scan;
            LidarStatus st = drv.GetLaserScanData(scan, 1500);
            if (st == LidarStatus::NORMAL) {
                normal++;
                pts = scan.size();
                drv.GetLidarScanFreq(freq);
            } else if (st != LidarStatus::DATA_WAIT) {
                other++;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
    }
    stop = true;
    feeder.join();
    drv.Stop();
    printf("%d %d %d %zu %.2f\n", started ? 1 : 0, normal, other, pts, freq);
    return 0;
}
"""


@pytest.fixture(scope="module")
def harness(tmp_path_factory):
    if not os.path.isdir(DRIVER):
        pytest.skip("ldlidar_stl_ros2 is not staged "
                    "(run scripts/prepare_docker_vendor.sh)")
    cxx = shutil.which("g++") or shutil.which("clang++")
    if not cxx:
        pytest.skip("no host C++ compiler")
    d = tmp_path_factory.mktemp("ld19net")
    src = os.path.join(d, "net.cpp")
    open(src, "w").write(HARNESS)
    exe = os.path.join(d, "net")
    cmd = [cxx, "-O0", "-w", "-std=c++17", "-o", exe, src,
           os.path.join(DRIVER, "src", "core", "ldlidar_driver.cpp"),
           os.path.join(DRIVER, "src", "dataprocess", "lipkg.cpp"),
           os.path.join(DRIVER, "src", "filter", "tofbf.cpp"),
           os.path.join(DRIVER, "src", "logger", "log_module.cpp"),
           os.path.join(DRIVER, "src", "networkcom",
                        "network_socket_interface_linux.cpp"),
           os.path.join(DRIVER, "src", "serialcom",
                        "serial_interface_linux.cpp")]
    for inc in ("core", "dataprocess", "filter", "logger", "networkcom", "serialcom"):
        cmd += ["-I", os.path.join(DRIVER, "include", inc)]
    cmd += ["-lpthread"]
    res = subprocess.run(cmd, capture_output=True, text=True)
    if res.returncode != 0:
        pytest.skip("cannot build the vendored driver here: %s"
                    % res.stderr.strip().splitlines()[-3:])
    return exe


def test_a_udp_server_lidar_delivers_scans(harness):
    res = subprocess.run([harness, "18899"], capture_output=True, text=True,
                         timeout=90, check=True)
    # The driver logs to stdout as well; our result is the last line.
    out = res.stdout.strip().splitlines()[-1].split()
    started, normal, other, points, freq = (int(out[0]), int(out[1]), int(out[2]),
                                            int(out[3]), float(out[4]))
    assert started, "the driver refused to start as a UDP server"
    assert other == 0, (
        "%d calls returned neither NORMAL nor DATA_WAIT -- LidarStatus::STOP, "
        "which demo.cpp's switch discards silently: the network Start() is not "
        "setting is_start_flag_" % other)
    assert normal >= 3, (
        "only %d scans in 6 s from a 10 Hz stream" % normal)
    assert points == 456, "a scan carried %d points, not 456" % points
    assert freq > 0, (
        "scan frequency %.2f -- ToLaserscanMessagePublish() skips the publish "
        "entirely when it is not positive, and GetLidarScanFreq() returns "
        "early on the same flag" % freq)


def test_the_patch_is_in_the_staged_source(harness):
    """The fix lives in scripts/prepare_docker_vendor.sh, so a re-vendor that
    stops applying it has to fail here rather than on a robot."""
    src = open(os.path.join(DRIVER, "src", "core", "ldlidar_driver.cpp")).read()
    assert src.count("is_start_flag_ = true;") >= 3, (
        "the network Start() is not setting is_start_flag_ -- did "
        "prepare_docker_vendor.sh fail to patch it?")
