"""Two faults the Yahboom real-sensor legs found on 2026-10-06 (real wheels, no motors fitted).

1. /odom/unfiltered ran away to x = -4.7e11 m. createEncoder() builds a RealEncoder with
   `new`; heap memory is not zeroed, and the ESP32 and Pico Encoder classes never set
   prev_update_time_ / prev_encoder_ticks_. The first getRPM() subtracted a garbage tick
   count over a garbage interval -- one enormous, finite RPM that the odometry integrated
   once and kept. The GenDrv, same class, happened to get zeroed heap.
2. imu: AUTO reported "no IMU" on a board that had just printed its gyro bias. The check
   waited on /odom -- the EKF on the robot computer, up from the start of bringup -- not on
   the board's /odom/unfiltered, then gave /imu/data one second.
"""
import os
import re

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ENCODER_H = os.path.join(ROOT, "firmware", "common", "lib", "encoder", "encoder.h")
VERIFY = os.path.join(ROOT, "scripts", "verify_topics.py")


def _class_body(src, guard):
    a = src.index(guard)
    return src[a:src.index("#elif", a + len(guard)) if "#elif" in src[a + len(guard):] else len(src)]


def test_every_encoder_starts_from_now_and_zero_ticks():
    src = open(ENCODER_H).read()
    for guard in ("#ifdef ESP32", "#elif defined(PICO)", "#elif defined(LINO_ZEPHYR)"):
        body = _class_body(src, guard)
        assert re.search(r"unsigned long prev_update_time_\s*=\s*0;", body), guard
        assert re.search(r"int(32|64)_t prev_encoder_ticks_\s*=\s*0;", body), guard
        assert "prev_update_time_ = micros();" in body, f"{guard}: the first interval must start at construction"


def test_imu_auto_waits_for_the_board_not_the_ekf():
    src = open(VERIFY).read()
    a = src.index("if args.imu_auto and not args.no_imu:")
    block = src[a:src.index("imu_absent_note = (", a)]
    code = "\n".join(ln for ln in block.splitlines() if not ln.strip().startswith("#"))
    assert 'count_publishers("/odom/unfiltered")' in code
    assert 'count_publishers("/odom")' not in code
    assert "IMU_AUTO_GRACE_S" in code


def test_the_grace_sits_past_the_gap_that_was_measured():
    import sys
    sys.path.insert(0, os.path.join(ROOT, "scripts"))
    src = open(VERIFY).read()
    grace = float(re.search(r"^IMU_AUTO_GRACE_S = ([0-9.]+)", src, re.M).group(1))
    wait = float(re.search(r"^IMU_AUTO_BOARD_WAIT_S = ([0-9.]+)", src, re.M).group(1))
    assert grace >= 5.0          # the gap was over 1 s; the old bound was 1 s
    assert wait >= 10.0
