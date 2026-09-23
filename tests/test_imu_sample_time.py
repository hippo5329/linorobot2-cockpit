"""The IMU stamp is the sample's time, not the publisher's.

main.cpp takes ONE getTime() after every sensor on the bus has been polled and
puts it on odom, imu and mag alike. For the IMU that is the time the MCU got
round to publishing, and the error is not constant: it moves with bus traffic,
with how many optional sensors are fitted, and with whatever else the control
loop did that cycle. bringup.launch.py runs madgwick with constant_dt: 0.0, so
the filter integrates the gyro over the interval between those stamps, and the
EKF then takes the resulting heading as absolute (imu0_config[5]).

Two sources of the true sample time, best first: the chip's own timestamp
counter -- ICM-42670 (16-bit, ~1 us tick), LSM6DSOX (32-bit, 25 us), BMI270
(24-bit) all put one in the FIFO beside the sample -- and the DATA_RDY edge,
which the ISR times and which works on any chip with the line wired.
"""
import os
import re

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
IFACE_H = os.path.join(ROOT, "firmware", "common", "lib", "imu", "imu_interface.h")
IFACE_C = os.path.join(ROOT, "firmware", "common", "lib", "imu", "imu_interface.cpp")
MAIN = os.path.join(ROOT, "firmware", "src", "main.cpp")


def _read(p):
    with open(p, encoding="utf-8") as fh:
        return fh.read()


def test_the_chip_counter_is_the_only_source():
    """There were two: the chip's counter and the DATA_RDY edge. The interrupt
    path went on 2026-09-24 and the edge with it, because an ISR that cannot
    read the bus times an edge whose SAMPLE is then uncertain -- a precise time
    paired with the wrong reading."""
    h = _read(IFACE_H)
    blk = h[h.index("uint32_t sampleAgeUs()"):h.index("// Running statistics")]
    assert "chipSampleAgeUs()" in blk
    assert "sample_time_known_" not in blk, "the edge source is back"
    assert "micros()" not in blk, "the host clock is being used as a sample time again"


def test_an_age_not_a_counter():
    """The chip's oscillator is not the MCU's and drifts against it.

    An age is used across one sample interval, where even a 2% clock error is
    under a microsecond; a raw counter turned into a stamp needs a linear fit
    that has to be re-anchored forever.
    """
    h = _read(IFACE_H)
    assert "virtual uint32_t chipSampleAgeUs() { return 0; }" in h
    assert "An AGE, not the counter itself" in h


def test_a_driver_that_cannot_say_returns_zero_and_zero_means_no_correction():
    h, m = _read(IFACE_H), _read(MAIN)
    assert "virtual uint32_t chipSampleAgeUs() { return 0; }" in h
    # no pin, or a line that never fired: sampleAgeUs answers 0
    blk = h[h.index("uint32_t sampleAgeUs()"):h.index("// Running statistics")]
    assert "return 0;" in blk, "a driver with no counter must answer 0, not guess"
    # and main.cpp subtracting 0 leaves every existing board exactly as it was
    assert "- (int64_t)imu_age_us * 1000LL" in m


def test_the_age_is_clamped_so_a_fault_cannot_move_a_stamp_anywhere():
    h = _read(IFACE_H)
    assert "IMU_SAMPLE_AGE_MAX_US 50000" in h
    blk = h[h.index("uint32_t sampleAgeUs()"):h.index("// Running statistics")]
    assert blk.count("IMU_SAMPLE_AGE_MAX_US") >= 2, "the chip's counter must be clamped"


def test_the_report_does_not_wait_for_an_agent():
    """It hung off the data-ready verdict, which ran in publishData() -- so on a
    board whose console shares the micro-ROS port it was printed only after the
    session opened, into the XRCE stream, and was never readable. Reported from
    loop() now."""
    m = _read(MAIN)
    assert "static void reportSampleAge()" in m
    loop = m[m.index("void loop() {"):]
    loop = loop[:min(loop.index(t) for t in ("\nvoid ", "\nstatic ") if t in loop)]
    assert "reportSampleAge();" in loop


def test_the_imu_stamp_is_no_longer_odoms():
    m = _read(MAIN)
    # odom keeps the publish time; the IMU is dated back
    assert "odom_msg->header.stamp.nanosec = time_stamp.tv_nsec;" in m
    assert re.search(r"imu_msg->header\.stamp\.sec\s*=\s*\(int32_t\)\(imu_ns / 1000000000LL\)", m)
    assert "imu_msg->header.stamp.sec = time_stamp.tv_sec;" not in m


def test_a_stamp_is_never_pushed_before_the_epoch():
    """Before the agent's epoch sync lands, getTime() is small."""
    m = _read(MAIN)
    assert "if (imu_ns < 0) imu_ns = 0;" in m


def test_the_bench_can_see_the_correction_and_which_source_made_it():
    """A feature that cannot be observed cannot be verified."""
    h, m = _read(IFACE_H), _read(MAIN)
    for fn in ("noteSampleAge", "ageMinUs", "ageMaxUs", "ageMeanUs", "ageSource"):
        assert fn in h, fn
    assert "imu->noteSampleAge(imu_age_us);" in m
    assert "sample age from %s" in m
    # the spread, not just the mean: a constant age is a constant offset
    assert "%lu..%lu us, mean %lu" in m
    for src in ("chip timestamp", "none - stamped at publish"):
        assert src in h, src
    # The edge source must be gone from the CODE, not from the prose: the
    # comment explaining why it was removed is worth more than the silence, and
    # a check that forbids the word forbids the explanation too.
    for line in h.splitlines():
        if "DATA_RDY" in line:
            assert line.lstrip().startswith("//"), f"DATA_RDY in live code: {line.strip()}"
