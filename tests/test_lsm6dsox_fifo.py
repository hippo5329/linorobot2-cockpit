"""The LSM6DSOX is read through its FIFO, so a sample and its time match.

Reading the output registers takes three transactions -- accel, gyro, then
TIMESTAMP0 -- and nothing ties them together, so at a 104 Hz ODR the timestamp
can belong to a later sample than the data. The FIFO tags every word and
interleaves a timestamp word, so both come out of ONE read, latched by the part.

This is also the first implementation of chipSampleAgeUs() in the project. The
hook existed on IMUInterface and every driver inherited `return 0`, so the stamp
correction it feeds has never actually run on any board -- ageSource() would say
"none - stamped at publish" on every run to date.

Register values confirmed against ST's own lsm6dsox_reg.h, not from memory:
FIFO_CTRL3 0x09 (BDR_XL 3:0, BDR_GY 7:4), FIFO_CTRL4 0x0A (FIFO_MODE 2:0,
ODR_TS 7:6), FIFO_STATUS1 0x3A / STATUS2 0x3B with DIFF_FIFO split 7:0 + 9:8 and
FIFO_OVR_IA at bit 6, FIFO_DATA_OUT_TAG 0x78 with TAG_SENSOR in bits 7:3, and
tag values 1 = gyro, 2 = accel, 4 = timestamp, STREAM_MODE = 6, 104 Hz BDR = 4.
"""
import os
import re

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
IMU_H = os.path.join(ROOT, "firmware", "common", "lib", "imu", "default_imu.h")


def _cls():
    s = open(IMU_H, encoding="utf-8").read()
    start = s.index("class LSM6DSOXIMU")
    return s[start:s.index("\nclass ", start + 10)]


def _fn(name):
    cls = _cls()
    start = cls.index(name)
    i = cls.index("{", start)
    depth, j = 0, i
    while True:
        if cls[j] == "{":
            depth += 1
        elif cls[j] == "}":
            depth -= 1
            if depth == 0:
                break
        j += 1
    return cls[start:j + 1]


def test_the_registers_match_the_datasheet():
    cls = _cls()
    for name, addr in (("REG_FIFO_CTRL3", "0x09"), ("REG_FIFO_CTRL4", "0x0A"),
                       ("REG_FIFO_STATUS1", "0x3A"), ("REG_FIFO_STATUS2", "0x3B"),
                       ("REG_FIFO_TAG", "0x78")):
        assert re.search(rf"{name}\s*=\s*{addr}", cls), f"{name} is not {addr}"
    for name, val in (("TAG_GYRO", "0x01"), ("TAG_ACCEL", "0x02"), ("TAG_TIMESTAMP", "0x04")):
        assert re.search(rf"{name}\s*=\s*{val}", cls), f"{name} is not {val}"


def test_the_fifo_is_configured_for_continuous_mode_with_timestamps():
    start = _fn("bool startSensor()")
    assert "writeReg(REG_FIFO_CTRL3, 0x44)" in start, "BDR is not 104 Hz on both sensors"
    assert "writeReg(REG_FIFO_CTRL4, 0x46)" in start, "not continuous mode with ODR_TS batching"
    # the timestamp counter must be running or every word's time is zero
    assert "writeReg(REG_CTRL10_C, 0x20)" in start, "TIMESTAMP_EN is not set"


def test_the_batch_rate_matches_the_sensor_rate():
    """Batching faster than the sensors produce pads the FIFO with repeats;
    slower discards samples inside the chip, where nothing can see it."""
    start = _fn("bool startSensor()")
    assert "writeReg(REG_CTRL1_XL, 0x40)" in start   # accel 104 Hz
    assert "writeReg(REG_CTRL2_G,  0x4C)" in start   # gyro 104 Hz
    assert "0x44" in start                           # BDR 104 Hz for both


def test_the_tag_is_taken_from_the_top_five_bits():
    """TAG_SENSOR is bits 7:3 -- bits 2:1 are TAG_CNT and bit 0 is parity, so a
    driver comparing the raw byte would match nothing."""
    body = _fn("int drainFifo()")
    assert "w[0] >> 3" in body, "the tag byte is compared without shifting off TAG_CNT and parity"


def test_the_timestamp_is_little_endian_over_four_bytes():
    body = _fn("int drainFifo()")
    assert re.search(r"w\[1\].*w\[2\] << 8.*w\[3\] << 16.*w\[4\] << 24", body, re.S), \
        "the timestamp word is not assembled little-endian"


def test_an_overrun_is_counted_rather_than_ignored():
    """Continuous mode drops the OLDEST when it fills, which is the right end to
    lose from -- but silently losing samples is how a rate looks healthy while
    the data has holes."""
    body = _fn("int drainFifo()")
    assert "0x40" in body, "FIFO_OVR_IA is never checked"
    assert "fifo_overruns_++" in body


def test_the_drain_is_bounded():
    """One stall must not turn into an unbounded burst of I2C in the publish
    path. The FIFO holds 512 words."""
    body = _fn("int drainFifo()")
    assert re.search(r"if \(words > \d+\)", body), "the drain is unbounded"


def test_only_one_read_per_publish():
    """IMUInterface::getData() calls readGyroscope() then readAccelerometer().
    Draining in both would read the FIFO twice per sample and hand the
    accelerometer a different sample from the gyro -- the same straddling this
    change exists to remove."""
    gyro = _fn("geometry_msgs__msg__Vector3 readGyroscope() override")
    accel = _fn("geometry_msgs__msg__Vector3 readAccelerometer() override")
    assert "drainFifo()" in gyro
    assert "drainFifo()" not in accel, "the FIFO is drained twice per sample"
    assert "return accel_;" in accel


def test_the_register_path_survives_as_a_fallback():
    """If the FIFO was never configured -- a part that answered WHO_AM_I and then
    failed its writes -- the driver must still read something rather than publish
    the same stale struct for ever."""
    gyro = _fn("geometry_msgs__msg__Vector3 readGyroscope() override")
    accel = _fn("geometry_msgs__msg__Vector3 readAccelerometer() override")
    assert "REG_OUTX_L_G" in gyro and "REG_OUTX_L_A" in accel
    assert "fifo_ok_" in gyro and "fifo_ok_" in accel


def test_the_age_comes_from_the_chips_own_clock_on_both_sides():
    """now and the sample time must come from the SAME counter, or the answer is
    a host/chip clock difference rather than an age."""
    body = _fn("uint32_t chipSampleAgeUs() override")
    assert "readTimestampRaw()" in body
    assert "sample_ts_raw_" in body
    assert "* 25u" in body, "the 25 us tick is not applied"
    # and it must answer 0 rather than guess when nothing has been drained
    assert "!sample_ts_raw_" in body


def test_the_age_multiply_cannot_overflow_into_a_plausible_number():
    body = _fn("uint32_t chipSampleAgeUs() override")
    assert "0xFFFFFFFFu / 25u" in body, "a garbage counter read can overflow the multiply"


def test_this_is_the_first_chip_timestamp_source_in_the_project():
    """Recorded because it changes what ageSource() reports: every run to date
    said "none - stamped at publish"."""
    s = open(IMU_H, encoding="utf-8").read()
    assert s.count("uint32_t chipSampleAgeUs() override") == 1, \
        "another driver implements it now -- update this note and the docs"
