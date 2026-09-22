"""Fake wheels no longer imply a fake IMU.

A bare custom board (Yahboom YB-EET01 on the bench: no encoders, a real
QMI8658) used to have its real IMU skipped entirely because the wheels were
simulated -- the measured /imu/data_raw was the simulation, and the driver
under test never ran. Now only the sensors that are themselves fake are
synthesised from the wheels, and a real one that fails to init on a
fake-wheel board falls back to the simulation with a loud line instead of
the fatal LED loop.
"""
import os
import re

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MAIN = os.path.join(ROOT, "firmware", "src", "main.cpp")
PROBE = os.path.join(ROOT, "firmware", "common", "lib", "i2c_probe", "i2c_probe.cpp")


def _read(p):
    with open(p, encoding="utf-8") as fh:
        return fh.read()


def test_the_simulation_follows_the_sensor_not_the_wheels():
    m = _read(MAIN)
    assert "sim_imu = fake_wheels && imu_is_fake;" in m
    assert 'sim_mag = fake_wheels && (strcasecmp(mag_name, "fake") == 0);' in m
    # the real IMU is initialised and its DATA_RDY attached whenever it is not simulated
    assert re.search(r"if \(!sim_imu\) \{\s*if \(!imu->init\(\)\)", m)
    assert re.search(r"if \(!sim_imu\) \{\s*// The data-ready line", m)


def test_a_bare_board_whose_imu_fails_keeps_running_on_the_simulation():
    m = _read(MAIN)
    assert "init FAILED on a fake-wheel board - falling back to the simulated IMU" in m
    assert "init FAILED on a fake-wheel board - falling back to the simulated field" in m
    blk = m[m.index("if (!imu->init())"):]
    blk = blk[:blk.index("if (!sim_imu) {", 1)]
    assert "if (fake_wheels) {" in blk and "sim_imu = true;" in blk and "flashLED(3)" in blk


def test_publish_reads_the_real_sensor_when_only_the_wheels_are_fake():
    m = _read(MAIN)
    pub = m[m.index("void publishData()"):]
    pub = pub[:pub.index("// Hard-iron offsets")]
    assert "if (sim_imu) {" in pub and "fake_imu.apply(*imu_msg);" in pub
    assert "if (sim_mag) {" in pub and "fake_imu.applyMag(*mag_msg);" in pub
    assert "*imu_msg = imu->getData();" in pub and "*mag_msg = mag->getData();" in pub
    assert "if (fake_wheels)" not in pub
    assert "if (!sim_imu && imu) imu->applyEnvFrames();" in m


def test_the_probe_names_an_unknown_device_by_its_registers():
    p = _read(PROBE)
    assert 'unidentified: reg00=%02X reg0F=%02X reg75=%02X' in p
