"""Sim wheels no longer imply a sim IMU.

A bare custom board (Yahboom YB-EET01 on the bench: no encoders, a real
QMI8658) used to have its real IMU skipped entirely because the wheels were
simulated -- the measured /imu/data_raw was the simulation, and the driver
under test never ran. Now only the sensors that are themselves sim are
synthesised from the wheels, and a real one that fails to init on a
sim-wheel board falls back to the simulation with a loud line instead of
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
    assert "imu_from_wheels = sim_wheels && imu_is_sim;" in m
    assert 'mag_from_wheels = sim_wheels && (strcasecmp(mag_name, "sim") == 0);' in m
    # the real IMU is initialised and its DATA_RDY attached whenever it is not simulated
    assert re.search(r"if \(!imu_from_wheels\) \{\s*if \(!imu->init\(\)\)", m)


def test_a_bare_board_whose_imu_fails_keeps_running_on_the_simulation():
    m = _read(MAIN)
    assert "init FAILED on a sim-wheel board - falling back to the simulated IMU" in m
    assert "init FAILED on a sim-wheel board - falling back to the simulated field" in m
    blk = m[m.index("if (!imu->init())"):]
    # Bounded by the MAG init, which is the next thing setup() does. It used to
    # slice to the second `if (!imu_from_wheels) {` -- the data-ready attach block --
    # which vanished with the interrupt path on 2026-09-24, and the test then
    # failed inside its own slicing rather than on anything it asserts.
    blk = blk[:blk.index("if (!mag_from_wheels) {")]
    assert "if (sim_wheels) {" in blk and "imu_from_wheels = true;" in blk and "flashLED(3)" in blk


def test_publish_reads_the_real_sensor_when_only_the_wheels_are_sim():
    m = _read(MAIN)
    pub = m[m.index("void publishData()\n{"):]
    pub = pub[:pub.index("// Hard-iron offsets")]
    assert "if (imu_from_wheels) {" in pub and "sim_imu.apply(*imu_msg);" in pub
    assert "if (mag_from_wheels) {" in pub and "sim_imu.applyMag(*mag_msg);" in pub
    assert "*imu_msg = imu->getData();" in pub and "*mag_msg = mag->getData();" in pub
    assert "if (sim_wheels)" not in pub
    assert "if (!imu_from_wheels && imu) imu->applyEnvFrames();" in m


def test_the_probe_names_an_unknown_device_by_its_registers():
    p = _read(PROBE)
    assert 'unidentified: reg00=%02X reg0F=%02X reg75=%02X' in p
