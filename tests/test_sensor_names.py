"""One spelling for a sensor, from the YAML to the dropdown to the driver.

A sensor name is written down in three places that cannot import each other:
sensor_factory.cpp dispatches on it, the Sensors tab offers it, and the I2C
probe emits it as the name to adopt when the chip answers. Nothing links them,
so a driver can be added to the firmware and stay unreachable from the UI, or
an option can be offered that falls back to `fake` at boot with only a serial
line to say so.

These tests are that link.
"""
import os
import re

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FACTORY = os.path.join(REPO_ROOT, "firmware", "common", "lib", "sensor_factory",
                       "sensor_factory.cpp")
PROBE = os.path.join(REPO_ROOT, "firmware", "common", "lib", "i2c_probe", "i2c_probe.cpp")
INDEX = os.path.join(REPO_ROOT, "web", "frontend", "index.html")


def _read(path):
    with open(path) as fh:
        return fh.read()


def _factory_names(kind):
    """The names sensor_factory.cpp will dispatch on, uppercased."""
    return {m.upper() for m in re.findall(rf'\{{"(\w+)",\s*make{kind}<', _read(FACTORY))}


def _dropdown(select_id):
    """The values offered by one <select>, minus the NONE placeholder."""
    html = _read(INDEX)
    block = re.search(rf'id="{select_id}".*?</select>', html, re.S)
    assert block, f"no <select id={select_id}> in index.html"
    return set(re.findall(r'value="([A-Za-z0-9_]+)"', block.group(0))) - {"NONE"}


def test_every_imu_driver_is_offered_and_every_option_has_a_driver():
    assert _dropdown("cfg-imu") == _factory_names("IMU")


def test_every_mag_driver_is_offered_and_every_option_has_a_driver():
    assert _dropdown("cfg-mag") == _factory_names("MAG")


def test_the_probe_only_adopts_names_the_factory_knows():
    """i2cProbeSelect() overrides the config with the probe's `driver` string.

    If that string is not a factory row, detection silently downgrades a real
    sensor to the fake one -- the worst outcome available, because the bus was
    read correctly and the answer was then thrown away.
    """
    # Only the imu/mag roles go through sensor_factory. The "current" and "env"
    # devices are answered by battery.cpp and env.cpp, which probe for
    # themselves and have no name table.
    adopted = re.findall(r'sink\.add\(\s*\w+,\s*"(imu|mag)",\s*"[^"]*",\s*"(\w*)"',
                         _read(PROBE))
    drivers = {d.upper() for _, d in adopted if d}
    known = _factory_names("IMU") | _factory_names("MAG")
    assert drivers, "found no imu/mag rows in the probe at all"
    assert drivers <= known, f"probe adopts names no driver answers to: {drivers - known}"


def test_the_icm20948_is_registered_as_both_an_imu_and_a_magnetometer():
    """Its AK09916 is on the IMU's internal aux bus and never ACKs a bus scan.

    Probing alone would report a 9-axis part as 6-axis, leave mag_name at
    whatever the config said, and never publish /imu/mag. The probe therefore
    adds the magnetometer against the IMU's own address -- which is exactly how
    ICM20948MAG reaches it.
    """
    probe = _read(PROBE)
    block = re.search(r'who0 == 0xEA.*?return;', probe, re.S)
    assert block, "the ICM-20948 WHO_AM_I branch moved"
    roles = re.findall(r'sink\.add\(addr,\s*"(\w+)"', block.group(0))
    assert roles == ["imu", "mag"], roles
