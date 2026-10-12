"""A tool on a real robot never creates a simulated IMU or magnetometer for "none".

createIMU()/createMAG() return a simulated part for any name they do not know, "none"
included; `base` never asks them for "none" (main.cpp). The tools did: on a mecanum robot
with `mag: none`, test_sensors printed "[+] Magnetometer initialized successfully" and
streamed a simulated field, and with `imu: none` test_acc reported a simulated IMU's
acceleration as IMU ACC (2026-10-12). Every tool now asks sensorFitted() first and creates
nothing for "none". test_sensors also prints RANGE none for a sonar that is not fitted
(it printed the seeded max range, 4.84 m, as if clear).
"""
import os
import re

import pytest

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TOOLS_DIR = os.path.join(REPO_ROOT, "firmware", "src", "tools")
FACTORY = os.path.join(REPO_ROOT, "firmware", "common", "lib", "sensor_factory", "sensor_factory.cpp")


def _src(name):
    with open(os.path.join(TOOLS_DIR, name + ".cpp"), encoding="utf-8") as f:
        return f.read()


def test_sensor_fitted_is_false_for_none_and_blank():
    with open(FACTORY, encoding="utf-8") as f:
        body = f.read()
    m = re.search(r"bool sensorFitted\(const char \*name\)\s*\{(.*?)\n\}", body, re.S)
    assert m, "sensorFitted() missing from the sensor factory"
    assert 'strcasecmp(name, "none") != 0' in m.group(1)
    assert "name && *name" in m.group(1)


@pytest.mark.parametrize("tool", ["test_sensors", "test_acc", "test_motors"])
@pytest.mark.parametrize("kind", ["IMU", "MAG"])
def test_every_create_is_behind_sensor_fitted(tool, kind):
    src = _src(tool)
    calls = [m.start() for m in re.finditer(r"= create%s\(" % kind, src)]
    assert calls, f"{tool} creates no {kind}"
    for pos in calls:
        cond = src[src.rfind("if (", 0, pos):pos]          # the nearest condition above the call
        cond = cond[:cond.find("\n")] if "\n" in cond else cond
        assert "sensorFitted(" in cond, f"{tool}: create{kind}() not guarded by sensorFitted()"


@pytest.mark.parametrize("tool", ["test_sensors", "test_acc"])
def test_reads_are_guarded(tool):
    src = _src(tool)
    for m in re.finditer(r"\b(imu|mag)->getData\(\)", src):
        line_start = src.rfind("\n", 0, m.start())
        ctx = src[max(0, line_start - 300):m.start()]
        assert re.search(r"if \(!?%s\b|if \(%s\)|!%s \|\|" % ((m.group(1),) * 3), ctx), \
            f"{tool}: {m.group(0)} at {m.start()} without a null check"


def test_test_sensors_prints_range_none_without_a_sonar():
    src = _src("test_sensors")
    assert "RANGE: none" in src and "rangePresent()" in src


def test_test_acc_says_none_without_an_imu():
    assert 'IMU ACC none (no IMU)' in _src("test_acc")
