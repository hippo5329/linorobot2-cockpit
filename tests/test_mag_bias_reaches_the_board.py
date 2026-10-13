"""The Sensors tab's hard-iron bias reaches a release image, in the firmware's units.

The tab's three fields save `sensors.mag_bias` in uT. Until 2026-10-13 only a local
build read that key (MAG_BIAS), and it compiled the uT numbers into a subtraction
from a field in tesla; the env -- all a release image reads -- came only from
`imu_tuning.mag_bias`. A bias typed on the tab never reached a prebuilt board.
"""
import copy
import os
import re
import sys

import pytest
import yaml

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO_ROOT, "scripts"))

import mcu_env  # noqa: E402


def _params(name="gendrv"):
    with open(os.path.join(REPO_ROOT, "config", "reference", f"{name}_config.yaml")) as fh:
        return yaml.safe_load(fh)


def _bias(params):
    return [float(v) for v in mcu_env.hardware_env(params)["mag_bias"].split(",")]


def test_the_sensors_tab_bias_reaches_the_env_in_tesla():
    p = copy.deepcopy(_params())
    p["base_controller"].setdefault("sensors", {})["mag_bias"] = [20.0, -4.5, 1.25]
    p["base_controller"].pop("imu_tuning", None)
    assert _bias(p) == pytest.approx([20e-6, -4.5e-6, 1.25e-6], rel=1e-6)


def test_imu_tuning_wins_over_the_tab():
    p = copy.deepcopy(_params())
    p["base_controller"].setdefault("sensors", {})["mag_bias"] = [20.0, 0.0, 0.0]
    p["base_controller"]["imu_tuning"] = {"mag_bias": [1e-6, 2e-6, 3e-6]}
    assert _bias(p) == pytest.approx([1e-6, 2e-6, 3e-6], rel=1e-6)


def test_no_bias_anywhere_writes_none():
    p = copy.deepcopy(_params())
    p["base_controller"].get("sensors", {}).pop("mag_bias", None)
    p["base_controller"].pop("imu_tuning", None)
    assert "mag_bias" not in mcu_env.hardware_env(p)


def test_every_reference_design_names_a_chip_the_firmware_has():
    """pico2_mecanum said `mag: AK09916` -- the ICM-20948's magnetometer, but not a
    driver name: the board got an unknown mag. Every design's imu/mag must be a key
    of the firmware's tables (or none/auto)."""
    ref = os.path.join(REPO_ROOT, "config", "reference")
    tables = {}
    src = open(os.path.join(REPO_ROOT, "firmware", "common", "lib", "sensor_factory",
                            "sensor_factory.cpp")).read()
    starts = [m.start() for m in re.finditer(r"\{\"sim\",", src)]
    assert len(starts) >= 2, "expected an IMU table and a MAG table"
    for kind, start in zip(("imu", "mag"), starts[:2]):
        block = src[start:src.index("};", start)]
        tables[kind] = set(re.findall(r'\{"([a-z0-9_]+)",', block))
    bad = []
    for fn in sorted(os.listdir(ref)):
        if not fn.endswith("_config.yaml"):
            continue
        with open(os.path.join(ref, fn)) as fh:
            p = yaml.safe_load(fh) or {}
        env = mcu_env.hardware_env(p)
        for kind in ("imu", "mag"):
            v = env.get(kind)
            if v in (None, "none", "auto", "sim"):
                continue
            if v not in tables[kind]:
                bad.append(f"{fn}: {kind} {v!r}")
    assert not bad, bad
