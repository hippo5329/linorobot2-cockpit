"""If the fake magnetometer is fused, it must also be published.

bringup.launch.py decides whether madgwick fuses /imu/mag; scripts/mcu_env.py
decides, at flash time, whether the firmware publishes it. They were derived
from different fields and disagreed: `mag: NONE` (no chip) set pub_mag=0 while
`use_fake_mag: true` made the launcher fuse it. Madgwick then waited forever
for a synchronised imu/data_raw + imu/mag pair, published nothing, and the EKF
lost its IMU. Measured 2026-09-22 on a bare Pico 2: /imu/mag 0 publishers,
/imu/data 0 Hz, "Still waiting for data on topics imu/data_raw and imu/mag".
"""
import os
import re
import sys

import yaml

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO_ROOT, "scripts"))
import mcu_env  # noqa: E402

LAUNCH = os.path.join(REPO_ROOT, "launchers", "bringup.launch.py")
REF = os.path.join(REPO_ROOT, "config", "reference")


def env_for(sensors):
    params = {"robot": {"name": "t"}, "base_controller": {"name": "pico2", "sensors": sensors}}
    return mcu_env.hardware_env(params)


def launcher_use_mag(mag_sensor, use_fake_mag):
    src = open(LAUNCH, encoding="utf-8").read()
    m = re.search(r"else:\s*\n(?:\s*#.*\n)*\s*use_mag\s*=\s*(.+)", src)
    return bool(eval(m.group(1).strip(), {}, {"mag_sensor": mag_sensor, "use_fake_mag": use_fake_mag}))


def test_fake_mag_on_a_bare_module_is_published():
    env = env_for({"imu": "FAKE", "mag": "NONE", "use_fake_imu": True, "use_fake_mag": True})
    assert int(env.get("pub_mag", 0)) == 1, "mag: NONE must not silence the simulated magnetometer"


def test_no_mag_and_no_fake_stays_silent():
    env = env_for({"imu": "NONE", "mag": "NONE", "use_fake_mag": False})
    assert int(env.get("pub_mag", 0)) == 0


def test_launcher_and_env_agree_on_every_reference_config():
    """The fusion decision and the publish decision derive from the same facts."""
    for f in sorted(os.listdir(REF)):
        if not f.endswith("_config.yaml"):
            continue
        with open(os.path.join(REF, f)) as fh:
            params = yaml.safe_load(fh) or {}
        sensors = (params.get("base_controller") or {}).get("sensors") or {}
        mag = str(sensors.get("mag", "NONE")).upper()
        fake = bool(sensors.get("use_fake_mag", False))
        if mag == "AUTO":
            continue                      # the bus decides; nothing to compare statically
        env = env_for(sensors)
        fused = launcher_use_mag(mag, fake)
        published = int(env.get("pub_mag", 0)) == 1
        assert fused == published, (
            f"{f}: launcher fuses /imu/mag={fused} but the env publishes it={published} "
            f"(mag={mag}, use_fake_mag={fake}) -- madgwick would wait forever or a real mag would be ignored"
        )
