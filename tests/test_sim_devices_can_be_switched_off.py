"""In Sim mode every device is simulated by default, and the user may switch any one off.

User, 2026-10-06: "in sim mode, default enable all sim devices, allow user to disable some
devices, eg, no imu no mag" / "no sonar, no battery, ... etc." Only a bare module runs in Sim
mode, so a device it does not simulate cannot be real: it is ABSENT. The wheels always are
simulated -- the base must move.
"""
import os
import sys

import pytest
import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "scripts"))

import gen_bare_config  # noqa: E402
import mcu_env  # noqa: E402


def sim_env(tmp_path, **off):
    cfg = gen_bare_config.bare_config("esp32")
    cfg["robot"]["name"] = "my_rover"
    for k in off:
        cfg["base_controller"]["sensors"][f"use_sim_{k}"] = False
    path = tmp_path / "my_rover_config.yaml"
    path.write_text(yaml.safe_dump(cfg))
    env = mcu_env.hardware_env(cfg)
    mcu_env.apply_sensor_mode(env, "sim", str(path))
    return cfg, path, env


def test_by_default_every_device_is_simulated(tmp_path):
    _, _, env = sim_env(tmp_path)
    assert env["imu"] == "sim" and env["mag"] == "sim"
    assert env["sim_wheel"] == env["sim_ld19"] == env["sim_env"] == env["sim_battery"] == "1"


def test_no_imu_means_no_magnetometer_either(tmp_path):
    cfg, path, env = sim_env(tmp_path, imu=True)
    assert env["imu"] == "none" and env["mag"] == "none"
    assert not mcu_env.robot_has_imu(cfg, "sim", str(path))   # the EKF and the topic gate agree


def test_no_magnetometer_keeps_the_imu(tmp_path):
    _, _, env = sim_env(tmp_path, mag=True)
    assert env["imu"] == "sim" and env["mag"] == "none" and env["pub_mag"] == 0


@pytest.mark.parametrize("device, key", [("env", "sim_env"), ("battery", "sim_battery"),
                                         ("ld19", "sim_ld19"), ("sonar", "sim_sonar")])
def test_any_device_can_be_switched_off(tmp_path, device, key):
    _, _, env = sim_env(tmp_path, **{device: True})
    assert str(env[key]) == "0", (device, env.get(key))


def test_the_wheels_always_are_simulated(tmp_path):
    _, _, env = sim_env(tmp_path, wheel=True)
    assert env["sim_wheel"] == "1"


def test_switching_devices_off_keeps_it_a_bare_module(tmp_path):
    import one_click_pipeline as ocp
    cfg, _, _ = sim_env(tmp_path, imu=True, mag=True, sonar=True, battery=True)
    assert ocp.is_bare_config(cfg) and not ocp.is_real_robot(cfg)


def test_an_absent_device_has_no_topic(tmp_path):
    """User, 2026-10-06: "when a device is absent, the corresponding topic is disabled"."""
    import depth_camera
    # the firmware's own publishers
    _, _, env = sim_env(tmp_path, imu=True, env=True, battery=True)
    assert env["imu"] == "none" and env["pub_env"] == 0 and env["pub_battery"] == 0
    _, _, env = sim_env(tmp_path, mag=True)
    assert env["pub_mag"] == 0
    # the LiDAR: no driver, nothing waits for /scan
    cfg, _, _ = sim_env(tmp_path, ld19=True)
    assert depth_camera.scan_source(cfg["base_controller"]) is None
    cfg, _, _ = sim_env(tmp_path)
    assert depth_camera.scan_source(cfg["base_controller"]) == "lidar"
