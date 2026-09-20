"""Covariance and the simulated world are configuration, not build constants.

Ported from linorobot2_hardware's robot_config_engine, which carries all of
this in its spec and emits compile-time macros. Here it goes in the ENV block,
because in this project a robot is a configuration and not a build: one
released image has to serve a board with an MPU6050 and a board with a BNO085,
whose accelerometer variances differ by a factor of seven.

The firmware keeps its `#ifndef` defaults as the fallback -- `envFloatVec()`
leaves them alone when a key is absent -- so a blank env still boots with sane
values. A scalar expands to every axis, which is the config engine's rule and
worth keeping: most people have one number, and the ones who measured per-axis
values must not have to average them.

Key names follow the config engine's schema.json exactly (`robot_mass`,
`wheel_noise_rpm`, `map_width`, `wall_x1`, …) so a spec can move between the
two projects.
"""
import copy
import os
import sys
import tempfile

import pytest
import yaml

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO_ROOT, "scripts"))

import mcu_env  # noqa: E402
from gen_bare_config import bare_config  # noqa: E402


def _env(cfg):
    d = tempfile.mkdtemp()
    secrets = os.path.join(d, "secrets.yaml")
    open(secrets, "w").write("{}\n")
    params = os.path.join(d, "robot.yaml")
    open(params, "w").write(yaml.safe_dump(cfg))
    return mcu_env.env_from_config(params, secrets)


def _with(**controller_keys):
    cfg = copy.deepcopy(bare_config("esp32"))
    cfg["base_controller"].update(controller_keys)
    return cfg


def test_a_named_imu_brings_its_datasheet_variance():
    """Without this a config that names its IMU still ships the firmware's
    1e-5 placeholder, which tells the EKF a cheap MPU6050 is as trustworthy as
    a BNO085."""
    cfg = _with()
    cfg["base_controller"]["sensors"]["imu"] = "MPU6050"
    env = _env(cfg)
    assert env["accel_cov"] == "0.0015"
    assert env["gyro_cov"] == "3e-06"


def test_a_sensor_without_orientation_gets_no_orientation_variance():
    cfg = _with()
    cfg["base_controller"]["sensors"]["imu"] = "MPU6050"
    assert "ori_cov" not in _env(cfg)
    cfg["base_controller"]["sensors"]["imu"] = "BNO085"
    assert _env(cfg)["ori_cov"] == "0.004"


def test_an_explicit_value_beats_the_datasheet():
    cfg = _with(imu_tuning={"accel_cov": 0.5})
    cfg["base_controller"]["sensors"]["imu"] = "MPU6050"
    assert _env(cfg)["accel_cov"] == "0.5"


def test_a_scalar_is_left_for_the_firmware_to_expand():
    """One number means "the same on every axis"; the firmware's envFloatVec
    does the expanding, so the env stays small."""
    env = _env(_with(imu_tuning={"twist_cov": 0.001}))
    assert env["twist_cov"] == "0.001"


def test_a_full_vector_is_written_in_order():
    env = _env(_with(imu_tuning={"pose_cov": [1, 2, 3, 4, 5, 6]}))
    assert env["pose_cov"] == "1,2,3,4,5,6"


def test_a_wrong_length_vector_is_refused():
    """Silently zero-filling a covariance would tell the EKF the robot is
    perfectly certain about the axes the user forgot."""
    assert "pose_cov" not in _env(_with(imu_tuning={"pose_cov": [1, 2, 3]}))


def test_an_all_zero_bias_is_not_a_calibration():
    assert "mag_bias" not in _env(_with(imu_tuning={"mag_bias": [0, 0, 0]}))
    assert "mag_bias" not in _env(_with(imu_tuning={"mag_bias": [1.0, 2.0]}))
    assert _env(_with(imu_tuning={"mag_bias": [1.5, -2.25, 0.75]}))["mag_bias"] \
        == "1.5,-2.25,0.75"


def test_the_barometer_address_can_be_named():
    """0x77 on the Waveshare General Driver board, 0x76 on most breakouts."""
    assert _env(_with(bmp280_addr="0x76"))["bmp280_addr"] == 0x76
    assert "bmp280_addr" not in _env(_with())


def test_the_simulated_room_is_configurable():
    """Fake mode is this project's default, so the room the emulator raycasts
    is configuration: a Nav2 test wants the obstacle wall somewhere else
    without rebuilding."""
    env = _env(_with(simulation={"map_width": 8.0, "wall_obstacle": 0,
                                 "wall_x1": 1.25, "robot_mass": 12,
                                 "wheel_noise_rpm": 0.25}))
    assert env["fake_map_w"] == "8"
    assert env["fake_wall"] == 0
    assert env["fake_wall_x1"] == "1.25"
    assert env["fake_mass"] == "12"
    assert env["fake_noise_rpm"] == "0.25"


def test_an_unconfigured_robot_adds_none_of_these_keys():
    """Every one of them costs bytes in a 4 KB partition, and the firmware's
    own default is the right answer when the config is silent."""
    env = _env(_with())
    for key in ("accel_cov", "gyro_cov", "ori_cov", "mag_cov", "pose_cov",
                "twist_cov", "env_cov", "mag_bias", "bmp280_addr",
                "fake_map_w", "fake_wall", "fake_mass", "fake_noise_rpm"):
        assert key not in env, f"{key} was written for a config that never asked"


def test_the_key_names_match_the_config_engines_schema():
    """So a spec can move between the two projects without translation."""
    schema = os.path.expanduser(
        "~/code/linorobot2_hardware/tools/robot_config_engine/schema.json")
    if not os.path.isfile(schema):
        pytest.skip("the config engine is not checked out beside this repo")
    import json
    props = json.load(open(schema))["properties"]["simulation"]["properties"]
    for key in ("robot_mass", "wheel_noise_rpm", "map_width", "map_height",
                "wall_obstacle", "wall_x1", "wall_y1", "wall_x2", "wall_y2"):
        assert key in props, f"{key} is not what the config engine calls it"
