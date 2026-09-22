"""The env partition: layout, CRC, and what a config turns into."""
import os
import zlib

import copy

import mcu_env
import yaml

from gen_bare_config import bare_config

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
REF = os.path.join(REPO_ROOT, "config", "reference")
SECRETS_EXAMPLE = os.path.join(REPO_ROOT, "config", "secrets.yaml.example")


def test_encode_decode_roundtrip():
    env = {"baud": "1500000", "wifi_ssid": "bench", "m1_pwm": "25", "node": "x_base_node"}
    blob = mcu_env.encode(env)
    assert len(blob) == mcu_env.ENV_SIZE
    assert mcu_env.decode(blob) == env


def test_layout_is_uboot_crc32_then_nul_separated_pairs():
    blob = mcu_env.encode({"a": "1", "b": "two"})
    crc = int.from_bytes(blob[:4], "little")
    assert crc == zlib.crc32(blob[4:]) & 0xFFFFFFFF
    body = blob[4:]
    assert body.startswith(b"a=1\0b=two\0\0")
    assert set(body[len(b"a=1\0b=two\0\0"):]) == {0xFF}


def test_corrupt_crc_is_rejected():
    import pytest
    blob = bytearray(mcu_env.encode({"a": "1"}))
    blob[0] ^= 0xFF
    # decode() is CLI-shaped: a bad CRC is a hard stop, not a value.
    with pytest.raises(SystemExit):
        mcu_env.decode(bytes(blob))


def _env(name):
    return mcu_env.env_from_config(os.path.join(REF, f"{name}_config.yaml"), SECRETS_EXAMPLE, "192.0.2.1")


def reference_params(name):
    with open(os.path.join(REF, f"{name}_config.yaml")) as fh:
        return yaml.safe_load(fh)


def _with(params, **sections):
    """A copy of `params` with one or more sub-dicts of base_controller merged."""
    out = copy.deepcopy(params)
    for key, patch in sections.items():
        out["base_controller"].setdefault(key, {}).update(patch)
    return out


def _env_from_params(params):
    import tempfile
    fd, path = tempfile.mkstemp(suffix="_config.yaml")
    with os.fdopen(fd, "w") as fh:
        yaml.safe_dump(params, fh, sort_keys=False)
    try:
        return mcu_env.env_from_config(path, SECRETS_EXAMPLE, "192.0.2.1")
    finally:
        os.unlink(path)


def _bare_env(mcu, tmp_path):
    """The generated bare config, through the same path as a shipped one.

    There is no bare *file* any more -- scripts/gen_bare_config.py is the rule
    -- so write the generated config out and read it back exactly as
    env_from_config would read a reference, rather than testing a dict.
    """
    path = tmp_path / f"bare_{mcu}_config.yaml"
    path.write_text(yaml.safe_dump(bare_config(mcu), sort_keys=False))
    return mcu_env.env_from_config(str(path), SECRETS_EXAMPLE, "192.0.2.1")


def test_gendrv_carries_the_boards_facts():
    env = _env("gendrv")
    assert env["baud"] == 921600
    assert env["transport"] == "serial"
    assert env["motor_driver"] == "bts7960"
    assert (env["m1_pwm"], env["m1_in_a"], env["m1_in_b"]) == (25, 21, 17)
    assert (env["m2_pwm"], env["m2_in_a"], env["m2_in_b"]) == (26, 22, 23)
    assert (env["m1_enc_a"], env["m1_enc_b"], env["m2_enc_a"], env["m2_enc_b"]) == (34, 35, 16, 27)
    assert (env["i2c_sda"], env["i2c_scl"]) == (32, 33)
    assert env["node"] == "gendrv_base_node"
    assert env["lidar_rx"] == 4 and env["lidar_baud"] == 230400
    assert "gpio_out" not in env, "the BTS7960 driver drives its own enable pins now"


def test_dual_core_key_follows_the_config():
    # Booleans reach the env as "1"/"0" strings; pins and rates as ints.
    # gendrv is the only shipped ESP32 reference now and it has dual core off,
    # so the "on" case is built rather than read from a file.
    assert str(_env("gendrv")["dual_core"]) == "0"         # use_dual_core: false
    params = reference_params("gendrv")
    params["base_controller"]["use_dual_core"] = True
    assert str(_env_from_params(params)["dual_core"]) == "1"
    # RP2 has no dual-core port, so the key never reaches the env at all.
    assert "dual_core" not in _env("pico2_mecanum")


def test_sensor_three_state_mapping():
    # gendrv declares every chip on the Waveshare board: publish flags are 1s.
    env = _env("gendrv")
    assert (env["pub_mag"], env["pub_battery"], env["pub_env"]) == (1, 1, 1)
    # pico2_mecanum declares no mag but does declare a battery divider and a
    # BMP280, so only pub_mag is off.
    env = _env("pico2_mecanum")
    assert (env["pub_mag"], env["pub_battery"], env["pub_env"]) == (0, 1, 1)


def test_bare_config_has_no_real_pins(tmp_path):
    """Generated, not stored -- and the rule holds on every board.

    This used to read rover_pico2_config.yaml. The bare designs are generated
    now (one rule, four boards), so assert the rule on all of them: nothing
    driven, nothing on the bus, every sensor simulated.
    """
    for mcu in ("pico", "pico2", "esp32", "esp32s3"):
        env = _bare_env(mcu, tmp_path)
        assert str(env["fake_wheel"]) == "1", mcu
        # There is no fake_imu key: the IMU is chosen by name at runtime
        # (sensor_factory), and "fake" is the name of the simulated driver.
        assert str(env["imu"]).lower() == "fake", mcu
        assert str(env["fake_ld19"]) == "1", mcu
        assert all(env[f"m{i}_pwm"] == -1 for i in range(1, 5)), mcu
        # No battery, no environmental sensor -- but the simulated magnetometer
        # IS published. use_fake_mag rotates a world field to the room heading
        # for one purpose, anchoring madgwick, and `mag: NONE` here means "no
        # chip", not "silence the simulation of one". The old expectation of
        # pub_mag == 0 was the bug: /imu/mag had no publisher, madgwick with a
        # mag waited forever, /imu/data went to 0 Hz and the EKF lost its IMU.
        assert (env["pub_mag"], env["pub_battery"], env["pub_env"]) == (1, 0, 0), mcu


def test_mecanum_reference_has_four_motors():
    env = _env("pico2_mecanum")
    assert env["base"] == "mecanum"
    for i in range(1, 5):
        assert env[f"m{i}_in_a"] >= 0 and env[f"m{i}_enc_a"] >= 0
    assert env["m1_cpr"] == 1320


def test_redact_hides_the_psk():
    env = {"wifi_psk": "hunter2", "wifi_ssid": "bench"}
    r = mcu_env.redact(env)
    assert r["wifi_psk"] != "hunter2" and r["wifi_ssid"] == "bench"


def test_battery_fake_lidar_and_geometry_reach_the_env():
    env = _env("pico2_mecanum")
    assert (env["battery_pin"], env["bat_r1"], env["bat_r2"]) == (26, 30000, 7500)
    assert env["fake_ld19"] == "0"          # a real LD19 on the robot computer
    assert "lidar_x" in env
    # gendrv drives a real LD19 now (the merge with gendrv_real), so flip the
    # flag on a copy: what is under test is that the emulator raycasts from the
    # config's LiDAR pose, not which board happens to ship with it on.
    fake = _env_from_params(_with(reference_params("gendrv"),
                                  sensors={"use_fake_ld19": True}))
    assert fake["fake_ld19"] == "1"
    assert float(fake["lidar_x"]) == 0.12   # the emulator raycasts from the config's LiDAR pose
    assert "pwm_min" not in fake and "pwm_max" not in fake   # derived from pwm_bits on the board


def test_fake_mode_overrides_a_config_that_names_real_hardware():
    """--mode fake means simulate what the bench lacks, whatever the YAML says.

    gendrv_config.yaml describes a real LD19 on GPIO 4 (use_fake_ld19: false).
    Under --mode fake the env still carried fake_ld19 0, the board emitted
    nothing on the LiDAR bridge, and /scan could never arrive -- both distros.
    """
    env = _env("gendrv")
    assert env["fake_ld19"] == "0" and env["imu"] != "fake"      # the config, as written
    changed = mcu_env.apply_sensor_mode(env, "fake")
    assert env["fake_ld19"] == "1" and env["fake_wheel"] == "1" and env["fake_env"] == "1"
    assert env["imu"] == "fake" and env["mag"] == "fake"
    assert "fake_ld19" in changed and "imu" in changed
    # the LED is not a sensor: fake mode drives the real one
    assert env.get("led") == mcu_env.env_from_config(
        os.path.join(REF, "gendrv_config.yaml"), SECRETS_EXAMPLE, "192.0.2.1").get("led")


def test_real_mode_restores_the_named_drivers_and_config_mode_is_a_no_op():
    env = _env("gendrv")
    mcu_env.apply_sensor_mode(env, "fake")
    mcu_env.apply_sensor_mode(env, "real", os.path.join(REF, "gendrv_config.yaml"))
    assert env["fake_ld19"] == "0" and env["fake_wheel"] == "0"
    assert env["imu"] == "qmi8658" and env["mag"] == "ak09918"
    untouched = _env("gendrv")
    assert mcu_env.apply_sensor_mode(dict(untouched), "config") == []
    assert mcu_env.apply_sensor_mode(dict(untouched), None) == []


def test_the_pipeline_and_the_flasher_carry_the_mode_to_the_env():
    pipe = open(os.path.join(REPO_ROOT, "scripts", "one_click_pipeline.py")).read()
    assert 'return {"fake": "fake", "real": "real"}.get(mode)' in pipe
    assert pipe.count('argv += ["--sensors", sensors]') == 2, "both flash paths must forward it"
    assert "sensors=sensors_for_mode(args.mode)" in pipe
    flash = open(os.path.join(REPO_ROOT, "scripts", "flash_mcu.py")).read()
    assert 'cmd += ["--sensors", sensors]' in flash
