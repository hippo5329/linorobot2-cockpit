"""The env partition: layout, CRC, and what a config turns into."""
import os
import zlib

import mcu_env

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


def test_gendrv_real_carries_the_boards_facts():
    env = _env("gendrv_real")
    assert env["baud"] == 1500000
    assert env["transport"] == "serial"
    assert env["motor_driver"] == "bts7960"
    assert (env["m1_pwm"], env["m1_in_a"], env["m1_in_b"]) == (25, 21, 17)
    assert (env["m2_pwm"], env["m2_in_a"], env["m2_in_b"]) == (26, 22, 23)
    assert (env["m1_enc_a"], env["m1_enc_b"], env["m2_enc_a"], env["m2_enc_b"]) == (34, 35, 16, 27)
    assert (env["i2c_sda"], env["i2c_scl"]) == (32, 33)
    assert env["node"] == "gendrv_real_base_node"
    assert env["lidar_rx"] == 4 and env["lidar_baud"] == 230400
    assert "gpio_out" not in env, "the BTS7960 driver drives its own enable pins now"


def test_dual_core_key_follows_the_config():
    # Booleans reach the env as "1"/"0" strings; pins and rates as ints.
    assert str(_env("gendrv_real")["dual_core"]) == "1"
    assert str(_env("esp32_wifi")["dual_core"]) == "0"
    assert "dual_core" not in _env("pico")


def test_sensor_three_state_mapping():
    # gendrv_real declares every chip: publish flags are explicit 1s.
    env = _env("gendrv_real")
    assert (env["pub_mag"], env["pub_battery"], env["pub_env"]) == (1, 1, 1)
    # rover_pico2 says NONE for mag/current/env: explicit 0s.
    env = _env("rover_pico2")
    assert (env["pub_mag"], env["pub_battery"], env["pub_env"]) == (0, 0, 0)


def test_fake_reference_has_no_real_pins():
    env = _env("rover_pico2")
    assert str(env["fake_wheel"]) == "1"
    assert all(env[f"m{i}_pwm"] == -1 for i in range(1, 5))


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
    fake = _env("gendrv")
    assert fake["fake_ld19"] == "1"
    assert float(fake["lidar_x"]) == 0.12   # the emulator raycasts from the config's LiDAR pose
    assert "pwm_min" not in fake and "pwm_max" not in fake   # derived from pwm_bits on the board
