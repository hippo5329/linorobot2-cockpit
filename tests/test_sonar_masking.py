"""A simulated robot must not drive a real sonar.

Two bare Picos came up on the bench announcing an HC-SR04 on GP27/GP28 and
publishing /sonar at 8.45 Hz. Neither had one. Two causes, both fixed:

  - the release was built from a WIRED reference (pico2_mecanum wires a sonar
    there), so the header's TRIG_PIN/ECHO_PIN were the fallback for any config
    that said nothing;
  - mcu_env omitted the keys entirely when a config had no sonar, so there was
    nothing to override that fallback with.

And the rule that follows: in fake mode the sonar is the simulated cone, not
the pins -- otherwise /sonar and /scan describe two different worlds.
"""
import os
import re
import sys

import yaml

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO_ROOT, "scripts"))

import mcu_env                      # noqa: E402
from gen_bare_config import bare_config, bare_pins   # noqa: E402

SECRETS = os.path.join(REPO_ROOT, "config", "secrets.yaml.example")
MAIN = os.path.join(REPO_ROOT, "firmware", "src", "main.cpp")
RANGE = os.path.join(REPO_ROOT, "firmware", "common", "lib", "range", "range.cpp")


def _env(params, tmp_path, name="x"):
    p = tmp_path / f"{name}_config.yaml"
    p.write_text(yaml.safe_dump(params, sort_keys=False))
    return mcu_env.env_from_config(str(p), SECRETS, "192.0.2.1")


def test_the_bare_module_has_no_sonar_pins():
    assert bare_pins()["sonar"] == {"trigger": -1, "echo": -1}


def test_a_bare_config_says_minus_one_rather_than_staying_silent(tmp_path):
    """Silence let the firmware fall back to the image's compiled pins."""
    for mcu in ("pico", "pico2", "esp32", "esp32s3"):
        env = _env(bare_config(mcu), tmp_path, mcu)
        assert env["sonar_trig"] == -1, mcu
        assert env["sonar_echo"] == -1, mcu


def test_a_config_with_no_sonar_block_still_writes_the_keys(tmp_path):
    params = yaml.safe_load(open(os.path.join(REPO_ROOT, "config", "reference",
                                              "gendrv_config.yaml")))
    assert "sonar" not in (params["base_controller"]["pins"]), "gendrv grew a sonar"
    env = _env(params, tmp_path, "gendrv")
    assert (env["sonar_trig"], env["sonar_echo"]) == (-1, -1)


def test_a_wired_sonar_still_reaches_the_board(tmp_path):
    params = yaml.safe_load(open(os.path.join(REPO_ROOT, "config", "reference",
                                              "pico2_mecanum_config.yaml")))
    env = _env(params, tmp_path, "mecanum")
    assert (env["sonar_trig"], env["sonar_echo"]) == (27, 28)


def test_fake_mode_drops_the_pins_before_anything_is_configured():
    """initRange(allow_hardware=false) must zero them, not merely skip publishing."""
    src = open(RANGE).read()
    body = src[src.index("void initRange("):]
    assert "if (!allow_hardware)" in body
    guard = body[body.index("if (!allow_hardware)"):]
    assert guard.index("trig_pin = -1") < guard.index("pinMode(trig_pin")
    assert guard.index("echo_pin = -1") < guard.index("attachInterrupt")


def test_main_masks_the_sonar_whenever_anything_is_simulated():
    src = open(MAIN).read()
    m = re.search(r"const bool sonar_faked = ([^;]+);", src)
    assert m, "the fake-mode mask is gone"
    assert "fake_wheels" in m.group(1) and "fake_ld19" in m.group(1)
    assert "initRange(!sonar_faked)" in src


def test_the_bare_scan_sink_follows_the_silicon():
    """An RP2 can carry the scan over micro-ROS; an ESP32 cannot.

    Measured on the bench 2026-09-20. RP2: /odom and /imu at 50.0 Hz with
    /raw_scan at 85-100 Hz alongside. ESP32, same configuration: every topic
    drops to 40-45 Hz, and 33 Hz on a GenDrv that also reads four I2C sensors,
    because one 921600 link is carrying both. So a bare ESP32 defaults to the
    UART sink -- with no LIDAR_RXD wired it simply has no scan, which is honest
    -- rather than to a mode that halves its control rate.
    """
    from gen_bare_config import bare_config
    for mcu in ("pico", "pico2"):
        assert bare_config(mcu)["base_controller"]["lidar"]["comm_mode"] == "topic", mcu
    for mcu in ("esp32", "esp32s3"):
        assert bare_config(mcu)["base_controller"]["lidar"]["comm_mode"] == "serial", mcu
