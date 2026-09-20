"""Fields the config panel shows must actually reach the board.

Seven controls on the Base & MCU / Sensors panels were rendered by
`web/frontend/index.html`, read by no JavaScript, and written by nothing: you
could type in them, save, reload, and the value was gone. Each names a real
firmware feature, and for three of them the path did not exist on the backend
either -- the firmware read a key the tooling never wrote.

  cfg-sonar          the enable select; the pins were saved, the choice was not
  cfg-bat-nom        nominal pack voltage
  cfg-bat-cap-val    ADC filter capacitor
  cfg-dac-pin        the DAC pin adc_calibrate sweeps -- read by the firmware
                     as envU16("dac_pin", DAC_PIN), written by no one
  cfg-mag-bias-x/y/z hard-iron offsets -- main.cpp has carried
                     `#ifdef MAG_BIAS` for a long time and nothing defined it

These cover the two backend halves. The frontend half is in app.js.
"""
import copy
import os
import sys
import tempfile

import yaml

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO_ROOT, "scripts"))

import gen_firmware_header  # noqa: E402
import mcu_env  # noqa: E402
from gen_bare_config import bare_config  # noqa: E402


def _env_for(cfg):
    d = tempfile.mkdtemp()
    secrets = os.path.join(d, "secrets.yaml")
    open(secrets, "w").write("{}\n")
    params = os.path.join(d, "robot.yaml")
    open(params, "w").write(yaml.safe_dump(cfg))
    return mcu_env.env_from_config(params, secrets)


def test_the_dac_pin_reaches_the_env():
    """adc_calibrate.cpp reads dac_pin; until now nothing wrote it, so the
    selector on the ADC panel could not change what the tool swept."""
    cfg = copy.deepcopy(bare_config("esp32"))
    cfg["base_controller"].setdefault("pins", {})["dac"] = 26
    assert _env_for(cfg).get("dac_pin") == 26


def test_no_dac_pin_means_the_key_is_absent():
    """Absent, not -1: the firmware's own default is the fallback, and an env
    that names every key it does not have is how a bare board ends up driving
    a pin nobody wired."""
    cfg = bare_config("esp32")
    assert "dac_pin" not in _env_for(cfg)
    cfg2 = copy.deepcopy(cfg)
    cfg2["base_controller"].setdefault("pins", {})["dac"] = -1
    assert "dac_pin" not in _env_for(cfg2)


def _header(cfg):
    return gen_firmware_header.generate_header(
        cfg, {}, cfg["base_controller"]["name"])


def test_mag_bias_is_emitted_when_the_config_carries_it():
    cfg = copy.deepcopy(bare_config("pico2"))
    cfg["base_controller"].setdefault("sensors", {})["mag_bias"] = [1.5, -2.25, 0.75]
    header = _header(cfg)
    assert "#define MAG_BIAS {1.5f, -2.25f, 0.75f}" in header


def test_an_uncalibrated_robot_gets_no_correction():
    """The `#ifdef` in main.cpp is the point: a robot that has never been
    calibrated must not have a bias subtracted from its magnetometer."""
    assert "MAG_BIAS" not in _header(bare_config("pico2"))
    cfg = copy.deepcopy(bare_config("pico2"))
    cfg["base_controller"].setdefault("sensors", {})["mag_bias"] = [0.0, 0.0, 0.0]
    assert "MAG_BIAS" not in _header(cfg), "all-zero bias is not a calibration"


def test_a_malformed_bias_does_not_break_the_build():
    cfg = copy.deepcopy(bare_config("pico2"))
    cfg["base_controller"].setdefault("sensors", {})["mag_bias"] = ["x", None, 3]
    assert "MAG_BIAS" not in _header(cfg)
    cfg["base_controller"]["sensors"]["mag_bias"] = [1.0, 2.0]
    assert "MAG_BIAS" not in _header(cfg), "three axes or nothing"
