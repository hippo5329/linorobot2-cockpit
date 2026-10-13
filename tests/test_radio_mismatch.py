"""The flash refuses a W config on a board whose radio did not answer, and the other way.

Every RP2 image is built from a W env, so every RP2 board enumerates as a W and
arduino-pico assumes one when it cannot tell. Given the same image, a Pico 2 W answered
cyw43439r5 and a plain Pico 2 answered noanswer(-6) -- the only evidence there is. A
config that disagrees with it is a mix-up.
"""
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "scripts"))

import flash_mcu  # noqa: E402

W = "cyw43439r5:2CCF67E7AFA0"


def test_a_w_config_on_a_board_with_no_radio():
    msg = flash_mcu.radio_mismatch("pico2w", "noanswer(-6)")
    assert msg and msg.startswith("[NO RADIO]") and "'pico2'" in msg
    assert flash_mcu.radio_mismatch("xrp", "none").startswith("[NO RADIO]")
    assert flash_mcu.radio_mismatch("picow", "noanswer(-1)").startswith("[NO RADIO]")


def test_a_plain_config_on_a_w():
    msg = flash_mcu.radio_mismatch("pico2", W)
    assert msg and msg.startswith("[IS A W]") and "'pico2w'" in msg
    assert flash_mcu.radio_mismatch("pico", W).startswith("[IS A W]")


def test_agreement_and_silence_pass():
    assert flash_mcu.radio_mismatch("pico2w", W) is None
    assert flash_mcu.radio_mismatch("xrp", "cyw43439r5:28CDC1198E66") is None
    assert flash_mcu.radio_mismatch("pico2", "noanswer(-6)") is None
    assert flash_mcu.radio_mismatch("pico2", "none") is None
    assert flash_mcu.radio_mismatch("pico2w", None) is None      # an older image: no radio= field
    assert flash_mcu.radio_mismatch("esp32", "anything") is None  # not an RP2


def test_config_mcu_reads_the_robot_config(tmp_path):
    p = tmp_path / "r_config.yaml"
    p.write_text("base_controller:\n  mcu: pico2w\n")
    assert flash_mcu.config_mcu(str(p)) == "pico2w"
    assert flash_mcu.config_mcu(str(tmp_path / "missing.yaml")) is None
