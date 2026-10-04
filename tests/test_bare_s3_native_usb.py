"""The bare ESP32-S3 talks on its native USB, so that is where its config looks.

The S3 image is built with ARDUINO_USB_CDC_ON_BOOT=1: its console and micro-ROS go out
the chip's own USB (USB-Serial/JTAG, a CDC-ACM device -- /dev/ttyACM0) unless the env
key `console` says `uart0`. The generated bare config pointed every ESP32 at
/dev/ttyUSB0, a bridge chip, so a bare S3 on its native port was opened at a node
that carried nothing. A board whose only USB is a UART0 bridge -- the Yahboom
YB-EET01 -- keeps saying so in its own config. 2026-10-04.
"""
import os
import sys

import yaml

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO_ROOT, "scripts"))
import gen_bare_config  # noqa: E402
import gen_firmware_header  # noqa: E402
import mcu_env  # noqa: E402


def _bc(mcu):
    return gen_bare_config.bare_config(mcu)["base_controller"]


def test_bare_s3_defaults_to_native_usb():
    bc = _bc("esp32s3")
    assert bc["serial_port"] == "/dev/ttyACM0"
    assert bc["console"] == "usb"


def test_bare_s3_env_puts_the_console_on_usb():
    env = mcu_env.hardware_env(gen_bare_config.bare_config("esp32s3"))
    assert env.get("console") == "usb"


def test_bridged_esp32_and_the_rp2s_are_unchanged():
    assert _bc("esp32")["serial_port"] == "/dev/ttyUSB0"
    assert "console" not in _bc("esp32")
    for mcu in gen_firmware_header.RP2_MCUS:
        if mcu in gen_bare_config.BOARDS:
            assert _bc(mcu)["serial_port"] == "/dev/ttyACM0", mcu


def test_the_uart_option_stays_the_yahbooms():
    ref = os.path.join(REPO_ROOT, "config", "reference", "yb_eet01_config.yaml")
    bc = yaml.safe_load(open(ref))["base_controller"]
    assert bc["console"] == "uart0"
    assert bc["serial_port"] == "/dev/ttyUSB0"
