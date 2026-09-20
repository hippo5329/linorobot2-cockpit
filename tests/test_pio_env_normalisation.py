"""A board name is not a PlatformIO env.

The USB probe answers with the board it can see -- `gendrv` for a CP2102N --
and the Sensors tab puts that answer in cfg-mcu. The flash path then used it as
the env, so on a GenDrv the cockpit asked GitHub for
`linorobot2-firmware-gendrv-jazzy.tar.gz`, got a 404, and firmware upload was
broken on the only ESP32 board with a reference design in this repo.
"""
import os
import re
import sys

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO_ROOT, "scripts"))

import mcu_identity  # noqa: E402


def _pio_envs():
    ini = os.path.join(REPO_ROOT, "firmware", "platformio.ini")
    return set(re.findall(r"^\[env:([^\]]+)\]", open(ini).read(), re.M))


def test_a_board_name_resolves_to_the_env_of_its_silicon():
    assert mcu_identity.pio_env_for("gendrv") == "esp32"


def test_a_real_env_is_returned_unchanged():
    for env in ("esp32", "pico2", "esp32s3", "pico2_lyrical", "esp32_lyrical"):
        assert mcu_identity.pio_env_for(env) == env, env


def test_the_result_is_always_something_platformio_has():
    envs = _pio_envs()
    for name in ("gendrv", "esp32", "pico2", "", None, "not-a-board"):
        assert mcu_identity.pio_env_for(name, "pico2") in envs, name


def test_every_name_the_usb_probe_can_emit_maps_to_an_env():
    """classify_usb() is the only source of these hints, so enumerate it."""
    envs = _pio_envs()
    hints = set()
    for vid, pid in (("2e8a", "000a"), ("2e8a", "000f"), ("10c4", "ea60"),
                     ("1a86", "7523"), ("0403", "6001"), ("303a", "1001")):
        family, _label, _decisive = mcu_identity.classify_usb(vid, pid)
        if family:
            hints.add(family)
    assert hints, "classify_usb recognised nothing at all"
    for h in hints:
        assert mcu_identity.pio_env_for(h, "pico2") in envs, h
