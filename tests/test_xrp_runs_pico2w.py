"""The SparkFun XRP Controller runs the Pico 2 W image, with an XRP env block.

There is no XRP image: the Pico 2 images are built for the RP2350B's 48 GPIO
(firmware/variants/rpipico2*_lino), and what the XRP wires differently from a
Pico 2 W -- the RM2 radio on GP26-29, the IMU's I2C1 pair on GP38/39, the
battery on GP46 -- reaches the firmware in the env. These pin that down: an
`xrp` env or release profile coming back, or the XRP's env landing anywhere but
pico2w's sector, would be a second image for one silicon.
"""
import os
import re
import sys

import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "scripts"))

import fetch_prebuilt  # noqa: E402
import mcu_env  # noqa: E402
import mcu_identity  # noqa: E402

XRP = os.path.join(ROOT, "config", "reference", "xrp_config.yaml")


def _xrp():
    with open(XRP) as fh:
        return yaml.safe_load(fh)


def test_the_xrp_controller_builds_and_downloads_the_pico2w_image():
    env = mcu_identity.pio_env_for("xrp")
    assert env == "pico2w"
    assert fetch_prebuilt.profile_for_env(env) == "pico2-jazzy"
    assert fetch_prebuilt.profile_for_env(env + "_lyrical") == "pico2-lyrical"


def test_there_is_no_xrp_env_or_release_profile():
    with open(os.path.join(ROOT, "firmware", "platformio.ini")) as fh:
        envs = re.findall(r"^\[env:([^\]]+)\]", fh.read(), re.M)
    assert not [e for e in envs if "xrp" in e]
    with open(os.path.join(ROOT, ".github", "workflows", "release.yml")) as fh:
        release = yaml.safe_load(fh)
    profiles = release["jobs"]["firmware"]["strategy"]["matrix"]["profile"]
    assert not [p for p in profiles if "xrp" in p]
    assert "pico2-jazzy" in profiles and "pico2-lyrical" in profiles


def test_the_xrp_env_block_carries_its_radio_and_i2c1_pins():
    env = mcu_env.hardware_env(_xrp())
    assert env["cyw43_pins"] == "26,29,28,27"        # REG_ON, DATA, CLOCK, CS
    assert (env["i2c_sda"], env["i2c_scl"]) == (38, 39)
    assert ((38 >> 1) & 1, (39 >> 1) & 1) == (1, 1)  # an I2C1 pair
    assert env["battery_pin"] == 46


def test_a_pico2w_env_block_leaves_the_radio_at_its_defaults():
    with open(os.path.join(ROOT, "config", "reference", "pico2_mecanum_config.yaml")) as fh:
        env = mcu_env.hardware_env(yaml.safe_load(fh))
    assert "cyw43_pins" not in env


def test_the_xrp_env_goes_to_the_pico2w_sector():
    # The image keeps its EEPROM sector at 4 MB whatever the chip holds; a write
    # at the XRP's 16 MB top would land where nothing reads it.
    assert "xrp" not in mcu_env.RP2_ENV_OFFSETS
    assert mcu_env.RP2_ENV_OFFSETS[mcu_identity.pio_env_for("xrp")] == 0x103FF000


def test_an_xrp_on_sparkfun_firmware_is_an_rp2350_like_a_pico2():
    family, label, decisive = mcu_identity.classify_usb("1b4f", "0046", "")
    assert family == mcu_identity.env_family("pico2w") == "pico2"
    assert "XRP" in label and decisive


def test_the_pico2_variants_number_the_rp2350b_gpio():
    for v in ("rpipico2_lino", "rpipico2w_lino"):
        with open(os.path.join(ROOT, "firmware", "variants", v, "pins_arduino.h")) as fh:
            src = fh.read()
        assert re.search(r"^#define PICO_RP2350A 0\b", src, re.M), v
        assert "#include <generic/common.h>" in src, v
    with open(os.path.join(ROOT, "firmware", "variants", "rpipico2w_lino", "init.cpp")) as fh:
        init = fh.read()
    # the radio's pins are set from the env before the driver starts
    assert init.index("lino_cyw43_pins()") < init.index("init_cyw43_wifi()")
