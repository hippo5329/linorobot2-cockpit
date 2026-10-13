"""The banner names the board's wireless chip on RP2 W images (radio=).

arduino-pico's CheckPicoW() assumes a W whenever it cannot tell, so "isPicoW" proves
nothing. On 2026-10-13 a bench board recorded as a Pico 2 W never answered its radio
query, while a real one answered CYW43439 rev 5 with a Raspberry Pi MAC. The banner
says which, and the parser hands it on; older banners still parse, with no radio.
"""
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "scripts"))

import mcu_probe  # noqa: E402

BASE = ("[fw] linorobot2_hardware app=base distro=jazzy built=2026-10-13 git=51641eb "
        "uid=3CC50D3C7E503DE6 envcrc=1A2B3C4D")


def test_a_radio_that_answered():
    b = mcu_probe.parse_banner(BASE + " radio=cyw43439r5:2CCF67E7AFA0\n")
    assert b["board_id"] == "3CC50D3C7E503DE6"
    assert b["radio"] == "cyw43439r5:2CCF67E7AFA0"


def test_no_w_and_no_answer_are_different_answers():
    assert mcu_probe.parse_banner(BASE + " radio=none\n")["radio"] == "none"
    assert mcu_probe.parse_banner(BASE + " radio=noanswer(-6)\n")["radio"] == "noanswer(-6)"


def test_an_older_banner_has_no_radio_and_still_parses():
    b = mcu_probe.parse_banner(BASE + "\n")
    assert b["git"] == "51641eb" and b["radio"] is None


def test_the_radio_of_the_last_boot_only():
    text = BASE + " radio=noanswer(-6)\nnoise radio=bogus\n" + BASE + "\n"
    assert mcu_probe.parse_banner(text)["radio"] is None


def test_the_firmware_writes_the_field_on_rp2_w_images_only():
    src = open(os.path.join(ROOT, "firmware", "src", "main.cpp")).read()
    assert "static void radioField(char *buf, size_t n)" in src
    assert src.index("#if defined(PICO_CYW43_SUPPORTED)\nextern bool __isPicoW;") < src.index("static void formatBanner")
    assert "radioField(radio, sizeof(radio));" in src
