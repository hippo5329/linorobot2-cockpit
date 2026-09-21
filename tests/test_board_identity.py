"""The board says which board it is, and the key says what was identified.

Two identical boards on one bench are indistinguishable from their output, which
has cost real time. The boot banner now carries an identity field — but only two
of the three families can name their own silicon, so the KEY carries the claim:

    uid=      the SILICON's id. RP2350 reads chip info from ROM; ESP32 has the
              48-bit eFuse MAC, burned per chip.
    flashid=  the external FLASH chip's id. An RP2040 has nothing else:
              pico_get_unique_board_id() there returns flash_get_unique_id().
              It names this board, but it moves when the flash is replaced.

Merging them would make a flash swap read as the same chip. They stay apart.
"""
import os
import re
import sys

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO_ROOT, "scripts"))

import mcu_probe  # noqa: E402

MAIN_CPP = os.path.join(REPO_ROOT, "firmware", "src", "main.cpp")

BANNER = "[fw] linorobot2_hardware app={app} distro=jazzy built=2026-09-21 git=e52c98a{tail}"


def test_a_uid_banner_parses_as_silicon():
    b = mcu_probe.parse_banner(BANNER.format(app="base", tail=" uid=E66138935F63A22A"))
    assert b["id_kind"] == "uid"
    assert b["board_id"] == "E66138935F63A22A"


def test_a_flashid_banner_parses_as_the_flash_chip():
    b = mcu_probe.parse_banner(BANNER.format(app="base", tail=" flashid=E6614103E769A52B"))
    assert b["id_kind"] == "flashid", "an RP2040's id must not be reported as a chip uid"
    assert b["board_id"] == "E6614103E769A52B"


def test_a_banner_without_an_identity_still_parses():
    """An older image emits no such field; that is not a parse failure."""
    b = mcu_probe.parse_banner(BANNER.format(app="base", tail=""))
    assert b["app"] == "base" and b["git"] == "e52c98a"
    assert b.get("board_id") is None and b.get("id_kind") is None


def test_the_identity_is_appended_so_the_trailing_note_still_parses():
    b = mcu_probe.parse_banner(
        BANNER.format(app="base", tail=" uid=7CDFA1B2C3D4")
        + " (env blank or invalid - using header defaults)")
    assert b["board_id"] == "7CDFA1B2C3D4"


def test_the_last_banner_wins_when_a_board_rebooted():
    text = (BANNER.format(app="base", tail=" uid=AAAA00000001") + "\n"
            + BANNER.format(app="i2c_detect", tail=" uid=AAAA00000001"))
    assert mcu_probe.parse_banner(text)["app"] == "i2c_detect"


def read_main():
    with open(MAIN_CPP, encoding="utf-8") as fh:
        return fh.read()


def test_rp2040_emits_flashid_and_never_uid():
    """The whole point: an RP2040 must not claim a silicon id."""
    src = read_main()
    fn = re.search(r"static void identityField\(.*?\n\}", src, re.S)
    assert fn, "identityField() is gone; the banner needs one place that decides this"
    body = fn.group(0)
    rp2040 = body.split("ARDUINO_ARCH_RP2040", 1)
    assert len(rp2040) == 2, "no RP2040 branch"
    assert "flashid=" in rp2040[1], "the RP2040 branch must emit flashid="
    assert "uid=" not in rp2040[1].split("#else", 1)[0], (
        "the RP2040 branch must never emit uid= -- its id belongs to the flash chip"
    )


def test_rp2350_and_esp32_emit_uid():
    body = re.search(r"static void identityField\(.*?\n\}", read_main(), re.S).group(0)
    esp = body.split("#if defined(ESP32)", 1)[1].split("#elif", 1)[0]
    assert "uid=" in esp and "getEfuseMac" in esp
    rp2350 = body.split("ARDUINO_ARCH_RP2350", 1)[1].split("#elif", 1)[0]
    assert "uid=" in rp2350


def test_the_banner_appends_the_identity_after_git():
    """Order matters: an older host parsing a newer board must still match."""
    src = read_main()
    fmt = re.search(r'Serial\.printf\("\\n\[fw\] linorobot2_hardware ([^"]*)"', src)
    assert fmt, "the banner format string moved"
    assert fmt.group(1).index("git=%s") < fmt.group(1).rindex("%s"), (
        "the identity field must come after git=, not be inserted before it"
    )
