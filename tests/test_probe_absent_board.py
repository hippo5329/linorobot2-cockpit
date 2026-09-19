"""A board that is mid-re-enumeration must not be reported, or treated, as absent.

The probe runs moments after the previous ROS stack was torn down and the serial
port released. On that boundary an RP2 can be off the USB bus for a second or
two. Answering "absent" there is not a harmless inaccuracy: "absent" is the one
verdict that makes one_click_pipeline write NOTHING -- not the firmware, not the
4 KB env block -- so the run proceeds to test whatever image and whatever env
the board happens to be carrying, and says "Nothing to write: the board already
runs this build with this config" while doing it.

That is what happened to a pico2 bench in the rc-20260919 release test. Its env
block had never been written, so `fake_wheel` was not set, so the firmware's
pose reset at the micro-ROS session did not fire, so the emulator was still
parked where an earlier run had left it: odom x=4.800 against a room wall five
metres out, /scan minimum 0.20 m. Nav2 accepted the goal and then issued 41
consecutive zero-velocity commands, because there was nowhere to go. Every
symptom pointed at Nav2 or at the firmware.
"""
import os
import re
import sys

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO_ROOT, "scripts"))


def test_usb_mode_waits_for_a_board_that_is_still_coming_back(monkeypatch):
    import flash_mcu
    import mcu_probe

    calls = []

    def flaky():
        calls.append(1)
        return "absent" if len(calls) < 3 else "app"

    monkeypatch.setattr(flash_mcu, "rp2_usb_mode", flaky)
    monkeypatch.setattr(mcu_probe, "PROBE_ENUMERATE_WAIT", 5.0)

    assert mcu_probe.usb_mode("pico2", "/dev/ttyACM0") == "app"
    assert len(calls) >= 3, "usb_mode gave up on the first answer"


def test_usb_mode_still_gives_up_on_a_board_that_is_really_gone(monkeypatch):
    """The wait must be bounded: an unplugged board has to report absent."""
    import time

    import flash_mcu
    import mcu_probe

    monkeypatch.setattr(flash_mcu, "rp2_usb_mode", lambda: "absent")
    monkeypatch.setattr(mcu_probe, "PROBE_ENUMERATE_WAIT", 1.0)

    started = time.time()
    assert mcu_probe.usb_mode("pico2", "/dev/ttyACM0") == "absent"
    assert time.time() - started < 10, "usb_mode never gave up"


def test_an_absent_board_is_flashed_rather_than_assumed_correct():
    """`absent` and `no_firmware` both mean "the probe cannot account for this board".

    If it really is unplugged the flash fails and the pipeline halts saying so,
    which is the honest outcome; silently testing an unknown board is not.
    """
    text = open(os.path.join(REPO_ROOT, "scripts", "one_click_pipeline.py")).read()
    m = re.search(r"blank_board\s*=\s*bool\(board and board\.get\(\"verdict\"\)(.*?)\)\n", text, re.S)
    assert m, "blank_board is no longer derived from the probe verdict"
    assert "absent" in m.group(1), (
        "an absent board no longer counts as blank, so the pipeline will write "
        "neither firmware nor env and will test whatever the board is carrying"
    )
