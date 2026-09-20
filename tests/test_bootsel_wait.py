"""Waiting for BOOTSEL is a condition, not a duration.

The board reaches BOOTSEL in well under a second -- the host logs
`Product: RP2350 Boot` immediately. What takes longer is everything between
this process and the USB device: on the bench, Incus hot-plugging the
re-enumerated device into the container, measured at ~2 s. The old code slept a
flat 2.5 s and then tried picotool exactly once, so a board that was sitting in
BOOTSEL was reported as "No accessible RP-series devices in BOOTSEL mode were
found" and the flash failed.
"""
import os
import subprocess
import sys

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO_ROOT, "scripts"))

import flash_mcu  # noqa: E402


class _Res:
    def __init__(self, rc):
        self.returncode = rc
        self.stdout = b""
        self.stderr = b""


def test_it_returns_as_soon_as_picotool_can_see_the_board(monkeypatch):
    """The point of the change: no fixed wait once the device is there."""
    calls = {"n": 0}

    def fake_run(argv, **kw):
        calls["n"] += 1
        return _Res(0)

    monkeypatch.setattr(flash_mcu, "find_picotool_binaries", lambda: ["/usr/bin/picotool"])
    monkeypatch.setattr(subprocess, "run", fake_run)
    assert flash_mcu.wait_for_bootsel(timeout_s=5.0) is True
    assert calls["n"] == 1, "should stop at the first success, not keep polling"


def test_it_keeps_looking_while_the_device_is_still_attaching(monkeypatch):
    """The whole bug: one look was not enough."""
    seq = [1, 1, 1, 0]          # fails three times, then the device appears
    calls = {"n": 0}

    def fake_run(argv, **kw):
        rc = seq[min(calls["n"], len(seq) - 1)]
        calls["n"] += 1
        return _Res(rc)

    monkeypatch.setattr(flash_mcu, "find_picotool_binaries", lambda: ["/usr/bin/picotool"])
    monkeypatch.setattr(subprocess, "run", fake_run)
    assert flash_mcu.wait_for_bootsel(timeout_s=10.0) is True
    assert calls["n"] == 4


def test_it_gives_up_rather_than_hanging(monkeypatch):
    monkeypatch.setattr(flash_mcu, "find_picotool_binaries", lambda: ["/usr/bin/picotool"])
    monkeypatch.setattr(subprocess, "run", lambda argv, **kw: _Res(1))
    assert flash_mcu.wait_for_bootsel(timeout_s=1.0) is False


def test_with_no_picotool_it_falls_back_to_the_interface_classes(monkeypatch):
    """sysfs cannot prove this process may OPEN the device, but it can prove a
    board is there -- which is better than nothing when picotool is missing."""
    monkeypatch.setattr(flash_mcu, "find_picotool_binaries", lambda: [])
    monkeypatch.setattr(flash_mcu, "rp2_usb_mode", lambda: "bootsel")
    assert flash_mcu.wait_for_bootsel(timeout_s=2.0) is True


def test_the_touch_no_longer_naps_a_fixed_two_and_a_half_seconds():
    """Guard against the flat sleep coming back."""
    src = open(os.path.join(REPO_ROOT, "scripts", "flash_mcu.py")).read()
    body = src[src.index("def pulse_1200_baud"): src.index("# USB interface classes")]
    assert "time.sleep(2.5)" not in body, "the fixed re-enumeration nap is back"
    assert "wait_for_bootsel()" in body
