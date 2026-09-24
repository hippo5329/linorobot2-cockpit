"""An absent port is not evidence about any board.

/sys is the HOST's inside a container, so /sys/class/tty/ttyACM0 resolves
whether or not this box has a ttyACM0 -- and on a bench with more than one
board it resolves to somebody else's. A board in BOOTSEL has no tty at all,
which is exactly when an image is about to be written.

Measured: a box whose own RP2040 was sitting in BOOTSEL, ready to flash, asked
about its absent /dev/ttyACM0 and was told "Raspberry Pi Pico 2 (RP2350)",
decisive -- the other box's board, on the other side of the machine. The run
was refused with an MCU mismatch against a board that was correct.

The hazard only appears when the name EXISTS in the host's sysfs while the
device node does not, so that is what these set up.
"""
import os
import sys

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO_ROOT, "scripts"))

import mcu_identity  # noqa: E402

PORT = "/dev/ttyACM0"
NAMESAKE = "/sys/class/tty/ttyACM0/device"


def _bench(monkeypatch):
    """This box has no ttyACM0; the host has one, and it is a Pico 2."""
    real_exists = os.path.exists

    def stub_exists(path):
        if path == PORT:
            return False
        if path == NAMESAKE:
            return True
        return real_exists(path)

    monkeypatch.setattr(mcu_identity.os.path, "exists", stub_exists)
    monkeypatch.setattr(mcu_identity, "read_usb_ids",
                        lambda p: ("2e8a", "000f", "Pico 2"))


def test_an_absent_port_does_not_borrow_its_namesake(monkeypatch):
    _bench(monkeypatch)
    assert mcu_identity.sysfs_device_link(PORT) == "", (
        "a missing port fell through to the name-based sysfs path, which "
        "describes whichever board the HOST happens to have at that name"
    )


def test_an_absent_port_identifies_nothing(monkeypatch):
    _bench(monkeypatch)
    family, chip, decisive = mcu_identity.identify_port(PORT)
    assert (family, decisive) == (None, False), (
        f"an absent port claimed to be {chip!r} (decisive={decisive})"
    )


def test_an_absent_port_cannot_block_a_flash(monkeypatch):
    """The whole point: no mismatch may be raised on the strength of nothing."""
    _bench(monkeypatch)
    family, _chip, decisive = mcu_identity.identify_port(PORT)
    assert mcu_identity.mismatch("pico", family, decisive) is False, (
        "a board in BOOTSEL, ready to flash, was refused because a DIFFERENT "
        "board answered for it"
    )


def test_a_real_port_still_resolves():
    """The fix must not blind the normal case: an existing node still resolves."""
    link = mcu_identity.sysfs_device_link("/dev/null")
    assert link != "", "an existing device node stopped resolving"
    assert "/sys/" in link
