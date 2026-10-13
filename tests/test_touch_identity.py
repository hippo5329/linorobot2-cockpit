"""A 1200-baud touch asks the board's identity first, every time.

A gate run (2026-10-13): two cells on one USB controller flashed at once, and the host
handed /dev/ttyACM0 back and forth between their boards. The Pico 2's load died, and it sat
in BOOTSEL with no tty while its cell's /dev/ttyACM0 was the other cell's Pico W. The retry
path then reset and touched that node. The guard at the start of the flash had read the tty
once, so it could not see this.
"""
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "scripts"))

import flash_mcu  # noqa: E402
import pytest  # noqa: E402

PICO_2W = "A2898C2992D4D2FE"
OTHER_PICO_W = "D664CC2416264435"


@pytest.fixture(autouse=True)
def _state(monkeypatch, tmp_path):
    for name in ("_EXPECTED_IDS", "_APP_USB_SERIAL", "_IDENTITY_CONFIRMED", "_TARGET_USB_PATH"):
        monkeypatch.setattr(flash_mcu, name, getattr(flash_mcu, name))
    tty = tmp_path / "ttyACM0"
    tty.write_text("")
    calls = {"reset": 0, "open": 0}
    monkeypatch.setattr(flash_mcu, "usb_reset_target", lambda p: calls.__setitem__("reset", calls["reset"] + 1) or True)
    monkeypatch.setattr(flash_mcu, "wait_for_bootsel", lambda *a, **k: True)

    class FakeSerial:
        def __init__(self, *a, **k):
            calls["open"] += 1
            self.dtr = True

        def close(self):
            pass

    import types
    monkeypatch.setitem(sys.modules, "serial", types.SimpleNamespace(Serial=FakeSerial))
    monkeypatch.setattr(flash_mcu.time, "sleep", lambda s: None)
    return str(tty), calls


def _board_behind(monkeypatch, serial):
    monkeypatch.setattr(flash_mcu, "usb_path_for_tty", lambda p: "3-2" if serial else None)
    monkeypatch.setattr(flash_mcu, "_sysfs_serial", lambda path, root=None: serial)


def test_the_node_now_leads_to_another_cells_board(_state, monkeypatch):
    tty, calls = _state
    monkeypatch.setattr(flash_mcu, "_EXPECTED_IDS", {PICO_2W})
    _board_behind(monkeypatch, OTHER_PICO_W)
    assert flash_mcu.pulse_1200_baud(tty) is False
    assert calls == {"reset": 0, "open": 0}


def test_no_readable_board_behind_the_node_is_not_a_yes(_state, monkeypatch):
    tty, calls = _state
    monkeypatch.setattr(flash_mcu, "_EXPECTED_IDS", {PICO_2W})
    _board_behind(monkeypatch, None)
    assert flash_mcu.pulse_1200_baud(tty) is False
    assert calls == {"reset": 0, "open": 0}


def test_the_right_board_is_touched(_state, monkeypatch):
    tty, calls = _state
    monkeypatch.setattr(flash_mcu, "_EXPECTED_IDS", {PICO_2W})
    _board_behind(monkeypatch, PICO_2W)
    assert flash_mcu.pulse_1200_baud(tty) is True
    assert calls == {"reset": 1, "open": 1}


def test_the_board_changes_between_the_reset_and_the_touch(_state, monkeypatch):
    tty, calls = _state
    monkeypatch.setattr(flash_mcu, "_EXPECTED_IDS", {PICO_2W})
    _board_behind(monkeypatch, PICO_2W)

    def reset_then_swap(p):
        calls["reset"] += 1
        _board_behind(monkeypatch, OTHER_PICO_W)
        return True
    monkeypatch.setattr(flash_mcu, "usb_reset_target", reset_then_swap)
    assert flash_mcu.pulse_1200_baud(tty) is False
    assert calls == {"reset": 1, "open": 0}


def test_nothing_on_record_keeps_the_old_behaviour(_state, monkeypatch):
    tty, calls = _state
    monkeypatch.setattr(flash_mcu, "_EXPECTED_IDS", set())
    _board_behind(monkeypatch, None)
    assert flash_mcu.pulse_1200_baud(tty) is True
    assert calls["open"] == 1
