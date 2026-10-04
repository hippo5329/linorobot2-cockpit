"""Flash THIS board, not whichever one is in BOOTSEL.

On the bench (gate g4, 2026-10-04) two cells flashed their Picos at once, and both cells
see both boards (the passthrough is by vendor). While one board re-enumerated into
BOOTSEL its port had no device for a moment; the flasher's target came back empty,
the "is it in BOOTSEL yet" question went out untargeted and was answered at 0.0 s
by the OTHER cell's Pico, and the env block was written there. The board this run
was flashing then booted the new app on its old env (no IMU, no magnetometer) and
failed the topic gate. Two guards: an absent port is waited for, never treated as
"no target"; and the board in BOOTSEL must answer to the identity the board had
while it was running -- its USB serial, which on an RP2040 is the flash id
picotool reads, and on an RP2350 is also its BOOTSEL serial.
"""
import os
import stat
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "scripts"))

import flash_mcu  # noqa: E402
import pytest  # noqa: E402


@pytest.fixture(autouse=True)
def _restore_identity_state(monkeypatch):
    """remember_board_identity() sets module state; no test may leak it into the next."""
    for name in ("_EXPECTED_IDS", "_APP_USB_SERIAL", "_IDENTITY_CONFIRMED",
                 "_TARGET_USB_PATH", "_ESP_EXPECTED_UID"):
        monkeypatch.setattr(flash_mcu, name, getattr(flash_mcu, name))

PICO_W = "D665C007DA2A1336"        # app USB serial == banner flashid == picotool flash id
PICO_W_BOOTROM = "E0C9125B0D9B"    # what its RP2040 bootrom reports instead
OTHER_PICO_W = "D664CC2416264435"
PICO_2W = "A2898C2992D4D2FE"       # RP2350: the same serial in the app and in BOOTSEL


def _sysfs(tmp_path, port, serial, bus="1", addr="58"):
    d = tmp_path / "usb" / port
    d.mkdir(parents=True, exist_ok=True)
    (d / "serial").write_text(serial + "\n")
    (d / "busnum").write_text(bus + "\n")
    (d / "devnum").write_text(addr + "\n")
    return str(tmp_path / "usb")


def _fake_picotool(tmp_path, flash_id):
    p = tmp_path / "picotool"
    body = "#!/bin/sh\n"
    if flash_id:
        body += f'echo "Device Information"\necho " type:        RP2040"\necho " flash id:    0x{flash_id}"\n'
    p.write_text(body)
    p.chmod(p.stat().st_mode | stat.S_IEXEC)
    return [str(p)]


def _point_sysfs(monkeypatch, root):
    real = flash_mcu._sysfs_serial
    monkeypatch.setattr(flash_mcu, "_sysfs_serial", lambda path, r=None: real(path, root))


def test_the_running_board_is_remembered_by_its_usb_serial(tmp_path, monkeypatch):
    root = _sysfs(tmp_path, "1-4", PICO_W)
    ids = flash_mcu.remember_board_identity("1-4", from_tty=True, stamp={}, root=root)
    assert ids == {PICO_W}
    assert flash_mcu._APP_USB_SERIAL == PICO_W        # and the stamp will record it


def test_a_board_already_in_bootsel_is_known_from_the_stamp(tmp_path):
    root = _sysfs(tmp_path, "1-4", PICO_W_BOOTROM)
    ids = flash_mcu.remember_board_identity(
        "1-4", from_tty=False, stamp={"usb_serial": PICO_W, "board_id": PICO_W, "id_kind": "flashid"},
        root=root)
    assert ids == {PICO_W}


def test_an_rp2040_is_confirmed_by_its_flash_id(tmp_path, monkeypatch):
    root = _sysfs(tmp_path, "1-4", PICO_W_BOOTROM)
    _point_sysfs(monkeypatch, root)
    monkeypatch.setattr(flash_mcu, "_TARGET_USB_PATH", "1-4")
    monkeypatch.setattr(flash_mcu, "_EXPECTED_IDS", {PICO_W})
    monkeypatch.setattr(flash_mcu, "_IDENTITY_CONFIRMED", None)
    assert flash_mcu.verify_board_identity(["--bus", "1", "--address", "58"],
                                           _fake_picotool(tmp_path, PICO_W))


def test_an_rp2350_is_confirmed_by_its_bootsel_serial(tmp_path, monkeypatch):
    root = _sysfs(tmp_path, "3-1", PICO_2W, bus="3", addr="30")
    _point_sysfs(monkeypatch, root)
    monkeypatch.setattr(flash_mcu, "_TARGET_USB_PATH", "3-1")
    monkeypatch.setattr(flash_mcu, "_EXPECTED_IDS", {PICO_2W})
    monkeypatch.setattr(flash_mcu, "_IDENTITY_CONFIRMED", None)
    assert flash_mcu.verify_board_identity(["--bus", "3", "--address", "30"],
                                           _fake_picotool(tmp_path, None))


def test_the_other_cells_board_is_refused_and_nothing_is_written(tmp_path, monkeypatch, capsys):
    root = _sysfs(tmp_path, "3-2", "E0C9125B0D9C", bus="3", addr="19")
    _point_sysfs(monkeypatch, root)
    monkeypatch.setattr(flash_mcu, "_TARGET_USB_PATH", "3-2")
    monkeypatch.setattr(flash_mcu, "_EXPECTED_IDS", {PICO_W})
    monkeypatch.setattr(flash_mcu, "_IDENTITY_CONFIRMED", None)
    assert not flash_mcu.verify_board_identity(["--bus", "3", "--address", "19"],
                                               _fake_picotool(tmp_path, OTHER_PICO_W))
    out = capsys.readouterr()
    assert "BOARD MISMATCH" in out.out + out.err and "Nothing was written" in out.out + out.err


def test_the_env_write_refuses_a_wrong_board(tmp_path, monkeypatch):
    calls = []
    monkeypatch.setattr(flash_mcu, "picotool_target", lambda **k: ["--bus", "3", "--address", "19"])
    monkeypatch.setattr(flash_mcu, "verify_board_identity", lambda target, pts=None: False)
    monkeypatch.setattr(flash_mcu, "find_picotool_binaries", lambda: ["/usr/bin/picotool"])
    monkeypatch.setattr(flash_mcu, "run_tool", lambda cmd, **kw: calls.append(cmd))
    env_bin = tmp_path / "env.bin"
    env_bin.write_bytes(b"\0" * 4096)
    assert not flash_mcu.flash_env_via_picotool(str(env_bin), "pico")
    assert not any("load" in c for c in calls), calls


def test_no_identity_on_record_still_flashes_by_port(monkeypatch):
    """A first flash, or an image older than the stamp field: the port is all there is."""
    monkeypatch.setattr(flash_mcu, "_EXPECTED_IDS", set())
    assert flash_mcu.verify_board_identity(["--bus", "1", "--address", "5"], [])


def test_the_bootsel_wait_does_not_ask_untargeted_while_the_port_is_empty(monkeypatch):
    asked = []
    monkeypatch.setattr(flash_mcu, "picotool_target", lambda **k: None)
    monkeypatch.setattr(flash_mcu, "find_picotool_binaries", lambda: ["/usr/bin/picotool"])
    monkeypatch.setattr(flash_mcu.subprocess, "run", lambda cmd, **kw: asked.append(cmd))
    assert not flash_mcu.wait_for_bootsel(timeout_s=1.0)
    assert asked == [], "an untargeted picotool question would be answered by any board"


# ---- ESP32: the chip's MAC, which esptool prints before it writes anything ----

def test_the_banner_uid_is_the_mac_reversed():
    # The bench S3: MAC a0:f2:62:f4:49:e0, banner uid=E049F462F2A0.
    assert flash_mcu.uid_from_mac("a0:f2:62:f4:49:e0") == "E049F462F2A0"


def test_esptool_is_stopped_at_the_mac_of_another_chip(monkeypatch):
    monkeypatch.setattr(flash_mcu, "_ESP_EXPECTED_UID", "E049F462F2A0")
    assert flash_mcu.esp_identity_watch("Chip is ESP32-S3 (QFN56) (revision v0.2)\n") is None
    assert flash_mcu.esp_identity_watch("MAC: a0:f2:62:f4:49:e0\n") is None
    why = flash_mcu.esp_identity_watch("MAC: 24:6f:28:aa:bb:cc\n")
    assert why and "BOARD MISMATCH" in why and "--new-board" in why


def test_no_uid_on_record_watches_nothing(monkeypatch):
    monkeypatch.setattr(flash_mcu, "_ESP_EXPECTED_UID", None)
    assert flash_mcu.esp_identity_watch("MAC: 24:6f:28:aa:bb:cc\n") is None


def test_the_watch_stops_the_tool_before_it_writes(tmp_path):
    tool = tmp_path / "esptool"
    marker = tmp_path / "wrote"
    tool.write_text(f"#!/bin/sh\necho 'Connecting....'\necho 'MAC: 24:6f:28:aa:bb:cc'\n"
                    f"sleep 2\necho 'Writing at 0x00010000'\ntouch {marker}\n")
    tool.chmod(tool.stat().st_mode | stat.S_IEXEC)
    res = flash_mcu.run_tool([str(tool)], timeout=20, echo=False,
                             watch=lambda line: "stop" if "MAC:" in line else None)
    assert res.returncode == 3
    assert not marker.exists(), "the tool reached its write after the watch said stop"


def test_new_board_is_a_cli_flag():
    src = open(os.path.join(ROOT, "scripts", "flash_mcu.py")).read()
    assert '"--new-board"' in src and "if is_esp_family(args.env) and not args.new_board" in src
