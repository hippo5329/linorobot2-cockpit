"""A flash must name the board it is flashing.

picotool refuses to guess between two RP-series devices:

    ERROR: Command requires a single RP-series device to be targeted.

It only refuses once BOTH are in BOOTSEL, which is why serial runs never saw
it. The first parallel run of the bench failed all four RP2 cells at once,
2026-09-20: each cell touched its own board into BOOTSEL, and from that moment
every picotool call in either cell saw two devices and wrote neither. The two
ESP32 cells in the same run passed 6/6, so the images were fine.

The key has to be the USB PORT PATH. Bus and address change when the board
re-enumerates into BOOTSEL, so they cannot be captured up front; the serial is
worse than useless, because an RP2040 bootrom reports a different one from the
running application -- E0C9125B0D9B in BOOTSEL against D665C007DA2A1336 as the
application, measured on this bench. The physical port does not move.
"""
import os
import subprocess
import sys

import pytest

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO_ROOT, "scripts"))

import flash_mcu  # noqa: E402


@pytest.fixture(autouse=True)
def _no_leaked_target(monkeypatch):
    monkeypatch.setattr(flash_mcu, "_TARGET_USB_PATH", None)


def _fake_sysfs(tmp_path, path="7-1.1", bus="7", addr="57"):
    root = tmp_path / "devices"
    dev = root / path
    dev.mkdir(parents=True)
    (dev / "busnum").write_text(bus + "\n")
    (dev / "devnum").write_text(addr + "\n")
    return root


def test_the_target_is_read_fresh_because_bootsel_re_enumerates(tmp_path, monkeypatch):
    """The address changes between the 1200-baud touch and the load, so it must
    not be captured with the path."""
    root = _fake_sysfs(tmp_path, addr="57")
    monkeypatch.setattr(flash_mcu, "_TARGET_USB_PATH", "7-1.1")
    assert flash_mcu.picotool_target(str(root)) == ["--bus", "7", "--address", "57"]

    (root / "7-1.1" / "devnum").write_text("58\n")
    assert flash_mcu.picotool_target(str(root)) == ["--bus", "7", "--address", "58"], (
        "the address was cached across a re-enumeration")


def test_no_target_means_no_flags_not_a_crash(monkeypatch):
    """Everything still works on a host with one board and no sysfs to read."""
    monkeypatch.setattr(flash_mcu, "_TARGET_USB_PATH", None)
    assert flash_mcu.picotool_target() == []
    monkeypatch.setattr(flash_mcu, "_TARGET_USB_PATH", "no-such-port")
    assert flash_mcu.picotool_target() == []


def test_the_tty_is_resolved_by_device_number_not_by_name(monkeypatch):
    """Inside a container the tty is a bind mount: the bench passes the host's
    /dev/ttyACM1 in as /dev/ttyACM0, so /sys/class/tty/ttyACM0 is the OTHER
    board. Matching on major:minor is what makes the answer right."""
    seen = {}

    class _St:
        st_rdev = os.makedev(166, 1)

    monkeypatch.setattr(flash_mcu.os, "stat", lambda p: _St())

    def fake_glob(pattern):
        seen["pattern"] = pattern
        return ["/sys/class/tty/ttyACM0/dev", "/sys/class/tty/ttyACM1/dev"]

    monkeypatch.setattr(flash_mcu.glob, "glob", fake_glob)
    contents = {"/sys/class/tty/ttyACM0/dev": "166:0\n",
                "/sys/class/tty/ttyACM1/dev": "166:1\n"}
    real_open = open

    def fake_open(path, *a, **kw):
        if path in contents:
            import io
            return io.StringIO(contents[path])
        return real_open(path, *a, **kw)

    monkeypatch.setattr("builtins.open", fake_open)
    monkeypatch.setattr(flash_mcu.os.path, "realpath",
                        lambda p: "/sys/devices/pci/usb7/7-1/7-1.1/7-1.1:1.0"
                        if "ttyACM1" in p else "/sys/devices/pci/usb3/3-2/3-2:1.0")
    monkeypatch.setattr(flash_mcu.os.path, "isfile",
                        lambda p: p.endswith("7-1.1/busnum"))
    assert flash_mcu.usb_path_for_tty("/dev/ttyACM0") == "7-1.1", (
        "resolved through the name instead of the device numbers")


def test_the_load_carries_the_target(tmp_path, monkeypatch):
    """The whole point: picotool is told which board, on every form of the
    command it tries."""
    uf2 = tmp_path / "firmware.uf2"
    uf2.write_bytes(b"\x00")
    cmds = []

    class _Res:
        returncode = 1
        stdout = ""
        stderr = ""

    def fake_run(cmd, **kw):
        cmds.append(cmd)
        return _Res()

    monkeypatch.setattr(flash_mcu, "find_picotool_binaries", lambda: ["/usr/bin/picotool"])
    monkeypatch.setattr(flash_mcu, "picotool_target",
                        lambda: ["--bus", "7", "--address", "57"])
    monkeypatch.setattr(flash_mcu, "run_tool", fake_run)
    monkeypatch.setattr(subprocess, "run", lambda *a, **k: _Res())

    assert flash_mcu.flash_via_picotool(str(uf2), "pico2w") is False
    assert cmds, "nothing was attempted"
    for cmd in cmds:
        assert "--bus" in cmd and "--address" in cmd, cmd
        # picotool's synopsis puts device selection last, after the filename.
        assert cmd.index("--bus") > cmd.index(str(uf2)), cmd


def test_the_env_write_targets_the_same_board(tmp_path, monkeypatch):
    """An env block on the wrong board is worse than none: it configures a
    robot nobody is testing, silently."""
    env_bin = tmp_path / "env.bin"
    env_bin.write_bytes(b"\x00" * 4096)
    cmds = []

    class _Res:
        returncode = 1
        stdout = ""
        stderr = ""

    monkeypatch.setattr(flash_mcu, "find_picotool_binaries", lambda: ["/usr/bin/picotool"])
    monkeypatch.setattr(flash_mcu, "picotool_target",
                        lambda: ["--bus", "3", "--address", "34"])
    monkeypatch.setattr(flash_mcu, "run_tool",
                        lambda cmd, **kw: (cmds.append(cmd), _Res())[1])

    flash_mcu.flash_env_via_picotool(str(env_bin), "picow")
    assert cmds
    for cmd in cmds:
        assert cmd[-4:] == ["--bus", "3", "--address", "34"], cmd


def test_the_bootsel_wait_asks_about_our_board(monkeypatch):
    """Otherwise the other cell's board in BOOTSEL reports ours as ready."""
    seen = []

    class _Res:
        returncode = 0
        stdout = b""
        stderr = b""

    monkeypatch.setattr(flash_mcu, "find_picotool_binaries", lambda: ["/usr/bin/picotool"])
    monkeypatch.setattr(flash_mcu, "picotool_target",
                        lambda: ["--bus", "7", "--address", "57"])
    monkeypatch.setattr(subprocess, "run",
                        lambda cmd, **kw: (seen.append(cmd), _Res())[1])
    assert flash_mcu.wait_for_bootsel(timeout_s=2.0) is True
    assert seen and seen[0][-4:] == ["--bus", "7", "--address", "57"]


def test_force_is_never_combined_with_an_explicit_device(tmp_path, monkeypatch):
    """picotool treats `-f` as its own way of choosing a board and rejects the
    pair outright:

        ERROR: unexpected option: --bus

    Every command form must pick one. Nothing is lost by dropping -f: it forces
    a RUNNING board to reset, which this firmware does not support anyway (it
    exposes no picotool reset interface), and a board we can name is one we
    already identified through its tty."""
    uf2 = tmp_path / "firmware.uf2"
    uf2.write_bytes(b"\x00")
    env_bin = tmp_path / "env.bin"
    env_bin.write_bytes(b"\x00" * 4096)
    cmds = []

    class _Res:
        returncode = 1
        stdout = ""
        stderr = ""

    monkeypatch.setattr(flash_mcu, "find_picotool_binaries", lambda: ["/usr/bin/picotool"])
    monkeypatch.setattr(flash_mcu, "picotool_target",
                        lambda: ["--bus", "7", "--address", "57"])
    monkeypatch.setattr(flash_mcu, "run_tool",
                        lambda cmd, **kw: (cmds.append(cmd), _Res())[1])
    monkeypatch.setattr(subprocess, "run", lambda *a, **k: _Res())

    flash_mcu.flash_via_picotool(str(uf2), "pico2w", env_bin=str(env_bin))
    flash_mcu.flash_env_via_picotool(str(env_bin), "pico2w")
    assert cmds
    for cmd in cmds:
        assert not ("-f" in cmd and "--bus" in cmd), cmd


def test_force_is_still_used_when_there_is_no_target(tmp_path, monkeypatch):
    """One board, no sysfs, an older container: the recovery ladder is intact."""
    uf2 = tmp_path / "firmware.uf2"
    uf2.write_bytes(b"\x00")
    cmds = []

    class _Res:
        returncode = 1
        stdout = ""
        stderr = ""

    monkeypatch.setattr(flash_mcu, "find_picotool_binaries", lambda: ["/usr/bin/picotool"])
    monkeypatch.setattr(flash_mcu, "picotool_target", lambda: [])
    monkeypatch.setattr(flash_mcu, "run_tool",
                        lambda cmd, **kw: (cmds.append(cmd), _Res())[1])
    monkeypatch.setattr(subprocess, "run", lambda *a, **k: _Res())

    flash_mcu.flash_via_picotool(str(uf2), "pico2w")
    assert any("-f" in cmd for cmd in cmds), "the forced retry disappeared"
