"""'I could not look' is not 'there is nothing there'.

rp2_usb_mode() used to shell out to lsusb and read any non-zero exit as
"absent". On an Ubuntu 26.04 bench lsusb cannot start at all --

    lsusb: error while loading shared libraries: libc.so.6: cannot apply
    additional memory protection after relocation: Permission denied

-- exit 127, no output. Every probe on that host therefore returned a
confident "nothing is on that port" about a board that was sitting there
running micro-ROS. `verdict: absent` writes neither firmware nor env block
while printing "Nothing to write", and it also suppressed
report_failed_bootsel_request(), which only fires on "app" -- so the message
explaining a failed flash never printed either.
"""
import os
import sys
import types

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO_ROOT, "scripts"))


def _sysfs(tmp_path, devices):
    """Build a sim /sys/bus/usb/devices. devices: {name: (vid, [ifclass, ...])}"""
    root = tmp_path / "sys" / "bus" / "usb" / "devices"
    root.mkdir(parents=True)
    for name, (vid, classes) in devices.items():
        dev = root / name
        dev.mkdir()
        (dev / "idVendor").write_text(vid + "\n")
        for n, cls in enumerate(classes):
            iface = dev / f"{name}:1.{n}"
            iface.mkdir()
            (iface / "bInterfaceClass").write_text(cls + "\n")
    return str(root)


def test_a_broken_lsusb_is_unknown_not_absent(monkeypatch):
    import flash_mcu

    monkeypatch.setattr(flash_mcu, "_rp2_mode_from_sysfs", lambda: None)
    monkeypatch.setattr(flash_mcu.subprocess, "run",
                        lambda *a, **k: types.SimpleNamespace(
                            returncode=127, stdout="", stderr="cannot apply additional memory protection"))
    assert flash_mcu.rp2_usb_mode() == "unknown", (
        "a tool that could not start must not be reported as an empty bus"
    )


def test_lsusb_that_ran_and_found_nothing_is_absent(monkeypatch):
    import flash_mcu

    monkeypatch.setattr(flash_mcu, "_rp2_mode_from_sysfs", lambda: None)
    monkeypatch.setattr(flash_mcu.subprocess, "run",
                        lambda *a, **k: types.SimpleNamespace(returncode=0, stdout="", stderr=""))
    assert flash_mcu.rp2_usb_mode() == "absent"


def test_sysfs_reads_an_application_board(tmp_path):
    import flash_mcu

    root = _sysfs(tmp_path, {"9-1": ("2e8a", ["02", "0a"]), "1-2": ("1d6b", ["09"])})
    assert flash_mcu._rp2_mode_from_sysfs(root) == "app"


def test_sysfs_reads_a_bootsel_board(tmp_path):
    import flash_mcu

    root = _sysfs(tmp_path, {"9-1": ("2e8a", ["08"])})
    assert flash_mcu._rp2_mode_from_sysfs(root) == "bootsel"


def test_sysfs_with_no_rp2_board_is_absent(tmp_path):
    import flash_mcu

    root = _sysfs(tmp_path, {"1-2": ("1d6b", ["09"])})
    assert flash_mcu._rp2_mode_from_sysfs(root) == "absent"


def test_sysfs_is_tried_before_lsusb(monkeypatch):
    """lsusb is the fallback, not the source. It must not run when sysfs answered."""
    import flash_mcu

    monkeypatch.setattr(flash_mcu, "_rp2_mode_from_sysfs", lambda: "bootsel")

    def boom(*a, **k):
        raise AssertionError("lsusb was called even though sysfs answered")

    monkeypatch.setattr(flash_mcu.subprocess, "run", boom)
    assert flash_mcu.rp2_usb_mode() == "bootsel"
