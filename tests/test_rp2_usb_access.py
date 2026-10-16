"""A cockpit that runs as a user needs write access to an RP2 board's raw USB device: picotool
reaches the bootloader there, not through the tty. A stock host gives root only, so the first gate
run as the app user failed every RP2 flash on a bench host with "Permission denied" and then
"sudo: a password is required" -- nothing said why. flash_mcu.py names it, and the repo ships the
host rule that fixes it (2026-10-16)."""
import os
import sys

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO_ROOT, "scripts"))
import flash_mcu  # noqa: E402


def fake_bus(tmp_path, vendor, mode):
    sysd = tmp_path / "sys" / "1-13"
    sysd.mkdir(parents=True)
    (sysd / "idVendor").write_text(vendor + "\n")
    (sysd / "busnum").write_text("1\n")
    (sysd / "devnum").write_text("122\n")
    node = tmp_path / "dev" / "001" / "122"
    node.parent.mkdir(parents=True)
    node.write_text("")
    node.chmod(mode)
    return str(tmp_path / "sys"), str(tmp_path / "dev"), str(node)


def test_an_unwritable_rp2_device_is_named_with_the_fix(tmp_path, monkeypatch):
    monkeypatch.setattr(os, "geteuid", lambda: 1000)
    sys_root, dev_root, node = fake_bus(tmp_path, "2e8a", 0o444)
    why = flash_mcu.rp2_usb_access_problem(sys_root, dev_root)
    assert why and node in why and "install_udev_rules.sh" in why and "[NO USB ACCESS]" in why


def test_a_writable_device_is_no_problem(tmp_path, monkeypatch):
    monkeypatch.setattr(os, "geteuid", lambda: 1000)
    sys_root, dev_root, _ = fake_bus(tmp_path, "2e8a", 0o666)
    assert flash_mcu.rp2_usb_access_problem(sys_root, dev_root) is None


def test_other_vendors_are_not_judged(tmp_path, monkeypatch):
    monkeypatch.setattr(os, "geteuid", lambda: 1000)
    sys_root, dev_root, _ = fake_bus(tmp_path, "303a", 0o444)
    assert flash_mcu.rp2_usb_access_problem(sys_root, dev_root) is None


def test_root_is_never_blocked(tmp_path, monkeypatch):
    monkeypatch.setattr(os, "geteuid", lambda: 0)
    sys_root, dev_root, _ = fake_bus(tmp_path, "2e8a", 0o444)
    assert flash_mcu.rp2_usb_access_problem(sys_root, dev_root) is None


def test_the_rule_gives_the_rp2_devices_to_dialout():
    rule = open(os.path.join(REPO_ROOT, "udev", "60-linorobot2-usb.rules")).read()
    lines = [ln for ln in rule.splitlines() if ln.strip() and not ln.startswith("#")]
    assert len(lines) == 1
    assert 'ATTRS{idVendor}=="2e8a"' in lines[0] and 'GROUP="dialout"' in lines[0] and 'MODE="0660"' in lines[0]
    assert os.access(os.path.join(REPO_ROOT, "scripts", "install_udev_rules.sh"), os.X_OK)


def test_the_compose_file_gives_the_container_raw_usb_and_dialout():
    compose = open(os.path.join(REPO_ROOT, "docker-compose.yml")).read()
    assert "c 189:* rwm" in compose and "- dialout" in compose
