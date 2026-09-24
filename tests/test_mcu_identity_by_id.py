"""A port may be named by its udev by-id path anywhere a tty is accepted."""
import os

import mcu_identity


def test_resolve_port_follows_a_symlink(tmp_path):
    real = tmp_path / "ttySIM0"
    real.write_text("")
    link = tmp_path / "usb-Board_1234-if00"
    os.symlink(real, link)
    assert mcu_identity.resolve_port(str(link)) == str(real)
    assert mcu_identity.resolve_port(str(real)) == str(real)


def test_resolve_port_keeps_a_name_that_does_not_resolve(tmp_path):
    # A stale by-id path must come back unchanged, so the caller reports the
    # name the user configured rather than a guess.
    missing = str(tmp_path / "usb-Gone-if00")
    assert mcu_identity.resolve_port(missing) == missing
    assert mcu_identity.resolve_port("") == ""


def test_by_id_map_prefers_the_first_name_for_a_port(tmp_path, monkeypatch):
    real = tmp_path / "ttySIM1"
    real.write_text("")
    for name in ("usb-Board_1234-if00", "usb-Board_1234-if00-port0"):
        os.symlink(real, tmp_path / name)
    monkeypatch.setattr(mcu_identity, "BY_ID_DIR", str(tmp_path))
    m = mcu_identity.by_id_map()
    assert m[str(real)].endswith("usb-Board_1234-if00")
    assert mcu_identity.by_id_for_port(str(real)).endswith("usb-Board_1234-if00")


def test_by_id_for_port_is_empty_without_udev(tmp_path, monkeypatch):
    monkeypatch.setattr(mcu_identity, "BY_ID_DIR", str(tmp_path / "nothing-here"))
    assert mcu_identity.by_id_for_port("/dev/ttyACM0") == ""
