"""A flash stamp is a record: failing to write it must not fail the flash.

On 2026-09-25 the UI's Start 1-Click died with PermissionError on a stamp file a
root-run leg had left behind -- the whole pipeline aborted over bookkeeping.
"""
import json
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "scripts"))
import mcu_probe  # noqa: E402


def test_writes_atomically(tmp_path, monkeypatch):
    monkeypatch.setattr(mcu_probe, "STAMP_DIR", str(tmp_path))
    assert mcu_probe.write_stamp("pico2w", "/dev/ttyACM0", {"git": "abc1234"}) is True
    data = json.load(open(tmp_path / "pico2w_ttyACM0.json"))
    assert data["git"] == "abc1234" and "flashed_at" in data
    assert not [f for f in os.listdir(tmp_path) if f.startswith(".stamp-")], "temp file left behind"


@pytest.mark.skipif(os.geteuid() == 0, reason="root ignores directory permissions")
def test_an_unwritable_stamp_dir_warns_instead_of_raising(tmp_path, monkeypatch, capsys):
    d = tmp_path / "flashed"; d.mkdir()
    (d / "pico2w_ttyACM0.json").write_text("{}")
    d.chmod(0o555)
    try:
        monkeypatch.setattr(mcu_probe, "STAMP_DIR", str(d))
        assert mcu_probe.write_stamp("pico2w", "/dev/ttyACM0", {"git": "abc1234"}) is False
        assert "flash stamp not recorded" in capsys.readouterr().err
    finally:
        d.chmod(0o755)
