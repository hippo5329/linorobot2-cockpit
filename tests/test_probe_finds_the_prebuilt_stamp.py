"""flash_mcu.py records a prebuilt flash under the profile's own pio_env
(pico2-jazzy is the pico2w image), while the pipeline probes as 'pico2'. Every
Start 1-Click then read "no record of flashing it" and reflashed a board that
was already running the build."""
import json
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))

import mcu_probe  # noqa: E402


def _setup(tmp_path, monkeypatch, recorded_as):
    stamps = tmp_path / "flashed"
    stamps.mkdir()
    monkeypatch.setattr(mcu_probe, "STAMP_DIR", str(stamps))
    (stamps / f"{recorded_as}_ttyACM0.json").write_text(json.dumps({"git": "e2e2a9b"}))
    pre = tmp_path / "pico2-jazzy"
    pre.mkdir()
    (pre / "manifest.json").write_text(json.dumps({"pio_env": "pico2w", "commit": "e2e2a9b"}))
    return str(pre)


def test_a_stamp_written_under_the_profile_env_is_found(tmp_path, monkeypatch):
    pre = _setup(tmp_path, monkeypatch, "pico2w")
    assert mcu_probe.stamp_for("pico2", "/dev/ttyACM0", pre) == {"git": "e2e2a9b"}


def test_the_requested_env_is_read_first(tmp_path, monkeypatch):
    pre = _setup(tmp_path, monkeypatch, "pico2")
    assert mcu_probe.stamp_for("pico2", "/dev/ttyACM0", pre) == {"git": "e2e2a9b"}


def test_no_prebuilt_dir_no_guess(tmp_path, monkeypatch):
    _setup(tmp_path, monkeypatch, "pico2w")
    assert mcu_probe.stamp_for("pico2", "/dev/ttyACM0", None) == {}


def test_probe_uses_it():
    src = open(os.path.join(os.path.dirname(__file__), "..", "scripts", "mcu_probe.py")).read()
    assert '"stamp": stamp_for(env_name, port, prebuilt_dir)' in src
