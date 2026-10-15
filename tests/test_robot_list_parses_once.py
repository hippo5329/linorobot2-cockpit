"""/api/status lists every robot on each poll: a config is parsed once, until it changes.

On an Arduino UNO Q the pure-Python YAML loader took 0.44 s a file, 15 files made a status call
22 s, and a Bringup or 1-Click press (which asks the status first) looked dead (2026-10-15).
"""
import os
import sys

import pytest

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO_ROOT, "scripts"))
sys.path.insert(0, os.path.join(REPO_ROOT, "web", "backend"))
pytest.importorskip("fastapi", reason="the cockpit backend needs fastapi/uvicorn")

import core  # noqa: E402


def test_a_config_is_parsed_once_until_it_changes(tmp_path, monkeypatch):
    f = tmp_path / "r_config.yaml"
    f.write_text("robot:\n  name: r\n  description: first\nbase_controller:\n  name: esp32\n")
    calls = []
    real = core._parse_robot_file
    monkeypatch.setattr(core, "_parse_robot_file", lambda p: (calls.append(p), real(p))[1])
    a = core._read_robot_file(str(f))
    b = core._read_robot_file(str(f))
    assert a == b and a[1] == "first" and len(calls) == 1
    f.write_text("robot:\n  name: r\n  description: second one\nbase_controller:\n  name: esp32\n")
    os.utime(f, ns=(os.stat(f).st_atime_ns, os.stat(f).st_mtime_ns + 1_000_000))
    assert core._read_robot_file(str(f))[1] == "second one" and len(calls) == 2


def test_the_c_loader_is_used_when_there():
    import yaml
    if getattr(yaml, "__with_libyaml__", False):
        assert core._YAML_LOADER is yaml.CSafeLoader
