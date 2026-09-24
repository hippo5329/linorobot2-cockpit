"""A config written before the `sim_` rename must be refused, not read as blank.

The simulation flags were `use_fake_*` until 2026-09-24. Every reader in
`mcu_env.py` tests for its key by name -- `if "use_sim_ld19" in sensors` -- so an
old config does not fail. The key is simply absent, the flag falls back to a
compiled-in default, and a bare module gets flashed to expect an IMU, a
magnetometer and a LiDAR that are not fitted.

That is the shape this project keeps paying for: an unrecognised key is not an
error, it is a default silently taken. The firmware got a loud warning for a
pre-rename ENV image; this is the same guard one step earlier, on the config that
produces it.

Refusing rather than translating is the deliberate part. A silent translation lets
two spellings live indefinitely, and the second spelling only ever surfaces after
somebody has spent an afternoon on a board that cannot see its sensors.
"""
import os
import subprocess
import sys

import yaml

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MCU_ENV = os.path.join(REPO_ROOT, "scripts", "mcu_env.py")
REFERENCE = os.path.join(REPO_ROOT, "config", "reference", "yb_eet01_config.yaml")


def _build(config_path, tmp_path):
    return subprocess.run(
        [sys.executable, MCU_ENV, "build", "--params", str(config_path),
         "--secrets", str(tmp_path / "no-secrets.yaml"),
         "--out", str(tmp_path / "env.bin")],
        capture_output=True, text=True, cwd=REPO_ROOT)


def test_a_current_config_still_builds(tmp_path):
    """The guard must not have broken the ordinary path."""
    proc = _build(REFERENCE, tmp_path)
    assert proc.returncode == 0, proc.stderr + proc.stdout
    assert (tmp_path / "env.bin").stat().st_size == 4096


def test_a_use_fake_key_anywhere_is_refused(tmp_path):
    """Found by walking the document, not by checking the two places we remember.

    The flags live under `sensors` today and an explicit one may sit under `lidar`
    -- and the next config revision may move them again. A guard that named the
    sections would rot exactly like the list that let `use_sim_sonar` through.
    """
    with open(REFERENCE, encoding="utf-8") as fh:
        cfg = yaml.safe_load(fh)

    # Rename one flag back, wherever it lives.
    def unrename(node):
        if isinstance(node, dict):
            for key in list(node):
                if isinstance(key, str) and key.startswith("use_sim_"):
                    node["use_fake_" + key[len("use_sim_"):]] = node.pop(key)
                else:
                    unrename(node[key])
        elif isinstance(node, list):
            for item in node:
                unrename(item)

    unrename(cfg)
    old = tmp_path / "old_config.yaml"
    old.write_text(yaml.safe_dump(cfg, sort_keys=False))

    proc = _build(old, tmp_path)
    assert proc.returncode != 0, (
        "a pre-rename config was accepted; every simulation flag was silently at "
        "its compiled-in default and the board would be flashed to expect hardware "
        "it does not have")
    text = proc.stderr + proc.stdout
    assert "use_fake_" in text, "the message must name the offending keys"
    assert "use_sim_" in text, "the message must say what to rename them to"
    assert not (tmp_path / "env.bin").exists(), \
        "nothing may be written from a config that was refused"


def test_a_nested_use_fake_key_is_found_too(tmp_path):
    """One key, buried, in a config that is otherwise current."""
    with open(REFERENCE, encoding="utf-8") as fh:
        cfg = yaml.safe_load(fh)
    cfg.setdefault("lidar", {})["use_fake_ld19"] = True
    buried = tmp_path / "buried.yaml"
    buried.write_text(yaml.safe_dump(cfg, sort_keys=False))

    proc = _build(buried, tmp_path)
    assert proc.returncode != 0, "a single buried use_fake_ key slipped through"
    assert "lidar.use_fake_ld19" in (proc.stderr + proc.stdout), \
        "the message must say WHERE the key is, not just that one exists"
