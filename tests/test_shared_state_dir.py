"""State the two users share must not be keyed on $HOME.

The one-click pipeline runs as container-root and the cockpit's backend as the
container user. Anything under ~ resolves to /root/... for one and
/home/ubuntu/... for the other, so neither sees what the other wrote. The
flash stamp did exactly that: a board flashed by the pipeline still probed as
"the board has not said, and this host has no record of flashing it" from the
UI, and the verdict fell back to `unknown` across a whole release matrix.
"""
import importlib
import os
import sys

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO_ROOT, "scripts"))

import cockpit_paths  # noqa: E402


def test_state_dir_lives_with_the_config_not_with_the_user(tmp_path, monkeypatch):
    monkeypatch.setenv("COCKPIT_CONFIG_DIR", str(tmp_path / "cfg"))
    d = cockpit_paths.state_dir()
    assert d.startswith(str(tmp_path / "cfg")), d
    assert os.path.isdir(d)


def test_two_users_resolve_the_same_stamp_dir(tmp_path, monkeypatch):
    """Same config dir, different HOME -> the same place."""
    monkeypatch.setenv("COCKPIT_CONFIG_DIR", str(tmp_path / "cfg"))
    monkeypatch.delenv("LINO_STAMP_DIR", raising=False)

    monkeypatch.setenv("HOME", "/root")
    import mcu_probe
    importlib.reload(mcu_probe)
    as_root = mcu_probe.STAMP_DIR

    monkeypatch.setenv("HOME", str(tmp_path / "home-ubuntu"))
    importlib.reload(mcu_probe)
    as_user = mcu_probe.STAMP_DIR

    assert as_root == as_user, (
        f"the flash stamp still follows $HOME: root sees {as_root}, the "
        f"cockpit user sees {as_user}"
    )
    assert "/.cache/" not in as_root, as_root


def test_an_explicit_override_still_wins(tmp_path, monkeypatch):
    monkeypatch.setenv("COCKPIT_CONFIG_DIR", str(tmp_path / "cfg"))
    monkeypatch.setenv("LINO_STAMP_DIR", str(tmp_path / "elsewhere"))
    import mcu_probe
    importlib.reload(mcu_probe)
    assert mcu_probe.STAMP_DIR == str(tmp_path / "elsewhere")
