import os
import sys

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for sub in ("scripts", os.path.join("web", "backend")):
    p = os.path.join(REPO_ROOT, sub)
    if p not in sys.path:
        sys.path.insert(0, p)

import pytest  # noqa: E402


@pytest.fixture(autouse=True)
def isolated_config_dir(tmp_path, monkeypatch):
    """Every test gets its own config dir; nothing touches ~/linorobot2-config."""
    monkeypatch.setenv("COCKPIT_CONFIG_DIR", str(tmp_path / "cfg"))
    yield tmp_path / "cfg"


@pytest.fixture
def reference(tmp_path):
    """Load a shipped reference config by name."""
    import yaml

    def _load(name):
        with open(os.path.join(REPO_ROOT, "config", "reference", f"{name}_config.yaml")) as f:
            return yaml.safe_load(f)
    return _load
