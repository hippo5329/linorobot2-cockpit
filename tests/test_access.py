"""The access token and the path allowlist (web/backend/access.py)."""
import os
import stat

import access


def test_token_is_created_once_and_private(tmp_path):
    t1 = access.load_or_create_token(str(tmp_path))
    t2 = access.load_or_create_token(str(tmp_path))
    assert t1 == t2 and len(t1) >= 32
    mode = stat.S_IMODE(os.stat(access.token_path(str(tmp_path))).st_mode)
    assert mode == 0o600


def test_environment_token_wins(tmp_path, monkeypatch):
    monkeypatch.setenv("COCKPIT_TOKEN", "from-env")
    assert access.load_or_create_token(str(tmp_path)) == "from-env"
    assert not os.path.exists(access.token_path(str(tmp_path)))


def test_needs_token_policy():
    assert access.needs_token("POST", "/api/exec")
    assert access.needs_token("POST", "/api/kill")
    assert access.needs_token("GET", "/api/secrets")
    assert access.needs_token("GET", "/api/list_dir")
    # GET streams that do something: the pipeline flashes and launches, the
    # LiDAR viewer starts a subscriber.
    assert access.needs_token("GET", "/api/workflow/one-click/stream")
    assert access.needs_token("GET", "/api/lidar_stream")
    assert not access.needs_token("GET", "/api/status")
    assert not access.needs_token("GET", "/api/bringup/stream")
    assert not access.needs_token("GET", "/index.html")
    assert not access.needs_token("OPTIONS", "/api/exec")


def test_token_compare():
    assert access.token_matches("abc", "abc")
    assert not access.token_matches("", "abc")
    assert not access.token_matches("abd", "abc")


def test_auth_toggle(monkeypatch):
    monkeypatch.delenv("COCKPIT_AUTH", raising=False)
    assert access.auth_enabled()
    for off in ("off", "0", "false", "NO"):
        monkeypatch.setenv("COCKPIT_AUTH", off)
        assert not access.auth_enabled()


def test_path_allowlist(tmp_path):
    cfg = str(tmp_path / "cfg"); repo = str(tmp_path / "repo")
    os.makedirs(cfg); os.makedirs(repo)
    assert access.path_allowed(cfg, cfg, repo)
    assert access.path_allowed(os.path.join(repo, "maps"), cfg, repo)
    assert access.path_allowed("/dev", cfg, repo)
    assert access.path_allowed("~", cfg, repo)
    assert not access.path_allowed("/etc", cfg, repo)
    assert not access.path_allowed("/etc/passwd", cfg, repo)
    # pytest's tmp_path lives under /tmp, itself an allowed root, so the
    # traversal case has to climb out of every root.
    assert not access.path_allowed(cfg + "/../../../../../../etc", cfg, repo)
