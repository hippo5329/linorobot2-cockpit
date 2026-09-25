"""Three faults the every-control browser walk found (2026-09-25), each silent.

* Every ROS package read as missing: the supervisor runs unsourced, `ros2` is
  not on its PATH, and the FileNotFoundError was swallowed as "not installed".
  Start SLAM / Start Navigation then refused on an image that has both.
* The folder picker read `entries` and `parent`; list_dir returned `items` and
  no parent. Every picker showed "(empty)" with Up disabled, since the first
  commit. A field whose value lay outside the fence (/opt/lino_ws) got a 403.
* "Installing Rootless Docker" installed Ubuntu's docker.io, which has no
  rootless support at all.
"""
import os
import re
import sys

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BACKEND = os.path.join(REPO_ROOT, "web", "backend")


def read(*p):
    return open(os.path.join(REPO_ROOT, *p)).read()


def test_package_checks_run_in_a_sourced_ros_shell():
    src = read("web", "backend", "system_utils.py")
    assert '["ros2", "pkg"' not in src, "a bare ros2 call: the supervisor's PATH has no ros2"
    body = src[src.index("def ros_pkg_installed"):]
    body = body[:body.index("\ndef ", 1)]
    assert "ros_setup_shell(" in body


def test_ros_pkg_installed_is_false_not_an_exception_without_ros(monkeypatch):
    sys.path.insert(0, BACKEND)
    import system_utils
    monkeypatch.setattr(system_utils, "ros_setup_shell", lambda d="auto": "true")
    monkeypatch.setenv("PATH", "/nonexistent")
    assert system_utils.ros_pkg_installed("nav2_bringup") is False


def test_list_dir_speaks_the_pickers_shape(tmp_path):
    sys.path.insert(0, BACKEND)
    import system_utils
    (tmp_path / "sub").mkdir()
    out = system_utils.list_dir(str(tmp_path), only="dir")
    assert [e["name"] for e in out["entries"]] == ["sub"]
    ui = read("web", "frontend", "app-studio.js")
    assert "data.entries" in ui and "data.parent" in ui
    route = read("web", "backend", "routes_status.py")
    route = route[route.index("def api_list_dir"):]
    assert 'out["parent"]' in route[:2000]
    assert "loadDir((target.value || \"\").trim(), true)" in ui, "the field's value is only where the picker opens"


def test_docker_is_installed_rootless_by_the_repos_script():
    src = read("web", "backend", "system_utils.py")
    body = src[src.index("def install_container_engine"):]
    body = body[:body.index("\ndef ", 1)]
    assert "install_docker.sh" in body
    assert not re.search(r'pkg_name\s*=.*docker\.io', body), "docker.io has no rootless support"
