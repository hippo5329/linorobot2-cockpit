"""The SLAM card's Save map writes where every reader of saved maps looks.

The button saved into the colcon workspace's linorobot2_navigation/maps, while the
Saved Maps gallery, the Nav2 map list and the 1-Click's map saver all use <repo>/maps:
a map saved from the SLAM card never appeared anywhere in the UI. One directory now,
cockpit_paths.maps_dir(), for every saver and every reader.
"""
import os
import re

import pytest

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def read(*p):
    return open(os.path.join(REPO_ROOT, *p)).read()


def test_the_one_directory_is_the_repos_maps():
    import cockpit_paths
    assert cockpit_paths.maps_dir() == os.path.join(REPO_ROOT, "maps")


def test_save_map_writes_there_whatever_the_page_sends():
    import actions
    import cockpit_paths
    d = cockpit_paths.maps_dir()
    for args in ({"name": "kitchen"},
                 {"name": "kitchen", "maps_dir": "~/linorobot2_ws/src/linorobot2/linorobot2_navigation/maps"}):
        cmd = actions.build("map_save", args)
        assert f"map_saver_cli -f {d}/kitchen" in cmd
        assert "linorobot2_navigation" not in cmd


def test_the_page_names_no_directory():
    js = read("web", "frontend", "app-nav-viz.js")
    body = js[js.index('getElementById("btn-map-save")'):]
    body = body[:body.index("\n});")]
    assert "linorobot2_navigation/maps" not in body
    assert not re.search(r"maps_dir\s*:", body)


def test_the_1click_saver_uses_the_same_directory():
    src = read("scripts", "one_click_pipeline.py")
    m = re.search(r'"--map-output", default=([^\n]+)', src)
    assert m and "cockpit_paths.maps_dir()" in m.group(1)


def test_the_readers_use_the_same_directory(tmp_path, monkeypatch):
    pytest.importorskip("fastapi")
    import cockpit_paths
    import routes_hardware
    monkeypatch.setattr(cockpit_paths, "MAPS_DIR", str(tmp_path))
    (tmp_path / "kitchen.yaml").write_text("image: kitchen.pgm\n")
    (tmp_path / "kitchen.pgm").write_bytes(b"P5\n1 1\n255\n\x00")
    listed = routes_hardware.list_maps()
    assert listed["maps_dir"] == str(tmp_path)
    assert [m["name"] for m in listed["maps"]] == ["kitchen"]
    assert routes_hardware.saved_map_names() == ["kitchen"]
    assert routes_hardware._map_path("kitchen.pgm") == str(tmp_path / "kitchen.pgm")
