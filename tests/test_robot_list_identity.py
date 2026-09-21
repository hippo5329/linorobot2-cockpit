"""A robot is what its file SAYS it is, and no file is ever dropped.

`robot.name` inside the YAML is the identity; the filename is only where that
content lives. Two files can therefore claim one name — a conflict the user has
to see, not one for the listing to resolve quietly.

The old code keyed a `seen` set on the name and `continue`d past a repeat: no
log, no warning, the file simply was not in the list. Measured on a bench box
2026-09-21, three of nineteen configs were invisible in the cockpit, including
the real-hardware config the CLI was driving at the time, because
`pico2_real_config.yaml`, `pico2_realhw_config.yaml` and `pico2w_real_config.yaml`
all declared `robot: {name: pico2_real}`. The CLI resolves a path from
`--robot <stem>` and never lost them, so only the UI came up short.
"""
import os
import sys

import pytest
import yaml

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO_ROOT, "web", "backend"))

# core.py pulls in the FastAPI stack and calls sys.exit(1) when it is missing, so
# probe the dependency first -- importorskip does not catch SystemExit. A dev host
# without fastapi still runs the rest of the suite, and tests/api/test_cockpit_api.py
# covers the same ground against a live cockpit, where those packages are installed
# by definition.
pytest.importorskip("fastapi", reason="the cockpit backend needs fastapi/uvicorn")
import core  # noqa: E402


def write(directory, fname, robot_name, controller="pico2"):
    body = {"base_controller": {"name": controller, "mcu": controller}}
    if robot_name is not None:
        body["robot"] = {"name": robot_name, "description": f"desc for {robot_name}"}
    with open(os.path.join(directory, fname), "w") as fh:
        yaml.safe_dump(body, fh)


@pytest.fixture()
def listing(tmp_path, monkeypatch):
    """core.get_robots_list bound to a scratch config dir.

    CONFIG_DIR is read at import time, so point the module's own binding at the
    scratch directory rather than re-importing (which fights conftest's autouse
    fixture over sys.modules).
    """
    d = tmp_path / "robots"
    d.mkdir()
    monkeypatch.setattr(core, "CONFIG_DIR", str(d), raising=False)
    monkeypatch.setattr(core, "ACTIVE_ROBOT_NAME", "", raising=False)
    monkeypatch.setattr(core, "get_active_params_path",
                        lambda *a, **k: str(d / "__none__.yaml"), raising=False)
    return core, str(d)


def test_no_file_is_dropped_when_three_declare_one_name(listing):
    core, d = listing
    for f in ("pico2_real_config.yaml", "pico2_realhw_config.yaml", "pico2w_real_config.yaml"):
        write(d, f, "pico2_real")
    entries = core.get_robots_list()
    assert len(entries) == 3, "a file must not vanish because another claimed its name"
    assert {e["filename"] for e in entries} == {
        "pico2_real_config.yaml", "pico2_realhw_config.yaml", "pico2w_real_config.yaml"}


def test_the_name_comes_from_the_content_not_the_filename(listing):
    core, d = listing
    write(d, "pico2_realhw_config.yaml", "rover_two")
    (entry,) = core.get_robots_list()
    assert entry["name"] == "rover_two", "robot.name identifies the robot"
    assert entry["filename"] == "pico2_realhw_config.yaml"
    assert entry["select"] == "rover_two", "a unique name is its own handle"
    assert entry["conflict"] is None


def test_a_collision_is_reported_on_every_file_that_claims_the_name(listing):
    core, d = listing
    write(d, "a_config.yaml", "shared")
    write(d, "b_config.yaml", "shared")
    write(d, "c_config.yaml", "alone")
    by_file = {e["filename"]: e for e in core.get_robots_list()}
    assert by_file["a_config.yaml"]["conflict"] == ["b_config.yaml"]
    assert by_file["b_config.yaml"]["conflict"] == ["a_config.yaml"]
    assert by_file["c_config.yaml"]["conflict"] is None


def test_a_colliding_file_is_still_reachable_by_its_stem(listing):
    core, d = listing
    write(d, "pico2_real_config.yaml", "pico2_real")
    write(d, "pico2_realhw_config.yaml", "pico2_real")
    by_file = {e["filename"]: e for e in core.get_robots_list()}
    assert by_file["pico2_realhw_config.yaml"]["select"] == "pico2_realhw"
    assert by_file["pico2_real_config.yaml"]["select"] == "pico2_real"


def test_a_file_with_no_declared_name_falls_back_to_its_stem(listing):
    core, d = listing
    write(d, "unnamed_config.yaml", None)
    (entry,) = core.get_robots_list()
    assert entry["name"] == "unnamed"


def test_dotfiles_are_not_robots(listing):
    """The cockpit keeps its own state here, and bench scripts leave things behind."""
    core, d = listing
    write(d, "bare_pico2_config.yaml", "bare_pico2")
    write(d, ".active_probe.yaml", "pico2_real")
    with open(os.path.join(d, ".active_robot"), "w") as fh:
        fh.write("bare_pico2_config.yaml\n")
    names = [r["name"] for r in core.get_robots_list()]
    assert names == ["bare_pico2"], f"a dotfile was offered as a robot: {names}"


def test_secrets_are_not_robots(listing):
    core, d = listing
    write(d, "bare_pico2_config.yaml", "bare_pico2")
    with open(os.path.join(d, "secrets.yaml"), "w") as fh:
        fh.write("wifi_password: hunter2\n")
    assert [r["name"] for r in core.get_robots_list()] == ["bare_pico2"]


def test_a_collision_does_not_light_up_the_wrong_robot_as_active(listing, monkeypatch):
    """With two files claiming a name, only the open FILE may read as active."""
    core, d = listing
    write(d, "a_config.yaml", "shared")
    write(d, "b_config.yaml", "shared")
    monkeypatch.setattr(core, "ACTIVE_ROBOT_NAME", "shared", raising=False)
    assert [e["active"] for e in core.get_robots_list()] == [False, False]

    monkeypatch.setattr(core, "get_active_params_path",
                        lambda *a, **k: os.path.join(d, "b_config.yaml"), raising=False)
    by_file = {e["filename"]: e for e in core.get_robots_list()}
    assert by_file["b_config.yaml"]["active"] is True
    assert by_file["a_config.yaml"]["active"] is False
