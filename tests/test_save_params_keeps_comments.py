"""The web UI's config save keeps the robot config's comments.

core.save_params dumped the whole file with yaml.dump, so every save from the UI -- a
distro switch in the header, the LiDAR page -- threw away the user's reasons written
beside the values (the tracked robot's DDS domain, LiDAR mask and BMS cutoff, 2026-10-16).
"""
import os
import sys

import pytest
import yaml

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO_ROOT, "web", "backend"))
pytest.importorskip("fastapi", reason="the cockpit backend needs fastapi/uvicorn")
os.environ["COCKPIT_AUTH"] = "off"
import core  # noqa: E402

TEXT = """robot:
  name: rover_gendrv
base_controller:
  # Its own DDS domain: two robots share the floor.
  domain_id: 10
  battery:
    min_v: 9.3   # BMS cutoff 3.0 V per cell
nav2:
  velocity_smoother:
    max_velocity:
    - 0.23
    - 0.0
    - 0.73
"""


def _comments(text):
    return [l[l.index("#"):] for l in text.splitlines() if "#" in l]


def test_a_distro_switch_keeps_every_comment(tmp_path):
    cfg = tmp_path / "robot_config.yaml"
    cfg.write_text(TEXT)
    data = core.load_params(str(cfg))
    data["ros_distro"] = "jazzy"                  # what the header's distro switch saves
    data["base_controller"]["battery"]["min_v"] = 9.5
    core.save_params(data, path=str(cfg))
    after = cfg.read_text()
    assert _comments(after) == _comments(TEXT)
    assert yaml.safe_load(after) == data
    assert "    min_v: 9.5   # BMS cutoff 3.0 V per cell\n" in after


def test_a_change_it_cannot_place_still_saves_the_data(tmp_path):
    cfg = tmp_path / "robot_config.yaml"
    cfg.write_text(TEXT)
    data = core.load_params(str(cfg))
    data["nav2"]["velocity_smoother"]["max_velocity"] = [0.2, 0.0]   # a shorter list
    core.save_params(data, path=str(cfg))
    assert yaml.safe_load(cfg.read_text()) == data


def test_a_new_file_is_written(tmp_path):
    cfg = tmp_path / "new_config.yaml"
    core.save_params({"robot": {"name": "x"}}, path=str(cfg))
    assert yaml.safe_load(cfg.read_text()) == {"robot": {"name": "x"}}
