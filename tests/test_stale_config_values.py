"""Configs from before a convention changed fail a run without saying so. On
2026-09-25 the UI's Start 1-Click ran the cells' own robots, not the gate's:
`pico` had the EKF parenting base_footprint (TF split, Nav2's controller never
configured), `rover_pico2` removed gravity a second time and named its IMU
FAKE. The pipeline refuses each; the migrator rewrites each as text."""
import os
import sys

import yaml

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(HERE, "scripts"))

import gen_bare_config  # noqa: E402
import migrate_config_schema as mig  # noqa: E402

# The shapes found in the cells, trimmed to what matters.
PICO = """\
base_controller:
  name: pico
  sensors:
    imu: MPU6050   # the old default
    mag: NONE
ekf:
  ekf_filter_node:
    ros__parameters:
      base_link_frame: base_footprint   # was the fashion
      world_frame: odom
slam:
  slam_toolbox:
    ros__parameters:
      base_frame: base_footprint
"""
ROVER = """\
base_controller:
  name: pico2
  sensors:
    imu: FAKE
    mag: "fake"
ekf:
  base_link_frame: base_link
  imu0_remove_gravitational_acceleration: true
"""


def _write(tmp_path, text, name="robot_config.yaml"):
    p = tmp_path / name
    p.write_text(text)
    return p


def test_each_stale_value_is_named():
    pico = mig.stale_faults(yaml.safe_load(PICO))
    assert len(pico) == 1 and "base_footprint" in pico[0]
    rover = mig.stale_faults(yaml.safe_load(ROVER))
    assert len(rover) == 3
    assert any("gravit" in f for f in rover)
    assert any("sensors.imu" in f for f in rover) and any("sensors.mag" in f for f in rover)


def test_the_migrator_fixes_them_and_keeps_comments(tmp_path):
    for text in (PICO, ROVER):
        p = _write(tmp_path, text)
        assert mig.fix_stale_values(str(p), dry_run=False)
        out = p.read_text()
        assert mig.stale_faults(yaml.safe_load(out)) == []
        assert not mig.fix_stale_values(str(p), dry_run=False), "a second pass must be a no-op"
    p = _write(tmp_path, PICO)
    mig.fix_stale_values(str(p), dry_run=False)
    out = p.read_text()
    assert "base_link_frame: base_link   # was the fashion" in out
    assert "imu: MPU6050   # the old default" in out
    # only the EKF's frame: SLAM's base_frame hangs off base_link and is valid
    assert "base_frame: base_footprint" in out


def test_dry_run_writes_nothing(tmp_path):
    p = _write(tmp_path, ROVER)
    assert mig.fix_stale_values(str(p), dry_run=True)
    assert p.read_text() == ROVER


def test_migrate_file_runs_it(tmp_path):
    p = _write(tmp_path, ROVER)
    assert mig.migrate_file(str(p), dry_run=False) == [str(p)]
    assert mig.stale_faults(yaml.safe_load(p.read_text())) == []


def test_nothing_shipped_is_stale():
    ref = os.path.join(HERE, "config", "reference")
    for f in sorted(os.listdir(ref)):
        if f.endswith(".yaml"):
            with open(os.path.join(ref, f)) as fh:
                assert mig.stale_faults(yaml.safe_load(fh)) == [], f
    for board in gen_bare_config.BOARDS:
        assert mig.stale_faults(gen_bare_config.bare_config(board)) == [], board
