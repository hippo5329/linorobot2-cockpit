"""Configs from older releases fail a run without saying so. On 2026-09-25 the
UI's Start 1-Click ran the cells' own robots, not the gate's: `pico` had the EKF
parenting base_footprint (TF split, Nav2's controller never configured),
`rover_pico2` removed gravity a second time and named its IMU FAKE. The pipeline
refuses each before touching the board, and says to REPLACE the file -- the
user's call on 2026-09-25: "overwrite. it is easier than migrating"."""
import os
import sys

import yaml

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(HERE, "scripts"))

import gen_bare_config  # noqa: E402
import one_click_pipeline as ocp  # noqa: E402

PICO = """\
base_controller:
  sensors: {imu: MPU6050, mag: NONE}
ekf:
  ekf_filter_node:
    ros__parameters:
      base_link_frame: base_footprint
"""
ROVER = """\
base_controller:
  sensors: {imu: FAKE, mag: fake, use_fake_wheel: true}
ekf:
  base_link_frame: base_link
  imu0_remove_gravitational_acceleration: true
"""


def test_each_stale_value_is_named():
    pico = ocp.stale_faults(yaml.safe_load(PICO))
    assert len(pico) == 1 and "base_footprint" in pico[0]
    rover = ocp.stale_faults(yaml.safe_load(ROVER))
    assert len(rover) == 4
    assert any("use_fake_wheel" in f for f in rover)
    assert any("gravit" in f for f in rover)
    assert any("sensors.imu" in f for f in rover) and any("sensors.mag" in f for f in rover)


def test_absent_keys_are_not_faults():
    # robot_localization's own default frame is base_link, and the launcher
    # writes imu0_remove_gravitational_acceleration=False when it is absent
    assert ocp.stale_faults({"ekf": {"two_d_mode": True}}) == []


def test_nothing_shipped_or_generated_is_stale():
    ref = os.path.join(HERE, "config", "reference")
    for f in sorted(os.listdir(ref)):
        if f.endswith(".yaml"):
            with open(os.path.join(ref, f)) as fh:
                assert ocp.stale_faults(yaml.safe_load(fh)) == [], f
    for board in gen_bare_config.BOARDS:
        assert ocp.stale_faults(gen_bare_config.bare_config(board)) == [], board


def test_the_refusal_says_replace_not_migrate():
    src = open(os.path.join(HERE, "scripts", "one_click_pipeline.py")).read()
    i = src.index("stale = stale_faults(params)")
    assert "Replace it with a fresh config" in src[i:i + 500]
    assert i < src.index("[1/6] [CONFIG]")
