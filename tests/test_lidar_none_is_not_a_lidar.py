"""`lidar.model: NONE` is not a LiDAR.

The pipeline decided has_lidar with bool(model), and "NONE" is truthy: a robot with
no LiDAR was made to wait for /scan and failed topic verification, aborted before
SLAM even in --topics-only. Found 2026-09-25 on a Pico 2 with a real LSM6DSOX and
no LiDAR. Same class as the `current: NONE` -> /battery defect NOT_FITTED exists for.
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "scripts"))
import one_click_pipeline as ocp  # noqa: E402


def test_none_spellings_are_not_a_lidar():
    for m in ("NONE", "none", "", None, "off", "false"):
        assert not ocp.lidar_fitted({"lidar": {"model": m}}), m


def test_a_named_lidar_is_one():
    assert ocp.lidar_fitted({"lidar": {"model": "ld19"}})


def test_the_simulated_lidar_counts_without_a_model():
    assert ocp.lidar_fitted({"lidar": {"model": "NONE"}, "sensors": {"use_sim_ld19": True}})
    assert not ocp.lidar_fitted({"sensors": {"use_sim_ld19": False}})


def test_the_pipeline_decides_with_it():
    src = open(ocp.__file__, encoding="utf-8").read()
    assert "has_lidar = lidar_fitted(controller_cfg)" in src
