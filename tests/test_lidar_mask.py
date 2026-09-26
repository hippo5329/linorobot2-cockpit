"""The LiDAR mask removes the robot's own structure from /scan, and the bench can prove it.

laser_filters between the driver (scan_raw) and /scan, as upstream documents,
with the two details that decide whether it works: NaN, never range_max + 1
(which the costmap reads as "clear to here" and clears through the mast), and
sectors emitted as raw intervals, because lyrical's laser_filters has no wrap
handling and the LD driver (0..2 pi) and the host laser (-pi..pi) disagree on
the angle range. The firmware's simulated LD19 and the host laser can be given
the posts to see (simulation.lidar_occlusion), so a Sim MCU run shows the mask
working.
"""
import math
import os
import sys

import pytest

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO_ROOT, "scripts"))

import lidar_mask as lm  # noqa: E402


def read(*p):
    return open(os.path.join(REPO_ROOT, *p)).read()


def masked_by(intervals, angle):
    return any(lo < angle < hi for lo, hi in intervals)


def test_a_sector_behind_the_robot_masks_both_scan_conventions():
    iv = lm.scan_intervals(150, 60)
    assert masked_by(iv, math.pi), "LD driver scan, 0..2 pi: straight behind is pi"
    assert masked_by(iv, -math.pi + 0.01), "host laser scan, -pi..pi: straight behind is -pi"
    assert not masked_by(iv, 0.0) and not masked_by(iv, math.radians(100))


def test_a_sector_across_zero_is_split_not_lost():
    iv = lm.scan_intervals(350, 20)
    for deg in (355, 5, -5):
        assert masked_by(iv, math.radians(deg)), deg
    assert not masked_by(iv, math.radians(20))


def test_the_mount_yaw_is_taken_off():
    """A LiDAR mounted turned 90 deg left sees the robot's 'behind' at its own -90."""
    iv = lm.scan_intervals(170, 20, laser_yaw_rad=math.pi / 2)
    assert masked_by(iv, math.radians(90)) and not masked_by(iv, math.pi)


def test_masked_beams_are_nan_never_range_max_plus_one():
    chain = lm.filter_chain_params({"lidar": {"mask": {"sectors": [[150, 210]]}}}, 0.0)
    stages = chain["scan_to_scan_filter_chain"]["ros__parameters"]
    angular = [f for f in stages.values() if f["type"].endswith("AngularBoundsFilterInPlace")]
    assert angular and all(f["params"]["replace_with_nan"] is True for f in angular)


def test_a_box_is_removed_in_base_link():
    box = {"min_x": -0.1, "max_x": 0.1, "min_y": -0.05, "max_y": 0.05, "min_z": -1, "max_z": 1}
    chain = lm.filter_chain_params({"lidar": {"mask": {"boxes": [box]}}}, 0.0)
    (f,) = chain["scan_to_scan_filter_chain"]["ros__parameters"].values()
    assert f["type"] == "laser_filters/LaserScanBoxFilter"
    assert f["params"]["box_frame"] == "base_link" and f["params"]["invert"] is False


def test_nothing_configured_is_no_chain():
    assert lm.filter_chain_params({"lidar": {"model": "ld19"}}, 0.0) == {}
    assert not lm.masked({"lidar": {}})


@pytest.mark.parametrize("bad", [[[10]], [[5, 5]], "150-210"])
def test_a_bad_sector_is_refused(bad):
    with pytest.raises(ValueError):
        lm.mask_config({"lidar": {"mask": {"sectors": bad}}})


def test_the_occlusion_reaches_the_board_and_the_host():
    params = {"base_controller": {"simulation": {"lidar_occlusion": [[150, 210], [350, 10]],
                                                 "lidar_occlusion_range": 0.1}}}
    assert lm.occlusion_env(params) == {"sim_occl": "150,60,350,20", "sim_occl_r": "0.1"}
    fw = read("firmware", "common", "lib", "lidar", "sim_ld19.h")
    assert 'envGet("sim_occl"' in fw and 'envFloat("sim_occl_r"' in fw
    assert "360.0f - beam_deg" in fw, "the LD19 angle runs clockwise; the sectors do not"
    assert lm.occluded(180, lm.occlusion(params)[0]) and not lm.occluded(90, lm.occlusion(params)[0])


def test_mcu_env_writes_the_occlusion():
    src = read("scripts", "mcu_env.py")
    assert "lidar_mask.occlusion_env" in src


def test_bringup_puts_the_filter_between_the_driver_and_scan():
    launch = read("launchers", "bringup.launch.py")
    assert 'lidar_topic = "scan_raw" if lidar_masked else "scan"' in launch
    assert launch.count('"topic_name": lidar_topic,') == 2
    assert 'remappings=[("scan", "scan_raw"), ("scan_filtered", "scan")]' in launch
    assert "ros-${ROS_DISTRO}-laser-filters" in read("docker", "Dockerfile")
