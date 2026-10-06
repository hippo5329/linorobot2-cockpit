"""The mask covers the beam next to each sector edge (2026-10-06).

The simulated LD19 occluding [105, 255) put a 0.12 m return at 104.4 deg into every scan
of an exploration leg: the LD driver publishes a beam up to one beam (0.79 deg) from
where it was measured, and the mask stopped exactly at 105. SLAM and the costmap drew
the robot's own trail as a wall and it closed the only door to the far rooms.
"""
import math
import os
import sys

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "scripts"))

import lidar_mask  # noqa: E402


def intervals(controller):
    chain = lidar_mask.filter_chain_params(controller, 0.0)["scan_to_scan_filter_chain"]["ros__parameters"]
    return [(f["params"]["lower_angle"], f["params"]["upper_angle"]) for f in chain.values()
            if f["type"].endswith("AngularBoundsFilterInPlace")]


def covered(angle_deg, ivs):
    a = math.radians(angle_deg)
    return any(lo <= a <= hi for lo, hi in ivs)


def ctrl(**mask):
    return {"lidar": {"mask": dict({"sectors": [[105, 255]]}, **mask)}}


def test_the_beam_published_just_outside_an_edge_is_masked():
    ivs = intervals(ctrl())
    assert covered(104.4, ivs), "the 104.4 deg beam that leaked in the exploration leg"
    assert covered(255.6, ivs)
    assert not covered(103.5, ivs) and not covered(256.5, ivs), "one beam, not more"


def test_a_zero_margin_is_the_bare_sector():
    ivs = intervals(ctrl(margin_deg=0))
    assert not covered(104.4, ivs) and covered(105.5, ivs)


def test_the_margin_is_validated():
    for bad in (-1, 45, "wide"):
        with pytest.raises(ValueError):
            lidar_mask.mask_config(ctrl(margin_deg=bad))


def test_a_sector_never_grows_to_the_whole_circle():
    ivs = intervals({"lidar": {"mask": {"sectors": [[0.5, 0.0]], "margin_deg": 5}}})
    span = sum(hi - lo for lo, hi in ivs if lo >= 0)
    assert span < 2 * math.pi
