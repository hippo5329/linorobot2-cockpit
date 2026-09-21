"""The simulator must not park the robot inside Nav2's own footprint.

`clampToRoom()` holds the simulated robot's centre `FAKE_ROBOT_RADIUS` from a
wall. Nav2 refuses to plan out of a cell its footprint overlaps. So if the
simulator's radius is smaller than Nav2's `robot_radius`, the simulation itself
places the robot in a lethal cell — and nothing gets it out.

That is not hypothetical. With the emulator's old hardcoded 0.20 m against
configs planning with 0.22–0.26 m, a soak watched a board sit at the wall for
154 consecutive failed goals while the planner kept emitting correct escape
paths the controller would not follow (48 plans in 80 s, 1.2 mm of travel).
"""
import os
import re
import sys

import pytest
import yaml

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO_ROOT, "scripts"))

import gen_firmware_header as gfh  # noqa: E402

REF = os.path.join(REPO_ROOT, "config", "reference")
FAKE_LD19 = os.path.join(REPO_ROOT, "firmware", "common", "lib", "lidar", "fake_ld19.h")


def reference_configs():
    for f in sorted(os.listdir(REF)):
        if f.endswith("_config.yaml"):
            with open(os.path.join(REF, f)) as fh:
                yield f, yaml.safe_load(fh) or {}


def nav2_radii(params):
    nav2 = params.get("nav2") or {}
    out = []
    for cm in ("local_costmap", "global_costmap"):
        node = nav2.get(cm) or {}
        node = node.get(cm) or node
        rp = node.get("ros__parameters", node) or {}
        try:
            out.append(float(rp.get("robot_radius")))
        except (TypeError, ValueError):
            pass
    return out


@pytest.mark.parametrize("name,params", list(reference_configs()))
def test_simulator_radius_clears_nav2_footprint(name, params):
    radii = nav2_radii(params)
    if not radii:
        pytest.skip(f"{name} declares no robot_radius")
    sim = gfh.nav2_robot_radius(params)
    assert sim >= max(radii), (
        f"{name}: the simulator would hold the robot {sim:.3f} m off a wall while "
        f"Nav2 plans with a {max(radii):.3f} m footprint — the clamp would park it "
        f"inside a lethal cell it cannot plan out of"
    )


def test_the_firmware_fallback_clears_every_shipped_radius():
    """An image built without a generated header must still be safe."""
    src = open(FAKE_LD19, encoding="utf-8").read()
    m = re.search(r"#define FAKE_ROBOT_RADIUS\s+([0-9.]+)f", src)
    assert m, "FAKE_ROBOT_RADIUS fallback is gone"
    fallback = float(m.group(1))
    worst = max((max(nav2_radii(p)) for _, p in reference_configs() if nav2_radii(p)),
                default=0.0)
    assert worst > 0.0
    assert fallback >= worst, (
        f"fallback {fallback} is under the widest shipped robot_radius {worst}; "
        f"a header-less build would park the robot inside Nav2's footprint"
    )


def test_a_wider_robot_widens_the_simulated_one():
    """Derivation, not a constant: the two must move together."""
    base = {"nav2": {"local_costmap": {"local_costmap": {"ros__parameters": {"robot_radius": 0.26}}}}}
    wide = {"nav2": {"local_costmap": {"local_costmap": {"ros__parameters": {"robot_radius": 0.50}}}}}
    assert gfh.nav2_robot_radius(wide) > gfh.nav2_robot_radius(base)


def test_the_larger_of_the_two_costmaps_wins():
    """Either costmap refusing to plan strands the robot, so take the worst."""
    p = {"nav2": {
        "local_costmap": {"local_costmap": {"ros__parameters": {"robot_radius": 0.20}}},
        "global_costmap": {"global_costmap": {"ros__parameters": {"robot_radius": 0.40}}}}}
    assert gfh.nav2_robot_radius(p) >= 0.40


def test_a_config_with_no_nav2_still_builds():
    assert gfh.nav2_robot_radius({}) == gfh.FAKE_RADIUS_FALLBACK_M
