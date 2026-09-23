"""The sonar has a consumer.

The firmware has published sensor_msgs/Range on `sonar` all along -- the fake
one is on by default (mcu_env.py: use_fake_sonar defaults true, armed whenever
the fake LiDAR runs) and a real HC-SR04 publishes the same topic. Nothing read
it. Not a costmap layer, not the collision monitor, nothing. The LiDAR was the
only obstacle input in the stack, so when it was wrong there was nothing to
contradict it -- and on 2026-09-23 a base asked to return to (0, 0) drove
11.235 m away instead, with no Nav2 complaint logged at all.

The frame is the part that fails silently: the firmware stamps Range with
envPrefixed("sonar_link"), and a source whose frame is not in the TF tree is
dropped without a word. So the description publishes sonar_link whether or not
a sonar is fitted.
"""
import glob
import os

import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LAUNCHER = os.path.join(ROOT, "launchers", "nav2.launch.py")
GEN = os.path.join(ROOT, "scripts", "gen_robot_description.py")


def _read(p):
    with open(p, encoding="utf-8") as fh:
        return fh.read()


def _configs():
    return sorted(glob.glob(os.path.join(ROOT, "config", "reference", "*_config.yaml")))


def test_every_reference_config_listens_to_the_sonar():
    for f in _configs():
        d = yaml.safe_load(_read(f))
        cm = (d.get("nav2") or {}).get("collision_monitor", {}).get("ros__parameters")
        if not cm:
            continue
        assert "sonar" in cm["observation_sources"], f
        src = cm["sonar"]
        assert src["type"] == "range", f
        assert src["topic"] == "sonar", f
        assert src["enabled"] is True, f


def test_the_beam_does_not_exceed_the_monitors_point_ceiling():
    """point_count = ceil(field_of_view / obstacles_angle) + 1, and the
    firmware's cone is FAKE_SONAR_CONE_DEG = 30 degrees."""
    import math
    for f in _configs():
        d = yaml.safe_load(_read(f))
        cm = (d.get("nav2") or {}).get("collision_monitor", {}).get("ros__parameters")
        if not cm:
            continue
        ang = cm["sonar"]["obstacles_angle"]
        assert ang > 0, f
        points = math.ceil(math.radians(30.0) / ang) + 1
        assert points < 100, (f, points)


def test_the_launcher_default_matches_so_a_bare_config_gets_it_too():
    src = _read(LAUNCHER)
    assert '"observation_sources": ["scan", "sonar"]' in src
    assert '"type": "range"' in src and '"topic": "sonar"' in src


def test_the_description_publishes_the_frame_the_firmware_stamps():
    """Silently dropped otherwise, which is the whole trap."""
    gen = _read(GEN)
    assert 'DEFAULT_SONAR_FRAME = "sonar_link"' in gen
    assert 'ET.SubElement(robot, "link", name=sonar_frame)' in gen
    assert 'f"{sonar_frame}_to_base_link", "fixed", BASE_FRAME, sonar_frame' in gen


def test_the_sonar_looks_forward_from_the_front_face():
    """A range sensor sunk into the body reads the chassis at its minimum
    range for ever, and the collision monitor would believe it."""
    import sys
    sys.path.insert(0, os.path.join(ROOT, "scripts"))
    import gen_robot_description as g
    params = {"kinematics": {"base_type": "2wd", "wheel_diameter": 0.1,
                             "lr_wheels_distance": 0.27, "fr_wheels_distance": 0.2}}
    geo = g.geometry(params) if hasattr(g, "geometry") else None
    if geo is None:                      # name differs; fall back to the source
        assert 'sonar.setdefault("x", round(_f(body["length"]) / 2.0, 4))' in _read(GEN)
        return
    assert geo["sonar"]["x"] > 0, geo["sonar"]
    assert geo["sonar"]["frame"] == "sonar_link"
