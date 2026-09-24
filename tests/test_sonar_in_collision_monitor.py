"""The sonar has a consumer -- on a robot that has a sonar.

The firmware has published sensor_msgs/Range on `sonar` all along (a real
HC-SR04 when the pins name one, a raycast from the sim LiDAR room otherwise)
and nothing read it. Not a costmap layer, not the collision monitor, nothing.
The LiDAR was the only obstacle input in the stack, so when it was wrong there
was nothing to contradict it.

Giving the collision monitor that second input is right, and giving it to a
board with no sonar is a parking brake. The node's documented fail-safe is:

    [collision_monitor]: Robot to stop due to invalid source.
    Either due to data not published yet, or to lack of new data

Correct for a sensor that died mid-drive; fatal for one that was never fitted.
On 2026-09-23 the source reached every bench board -- the bench variants
inherit the reference's whole Nav2 block and replace only base_controller, so
the source travelled while the pins did not -- and all three drivetrains
stalled mid-route while the drive suite scored 8/8 seconds later.

So the pins decide, in the launcher, over whatever the config says.

The frame is the other silent failure: the firmware stamps Range with
envPrefixed("sonar_link"), and a source whose frame is not in the TF tree is
dropped without a word. The description publishes sonar_link whether or not a
sonar is fitted -- it costs one static transform and removes the trap.
"""
import glob
import math
import os
import sys

import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LAUNCHER = os.path.join(ROOT, "launchers", "nav2.launch.py")
GEN = os.path.join(ROOT, "scripts", "gen_robot_description.py")

sys.path.insert(0, os.path.join(ROOT, "launchers"))


def _read(p):
    with open(p, encoding="utf-8") as fh:
        return fh.read()


def _configs():
    return sorted(glob.glob(os.path.join(ROOT, "config", "reference", "*_config.yaml")))


def _pins(d):
    return ((d.get("base_controller") or {}).get("pins") or {}).get("sonar") or {}


def _cm(d):
    return (d.get("nav2") or {}).get("collision_monitor", {}).get("ros__parameters")


def _launcher_func(name):
    """Just the one function, without importing the module.

    nav2.launch.py imports `launch`, which only exists inside a sourced ROS
    environment, and this suite runs on a bare host. Lifting the function out
    of the AST tests the real code without pretending the host is a robot.
    """
    import ast
    tree = ast.parse(_read(LAUNCHER))
    for node in tree.body:
        if isinstance(node, ast.FunctionDef) and node.name == name:
            ns = {}
            exec(compile(ast.Module([node], []), LAUNCHER, "exec"), ns)
            return ns[name]
    raise AssertionError(f"{name}() is gone from {LAUNCHER}")


def test_only_a_robot_with_sonar_pins_declares_the_source():
    """Both directions: fitted means listed, unfitted means absent."""
    seen_fitted = False
    for f in _configs():
        d = yaml.safe_load(_read(f))
        cm = _cm(d)
        if not cm:
            continue
        pins = _pins(d)
        fitted = int(pins.get("trigger", -1)) >= 0 and int(pins.get("echo", -1)) >= 0
        listed = "sonar" in cm["observation_sources"]
        assert listed == fitted, (
            f"{os.path.basename(f)}: pins say fitted={fitted} but the collision "
            f"monitor says listed={listed}"
        )
        if fitted:
            seen_fitted = True
            src = cm["sonar"]
            assert src["type"] == "range", f
            assert src["topic"] == "sonar", f
            assert src["enabled"] is True, f
        else:
            assert "sonar" not in cm, f"{f}: an unused source block invites re-adding it"
    assert seen_fitted, "no reference config exercises the fitted case any more"


def test_the_beam_does_not_exceed_the_monitors_point_ceiling():
    """point_count = ceil(field_of_view / obstacles_angle) + 1, and the
    firmware's cone is SIM_SONAR_CONE_DEG = 30 degrees."""
    for f in _configs():
        d = yaml.safe_load(_read(f))
        cm = _cm(d)
        if not cm or "sonar" not in cm:
            continue
        ang = cm["sonar"]["obstacles_angle"]
        assert ang > 0, f
        points = math.ceil(math.radians(30.0) / ang) + 1
        assert points < 100, (f, points)


def test_has_real_sonar_reads_the_pins():
    has_real_sonar = _launcher_func("has_real_sonar")
    assert has_real_sonar({"base_controller": {"pins": {"sonar": {"trigger": 27, "echo": 28}}}})
    assert not has_real_sonar({"base_controller": {"pins": {"sonar": {"trigger": -1, "echo": -1}}}})
    # one pin wired and one not is not a sonar, it is a wiring mistake
    assert not has_real_sonar({"base_controller": {"pins": {"sonar": {"trigger": 27, "echo": -1}}}})
    # a config that never mentions a sonar, and junk in the field
    assert not has_real_sonar({})
    assert not has_real_sonar({"base_controller": {"pins": {"sonar": {"trigger": "", "echo": ""}}}})


def test_the_launcher_default_follows_the_pins():
    src = _read(LAUNCHER)
    assert '"observation_sources": ["scan", "sonar"] if sonar_fitted else ["scan"]' in src
    assert '"type": "range"' in src and '"topic": "sonar"' in src


def test_a_config_may_not_ask_for_a_sonar_the_robot_does_not_have():
    """The bench variants inherit the reference's Nav2 block wholesale. The
    launcher has to overrule them, or the pins are advice rather than fact."""
    src = _read(LAUNCHER)
    assert 'if not sonar_fitted and "sonar" in (cm_params.get("observation_sources") or []):' in src
    assert 'cm_params.pop("sonar", None)' in src


def test_the_description_publishes_the_frame_the_firmware_stamps():
    """Silently dropped otherwise, which is the whole trap."""
    gen = _read(GEN)
    assert 'DEFAULT_SONAR_FRAME = "sonar_link"' in gen
    assert 'ET.SubElement(robot, "link", name=sonar_frame)' in gen
    assert 'f"{sonar_frame}_to_base_link", "fixed", BASE_FRAME, sonar_frame' in gen


def test_the_sonar_looks_forward_from_the_front_face():
    """A range sensor sunk into the body reads the chassis at its minimum
    range for ever, and the collision monitor would believe it."""
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
