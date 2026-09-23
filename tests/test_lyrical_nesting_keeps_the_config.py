"""The Lyrical nesting must carry the config's values, not substitute its own.

Lyrical (Nav2 >= 1.5.1 / Kilted) nests the primary controller under
FollowPath.primary_controller, where Jazzy reads a flat FollowPath block. The
launcher rewrites one into the other -- and every key it moves is taken from
the config with .pop(key, default) so the robot's own tuning survives.

transform_tolerance alone was hardcoded to 0.1 while the shipped configs say
0.3. Same robot, same link, same firmware, three times less tolerance for a
stale transform on one distro than the other -- and the config's value was not
overridden, it was discarded, so no amount of editing the YAML could change it.

It is not academic. map->odom does stall: 601 ms measured on a serial leg from
a publisher configured at 50 Hz. A 0.3 s tolerance loses to a stall that long
and so does 0.1, but 0.1 also loses to every shorter one -- and the GenDrv
Wi-Fi legs, which run both distros off the same board and link, passed 88% on
Jazzy against 75% on Lyrical across every run since 2026-09-22.
"""
import ast
import glob
import os

import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LAUNCHER = os.path.join(ROOT, "launchers", "nav2.launch.py")


def _src():
    with open(LAUNCHER, encoding="utf-8") as fh:
        return fh.read()


def test_the_tolerance_comes_from_the_config():
    src = _src()
    assert '"transform_tolerance": follow_path.pop("transform_tolerance", 0.3),' in src, \
        "the Lyrical nesting hardcodes a transform_tolerance again"


def test_no_key_in_the_nesting_silently_drops_a_configured_value():
    """Any key the flat block can carry must be popped, not invented.

    The trap is specific: a hardcoded value here does not override the config,
    it makes the config unreachable -- the flat key is left behind in a block
    the nested controller never reads.
    """
    src = _src()
    tree = ast.parse(src)
    nested = None
    for node in ast.walk(tree):
        if (isinstance(node, ast.Assign) and isinstance(node.value, ast.Dict)
                and any(isinstance(k, ast.Constant) and k.value == "plugin"
                        for k in node.value.keys if k is not None)):
            keys = [k.value for k in node.value.keys
                    if isinstance(k, ast.Constant) and isinstance(k.value, str)]
            if "transform_tolerance" in keys and "lookahead_dist" in keys:
                nested = node.value
                break
    assert nested is not None, "the Lyrical primary_controller block is gone"

    configured = set()
    for f in glob.glob(os.path.join(ROOT, "config", "reference", "*_config.yaml")):
        with open(f, encoding="utf-8") as fh:
            d = yaml.safe_load(fh)
        fp = ((d.get("nav2") or {}).get("controller_server", {})
              .get("ros__parameters", {}).get("FollowPath") or {})
        configured |= {k for k, v in fp.items() if not isinstance(v, dict)}

    popped = set()
    for k, v in zip(nested.keys, nested.values):
        if not (isinstance(k, ast.Constant) and isinstance(k.value, str)):
            continue
        if (isinstance(v, ast.Call) and isinstance(v.func, ast.Attribute)
                and v.func.attr == "pop"):
            popped.add(k.value)

    nested_keys = {k.value for k in nested.keys
                   if isinstance(k, ast.Constant) and isinstance(k.value, str)}
    # a key the configs actually set, that the nesting also sets, must be popped
    must_carry = (configured & nested_keys) - {"plugin", "primary_controller"}
    missing = sorted(must_carry - popped)
    assert not missing, (
        f"the Lyrical nesting hardcodes {missing}, which the reference configs set -- "
        f"the flat value is then unreachable rather than overridden")
