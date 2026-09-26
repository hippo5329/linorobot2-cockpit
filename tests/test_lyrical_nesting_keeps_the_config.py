"""The Lyrical nesting must carry the config's values -- and only keys Lyrical reads.

Lyrical (Nav2 1.5.1) nests the primary controller under
FollowPath.primary_controller, where Jazzy reads a flat FollowPath block. The
launcher rewrites one into the other, and every key it moves is taken from the
config with .pop(key, default) so the robot's own tuning survives.

It also used to carry transform_tolerance, max_robot_pose_search_dist and
stateful into that block, with a comment crediting transform_tolerance for the
Lyrical Wi-Fi pass rate. Nav2 1.5.1 moved all three out of RPP (Kilted ->
Lyrical, "Centralize Path Handler logic in Controller Server"): the image's
libnav2_regulated_pure_pursuit_controller.so declares none of them, where
Jazzy's declares all three. Writing them set nothing. A key that reads as
configuration and is not is worse than a missing one.
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


# Declared by Jazzy's RPP, not by Lyrical's (strings of the installed .so, 2026-09-26).
NOT_READ_BY_LYRICAL_RPP = {"transform_tolerance", "max_robot_pose_search_dist", "stateful"}


def test_the_nesting_writes_no_key_lyrical_does_not_read():
    nested = _nested_block()
    keys = {k.value for k in nested.keys if isinstance(k, ast.Constant) and isinstance(k.value, str)}
    dead = sorted(keys & NOT_READ_BY_LYRICAL_RPP)
    assert not dead, f"the Lyrical primary_controller block sets {dead}, which Nav2 1.5.1's RPP never reads"


def _nested_block():
    tree = ast.parse(_src())
    for node in ast.walk(tree):
        if (isinstance(node, ast.Assign) and isinstance(node.value, ast.Dict)
                and any(isinstance(k, ast.Constant) and k.value == "plugin"
                        for k in node.value.keys if k is not None)):
            keys = [k.value for k in node.value.keys
                    if isinstance(k, ast.Constant) and isinstance(k.value, str)]
            if "lookahead_dist" in keys and "desired_linear_vel" in keys:
                return node.value
    raise AssertionError("the Lyrical primary_controller block is gone")


def test_no_key_in_the_nesting_silently_drops_a_configured_value():
    """Any key the flat block can carry must be popped, not invented.

    The trap is specific: a hardcoded value here does not override the config,
    it makes the config unreachable -- the flat key is left behind in a block
    the nested controller never reads.
    """
    nested = _nested_block()

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
