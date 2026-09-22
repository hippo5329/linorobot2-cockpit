"""One default chassis, everywhere.

A Nav2 fake-mode failure can only be attributed to a board if every default config
shares the same chassis: the same body and sensor mounts (laser at the base origin),
the same costmap radii, consistent frame names, the same bare kinematics, and no
inherited hardware quirks. The Yahboom reference failed the Nav2 goal on both
transports and both distros while every bare board passed, and the config was the
only variable: a laser 0.12 m ahead, a different Nav2 template (0.22/0.7,
base_footprint for EKF/SLAM against base_link in the costmaps) and a motor invert
copied from the vendor's firmware. 2026-09-22.
"""
import glob
import os
import re
import sys

import yaml

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
REF = os.path.join(REPO_ROOT, "config", "reference")
sys.path.insert(0, os.path.join(REPO_ROOT, "scripts"))
import gen_robot_description  # noqa: E402
import gen_firmware_header    # noqa: E402

DEFAULT_KINEMATICS = {"wheel_diameter": 0.152, "lr_wheels_distance": 0.271,
                      "max_rpm": 140, "counts_per_rev": 4000}
ROBOT_RADIUS, INFLATION = 0.26, 0.55


def _refs():
    for path in sorted(glob.glob(os.path.join(REF, "*_config.yaml"))):
        with open(path) as fh:
            yield os.path.basename(path), yaml.safe_load(fh)


def _walk(node, key):
    """Every value of `key` anywhere in a nested dict."""
    if isinstance(node, dict):
        for k, v in node.items():
            if k == key:
                yield v
            yield from _walk(v, key)
    elif isinstance(node, list):
        for v in node:
            yield from _walk(v, key)


def test_no_default_config_inverts_a_motor_or_encoder():
    for name, params in _refs():
        pins = params.get("base_controller", {}).get("pins", {})
        for unit, cfg in pins.items():
            if isinstance(cfg, dict) and "invert" in cfg:
                assert cfg["invert"] is False, f"{name}: {unit}.invert is on -- the default is forward"
    presets = open(os.path.join(REPO_ROOT, "web", "frontend", "app-presets.js")).read()
    assert not re.search(r"invert:\s*true", presets), "a preset inverts a motor or encoder"


def test_every_reference_puts_the_laser_at_the_base_origin():
    for name, params in _refs():
        geo = gen_robot_description.effective_geometry(params)["laser"]
        assert (geo["x"], geo["y"], geo["yaw"]) == (0.0, 0.0, 0.0), \
            f"{name}: laser at ({geo['x']}, {geo['y']}, yaw {geo['yaw']}) -- same mounts everywhere"


def test_every_reference_shares_the_costmap_radii():
    for name, params in _refs():
        radii = set(_walk(params, "robot_radius"))
        infl = set(_walk(params, "inflation_radius"))
        assert radii <= {ROBOT_RADIUS}, f"{name}: robot_radius {radii}"
        assert infl <= {INFLATION}, f"{name}: inflation_radius {infl}"


def test_every_reference_agrees_on_the_base_frame():
    for name, params in _refs():
        costmap_base = set(_walk(params, "robot_base_frame"))
        for key in ("base_link_frame", "base_frame", "base_frame_id"):
            for v in _walk(params, key):
                assert {v} == costmap_base, \
                    f"{name}: {key}={v} but the costmaps use {costmap_base}"


def test_bare_kinematics_are_the_same_in_the_presets_and_the_release_image():
    presets = open(os.path.join(REPO_ROOT, "web", "frontend", "app-presets.js")).read()
    for m in re.finditer(r'id: "(bare_[a-z0-9]+)"(.*?)pins:', presets, re.S):
        block = m.group(2)
        for key, want in (("wheel_diameter", 0.152), ("lr_wheels_distance", 0.271),
                          ("max_rpm", 140), ("cpr", 4000)):
            got = re.search(rf"\b{key}: ([0-9.]+)", block)
            assert got and float(got.group(1)) == want, f"{m.group(1)}: {key} {got and got.group(1)}"
    bare = gen_firmware_header.release_params("esp32") if hasattr(gen_firmware_header, "release_params") else None
    if bare is None:
        src = open(os.path.join(REPO_ROOT, "scripts", "gen_firmware_header.py")).read()
        for key, want in DEFAULT_KINEMATICS.items():
            assert re.search(rf'"{key}": {want}\b', src), f"release bare config: {key} != {want}"
    else:
        for key, want in DEFAULT_KINEMATICS.items():
            assert bare["kinematics"][key] == want
