"""A reference robot is called what its file is called, and never `bare_*`.

`robot.name` is not a label: it becomes the firmware's ROS node
(`<name>_base_node`, mcu_env.py), the URDF and wiring-sheet name, and the key the
backend falls back to when it looks a robot up by name. xrp_config.yaml shipped as
`bare_xrp`, described as a bare module with every pin N/C -- the SparkFun XRP Kit,
with its motors, encoders and IMU all wired -- so the kit's node carried the name of
the zero-wiring default. `bare_<mcu>` names belong to gen_bare_config's rule.
2026-10-04.
"""
import glob
import os
import re

import yaml

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
REFERENCE = sorted(glob.glob(os.path.join(REPO_ROOT, "config", "reference", "*_config.yaml")))


def test_there_are_reference_configs():
    assert REFERENCE


def test_robot_name_is_the_file_name():
    for path in REFERENCE:
        stem = os.path.basename(path)[: -len("_config.yaml")]
        robot = (yaml.safe_load(open(path, encoding="utf-8")) or {}).get("robot") or {}
        assert robot.get("name") == stem, f"{os.path.basename(path)}: robot.name is {robot.get('name')!r}"


def test_no_reference_robot_claims_to_be_a_bare_module():
    for path in REFERENCE:
        robot = (yaml.safe_load(open(path, encoding="utf-8")) or {}).get("robot") or {}
        assert not str(robot.get("name", "")).startswith("bare_"), path
        assert "bare module" not in str(robot.get("description", "")).lower(), path


def test_operations_tab_names_the_stages_in_the_pipelines_order():
    """The Tab 11 card said `Map saver -> Nav2`; the pipeline runs Nav2 (stage 6),
    then the drive suite (6.5), and saves the map last."""
    html = open(os.path.join(REPO_ROOT, "web", "frontend", "index.html"), encoding="utf-8").read()
    hint = re.search(r"Executes the automated end-to-end mission[^<]*", html).group(0)
    assert hint.index("Nav2") < hint.index("Map saver")
    pipe = open(os.path.join(REPO_ROOT, "scripts", "one_click_pipeline.py"), encoding="utf-8").read()
    assert pipe.index("[6/6] [NAV2] Launching Nav2") < pipe.index("[MAP] Saving the map")
