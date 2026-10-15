"""A robot named with a hyphen (mc200-unoq) goes through every path that handles names.

Every fixture in the suite was named with underscores, so the hyphen was met first by the
real MC200: rcl refused its node name (fixed in base_node_name) and the web UI would not
select it (fixed in selectRobot). This keeps one hyphenated robot in the tests.
"""
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "scripts"))
import cockpit_paths  # noqa: E402
import mcu_env  # noqa: E402

NAME = "rover-unoq"
CFG = "robot:\n  name: rover-unoq\nbase_controller:\n  name: unoq\n"


def test_the_config_resolves_by_its_name(tmp_path):
    (tmp_path / f"{NAME}_config.yaml").write_text(CFG)
    path = cockpit_paths.robot_config_path(NAME, str(tmp_path))
    assert os.path.basename(path) == f"{NAME}_config.yaml"
    params = mcu_env.load_yaml(path)
    assert mcu_env.robot_name_of(params, path) == NAME


def test_the_firmware_node_name_is_legal():
    node = cockpit_paths.base_node_name(NAME)
    assert node == "rover_unoq_base_node" and "-" not in node

