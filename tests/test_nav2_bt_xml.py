"""A robot's own behaviour tree, named relative to its config, is found beside the config wherever it is installed."""
import os
import types

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _fn():
    src = open(os.path.join(ROOT, "launchers", "nav2.launch.py")).read()
    a = src.index("def resolve_relative_bt_xml(")
    ns = {"os": os}
    exec(src[a:src.index("\ndef ", a + 10)], ns)
    return types.SimpleNamespace(resolve=ns["resolve_relative_bt_xml"])


def _data(**bt):
    return {"bt_navigator": {"ros__parameters": dict(bt)}}


def test_relative_tree_beside_the_config_is_resolved(tmp_path):
    cfg = tmp_path / "robot_config.yaml"
    cfg.write_text("nav2: {}\n")
    (tmp_path / "trees").mkdir()
    (tmp_path / "trees" / "nav.xml").write_text("<root/>")
    d = _data(default_nav_to_pose_bt_xml="trees/nav.xml")
    assert _fn().resolve(d, str(cfg)) == ["default_nav_to_pose_bt_xml"]
    assert d["bt_navigator"]["ros__parameters"]["default_nav_to_pose_bt_xml"] == str(tmp_path / "trees" / "nav.xml")


def test_absolute_missing_and_absent_are_left_alone(tmp_path, capsys):
    cfg = tmp_path / "robot_config.yaml"
    cfg.write_text("nav2: {}\n")
    d = _data(default_nav_to_pose_bt_xml="/opt/x.xml", default_nav_through_poses_bt_xml="missing.xml")
    assert _fn().resolve(d, str(cfg)) == []
    p = d["bt_navigator"]["ros__parameters"]
    assert p["default_nav_to_pose_bt_xml"] == "/opt/x.xml" and p["default_nav_through_poses_bt_xml"] == "missing.xml"
    assert "missing.xml is not beside" in capsys.readouterr().out
    assert _fn().resolve({}, str(cfg)) == []
