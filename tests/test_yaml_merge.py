"""yaml_merge keeps the target's layout and only overlays scalars."""
import yaml_merge


TARGET = """# robot
kinematics:
  base_type: 2wd   # keep this comment
  max_rpm: 140
  pid:
    kp: 0.6
"""


def test_scalar_overlay_keeps_comments():
    merged, report = yaml_merge.merge_yaml(TARGET, "kinematics:\n  max_rpm: 200\n  pid:\n    kp: 1.0\n")
    assert "max_rpm: 200" in merged and "kp: 1.0" in merged
    assert "# keep this comment" in merged and merged.startswith("# robot")
    assert set(report["changed"]) == {"kinematics/max_rpm", "kinematics/pid/kp"}


def test_source_only_paths_are_reported_not_written():
    merged, report = yaml_merge.merge_yaml(TARGET, "kinematics:\n  wheel_diameter: 0.1\n")
    assert "wheel_diameter" not in merged
    assert report["source_only"] == ["kinematics/wheel_diameter"]
