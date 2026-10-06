"""/api/status lists the robots on every poll; it must not re-parse the reference designs.

robot_kind() asked reference_design_names() twice per robot, and that parsed every
config/reference/*_config.yaml each time: 91 YAML parses and 2.7 s per poll on a
4-core cell (2026-10-07), with the page's overlapping polls holding a core and a
walkthrough step that took 10 s taking 10 minutes.
"""
import os
import sys

import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "scripts"))
import one_click_pipeline as ocp  # noqa: E402


def _designs(tmp_path, **names):
    d = tmp_path / "config" / "reference"
    d.mkdir(parents=True, exist_ok=True)
    for stem, name in names.items():
        (d / f"{stem}_config.yaml").write_text(yaml.safe_dump({"robot": {"name": name}}))
    return d


def test_an_unchanged_design_is_parsed_once(tmp_path, monkeypatch):
    _designs(tmp_path, a="alpha", b="beta")
    calls = []
    real = yaml.safe_load
    monkeypatch.setattr(ocp.yaml, "safe_load", lambda fh: calls.append(1) or real(fh))
    for _ in range(5):
        assert ocp.reference_design_names(str(tmp_path)) == {"alpha", "beta"}
    assert len(calls) == 2


def test_an_added_or_edited_design_is_seen_at_once(tmp_path):
    d = _designs(tmp_path, a="alpha")
    assert ocp.reference_design_names(str(tmp_path)) == {"alpha"}
    _designs(tmp_path, b="beta")
    assert ocp.reference_design_names(str(tmp_path)) == {"alpha", "beta"}
    (d / "a_config.yaml").write_text(yaml.safe_dump({"robot": {"name": "alpha_two"}}))
    os.utime(d / "a_config.yaml", ns=(1, 1))
    assert ocp.reference_design_names(str(tmp_path)) == {"alpha_two", "beta"}
    (d / "b_config.yaml").unlink()
    assert ocp.reference_design_names(str(tmp_path)) == {"alpha_two"}


def test_the_robots_list_asks_the_kind_once_per_robot():
    core = open(os.path.join(ROOT, "web", "backend", "core.py")).read()
    assert core.count("one_click_pipeline.robot_kind(") == 1
