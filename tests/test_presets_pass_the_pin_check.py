"""Every reference design passes the same pin check the server applies on save.

The UI's Reference Designs picker applies a design FILE (config/reference/) to the user's
robot (/api/robot/apply_reference). Until 2026-10-06 it applied a JavaScript copy of the
designs kept in app-presets.js, which drifted from the files: one preset put the battery on
GPIO 36, which its own encoder2 already used, and applying it saved a config the catalogue
refuses. The copy is gone; the files are checked here exactly as a saved config is.
"""
import glob
import os
import sys

import yaml

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO, "scripts"))
import pin_catalog  # noqa: E402

DESIGNS = sorted(glob.glob(os.path.join(REPO, "config", "reference", "*_config.yaml")))


def test_every_design_passes_the_pin_check_it_will_be_saved_through():
    assert DESIGNS
    bad = {}
    for path in DESIGNS:
        cfg = yaml.safe_load(open(path))
        errors = [msg for level, msg in pin_catalog.check_config(cfg) if level == "error"]
        if errors:
            bad[os.path.basename(path)] = errors
    assert not bad, f"designs the server would refuse on save: {bad}"


def test_the_ui_keeps_no_copy_of_the_designs():
    js = open(os.path.join(REPO, "web", "frontend", "app-presets.js")).read()
    assert "REFERENCE_DESIGNS" not in js
    assert "/api/robot/apply_reference" in js
