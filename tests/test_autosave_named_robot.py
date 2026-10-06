"""Autosave: the named robot's config follows the screen (user, 2026-10-06).

Name the robot from the beginning; every selection and every action creates and updates its
configuration. Edits in the configuration tabs are saved to the file at once, and Flash,
1-Click and Bringup save first -- they used to run the last SAVED file, ignoring the screen.
Only a NAMED robot is written: a generated robot is rewritten on every run, and a reference
design is the shipped design, so editing either asks for a name.
"""
import os

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FRONT = os.path.join(ROOT, "web", "frontend")


def src(name):
    return open(os.path.join(FRONT, name)).read()


def test_every_action_saves_the_screen_first():
    assert 'configReadyOrWarn("Flash")' in src("app-hardware.js")
    assert 'configReadyOrWarn("1-Click")' in src("app-workflow.js")
    assert 'configReadyOrWarn("Bringup")' in src("app-agent-bringup.js")


def test_no_save_bypasses_the_naming_rule():
    for f in ("app-adc.js", "app-presets.js", "index.html"):
        assert "saveCurrentHardwareConfig()" not in src(f).replace("saveCurrentHardwareConfig({", ""), f
    hw = src("app-hardware.js")
    a = hw.index("function robotIsNamed(")
    body = hw[a:hw.index("\n}\n", a)]
    assert "isGeneratedRobot" in body and "isReferenceDesign" in body


def test_a_new_name_takes_the_edits_on_screen():
    core = src("app-core.js")
    a = core.index("async function selectRobot(")
    body = core[a:core.index("loadHardwareConfig", a)]
    assert "autosaveAfterNaming(wasNew)" in body
    assert body.index("autosaveAfterNaming") < len(body)   # before the form reloads from the file


def test_one_save_state_in_the_header_and_no_save_buttons():
    """User, 2026-10-06: with autosave, one save indicator in the header instead of Save buttons."""
    html = src("index.html")
    assert "Save robot settings" not in html
    assert 'id="hdr-save-state"' in html
    hw = src("app-hardware.js")
    a = hw.index("function setSavedIndicator(")
    assert 'getElementById("hdr-save-state")' in hw[a:a + 300]
