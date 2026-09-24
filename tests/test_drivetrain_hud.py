"""The performance HUD computes, it does not re-derive.

The Config Studio's Kinematics HUD showed circumference, ticks/m and
`pi*d*rpm/60`. None of that answers the question a person designing a robot
has -- how hard does it accelerate, how long to reach speed, and is the Nav2
tuning in this same config asking for more than the motors can give -- so the
answer used to require flashing test_acc and driving the board. It does not any
more: /api/drivetrain/performance runs the same model.

Which creates exactly one new way to be wrong, and it is the one this file
guards: a JavaScript reimplementation of the motor model. The browser would then
hold a second opinion about the robot, drifting from sim_wheel.h silently, and
a HUD that is confidently wrong is worse than the arithmetic it replaced. So the
numbers are computed server-side by scripts/drivetrain_report.py, which parses
its constants out of the firmware.

The other half is duller and bit us on the geometry form: a HUD wired to only
some of the fields that change its answer describes a robot the form does not.
"""
import os
import re

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ROUTES = os.path.join(REPO_ROOT, "web", "backend", "routes_config.py")
HW_JS = os.path.join(REPO_ROOT, "web", "frontend", "app-hardware.js")
ADC_JS = os.path.join(REPO_ROOT, "web", "frontend", "app-adc.js")
INDEX = os.path.join(REPO_ROOT, "web", "frontend", "index.html")


def _read(path):
    with open(path, encoding="utf-8") as fh:
        return fh.read()


def _hud_block():
    """Just the HUD's own functions.

    Slicing to end-of-file swept in updateAdcCalculations and the battery
    divider's constants, so the "no unexplained numbers" check below was really
    checking the whole file and failed on 12.6 V. A check that names too much is
    the same rot as one that names too little.
    """
    js = _read(HW_JS)
    start = js.index("function updateDrivetrainHUD")
    end = js.index("function updateAdcCalculations")
    assert start < end, "the HUD functions moved; re-anchor this slice"
    return js[start:end]


def test_the_endpoint_exists_and_delegates_to_the_report():
    src = _read(ROUTES)
    assert '"/api/drivetrain/performance"' in src
    assert "import drivetrain_report as dr" in src
    # The three calls that ARE the model. Reaching into the module's internals
    # rather than copying its arithmetic is the whole point.
    for call in ("dr.drivetrain(", "dr.performance(", "dr.demand_rpm("):
        assert call in src, call


def test_the_browser_does_not_reimplement_the_motor_model():
    """No torque-speed arithmetic and no model constants in the frontend."""
    # The performance half only; everything above it is the old arithmetic
    # HUD, which is allowed to do arithmetic.
    perf = _hud_block()
    assert "SIM_" not in perf, "a firmware model constant reached the browser"

    # No maths beyond formatting. exp/pow/sqrt would be the torque-speed curve
    # or the sag lag; round and toFixed are how a number becomes a label.
    maths = set(re.findall(r"Math\.(\w+)", perf))
    assert maths <= {"round"}, f"the HUD appears to compute, not display: {sorted(maths)}"

    # Every float literal in the block. A model constant (0.75 efficiency, 0.25
    # sag, 150 ms tau) would show up here; the only one allowed is the
    # near-1 threshold that decides whether to mention the voltage derate, and
    # the defaults for form fields that may be empty.
    floats = set(re.findall(r"(?<![\w.])\d+\.\d+", perf))
    allowed = {"0.999",                                      # "is the pack derating it?"
               "0.152", "0.271", "0.85", "0.0", "24.0", "12.0"}  # form-field fallbacks
    assert floats <= allowed, f"unexplained constants in the HUD: {sorted(floats - allowed)}"

    # It renders what the server sent, so every number it shows is a field of
    # the response.
    assert "d.accel_held" in perf and "d.max_linear" in perf


def test_the_hud_posts_every_kinematics_key_the_model_reads():
    """drivetrain() reads these; a key the HUD omits silently takes the SAVED
    config's value, so the HUD answers for a robot the form is not describing."""
    report = _read(os.path.join(REPO_ROOT, "scripts", "drivetrain_report.py"))
    body = report[report.index("def drivetrain("):report.index("def performance(")]
    read = set(re.findall(r'kine\.get\("(\w+)"', body))
    # angular_scale has no control in the UI (nobody measures their own scrub
    # from a web form), so it comes from the saved config and is not posted.
    read -= {"angular_scale"}
    posted = _hud_block()
    posted = posted[posted.index("async function fetchDrivetrainHUD"):]
    missing = sorted(k for k in read if f"{k}:" not in posted)
    assert not missing, f"the HUD never sends {missing}"


def test_every_field_that_changes_the_answer_retriggers_the_hud():
    listeners = _read(ADC_JS)
    block = listeners[listeners.index('["cfg-wheel-diameter"'):]
    block = block[:block.index("});")]
    for field in ("cfg-wheel-diameter", "cfg-track-width", "cfg-wheelbase", "cfg-max-rpm",
                  "cfg-headroom", "cfg-motor-voltage", "cfg-motor-max-voltage"):
        assert field in block, f"{field} does not refresh the HUD"
    # base_type is a <select>, so it is a change listener, not an input one.
    kine = listeners[listeners.index('getElementById("cfg-kinematics")'):]
    kine = kine[:kine.index("const elMcu")]
    assert "updateKinematicsHUD()" in kine, \
        "changing the drivetrain changes the rotation radius and must refresh the HUD"


def test_every_element_the_renderer_writes_exists_in_the_page():
    perf = _hud_block()
    perf = perf[perf.index("function renderDrivetrainHUD"):]
    ids = set(re.findall(r'set\("(hud-perf-[\w-]+)"', perf))
    ids |= set(re.findall(r'getElementById\("(hud-perf-[\w-]+)"\)', perf))
    html = _read(INDEX)
    missing = sorted(i for i in ids if f'id="{i}"' not in html)
    assert not missing, f"the renderer writes to elements that do not exist: {missing}"
    assert ids, "the renderer wrote nothing"


def test_a_stale_reply_cannot_overwrite_a_newer_one():
    """The HUD fires per keystroke, so replies race. Showing the answer for a
    half-typed wheel diameter after the finished one is a wrong HUD that looks
    settled."""
    perf = _hud_block()
    perf = perf[perf.index("async function fetchDrivetrainHUD"):]
    assert "++drivetrainHudSeq" in perf
    assert "if (seq !== drivetrainHudSeq) return;" in perf


def test_a_half_typed_config_is_not_an_error():
    """ready:false, not a 500. The HUD asks while the user is still typing."""
    src = _read(ROUTES)
    block = src[src.index('"/api/drivetrain/performance"'):]
    assert '"ready": False' in block
    assert block.index('"ready": False') < block.index("dr.performance(")


def test_the_badge_reports_a_verdict_rather_than_a_reassurance():
    """It read "85% Headroom Safe" whatever the numbers were."""
    html = _read(INDEX)
    assert 'id="hud-kinematics-badge"' in html
    perf = _hud_block()
    perf = perf[perf.index("function renderDrivetrainHUD"):]
    assert "d.over_budget" in perf, "the badge must follow the budget check"
