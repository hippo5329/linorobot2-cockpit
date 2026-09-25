"""The Sim MCU offers no action that needs a board, and the backend refuses them all.

User, 2026-09-25: the Sim MCU is there so a "user can run without board, no
flash error". A browser walk of every control found Auto-Detect Sensors and
Build still enabled on it, each ending in a red ❌: the probe asked for a flash
the backend refused, and Build fell past the Sim MCU check (which covered
upload only) into "PlatformIO is not installed ... pio run -e pico2".
"""
import os
import re

import pytest

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FRONTEND = os.path.join(REPO_ROOT, "web", "frontend")


def read(*p):
    return open(os.path.join(REPO_ROOT, *p)).read()


def test_every_hardware_action_is_refused_on_the_sim_mcu():
    src = read("web", "backend", "routes_hardware.py")
    body = src[src.index("async def api_hardware_test"):]
    nxt = body.find("\nasync def ", 1)
    body = body[:nxt] if nxt > 0 else body
    call = body.index("refuse_sim_flash(")
    # Not under `if action == ...`: the line before the call is not a condition.
    before = body[:call].rstrip().splitlines()[-1]
    assert not before.strip().startswith("if "), "the Sim MCU refusal must cover every action"
    assert call < body.index("pio_present()"), "the refusal must come before the build's pio check"


def test_the_refusal_names_the_sim_mcu():
    fastapi = pytest.importorskip("fastapi")
    import core
    with pytest.raises(fastapi.HTTPException) as e:
        core.refuse_sim_flash("sim")
    assert "simulated MCU" in e.value.detail
    core.refuse_sim_flash("pico2")


def test_the_board_only_buttons_are_disabled_on_the_sim_mcu():
    adc = open(os.path.join(FRONTEND, "app-adc.js")).read()
    html = open(os.path.join(FRONTEND, "index.html")).read()
    listed = re.search(r"const BOARD_ONLY_BUTTONS = \[(.*?)\];", adc, re.S).group(1)
    ids = set(re.findall(r'"([\w-]+)"', listed))
    # Every button that sends /api/hardware/test is bound in app-adc.js to
    # executeHardwareAction; each one must be in the list, and exist.
    bound = set(re.findall(r'getElementById\("([\w-]+)"\);\s*\n\s*if \(\w+\) \w+\.addEventListener\("click", \(\) => executeHardwareAction', adc))
    bound |= set(re.findall(r'\{ id: "([\w-]+)", firmware:', adc))
    bound |= {"btn-auto-detect-i2c", "btn-adc-run-cal"}
    assert bound <= ids, f"board actions not disabled on the Sim MCU: {sorted(bound - ids)}"
    for i in ids:
        assert f'id="{i}"' in html, f"{i} is listed but not in the page"
    assert html.count('class="hint sim-mcu-note"') >= 2
    # The guard runs whenever the robot's controller is (re)loaded.
    assert "updateBoardOnlyButtons()" in open(os.path.join(FRONTEND, "app-hardware.js")).read()


def test_sim_mode_cannot_be_switched_off_on_the_sim_mcu():
    """The walk pressed "Switch Sim Mode OFF" on bare_sim and left it with every
    simulated device off: no IMU, wheels, LiDAR, sonar or battery at all."""
    src = read("web", "backend", "routes_config.py")
    body = src[src.index("async def api_toggle_sim_mode"):]
    guard = body.index("if not enabled and")
    assert "SIM_MCU" in body[guard:guard + 120]
    assert guard < body.index('sensors["use_sim_wheel"] = enabled'), "refuse before writing anything"
    assert "SIM_MCU" in read("web", "backend", "routes_status.py").split("sim_mode_active = bool(")[1][:200], \
        "the Sim MCU must always report Sim Mode"
    ui = open(os.path.join(FRONTEND, "app-presets.js")).read()
    assert 'if (!enabled && state.robot_name === "bare_sim") return;' in ui
    assert "b.disabled = lock;" in ui
    assert "USE_SIM_" not in ui, "the sim switches are env keys, not build macros"


def test_loading_a_bare_robot_does_not_throw():
    """loadHardwareConfig read `targetMcu`, which was never defined, whenever
    the config had no port -- every bare robot -- so it threw and skipped its
    last third (pin safety, kinematics HUD, DAC availability, port refresh)."""
    src = open(os.path.join(FRONTEND, "app-hardware.js")).read()
    assert "targetMcu." not in src


def test_a_status_poll_from_before_a_robot_switch_is_dropped():
    """A poll answered with the old robot after selectRobot() had switched,
    putting state.robot_name back while Tabs 1-5 were filled for the new one."""
    src = open(os.path.join(FRONTEND, "app-core.js")).read()
    poll = src[src.index("async function refreshStatus"):]
    assert poll.index("if (epoch !== robotEpoch) return;") < poll.index("state.status = s;")
    sel = src[src.index("async function selectRobot"):]
    assert sel.index("robotEpoch++") < sel.index('fetch("/api/robot/select"') < sel.rindex("robotEpoch++", 0, sel.index("state.robot_name = res.active"))
