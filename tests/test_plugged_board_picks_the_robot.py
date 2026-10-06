"""The no-board switch: only a GENERATED robot is swapped for the Sim MCU.

Reference designs are real robots (user, 2026-10-06, restating an older rule): the user
selects any of them whatever MCU is detected, or with none attached, and the UI never
swaps one for the Sim MCU -- a 1-Click pressed on a design ran `bare_sim` instead. Only
a bare module (`bare_<silicon>`) falls back to the Sim MCU when nothing is on the bus.

For that bare robot, a board plugged in afterwards picks the robot for THAT board: the
robot we left if it is that silicon, else that silicon's bare robot (plugging an ESP32
into a Pico 2 setup once came back to a Pico 2 robot).

The test runs app-core.js's own switch code in node with the DOM stubbed, so
it exercises what the browser runs rather than a transcription of it.
"""
import json
import os
import shutil
import subprocess

import pytest

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
NODE = shutil.which("node") or shutil.which("nodejs")

HARNESS = r"""
const src = require("fs").readFileSync(process.argv[2], "utf8");
const pick = (start) => { const i = src.indexOf(start); return src.slice(i, src.indexOf("\n}\n", i) + 3); };
const g = pick("function isGeneratedRobot(") + pick("function isReferenceDesign(") +
          pick("function activeRobot(") + pick("function activeRobotIsReal(");
const a = src.indexOf("let noBoardSwitchedFrom = null;");
const b0 = src.indexOf("async function noBoardSwitch(s)");
const b = src.indexOf("\n}\n", b0) + 3;
if (a < 0 || b0 < 0 || b < 3) { console.log(JSON.stringify({error: "switch code not found"})); process.exit(0); }
const calls = [];
const robot = process.argv[3];
const kind = robot.startsWith("bare_") ? "generated" : (process.argv[6] || "design");
const state = { robot_name: robot, robots: [{ name: robot, kind, real: kind === "design" || kind === "real" }] };
let controllerPickedByHand = false;
const BARE_CHOICE = "__bare__";
async function applyReferenceDesign(v) { calls.push("apply:" + v); }
let loadedControllerName = process.argv[4];
const BOARD_SILICON = { gendrv: "esp32", yb_eet01: "esp32s3" };
function siliconOf(n) { n = String(n || "").toLowerCase(); return BOARD_SILICON[n] || n; }
async function selectRobot(name) { calls.push(name); state.robot_name = name; }
function escapeHtml(x) { return String(x); }
const document = { addEventListener() {}, getElementById() { return null; } };
const window = {};
eval(g + src.slice(a, b) + `
(async () => {
  for (const poll of JSON.parse(process.argv[5])) await noBoardSwitch(poll);
  console.log(JSON.stringify({ calls, robot: state.robot_name }));
})();`);
"""

NONE = {"board_on_bus": False, "mcu_detected": False}


def board(mcu):
    return {"board_on_bus": True, "mcu_detected": True, "detected_mcu": mcu}


def run(robot, controller, polls, tmp_path, kind=""):
    h = tmp_path / "harness.js"
    h.write_text(HARNESS)
    out = subprocess.run([NODE, str(h), os.path.join(REPO_ROOT, "web", "frontend", "app-core.js"),
                          robot, controller, json.dumps(polls), kind],
                         capture_output=True, text=True, timeout=30)
    assert out.returncode == 0, out.stderr
    res = json.loads(out.stdout.strip().splitlines()[-1])
    assert "error" not in res, res
    return res


pytestmark = pytest.mark.skipif(NODE is None, reason="node is not installed")


def _designs():
    """Every shipped reference design, discovered -- more will come, and a list here would
    silently skip them -- plus a robot a user saved under their own name."""
    import glob
    import yaml
    out = []
    for path in sorted(glob.glob(os.path.join(REPO_ROOT, "config", "reference", "*_config.yaml"))):
        c = yaml.safe_load(open(path))
        out.append((c["robot"]["name"], c["base_controller"]["name"]))
    return out


@pytest.mark.parametrize("design, controller", _designs())
def test_a_real_robot_is_never_swapped_for_the_sim_mcu(design, controller, tmp_path):
    """A reference design (or a robot the user saved) stays selected with no board at all."""
    res = run(design, controller, [NONE, NONE], tmp_path)
    assert res["calls"] == [] and res["robot"] == design, res


def test_a_bare_robot_falls_back_and_the_plugged_board_gets_its_default_robot(tmp_path):
    """User, 2026-10-06: with a board detected, the default robot is named for it (lino_esp32)."""
    res = run("bare_pico2", "pico2", [NONE, board("esp32")], tmp_path)
    assert res["calls"] == ["bare_sim", "lino_esp32", "apply:__bare__"], res
    assert res["robot"] == "lino_esp32"


def test_the_same_silicon_gets_its_default_robot_not_the_generated_one(tmp_path):
    res = run("bare_pico2", "pico2", [NONE, board("pico2")], tmp_path)
    assert res["robot"] == "lino_pico2", res


def test_a_board_that_does_not_name_its_silicon_yet_is_waited_for(tmp_path):
    """A board in BOOTSEL is on the bus with no tty; the next poll names it."""
    bootsel = {"board_on_bus": True, "mcu_detected": False, "detected_mcu": "pico2"}
    res = run("bare_pico2", "pico2", [NONE, bootsel], tmp_path)
    assert res["robot"] == "bare_sim", res
    res = run("bare_pico2", "pico2", [NONE, bootsel, board("pico")], tmp_path)
    assert res["robot"] == "lino_pico", res


def test_with_no_design_chosen_the_detected_board_decides(tmp_path):
    """User, 2026-10-06: no reference design selected -> a detected MCU defaults to its bare
    module, every device simulated; a detected UNO Q to its STM32 (bare_unoq)."""
    # ...as a robot NAMED for the board, created as its bare module: lino_<mcu>
    res = run("bare_sim", "sim", [board("esp32")], tmp_path)
    assert res["calls"] == ["lino_esp32", "apply:__bare__"] and res["robot"] == "lino_esp32", res
    assert run("bare_sim", "sim", [board("unoq")], tmp_path)["robot"] == "lino_unoq"
    assert run("bare_pico2", "pico2", [board("esp32s3")], tmp_path)["robot"] == "lino_esp32s3"


def test_a_default_robot_gives_way_to_the_next_boards_default_robot(tmp_path):
    """lino_esp32 does not become a Pico robot under its ESP32 name: lino_pico2 is selected."""
    res = run("lino_esp32", "esp32", [board("pico2")], tmp_path, "sim")
    assert res["calls"] == ["lino_pico2", "apply:__bare__"], res


@pytest.mark.parametrize("design, controller", _designs())
def test_a_design_is_never_replaced_by_the_detected_board(design, controller, tmp_path):
    res = run(design, controller, [board("esp32s3")], tmp_path)
    assert res["calls"] == [] and res["robot"] == design, res


def test_the_default_robot_is_the_sim_mcu_not_a_design():
    import sys
    sys.path.insert(0, os.path.join(REPO_ROOT, "scripts"))
    import cockpit_paths
    assert cockpit_paths.DEFAULT_ROBOT == "bare_sim"
    core = open(os.path.join(REPO_ROOT, "web", "backend", "core.py")).read()
    assert 'DEFAULT_ROBOT_NAME = "bare_sim"' in core


@pytest.mark.parametrize("kind", ["real", "design"])
def test_a_robot_built_from_a_design_is_never_moved_by_the_board(kind, tmp_path):
    res = run("my_rover", "pico2", [NONE, board("esp32s3")], tmp_path, kind)
    assert res["calls"] == [] and res["robot"] == "my_rover", res


def test_the_users_robot_with_no_design_follows_the_board(tmp_path):
    """A named robot with no design IS a bare module of its board (user, 2026-10-06)."""
    res = run("my_rover", "pico2", [board("esp32")], tmp_path, "sim")
    assert res["calls"] == ["apply:__bare__"], res
    res = run("my_rover", "unoq", [board("unoq")], tmp_path, "sim")
    assert res["calls"] == [], res          # already that board
