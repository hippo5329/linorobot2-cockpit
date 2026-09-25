"""A board plugged in after the no-board switch picks the robot for THAT board.

The header switches to the Sim MCU robot (bare_sim) when nothing is on the bus,
and used to switch straight back to the robot it left when anything appeared.
On a fresh install that robot is the default, pico2_mecanum -- so plugging an
ESP32 in selected a Pico 2 mecanum design. The robot must follow the silicon
the bus names: the robot we left if it is that silicon, else that silicon's
bare robot, the same rule a user's own pick of a controller follows.

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
const a = src.indexOf("let noBoardSwitchedFrom = null;");
const b0 = src.indexOf("async function noBoardSwitch(s)");
const b = src.indexOf("\n}\n", b0) + 3;
if (a < 0 || b0 < 0 || b < 3) { console.log(JSON.stringify({error: "switch code not found"})); process.exit(0); }
const calls = [];
const state = { robot_name: process.argv[3] };
let loadedControllerName = process.argv[4];
const BOARD_SILICON = { gendrv: "esp32", yb_eet01: "esp32s3" };
function siliconOf(n) { n = String(n || "").toLowerCase(); return BOARD_SILICON[n] || n; }
async function selectRobot(name) { calls.push(name); state.robot_name = name; }
function escapeHtml(x) { return String(x); }
const document = { addEventListener() {}, getElementById() { return null; } };
const window = {};
eval(src.slice(a, b) + `
(async () => {
  for (const poll of JSON.parse(process.argv[5])) await noBoardSwitch(poll);
  console.log(JSON.stringify({ calls, robot: state.robot_name }));
})();`);
"""

NONE = {"board_on_bus": False, "mcu_detected": False}


def board(mcu):
    return {"board_on_bus": True, "mcu_detected": True, "detected_mcu": mcu}


def run(robot, controller, polls, tmp_path):
    h = tmp_path / "harness.js"
    h.write_text(HARNESS)
    out = subprocess.run([NODE, str(h), os.path.join(REPO_ROOT, "web", "frontend", "app-core.js"),
                          robot, controller, json.dumps(polls)],
                         capture_output=True, text=True, timeout=30)
    assert out.returncode == 0, out.stderr
    res = json.loads(out.stdout.strip().splitlines()[-1])
    assert "error" not in res, res
    return res


pytestmark = pytest.mark.skipif(NODE is None, reason="node is not installed")


def test_an_esp32_plugged_into_a_fresh_install_gets_the_esp32_robot(tmp_path):
    res = run("pico2_mecanum", "pico2", [NONE, board("esp32")], tmp_path)
    assert res["calls"] == ["bare_sim", "bare_esp32"], res
    assert res["robot"] == "bare_esp32"


def test_the_same_silicon_goes_back_to_the_robot_it_left(tmp_path):
    res = run("pico2_mecanum", "pico2", [NONE, board("pico2")], tmp_path)
    assert res["robot"] == "pico2_mecanum", res


def test_a_reference_design_follows_its_silicon(tmp_path):
    """gendrv is an ESP32 board, so an ESP32 brings it back."""
    res = run("gendrv", "gendrv", [NONE, board("esp32")], tmp_path)
    assert res["robot"] == "gendrv", res
    res = run("gendrv", "gendrv", [NONE, board("pico")], tmp_path)
    assert res["robot"] == "bare_pico", res


def test_a_board_that_does_not_name_its_silicon_yet_is_waited_for(tmp_path):
    """A board in BOOTSEL is on the bus with no tty; the next poll names it."""
    bootsel = {"board_on_bus": True, "mcu_detected": False, "detected_mcu": "pico2"}
    res = run("pico2_mecanum", "pico2", [NONE, bootsel], tmp_path)
    assert res["robot"] == "bare_sim", res
    res = run("pico2_mecanum", "pico2", [NONE, bootsel, board("pico")], tmp_path)
    assert res["robot"] == "bare_pico", res
