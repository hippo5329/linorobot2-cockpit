"""A design or bare module is applied to the robot it was meant for, or to none.

The backend rebuilds WHICHEVER robot is active. The page's two robot switches raced
(2026-10-19): on load, with the Sim MCU robot active and an ESP32 plugged, the page moved to the
board's default robot (lino_esp32) and began making it a bare module; the user picked a Wi-Fi
robot meanwhile, and the bare module landed on the Wi-Fi robot -- serial, domain 0. The page
now names the robot in the request (the server refuses another, 409) and drops the apply when
the robot changed while it waited.

Runs app-presets.js's own applyReferenceDesign in node with the DOM stubbed.
"""
import json
import os
import shutil
import subprocess

import pytest

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
NODE = shutil.which("node") or shutil.which("nodejs")
pytestmark = pytest.mark.skipif(NODE is None, reason="node is not installed")

HARNESS = r"""
const src = require("fs").readFileSync(process.argv[2], "utf8");
const i = src.indexOf("async function applyReferenceDesign(");
const fn = src.slice(i, src.indexOf("\n}\n", i) + 3);
const [forRobot, switchTo] = JSON.parse(process.argv[3]);
const state = { robot_name: "lino_esp32", status: { mcu_detected: true, detected_mcu: "esp32" } };
const BARE_CHOICE = "__bare__";
let robotEpoch = 0, loadedControllerName = "esp32";
const posts = [];
function robotIsNamed() { return true; }
function askForRobotName() {}
function updateReferenceDesigns() {}
function siliconOf(n) { return String(n || ""); }
function logLine() {}
function showToast() {}
async function adoptRobotReply() {}
async function flushAutosave() { if (switchTo) state.robot_name = switchTo; }   // a switch lands here
async function fetch(url, opts) {
  posts.push({ url, body: JSON.parse(opts.body) });
  return { json: async () => ({ status: "ok", message: "" }) };
}
const document = { getElementById() { return null; } };
eval(fn + `
(async () => {
  await applyReferenceDesign(BARE_CHOICE, forRobot || undefined);
  console.log(JSON.stringify(posts));
})();`);
"""


def run(tmp_path, for_robot, switch_to):
    h = tmp_path / "h.js"
    h.write_text(HARNESS)
    out = subprocess.run([NODE, str(h), os.path.join(REPO_ROOT, "web", "frontend", "app-presets.js"),
                          json.dumps([for_robot, switch_to])], capture_output=True, text=True, timeout=30)
    assert out.returncode == 0, out.stderr
    return json.loads(out.stdout.strip().splitlines()[-1])


def test_the_request_names_the_robot(tmp_path):
    posts = run(tmp_path, "lino_esp32", None)
    assert [p["url"] for p in posts] == ["/api/robot/apply_bare"]
    assert posts[0]["body"] == {"mcu": "esp32", "robot": "lino_esp32"}
    # with no robot given, the active one at the call
    assert run(tmp_path, None, None)[0]["body"]["robot"] == "lino_esp32"


def test_a_robot_picked_while_it_waited_is_not_rebuilt(tmp_path):
    assert run(tmp_path, "lino_esp32", "uileg_udp4") == []
    assert run(tmp_path, None, "uileg_udp4") == []


def test_every_automatic_bare_apply_names_the_robot_it_meant():
    src = open(os.path.join(REPO_ROOT, "web", "frontend", "app-core.js")).read()
    calls = [l.strip() for l in src.splitlines() if "await applyReferenceDesign(BARE_CHOICE" in l]
    assert calls and all("BARE_CHOICE," in c for c in calls), calls
