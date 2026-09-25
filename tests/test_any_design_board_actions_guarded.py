"""Any reference design can be chosen; only actions that write a board need the board to match.

User, 2026-09-26: someone starting a new design picks it before the board is on
the desk, and the MCU follows the design. What needs the matching board -- a
flash, 1-Click, an app switch -- asks the bus at the moment of the action and
is refused with a warning, before any request is sent. This replaced
2026-09-25's filter, which offered only the detected silicon's designs.
"""
import json
import os
import shutil
import subprocess

import pytest

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FRONT = os.path.join(REPO_ROOT, "web", "frontend")
NODE = shutil.which("node") or shutil.which("nodejs")


def src(name):
    return open(os.path.join(FRONT, name)).read()


GUARD_HARNESS = r"""
const src = require("fs").readFileSync(process.argv[2], "utf8");
const a = src.indexOf("async function boardMatchesOrWarn(");
const b = src.indexOf("\n}\n", a) + 3;
const answer = JSON.parse(process.argv[3]);
const fetched = [], logged = [], banners = [];
async function fetch(url) {
  fetched.push(url);
  if (answer === "throw") throw new Error("offline");
  return { json: async () => answer };
}
function logLine(x) { logged.push(x); }
function showActionBanner(t, d) { banners.push([t, d]); }
eval(src.slice(a, b) + `
boardMatchesOrWarn(process.argv[4], "Flash").then((ok) =>
  console.log(JSON.stringify({ ok, fetched, banners, logged })));`);
"""

MISMATCH = {"mcu_mismatch": {"controller": "pico2", "expected": "RP2350", "detected": "RP2040",
                             "chip": "Raspberry Pi Pico (RP2040)"}}


def guard(tmp_path, answer, controller="pico2"):
    h = tmp_path / "guard.js"
    h.write_text(GUARD_HARNESS)
    out = subprocess.run([NODE, str(h), os.path.join(FRONT, "app-core.js"), json.dumps(answer), controller],
                         capture_output=True, text=True, timeout=30)
    assert out.returncode == 0, out.stderr
    return json.loads(out.stdout.strip().splitlines()[-1])


needs_node = pytest.mark.skipif(NODE is None, reason="node is not installed")


@needs_node
def test_a_mismatched_board_blocks_with_a_warning(tmp_path):
    r = guard(tmp_path, MISMATCH)
    assert r["ok"] is False
    assert r["fetched"] == ["/api/status?controller=pico2"], "ask about THIS action's controller"
    assert r["banners"] and "RP2040" in r["banners"][0][1] and "RP2350" in r["banners"][0][1]


@needs_node
def test_a_matching_or_absent_board_lets_the_action_through(tmp_path):
    assert guard(tmp_path, {"mcu_mismatch": None})["ok"] is True


@needs_node
def test_no_answer_is_no_evidence(tmp_path):
    """The flasher refuses on its own; a lost status call must not block a good board."""
    assert guard(tmp_path, "throw")["ok"] is True


@needs_node
def test_the_sim_mcu_needs_no_board(tmp_path):
    r = guard(tmp_path, MISMATCH, controller="sim")
    assert r["ok"] is True and r["fetched"] == []


def _body(text, head):
    a = text.index(head)
    return text[a:text.index("\n}\n", a)] if text[a:].startswith("async function") else text[a:a + 6000]


def test_every_board_writing_action_asks_before_its_first_request():
    hw = _body(src("app-hardware.js"), "async function executeHardwareAction(")
    g = hw.index('action === "upload" && !(await boardMatchesOrWarn(')
    assert g < hw.index("/api/agent/port_release") and g < hw.index("/api/hardware/test")
    oc = _body(src("app-workflow.js"), "async function runOneClick(")
    g = oc.index("await boardMatchesOrWarn(controller, \"1-Click\")")
    assert g < oc.index("/api/workflow/one-click/stream")


def test_every_design_is_offered_whatever_the_mcu():
    p = src("app-presets.js")
    body = p[p.index("function updateReferenceDesigns("):p.index("async function applyReferenceDesign(")]
    assert "...Object.keys(REFERENCE_DESIGNS).filter((k) => k !== family)" in body
