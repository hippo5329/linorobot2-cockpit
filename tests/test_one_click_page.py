"""The Operations tab's 1-Click (web/frontend/app-workflow.js), run under node with a stub DOM.

The review found: a double click started two runs, and the refused second click's cleanup
re-enabled Start and disabled Abort while the first drove; a refused start left the badge
on "Running..."; every success read "Map saved and Nav2 verified" and every failure
"Flashing or verification failed"; the Run options had no page at all. These tests click the
page's own handlers and read what it shows and sends.
"""
import json
import os
import shutil
import subprocess

import pytest

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FRONT = os.path.join(REPO_ROOT, "web", "frontend")
NODE = shutil.which("node") or shutil.which("nodejs")
needs_node = pytest.mark.skipif(NODE is None, reason="node is not installed")

# A DOM just big enough for initCockpitDashboard: every element exists (a stub), clicks are
# the listeners the page added, and fetch answers by URL from the scenario.
HARNESS = r"""
const src = require("fs").readFileSync(process.argv[2], "utf8");
const scenario = JSON.parse(process.argv[3]);
const els = {};
function el(id) {
  if (!els[id]) els[id] = {
    id, value: (scenario.values || {})[id] ?? "", checked: (scenario.checked || {})[id] ?? false,
    disabled: false, hidden: (scenario.hidden || {})[id] ?? false, style: {}, dataset: {},
    textContent: "", innerHTML: "", className: "", options: [], open: false, listeners: {},
    classList: { add() {}, remove() {}, toggle() {}, contains() { return false; } },
    addEventListener(t, f) { (this.listeners[t] = this.listeners[t] || []).push(f); },
    append() {}, scrollIntoView() {}, focus() {}, select() {},
  };
  return els[id];
}
const document = {
  getElementById: el, body: el("body"),
  querySelector(q) {
    const m = /name="cockpit-map-source"\]:checked/.test(q);
    return m ? { value: scenario.mapSource || "slam" } : null;
  },
  querySelectorAll() { return []; },
  addEventListener() {}, createElement() { return el("x" + Math.random()); },
};
const window = { confirm: (m) => { confirms.push(m); return scenario.confirm !== false; }, addEventListener() {} };
const confirms = [], fetched = [], toasts = [], logs = [];
const localStorage = { getItem() { return null; }, setItem() {} };
class EventSource { constructor() {} close() {} }
class Option { constructor(t, v) { this.text = t; this.value = v; } }
const state = { robot_name: "robot1", status: {} };
const robotRunMode = () => scenario.mode || "sim";
const controllerForRun = (v) => v || "pico2";
const configReadyOrWarn = async () => true;
const boardMatchesOrWarn = async () => true;
const logLine = (x) => logs.push(x);
const openTerminal = () => {};
const showActionBanner = () => {};
const hideActionBanner = () => {};
const refreshStatus = () => {};
// app-bringup.js's gate: its refresh can show the real-robot group for the first time.
const buOneClickGate = scenario.reveal ? async () => { el("grp-cockpit-real").hidden = false; return null; } : undefined;
let streamResolve = null;
async function fetch(url, opts) {
  fetched.push(url);
  if (url.startsWith("/api/workflow/one-click/stream")) {
    if (scenario.stream === "pending") return new Promise((r) => { streamResolve = r; });
    if (scenario.stream === "refused") return { ok: false, status: 409, body: null,
      text: async () => JSON.stringify({ detail: "A 1-Click run is already in progress: Abort it first." }) };
    const frames = (scenario.lines || []).map((l) => `event: output\ndata: ${JSON.stringify({ line: l })}\n\n`)
      .concat([`event: done\ndata: ${JSON.stringify({ exit_code: scenario.exit_code ?? 0 })}\n\n`]).join("");
    let sent = false;
    return { ok: true, status: 200, body: { getReader: () => ({ read: async () => sent
      ? { done: true } : (sent = true, { value: Buffer.from(frames), done: false }) }) } };
  }
  if (url === "/api/maps") return { ok: true, json: async () => ({ maps: [] }) };
  return { ok: true, status: 200, json: async () => ({}), text: async () => "" };
}
global.TextDecoder = require("util").TextDecoder;
eval(src + `
showToast = (m) => toasts.push(m);
initCockpitDashboard();
const click = (id) => Promise.all((el(id).listeners.click || []).map((f) => f()));
(async () => {
  const before = { start: el("btn-cockpit-oneclick").disabled };
  const runs = [];
  for (let i = 0; i < (scenario.clicks || 1); i++) runs.push(click(scenario.button || "btn-cockpit-oneclick"));
  await new Promise((r) => setTimeout(r, 50));
  const during = { start: el("btn-cockpit-oneclick").disabled, abort: el("btn-cockpit-stop-pipeline").disabled,
                   hdrAbort: el("btn-header-abort-pipeline").hidden, badge: el("cockpit-pipeline-badge").textContent };
  if (scenario.stream !== "pending") await Promise.all(runs);
  await new Promise((r) => setTimeout(r, 20));
  console.log(JSON.stringify({
    fetched: fetched.filter((u) => u.startsWith("/api/workflow/one-click/stream")), during, toasts, confirms,
    badge: el("cockpit-pipeline-badge").textContent, msg: el("cockpit-pipeline-msg").textContent,
    optsMsg: el("cockpit-opts-msg").textContent,
    after: { start: el("btn-cockpit-oneclick").disabled, abort: el("btn-cockpit-stop-pipeline").disabled,
             hdrAbort: el("btn-header-abort-pipeline").hidden },
  }));
  process.exit(0);
})();
`);
"""


def page(tmp_path, **scenario):
    h = tmp_path / "page.js"
    h.write_text(HARNESS)
    out = subprocess.run([NODE, str(h), os.path.join(FRONT, "app-workflow.js"), json.dumps(scenario)],
                         capture_output=True, text=True, timeout=30)
    assert out.returncode == 0, out.stderr
    return json.loads(out.stdout.strip().splitlines()[-1])


@needs_node
def test_a_double_click_starts_one_run_and_leaves_its_buttons_alone(tmp_path):
    r = page(tmp_path, clicks=2, stream="pending")
    assert len(r["fetched"]) == 1, r["fetched"]
    assert r["during"] == {"start": True, "abort": False, "hdrAbort": False, "badge": "Running..."}
    assert any("already starting or running" in t for t in r["toasts"])


@needs_node
def test_the_header_start_beside_the_cards_is_one_run(tmp_path):
    h = tmp_path / "x"
    h.mkdir()
    r = page(h, clicks=2, stream="pending", button="btn-header-start-pipeline")
    assert len(r["fetched"]) == 1


@needs_node
def test_a_refused_start_says_why_and_is_not_running(tmp_path):
    r = page(tmp_path, stream="refused")
    assert r["badge"] == "Not started"
    assert "already in progress" in r["msg"] and any("already in progress" in t for t in r["toasts"])
    assert r["after"] == {"start": False, "abort": True, "hdrAbort": True}


@needs_node
def test_the_verdict_follows_the_options_and_the_failures(tmp_path):
    ok = page(tmp_path, values={"cockpit-run-scope": "topics_only"}, exit_code=0)
    assert ok["msg"].startswith("✅ Topics verified; nothing was commanded")
    assert "Map saved" not in ok["msg"]
    bad = page(tmp_path, exit_code=1, lines=[
        "❌ 1-Click Pipeline finished WITH 2 FAILED STEP(S):", "   - Nav2: did not activate (timeout)",
        "   - Map saver: exit 1", "=================================================================="])
    assert bad["badge"] == "FAILED (1)"
    assert "Nav2: did not activate (timeout); Map saver: exit 1" in bad["msg"]
    assert "Flashing" not in bad["msg"]
    saved = page(tmp_path, checked={"cockpit-explore": True}, exit_code=0,
                 lines=["🎉 MAP SAVED: /x/maps/garage.yaml / .pgm"])
    assert "map saved (/x/maps/garage.yaml / .pgm)" in saved["msg"] and "frontier exploration" in saved["msg"]


@needs_node
def test_the_run_options_are_sent_and_empty_fields_are_not(tmp_path):
    r = page(tmp_path, checked={"cockpit-explore": False, "cockpit-require-goal": True},
             values={"cockpit-goal-round-trips": "0", "cockpit-home-yaw-tolerance": "30",
                     "cockpit-map-output": "garage_2", "cockpit-explore-timeout": "600"})
    url = r["fetched"][0]
    for q in ("goal_round_trips=0", "require_goal=true", "home_yaw_tolerance_deg=30", "map_output=garage_2",
              "explore=false"):
        assert q in url, q
    assert "explore_timeout" not in url, "exploration is off: its limit is not sent"
    assert "explore_sec" not in url, "the mapping spin is only for a run that skips Nav2"
    assert "goal_tolerance" not in url and "clearance" not in url


@needs_node
def test_a_bad_option_stops_before_anything_starts(tmp_path):
    r = page(tmp_path, checked={"cockpit-explore": True}, values={"cockpit-explore-timeout": "10"})
    assert r["fetched"] == []
    assert "Exploration time limit" in r["optsMsg"] and r["badge"] != "Running..."


@needs_node
def test_a_real_robot_is_confirmed_and_hidden_options_are_not_sent(tmp_path):
    # The real-robot group is hidden (its checklist says otherwise): its ticked boxes stay home.
    r = page(tmp_path, mode="real", hidden={"grp-cockpit-real": True},
             checked={"cockpit-test-acc": True, "cockpit-skip-bringup": True})
    assert len(r["confirms"]) == 1 and "REAL robot" in r["confirms"][0] and "Abort" in r["confirms"][0]
    assert r["fetched"] and "test_acc" not in r["fetched"][0] and "skip_bringup" not in r["fetched"][0]
    no = page(tmp_path, mode="real", confirm=False)
    assert no["fetched"] == [] and no["badge"] != "Running..."


# ---------------------------------------------------------------- the pure helpers
HELPERS = r"""
const src = require("fs").readFileSync(process.argv[2], "utf8");
const a = src.indexOf("// 1-CLICK RUN OPTIONS");
const b = src.indexOf("function initCockpitDashboard() {");
eval(src.slice(a, b) + `
console.log(JSON.stringify(oneClickRunOptions(JSON.parse(process.argv[3]))));`);
"""


def options(tmp_path, **v):
    h = tmp_path / "helpers.js"
    h.write_text(HELPERS)
    out = subprocess.run([NODE, str(h), os.path.join(FRONT, "app-workflow.js"), json.dumps(v)],
                         capture_output=True, text=True, timeout=30)
    assert out.returncode == 0, out.stderr
    return json.loads(out.stdout.strip().splitlines()[-1])


@needs_node
@pytest.mark.parametrize("v, sent, missing", [
    ({"scope": "topics_only"}, ["topics_only=true", "no_drive_test=true"], []),
    ({"scope": "topics_drive", "noDriveTest": True}, ["topics_only=true"], ["no_drive_test"]),
    ({"mapSource": "saved", "savedMap": "kitchen", "explore": True, "mapOutput": "x"},
     ["saved_map=kitchen"], ["explore_timeout", "map_output"]),
    ({"explore": True, "exploreTimeout": "900", "exploreMinArea": "0", "exploreBeyondRoom": True},
     ["explore_timeout=900", "explore_min_area=0"], ["explore_beyond_room"]),     # not real: not sent
    ({"explore": True, "real": True, "exploreBeyondRoom": True, "goalX": "2"}, ["explore_beyond_room=true"], ["goal_x"]),
    ({"explore": False, "real": True, "goalX": "2.5", "goalY": "-1"}, ["goal_x=2.5", "goal_y=-1"], []),
    ({"real": True, "inch": False, "inchMax": "0.8", "testAcc": True, "testAccClearance": "1.5"},
     ["inch=false", "inch_max=0.8", "test_acc_clearance=1.5"], []),
    ({"real": False, "inch": False, "clearance": "0.5"}, [], ["inch", "clearance"]),
])
def test_run_options(tmp_path, v, sent, missing):
    r = options(tmp_path, **v)
    assert r["errors"] == []
    for q in sent:
        assert q in r["params"], (q, r["params"])
    for k in missing:
        assert not any(p.startswith(k + "=") for p in r["params"]), (k, r["params"])


@needs_node
@pytest.mark.parametrize("v, word", [
    ({"explore": True, "exploreTimeout": "59"}, "Exploration time limit"),
    ({"explore": True, "exploreMinArea": "-1"}, "Minimum map area"),
    ({"explore": False, "roundTrips": "1.5"}, "Round trips"),
    ({"explore": False, "goalTolerance": "2"}, "Goal tolerance"),
    ({"explore": False, "homeYawDeg": "0"}, "Home heading"),
    ({"mapOutput": "../etc"}, "Save map as"),
    ({"mapSource": "saved", "savedMap": ""}, "Saved map"),
    ({"real": True, "clearance": "3"}, "Clearance"),
    ({"real": True, "testAcc": True, "testAccClearance": "0.5"}, "test_acc needs"),
])
def test_run_options_out_of_range(tmp_path, v, word):
    r = options(tmp_path, **v)
    assert any(word in e for e in r["errors"]), r


@needs_node
def test_zero_clearance_is_flagged_for_a_confirmation(tmp_path):
    assert options(tmp_path, real=True, clearance="0")["zeroClearance"] is True
    assert options(tmp_path, real=True, clearance="0.4")["zeroClearance"] is False


@needs_node
def test_options_the_gate_just_revealed_are_reviewed_before_a_start(tmp_path):
    r = page(tmp_path, mode="real", reveal=True, hidden={"grp-cockpit-real": True},
             checked={"cockpit-test-acc": True})
    assert r["fetched"] == [] and r["confirms"] == []
    assert "review its options" in r["msg"] and r["badge"] != "Running..."
