"""The tool's STOP and the bring-up buttons on the page (web/frontend/app-tool.js,
app-bringup.js), run under node with stubs.

The review found STOP saying "Stop sent." whatever the server answered -- and the server
answered 404 once the header showed another robot, while the wheel turned -- and the
bring-up's motion buttons starting a second run when clicked twice.
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

STUBS = r"""
const els = {};
function el(id) {
  if (!els[id]) els[id] = { id, value: "", disabled: false, hidden: false, textContent: "", dataset: {},
    classes: new Set(),
    classList: { toggle(c, on) { on ? els[id].classes.add(c) : els[id].classes.delete(c); },
                 add(c) { els[id].classes.add(c); }, remove(c) { els[id].classes.delete(c); },
                 contains(c) { return els[id].classes.has(c); } } };
  return els[id];
}
const document = { getElementById: el, body: el("body"), addEventListener() {},
  querySelector() { return null; }, querySelectorAll(q) { return /data-spin/.test(q) ? [el("spin1"), el("spin2")] : []; } };
const fetched = [], toasts = [];
const showToast = (m) => toasts.push(m);
const answer = JSON.parse(process.argv[3]);
async function fetch(url, opts) {
  fetched.push([url, opts && opts.body ? JSON.parse(opts.body) : null]);
  return { ok: answer.status < 300, status: answer.status, json: async () => answer.body || {} };
}
"""

STOP = STUBS + r"""
const src = require("fs").readFileSync(process.argv[2], "utf8");
eval(src.replace(/document\.addEventListener\("DOMContentLoaded", toolInit\);/, "") + `
tool.open = true; tool.robot = "robot1";
toolStop().then((ok) => console.log(JSON.stringify({ ok, fetched, toasts,
  status: el("tool-status").textContent, red: el("tool-status").classList.contains("tool-stop-failed") })));`);
"""


def run(tmp_path, harness, script, answer):
    h = tmp_path / "h.js"
    h.write_text(harness)
    out = subprocess.run([NODE, str(h), os.path.join(FRONT, script), json.dumps(answer)],
                         capture_output=True, text=True, timeout=30)
    assert out.returncode == 0, out.stderr
    return json.loads(out.stdout.strip().splitlines()[-1])


@needs_node
def test_stop_names_the_sessions_robot_and_says_when_it_failed(tmp_path):
    r = run(tmp_path, STOP, "app-tool.js", {"status": 404, "body": {"detail": "No tool session is open for robot1."}})
    assert r["fetched"] == [["/api/tool/stop", {"robot": "robot1"}]]
    assert r["ok"] is False and r["red"] is True
    assert r["status"].startswith("STOP FAILED -- press Esc / unplug the motors' power")
    assert "Stop sent" not in r["status"] and r["toasts"]


@needs_node
def test_a_stop_that_arrived_says_so(tmp_path):
    r = run(tmp_path, STOP, "app-tool.js", {"status": 200, "body": {"ok": True}})
    assert r["ok"] is True and r["status"] == "Stop sent." and r["red"] is False


SPIN = STUBS + r"""
const src = require("fs").readFileSync(process.argv[2], "utf8");
const tool = { open: true, jsonHooks: [] };
const sent = [];
async function toolSend(line) { sent.push(line); return true; }
async function toolOpen() { return true; }
eval(src.replace(/document\.addEventListener\("DOMContentLoaded", buInit\);/, "") + `
el("bu-mot-pct").value = "40"; el("bu-mot-secs").value = "2";
const first = buSpin(1), second = buSpin(2);
Promise.all([first, second]).then(() => {
  const during = { spin1: el("spin1").disabled, spin2: el("spin2").disabled, sent: [...sent] };
  setTimeout(() => console.log(JSON.stringify({ during, after: { spin1: el("spin1").disabled, sent } })), 2800);
});`);
"""


@needs_node
def test_a_wheel_spin_runs_once_and_its_buttons_wait_for_it(tmp_path):
    r = run(tmp_path, SPIN, "app-bringup.js", {"status": 200})
    assert r["during"]["sent"] == ["motor 1 40 2"], "the second click sent nothing"
    assert r["during"]["spin1"] is True and r["during"]["spin2"] is True
    assert r["after"]["spin1"] is False and r["after"]["sent"] == ["motor 1 40 2", "stop"]


ATTACH = STUBS + r"""
const src = require("fs").readFileSync(process.argv[2], "utf8");
eval(src.replace(/document\.addEventListener\("DOMContentLoaded", toolInit\);/, "") + `
toolAttach = () => { tool.attached = true; };
toolRefreshState().then(() => console.log(JSON.stringify({ attached: !!tool.attached,
  status: el("tool-status").textContent })));`);
"""


@needs_node
def test_a_session_already_open_says_so_when_the_page_attaches(tmp_path):
    r = run(tmp_path, ATTACH, "app-tool.js",
            {"status": 200, "body": {"open": True, "robot": "robot1", "wifi": False, "refusal": ""}})
    assert r["attached"] is True
    assert "already open" in r["status"] and "Back to robot" in r["status"]

