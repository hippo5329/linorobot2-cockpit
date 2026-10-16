"""The header selects a robot whose name has a hyphen (rover-unoq), and every place that takes a
robot's name holds one rule.

The backend's /api/robot/select accepted [a-zA-Z0-9_.-] and the firmware maps the name's hyphen
to "_" for its node name (rc-20261009.5), but selectRobot() refused anything outside
[a-z0-9_]: the robot could not be chosen in its own web UI (2026-10-15). The header field's
`pattern` still said [a-z0-9_]+ and marked a selected rover-unoq invalid (2026-10-16). The rule
is cockpit_paths.ROBOT_NAME_PATTERN; the field, selectRobot(), /api/robot/select and the tool's
routes all hold it.
"""
import json
import os
import re
import shutil
import subprocess
import sys

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "scripts"))
NODE = shutil.which("node") or shutil.which("nodejs")

import cockpit_paths  # noqa: E402

GOOD = ("rover-unoq", "rover_gendrv", "bare_sim", "rover2", "Rover.v2", "x" * 64)
BAD = ("-lead", ".hidden", "_lead", "a b", "../x", "a/b", "", "x" * 65, "rover\n")


def _js_rule():
    js = open(os.path.join(ROOT, "web", "frontend", "app-core.js")).read()
    m = re.search(r"const ROBOT_NAME_RE = /(.+?)/;", js)
    assert m, "selectRobot's name rule moved"
    body = js[js.index("async function selectRobot("):]
    assert "!ROBOT_NAME_RE.test(name)" in body[:400]
    return m.group(1)


def _html_pattern():
    html = open(os.path.join(ROOT, "web", "frontend", "index.html")).read()
    a = html.index('id="hdr-robot-name"')
    m = re.search(r'pattern="([^"]+)"', html[a:a + 400])
    assert m
    return m.group(1)


def test_the_backend_rule():
    for ok in GOOD:
        assert cockpit_paths.valid_robot_name(ok), ok
    for bad in BAD:
        assert not cockpit_paths.valid_robot_name(bad), bad


def test_the_page_states_the_backends_rule():
    assert _js_rule() == "^" + cockpit_paths.ROBOT_NAME_PATTERN + "$"
    # the field's pattern is the same class, `-` escaped for the browser's `v` flag
    assert _html_pattern().replace("\\-", "-") == cockpit_paths.ROBOT_NAME_PATTERN


@pytest.mark.skipif(NODE is None, reason="node is not installed")
def test_the_browser_agrees_name_by_name():
    """The field's pattern is compiled as the browser does (anchored, `v` flag) and selectRobot's
    regex as written; each must say what the backend says for every sample."""
    prog = r"""
const [pat, re, names] = [process.argv[1], new RegExp(process.argv[2]), JSON.parse(process.argv[3])];
const field = new RegExp("^(?:" + pat + ")$", "v");
console.log(JSON.stringify(names.map((n) => [field.test(n), re.test(n)])));
"""
    names = list(GOOD + BAD)
    out = subprocess.run([NODE, "-e", prog, _html_pattern(), _js_rule(), json.dumps(names)],
                         capture_output=True, text=True, timeout=30)
    assert out.returncode == 0, out.stderr
    for name, (field, sel) in zip(names, json.loads(out.stdout)):
        want = cockpit_paths.valid_robot_name(name)
        assert field == want and sel == want, (name, field, sel, want)


def test_the_routes_ask_the_one_rule():
    cfg = open(os.path.join(ROOT, "web", "backend", "routes_config.py")).read()
    sel = cfg[cfg.index('@app.post("/api/robot/select")'):]
    assert "cockpit_paths.valid_robot_name(name)" in sel[:600]
    tool = open(os.path.join(ROOT, "web", "backend", "routes_tool.py")).read()
    assert "cockpit_paths.valid_robot_name(robot)" in tool and "_ROBOT_NAME" not in tool
