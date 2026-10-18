"""Navigation runs beside SLAM: it has its own runner, "nav".

The SLAM & Nav page started SLAM and Navigation both on the "main" slot, so
Navigation was refused with a 409 ("Slot 'main' is busy") whenever SLAM was up --
and nav2.launch.py without a saved map navigates on the map SLAM publishes, so
that is exactly when it is needed (real robot, 2026-10-18). The map save had
already moved off "main" for the same reason.
"""
import asyncio
import json
import os
import re
import shutil
import subprocess
import sys

import pytest

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FRONTEND = os.path.join(REPO_ROOT, "web", "frontend")


def read(*p):
    return open(os.path.join(REPO_ROOT, *p)).read()


def _wiring(js, start_id):
    start = js.index(f'startBtn: document.getElementById("{start_id}")')
    return js[start:js.index("\n});\n", start)]


# --------------------------------------------------------------- the page
def test_navigation_has_its_own_slot_and_slam_keeps_main():
    js = read("web", "frontend", "app-nav-viz.js")
    assert re.search(r'^\s*slot: "nav",$', _wiring(js, "btn-nav-start"), re.M)
    assert re.search(r'^\s*slot: "main",$', _wiring(js, "btn-slam-start"), re.M)


def test_stop_navigation_kills_its_own_slot():
    # wireStartStop's Stop kills the slot it was given: "nav" reaches /api/kill,
    # which looks the runner up in RUNNERS.
    core_js = read("web", "frontend", "app-core.js")
    body = core_js[core_js.index("function wireStartStop("):]
    body = body[:body.index("\n}\n")]
    assert "killSlot(slot)" in body
    kill = core_js[core_js.index("function killSlot("):]
    kill = kill[:kill.index("\n}\n")]
    assert '"/api/kill"' in kill


def _state(tag, status):
    node = shutil.which("node")
    if not node:
        pytest.skip("node is not installed")
    js = read("web", "frontend", "app-core.js")
    start = js.index("function stackPartState(")
    fn = js[start:js.index("\n}\n", start) + 3]
    out = subprocess.run([node, "-e", fn + f"\nconsole.log(JSON.stringify(stackPartState("
                          f"{json.dumps(tag)}, {json.dumps(status)})));"],
                         capture_output=True, text=True, check=True)
    return json.loads(out.stdout)


def test_each_part_is_judged_by_its_own_runner():
    live = {"nav2": True, "slam": True, "bringup": True}
    # SLAM busy on "main", Nav2 started elsewhere: Nav2 is not this cockpit's to stop
    nav = _state("nav2", {"stack": [], "liveness": live, "main_busy": True, "nav_busy": False})
    assert nav["running"] and not nav["stoppable"]
    nav = _state("nav2", {"stack": [], "liveness": live, "main_busy": False, "nav_busy": True})
    assert nav["running"] and nav["stoppable"]
    slam = _state("slam", {"stack": [], "liveness": live, "main_busy": True, "nav_busy": False})
    assert slam["running"] and slam["stoppable"]
    bring = _state("bringup", {"stack": [], "liveness": live, "bringup_busy_console": True})
    assert bring["running"] and bring["stoppable"]


# --------------------------------------------------------------- the backend
def test_the_backend_has_a_nav_runner_and_reports_it():
    core = read("web", "backend", "core.py")
    assert 'nav_runner = ProcessRunner("nav")' in core
    assert '"nav": nav_runner' in core
    status = read("web", "backend", "routes_status.py")
    assert '"nav_busy": nav_runner.is_busy(),' in status


class _Req:
    def __init__(self, body):
        self._body = body

    async def json(self):
        return self._body


def _backend():
    sys.path.insert(0, os.path.join(REPO_ROOT, "web", "backend"))
    pytest.importorskip("fastapi", reason="the cockpit backend needs fastapi/uvicorn")
    os.environ["COCKPIT_AUTH"] = "off"
    import core
    import routes_exec
    return core, routes_exec


def test_navigation_starts_while_slam_holds_main(monkeypatch):
    core, routes_exec = _backend()
    from fastapi import HTTPException
    monkeypatch.setenv("COCKPIT_ALLOW_RAW_EXEC", "1")
    monkeypatch.setattr(core.main_runner, "is_busy", lambda: True)   # SLAM is up

    with pytest.raises(HTTPException) as exc:
        asyncio.run(routes_exec.api_exec(_Req({"command": "echo X", "slot": "main"})))
    assert exc.value.status_code == 409

    async def body():
        res = await routes_exec.api_exec(_Req({"command": "echo NAV_UP", "slot": "nav"}))
        out = []
        async for chunk in res.body_iterator:
            out.append(chunk if isinstance(chunk, str) else chunk.decode())
        return "".join(out)

    text = asyncio.run(body())
    assert 'data: {"line": "NAV_UP"}' in text
    assert 'event: done\ndata: {"exit_code": 0}' in text


def test_kill_nav_stops_the_nav_runner_only(monkeypatch):
    core, routes_exec = _backend()
    killed = []
    for name, runner in core.RUNNERS.items():
        monkeypatch.setattr(runner, "kill", lambda n=name: killed.append(n) or True)
    res = asyncio.run(routes_exec.api_kill(_Req({"slot": "nav"})))
    assert res["slot"] == "nav" and killed == ["nav"]
