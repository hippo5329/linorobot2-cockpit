"""Start Bringup judges the micro-ROS agent on a status read after it replaced the old stack.

Pressed while a 1-Click run's stack was up, the Bringup button stopped that stack -- and
with it the agent the stack's bringup had launched as its own child -- then decided from
the status read BEFORE the stop that an agent was alive: bringup started with
micro_ros:=false, no agent ran, and the robot said "/odom/unfiltered not advertised"
(real robot, 2026-10-18). Two halves: the page re-reads the status after the stop
(ensureAgentForBringup, web/frontend/app-agent-bringup.js), and the backend never lets a
probe that was already running when the stop landed answer as a fresh one
(invalidate_liveness, web/backend/core.py).
"""
import json
import os
import re
import shutil
import subprocess
import sys
import types

import pytest

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
JS = os.path.join(REPO_ROOT, "web", "frontend", "app-agent-bringup.js")


def _fn(src, name):
    m = re.search(r"^(async )?function %s\(\) \{.*?^\}" % name, src, re.S | re.M)
    assert m, name
    return m.group(0)


def _run(stale, fresh, engine="native", sim=False, docker=False):
    """Run ensureAgentForBringup then bringupLaunchCommand against stubbed page state.
    `fresh` is the list of statuses successive refreshStatus() calls return."""
    node = shutil.which("node")
    if not node:
        pytest.skip("node is not installed")
    src = open(JS).read()
    funcs = "\n".join(_fn(src, n) for n in ("isAgentAlive", "ensureAgentForBringup", "bringupLaunchCommand"))
    script = """
const state = {status: %s, robot_config: {path: "/cfg/r.yaml"}};
const fresh = %s;
const calls = [];
async function refreshStatus() { calls.push("refresh"); if (fresh.length) state.status = fresh.shift(); }
async function ensureAgentRunning() { calls.push("ensureAgentRunning"); }
function logLine() {}
function robotIsSimMcu() { return %s; }
function isDockerMode() { return %s; }
function getAgentEngine() { return %s; }
function getDistro() { return "jazzy"; }
function dockerDir() { return "/d"; }
%s
ensureAgentForBringup().then((verdict) => {
  const spec = bringupLaunchCommand();
  console.log(JSON.stringify({verdict, calls, micro_ros: spec.args.micro_ros}));
});
""" % (json.dumps(stale), json.dumps(fresh), json.dumps(sim), json.dumps(docker), json.dumps(engine), funcs)
    r = subprocess.run([node, "-e", script], capture_output=True, text=True, timeout=20)
    assert r.returncode == 0, r.stderr
    return json.loads(r.stdout)


STALE_ALIVE = {"agent_alive_external": True, "agent_busy_console": False}
DOWN = {"agent_alive_external": False, "agent_busy_console": False}


def test_a_native_bringup_starts_its_own_agent_when_the_stop_took_it():
    out = _run(STALE_ALIVE, [DOWN])
    assert out["verdict"] == "bringup" and out["micro_ros"] == "true"
    assert "ensureAgentRunning" not in out["calls"]


def test_a_container_agent_is_started_when_the_stop_took_it():
    out = _run(STALE_ALIVE, [DOWN, {"agent_alive_external": False, "agent_busy_console": True}],
               engine="docker")
    assert out["verdict"] == "started"
    assert out["calls"] == ["refresh", "ensureAgentRunning", "refresh"]
    assert out["micro_ros"] == "false"      # one agent: the one just started


def test_an_agent_that_survived_is_left_alone():
    out = _run(DOWN, [STALE_ALIVE], engine="docker")
    assert out["verdict"] == "alive" and out["calls"] == ["refresh"]
    assert out["micro_ros"] == "false"


def test_the_sim_mcu_never_gets_a_separate_agent():
    out = _run(STALE_ALIVE, [DOWN], engine="docker", sim=True)
    assert out["verdict"] == "sim" and "ensureAgentRunning" not in out["calls"]
    assert out["micro_ros"] == "true"


def test_the_bringup_button_judges_the_agent_after_the_stack_stop():
    src = open(JS).read()
    start = src.index('startBtn: document.getElementById("btn-bringup-start")')
    block = src[start:src.index("\n});\n", start)]
    assert block.index('"/api/stack/stop"') < block.index("await ensureAgentForBringup()") \
        < block.index("return bringupLaunchCommand()")
    # no status read before the stop that the launch could still be judged on
    assert "refreshStatus" not in block[:block.index('"/api/stack/stop"')]


def test_a_restarted_stale_bringup_rechecks_the_agent_too():
    src = open(JS).read()
    body = src[src.index("async function ensureBringupRunning("):]
    body = body[:body.index("\n}\n")]
    restart = body[body.index('await killSlot("bringup")'):]
    assert restart.index("await ensureAgentForBringup()") < restart.index("bringupLaunchCommand()")


# --------------------------------------------------------------- the backend
def test_a_probe_overtaken_by_a_stop_is_not_a_fresh_answer(monkeypatch):
    sys.path.insert(0, os.path.join(REPO_ROOT, "web", "backend"))
    pytest.importorskip("fastapi", reason="the cockpit backend needs fastapi/uvicorn")
    os.environ["COCKPIT_AUTH"] = "off"
    import core

    with_agent = "---AGENT---\n/opt/ros/jazzy/lib/micro_ros_agent/micro_ros_agent serial\n" \
                 "---BRINGUP---\n---SLAM---\n---NAV2---\n"
    without = "---AGENT---\n---BRINGUP---\n---SLAM---\n---NAV2---\n"

    def overtaken(*a, **k):
        core.invalidate_liveness()            # the Stop lands while this probe runs
        return types.SimpleNamespace(stdout=with_agent, returncode=0)

    monkeypatch.setattr(core.subprocess, "run", overtaken)
    core._refresh_liveness()
    assert core._liveness_cache["ts"] == 0.0, "an overtaken probe must not count as fresh"

    monkeypatch.setattr(core.subprocess, "run",
                        lambda *a, **k: types.SimpleNamespace(stdout=without, returncode=0))
    assert core.probe_stack_liveness()["agent"] is False   # measured again, not remembered
    core.invalidate_liveness()   # leave no canned probe behind for the next test
