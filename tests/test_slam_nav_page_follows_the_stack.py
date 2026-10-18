"""The SLAM & Nav page shows the stack that is really running, and Stop stops it.

A 1-Click run leaves bringup, SLAM and Nav2 up on purpose and records them in
scripts/robot_stack.py; they are not the backend's children. The page knew only
its own runner, so after a 1-Click run it offered "Start SLAM" with Stop greyed
out over a SLAM that was running (web UI walkthrough, 2026-10-16). /api/status
now carries the record (`stack`) and whether SLAM and Nav2 processes are up at
all (`liveness.slam`, `liveness.nav2`), and every status poll sets the buttons.
"""
import json
import os
import re
import shutil
import subprocess
import types

import pytest

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FRONTEND = os.path.join(REPO_ROOT, "web", "frontend")


def read(*p):
    return open(os.path.join(REPO_ROOT, *p)).read()


def _function(js, name):
    start = js.index(f"function {name}(")
    end = js.index("\n}\n", start) + len("\n}\n")
    return js[start:end]


def _node(src):
    node = shutil.which("node")
    if not node:
        pytest.skip("node is not installed")
    out = subprocess.run([node, "-e", src], capture_output=True, text=True, check=True)
    return json.loads(out.stdout)


def _state(tag, status):
    js = read("web", "frontend", "app-core.js")
    return _node(_function(js, "stackPartState")
                 + f"\nconsole.log(JSON.stringify(stackPartState({json.dumps(tag)}, {json.dumps(status)})));")


# --------------------------------------------------------------- the verdict
def test_a_stack_left_by_a_1click_run_is_running_and_stoppable():
    st = _state("slam", {"stack": [{"tag": "bringup", "started": 1.0}, {"tag": "slam", "started": 2.0}],
                         "liveness": {"slam": True}, "main_busy": False})
    assert st["running"] and st["stoppable"]
    assert "1-Click" in st["text"]


def test_the_record_counts_even_before_the_probe_sees_it():
    st = _state("nav2", {"stack": [{"tag": "nav2"}], "liveness": {"nav2": False}})
    assert st["running"] and st["stoppable"]


def test_a_stack_nobody_recorded_is_shown_but_not_offered_to_stop():
    st = _state("slam", {"stack": [], "liveness": {"slam": True}, "main_busy": False})
    assert st["running"] and not st["stoppable"]


def test_this_cockpits_own_run_is_stoppable():
    st = _state("nav2", {"stack": [], "liveness": {"nav2": True}, "nav_busy": True})
    assert st["running"] and st["stoppable"]


def test_nothing_running_is_not_running():
    st = _state("slam", {"stack": [{"tag": "bringup"}], "liveness": {"bringup": True}})
    assert not st["running"] and not st["stoppable"]
    assert not _state("slam", {})["running"]


# --------------------------------------------------------------- the wiring
def test_the_page_has_a_state_badge_per_part_and_passes_it():
    html = read("web", "frontend", "index.html")
    nav = read("web", "frontend", "app-nav-viz.js")
    for tag, el in (("slam", "slam-stack-state"), ("nav2", "nav-stack-state")):
        assert f'id="{el}"' in html
        assert re.search(rf'stackTag: "{tag}",\s*\n\s*stateEl: document\.getElementById\("{el}"\)', nav)


def test_every_status_poll_syncs_the_buttons():
    js = read("web", "frontend", "app-core.js")
    body = js[js.index("async function refreshStatus"):]
    body = body[:body.index("\n}\n")]
    assert "syncStackButtons(s)" in body
    sync = _function(js, "syncStackButtons")
    assert "stopBtn.disabled = !st.stoppable" in sync
    assert "startBtn.disabled = st.running" in sync
    # a start in flight keeps its own buttons: the probe lags the launch
    assert "if (w.ownRun) continue" in sync


def test_stop_reaches_the_recorded_stack_and_refreshes():
    js = read("web", "frontend", "app-core.js")
    body = js[js.index("function wireStartStop("):]
    body = body[:body.index("\n}\n")]
    stop = body[body.index("stopBtn.addEventListener"):]
    assert '"/api/stack/stop"' in stop and "tag: stackTag" in stop
    assert "refreshStatus()" in stop


# --------------------------------------------------------------- the backend
def test_the_status_carries_the_record():
    src = read("web", "backend", "routes_status.py")
    assert "robot_stack.load()" in src
    assert '"stack": stack,' in src


def test_the_probe_sees_slam_and_nav2(monkeypatch):
    pytest.importorskip("fastapi")
    import core

    out = "\n".join([
        "---AGENT---", "---BRINGUP---",
        "/usr/bin/python3 /opt/ros/jazzy/bin/ros2 launch linorobot2_cockpit bringup.launch.py",
        "---SLAM---",
        "/opt/ros/jazzy/lib/slam_toolbox/async_slam_toolbox_node --ros-args",
        "---NAV2---",
        "/opt/ros/jazzy/lib/rclcpp_components/component_container --ros-args -r __node:=nav2_container",
    ])
    monkeypatch.setattr(core.subprocess, "run",
                        lambda *a, **k: types.SimpleNamespace(stdout=out, returncode=0))
    live = core._refresh_liveness()
    assert live["bringup"] and live["slam"] and live["nav2"] and not live["agent"]

    out = "---AGENT---\n---BRINGUP---\n---SLAM---\n---NAV2---\n"
    core.invalidate_liveness()
    live = core._refresh_liveness()
    assert not live["slam"] and not live["nav2"]
    core.invalidate_liveness()   # leave no canned probe behind for the next test


@pytest.mark.parametrize("line,section", [
    ("ros2 launch linorobot2_cockpit slam.launch.py config_file:=/x.yaml", "SLAM"),
    ("/opt/ros/lyrical/lib/slam_toolbox/sync_slam_toolbox_node --ros-args", "SLAM"),
    ("ros2 launch linorobot2_cockpit nav2.launch.py autostart:=true", "NAV2"),
    ("/opt/ros/jazzy/lib/nav2_bt_navigator/bt_navigator --ros-args", "NAV2"),
])
def test_the_probe_patterns_match_the_launches(line, section):
    src = read("web", "backend", "core.py")
    m = re.search(rf"echo '---{section}---'; \"\n\s*f\"\{{live\}} \| grep -E '([^']+)'", src)
    assert m, f"no {section} probe"
    pat = m.group(1).replace("\\\\", "\\")
    assert re.search(pat, line)
    # and never the probe's own command line
    assert not re.search(pat, m.group(0).replace("\\\\", "\\"))


def test_a_stop_invalidates_the_cached_probe():
    src = read("web", "backend", "routes_exec.py")
    for route in ("def api_bringup_kill", "async def api_stack_stop"):
        body = src[src.index(route):]
        body = body[:body.index("\n@app.")]
        assert "invalidate_liveness()" in body, route
