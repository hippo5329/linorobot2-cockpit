"""The bringup health card must measure rates the way the pipeline's gate does.

It shelled out to `ros2 topic hz` per topic under a 3 s `timeout`. That CLI
first looks the publisher up in the graph to copy its QoS, and under the
cockpit's Fast DDS profile that lookup does not find the micro-ROS agent's
publishers: `ros2 topic list` showed /odom/unfiltered and /imu/data, `hz` on
either said "does not appear to be published yet", and the card drew two red
rows on a robot whose own 1-Click gate had just measured both at 50 Hz -- via
a direct sensor-data subscription, which needs no graph lookup. Measured
2026-09-22 on a bare Pico 2 launched from the UI.
"""
import ast
import os
import re
import sys

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
RUNNERS = os.path.join(REPO_ROOT, "web", "backend", "runners.py")


def _src():
    return open(RUNNERS, encoding="utf-8").read()


def _func(name, code_only=False):
    """Source of a top-level function; with code_only, its docstring removed,
    so a test about what the code DOES cannot be tripped by a comment that
    explains what it used to do."""
    src = _src()
    tree = ast.parse(src)
    for node in tree.body:
        if isinstance(node, ast.FunctionDef) and node.name == name:
            seg = ast.get_source_segment(src, node)
            doc = ast.get_docstring(node, clean=False)
            if code_only and doc:
                seg = seg.replace(doc, "", 1)
            return seg
    raise AssertionError(f"{name} not found")


def test_probe_no_longer_shells_out_to_topic_hz():
    body = _func("check_bringup_health", code_only=True)
    assert "ros2 topic hz" not in body, (
        "check_bringup_health went back to `ros2 topic hz`, whose graph lookup "
        "does not see micro-ROS publishers under the Fast DDS profile"
    )


def test_probe_subscribes_with_sensor_data_qos():
    assert "qos_profile_sensor_data" in _src(), (
        "the probe must subscribe with the sensor-data QoS so best-effort "
        "micro-ROS topics match"
    )


def test_every_health_topic_has_a_message_type():
    ns = {}
    src = _src()
    for name in ("BRINGUP_HEALTH_TOPICS", "BRINGUP_HEALTH_TYPES"):
        m = re.search(rf"^{name}\s*=\s*(\[.*?\]|\{{.*?\}})", src, re.S | re.M)
        assert m, name
        ns[name] = ast.literal_eval(m.group(1))
    keys = {t[0] for t in ns["BRINGUP_HEALTH_TOPICS"]}
    missing = keys - set(ns["BRINGUP_HEALTH_TYPES"])
    assert not missing, f"health topics with no message type, so never rated: {missing}"
    for typ in ns["BRINGUP_HEALTH_TYPES"].values():
        assert re.fullmatch(r"[a-z_]+/msg/[A-Z][A-Za-z0-9]+", typ), typ


def test_rate_from_stamps():
    sys.path.insert(0, os.path.join(REPO_ROOT, "web", "backend"))
    src = _func("rate_from_stamps")
    # Real typing objects: Python 3.14 defers annotations, but 3.12 (CI) evaluates
    # `Optional[float]` at def time, and a stand-in `object` is not subscriptable.
    from typing import List, Optional
    ns = {"List": List, "Optional": Optional}
    exec(src, ns)
    f = ns["rate_from_stamps"]
    assert f([]) is None and f([1.0]) is None
    assert abs(f([0.0, 0.02, 0.04, 0.06, 0.08]) - 50.0) < 1e-6
    assert f([5.0, 5.0]) is None                      # zero span, no divide


def _health_with(monkeypatch, rates, present=None, **kw):
    """check_bringup_health on a fake graph: `present` advertised (default: every health
    topic), `rates` Hz each."""
    import json
    import subprocess
    import types
    sys.path.insert(0, os.path.join(REPO_ROOT, "web", "backend"))
    import runners
    topics = [t for _, t, _, _ in runners.BRINGUP_HEALTH_TOPICS]
    listed = topics if present is None else present

    def fake_run(cmd, **_):
        stamps = {t: [i / hz for i in range(int(hz * 4))] if hz else [] for t, hz in
                  ((t, rates.get(t, 0.0)) for t in topics)}
        out = {"graph": ["/rosout", "/parameter_events"] + topics, "present": list(listed),
               "stamps": stamps}
        return types.SimpleNamespace(returncode=0, stdout=json.dumps(out), stderr="")
    monkeypatch.setattr(subprocess, "run", fake_run)
    monkeypatch.setattr(runners.subprocess, "run", fake_run)
    return runners.check_bringup_health(timeout=1.0, **kw)


def test_the_ekf_alone_is_not_ready(monkeypatch):
    """The EKF publishes /odom with no input at all; a silent board is not a ready robot
    (walkthrough 2026-10-06: /odom 33.5 Hz, /odom/unfiltered silent, read as ready)."""
    assert not _health_with(monkeypatch, {"/odom": 33.5})["ready"]
    assert _health_with(monkeypatch, {"/odom": 50.0, "/odom/unfiltered": 50.0, "/scan": 10.0})["ready"]


def test_a_missing_scan_fails_a_robot_with_a_lidar(monkeypatch):
    """The first check passed with /scan not advertised at all (walkthrough 2026-10-16)."""
    no_scan = ["/odom", "/odom/unfiltered", "/imu/data"]
    h = _health_with(monkeypatch, {"/odom": 50.0, "/odom/unfiltered": 50.0}, present=no_scan)
    assert not h["ready"]
    assert h["topics"]["scan"]["advertised"] is False
    assert "/scan not advertised" in h["summary"]
    # advertised but silent is not healthy either
    h = _health_with(monkeypatch, {"/odom": 50.0, "/odom/unfiltered": 50.0})
    assert not h["ready"] and "/scan no messages" in h["summary"]
    assert "/scan" in h["advertised_but_silent"]


def test_a_robot_with_no_scan_source_does_not_need_one(monkeypatch):
    h = _health_with(monkeypatch, {"/odom": 50.0, "/odom/unfiltered": 50.0},
                     present=["/odom", "/odom/unfiltered"], has_scan=False)
    assert h["ready"]
    assert h["topics"]["scan"]["fitted"] is False and h["topics"]["scan"]["ok"]


def test_no_answer_from_the_probe_is_no_graph(monkeypatch):
    import subprocess
    import types
    sys.path.insert(0, os.path.join(REPO_ROOT, "web", "backend"))
    import runners
    fake = lambda cmd, **_: types.SimpleNamespace(returncode=1, stdout="", stderr="ModuleNotFoundError: rclpy")
    monkeypatch.setattr(runners.subprocess, "run", fake)
    h = runners.check_bringup_health(timeout=1.0)
    assert h["status"] == "no_graph" and not h["ready"] and "rclpy" in h["summary"]

    def hang(cmd, **kw):
        raise subprocess.TimeoutExpired(cmd, kw.get("timeout"))
    monkeypatch.setattr(runners.subprocess, "run", hang)
    h = runners.check_bringup_health(timeout=1.0)
    assert h["status"] == "error" and "did not answer" in h["summary"]


def test_the_route_expects_a_scan_from_the_robots_scan_source():
    src = open(os.path.join(REPO_ROOT, "web", "backend", "routes_exec.py")).read()
    body = src[src.index("def api_bringup_health"):]
    body = body[:body.index("\n\n\n")]
    assert "depth_camera.scan_source(" in body and "has_scan=has_scan" in body


def test_bringup_start_stops_a_stack_left_running():
    js = open(os.path.join(REPO_ROOT, "web", "frontend", "app-agent-bringup.js")).read()
    start = js.index('startBtn: document.getElementById("btn-bringup-start")')
    block = js[start:js.index("\n});", start)]
    assert 'fetch("/api/stack")' in block and '"/api/stack/stop"' in block
    assert block.index("boardMatchesOrWarn") < block.index('"/api/stack/stop"'), \
        "a refused Bringup must not stop the running robot"
