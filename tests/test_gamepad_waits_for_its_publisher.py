"""The virtual gamepad says "publishing" only once its publisher node exists.

On an Arduino UNO Q the publisher's shell, ROS setup and rclpy took seconds; start() returned
at once, the page said "publishing", the commands queued in the pipe, and the robot got only
the last one -- the release, zeros -- and never moved (2026-10-15).
"""
import os
import sys
import time

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "web", "backend"))
sys.path.insert(0, os.path.join(ROOT, "scripts"))
pytest.importorskip("fastapi", reason="the cockpit backend needs fastapi/uvicorn")

import runners  # noqa: E402

FAKE = """import sys, time
time.sleep({delay})
{ready}
for line in sys.stdin:
    pass
"""


def _runner(tmp_path, delay, ready=True):
    (tmp_path / "scripts").mkdir()
    (tmp_path / "scripts" / "gamepad_publisher.py").write_text(
        FAKE.format(delay=delay, ready='print("GAMEPAD_READY", flush=True)' if ready else ""))
    r = runners.GamepadRunner(str(tmp_path))
    r.status_file = str(tmp_path / "gp.json")
    return r


def test_start_returns_once_the_publisher_is_ready(tmp_path, monkeypatch):
    monkeypatch.setattr(runners, "ros_setup_shell", lambda *a, **k: "true")
    r = _runner(tmp_path, 1.0)
    t = time.time()
    try:
        assert r.start("/cmd_vel") is True
        assert time.time() - t >= 0.9
        assert r.send(0.1, 0.0, 0.0) is True
    finally:
        r.kill()


def test_a_publisher_that_never_comes_up_is_not_started(tmp_path, monkeypatch):
    monkeypatch.setattr(runners, "ros_setup_shell", lambda *a, **k: "true")
    monkeypatch.setattr(runners, "GAMEPAD_READY_TIMEOUT_S", 1.5)
    r = _runner(tmp_path, 0.1, ready=False)
    assert r.start("/cmd_vel") is False
    assert r.is_running() is False


def test_the_publisher_prints_the_line_the_runner_waits_for():
    import gamepad_publisher
    assert gamepad_publisher.READY_LINE == runners.GAMEPAD_READY_LINE
    src = open(os.path.join(ROOT, "scripts", "gamepad_publisher.py")).read()
    body = src[src.index("def main():"):]
    assert body.index("print(READY_LINE, flush=True)") < body.index("rclpy.spin(node)")


def test_the_routes_start_it_off_the_event_loop():
    src = open(os.path.join(ROOT, "web", "backend", "routes_exec.py")).read()
    assert "await run_in_threadpool(gamepad_runner.start, topic)" in src
    assert "await run_in_threadpool(gamepad_runner.start)" in src
