"""The Rate & Echo audit (web/backend/routes_hardware.py /api/topics/verify) gives
verify_topics.py the time it takes, and says a timeout as a result.

verify_topics.py collects for --timeout and measures a short topic once more for as long again;
with rclpy's start-up that is past the 20 s the route allowed. Every audit that had a topic to
report then died as a 500, and the page said "no topics -- is the stack running?" while the
stack ran (mc200-yahboom, 2026-10-18)."""
import os
import subprocess
import sys

import pytest

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO_ROOT, "web", "backend"))

pytest.importorskip("fastapi", reason="the cockpit backend needs fastapi/uvicorn")
os.environ["COCKPIT_AUTH"] = "off"
import routes_hardware as rh  # noqa: E402


def test_the_deadline_sits_past_the_scripts_own_worst_case(monkeypatch):
    seen = {}

    def fake_run(cmd, **kw):
        seen["cmd"], seen["timeout"] = " ".join(cmd), kw.get("timeout")
        return subprocess.CompletedProcess(cmd, 0, stdout='{"all_passed": true, "topics": {}}\n', stderr="")

    monkeypatch.setattr(rh.subprocess, "run", fake_run)
    rh.api_verify_topics(distro="jazzy")
    worst = rh.VERIFY_TIMEOUT_S * (1 + rh.VERIFY_RETRIES)
    assert f"--timeout {rh.VERIFY_TIMEOUT_S:g}" in seen["cmd"] and f"--retries {rh.VERIFY_RETRIES}" in seen["cmd"]
    assert seen["timeout"] >= worst + 10


def test_a_timeout_is_a_result_not_a_500(monkeypatch):
    def slow(cmd, **kw):
        raise subprocess.TimeoutExpired(cmd, kw.get("timeout"))

    monkeypatch.setattr(rh.subprocess, "run", slow)
    out = rh.api_verify_topics(distro="jazzy")
    assert out["success"] is False and out["data"] is None and "did not finish" in out["raw"]
