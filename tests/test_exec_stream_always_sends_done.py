"""An exec stream ends with its `done` event, however late the runner queues it.

The stream looped while the runner's process was alive. The process exits before
the runner's thread queues `done`, and a check in that gap closed the stream
without it: on a slow UNO Q the web UI's LiDAR bring-up step hung after
AGENT_FOUND (2026-10-16). The stream now runs until the streaming thread ends.
"""
import asyncio
import os
import sys
import time

import pytest

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO_ROOT, "web", "backend"))
pytest.importorskip("fastapi", reason="the cockpit backend needs fastapi/uvicorn")
os.environ["COCKPIT_AUTH"] = "off"
import core  # noqa: E402
import routes_exec  # noqa: E402


# Called directly with a stub request: the robot image has fastapi but not httpx, so a
# TestClient test would skip in the one place the backend runs.
class _Req:
    def __init__(self, body):
        self._body = body

    async def json(self):
        return self._body


async def _body(handler, body):
    res = await handler(_Req(body))
    out = []
    async for chunk in res.body_iterator:
        out.append(chunk if isinstance(chunk, str) else chunk.decode())
    return "".join(out)


@pytest.mark.parametrize("handler,slot", [
    ("api_exec", "main"), ("api_agent_exec", "agent"), ("api_bringup_exec", "bringup"),
])
def test_a_late_done_still_reaches_the_page(monkeypatch, handler, slot):
    monkeypatch.setenv("COCKPIT_ALLOW_RAW_EXEC", "1")
    runner = core.RUNNERS[slot]
    real = runner._broadcast

    def slow_done(event_type, payload):
        if event_type == "done":
            time.sleep(1.5)       # the process has exited; `done` is not queued yet
        real(event_type, payload)

    monkeypatch.setattr(runner, "_broadcast", slow_done)
    text = asyncio.run(_body(getattr(routes_exec, handler),
                             {"command": "echo AGENT_FOUND", "slot": slot}))
    assert 'data: {"line": "AGENT_FOUND"}' in text
    assert 'event: done\ndata: {"exit_code": 0}' in text
