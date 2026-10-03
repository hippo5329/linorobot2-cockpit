"""The Teleop tab's virtual gamepad: what the page sends is what the routes answer.

The page read `started` from /api/gamepad/start, which returned only `running`,
so every press said "could not start the publisher", sent no command and kept
Stop disabled -- while the publisher it had started sent zero twists on
/cmd_vel at 20 Hz until the cockpit restarted, fighting Nav2. The page also
POSTed /api/gamepad/stall to a GET route, and its topic was ignored. Found by
driving the tab in a browser (2026-10-04), not by any test.
"""
import os
import re

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _read(*p):
    return open(os.path.join(ROOT, *p)).read()


def _start_route():
    src = _read("web", "backend", "routes_exec.py")
    a = src.index('@app.post("/api/gamepad/start")')
    return src[a:src.index("\n@app", a + 10)]


def test_start_answers_the_field_the_page_reads():
    js = _read("web", "frontend", "app-workflow.js")
    assert "!r.started" in js
    route = _start_route()
    assert re.search(r'"started":\s*started', route), "the page reads r.started"
    assert 'data.get("topic")' in route and "gamepad_runner.start(topic)" in route


def test_every_gamepad_call_the_page_makes_has_a_route_for_its_method():
    js = _read("web", "frontend", "app-workflow.js")
    calls = set(re.findall(r'vgpPost\("(/api/gamepad/[a-z]+)"', js))
    assert calls == {"/api/gamepad/start", "/api/gamepad/cmd", "/api/gamepad/stall", "/api/gamepad/kill"}
    src = _read("web", "backend", "routes_exec.py")
    for url in calls:   # vgpPost always POSTs
        assert (f'@app.post("{url}")' in src or
                re.search(rf'@app\.api_route\("{re.escape(url)}", methods=\[[^\]]*"POST"', src)), url


def test_a_failed_start_stops_what_it_started():
    js = _read("web", "frontend", "app-workflow.js")
    a = js.index("async function startGamepad()")
    body = js[a:js.index("vgpRunning = true;", a)]
    assert 'vgpPost("/api/gamepad/kill"' in body


def test_the_runner_restarts_on_another_topic():
    import sys
    sys.path.insert(0, os.path.join(ROOT, "web", "backend"))
    src = _read("web", "backend", "runners.py")
    a = src.index("    def start(self, topic")
    body = src[a:src.index("    def send(", a)]
    assert "self.topic == topic" in body and "self.kill()" in body and "self.topic = topic" in body
