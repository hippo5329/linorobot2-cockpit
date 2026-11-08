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
    assert 'data.get("topic")' in route and "run_in_threadpool(gamepad_runner.start, topic)" in route


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


def test_one_click_replaces_a_bringup_tab_bringup():
    """Start 1-Click over a running Bringup-tab bringup launched a second stack beside it."""
    src = _read("web", "backend", "routes_hardware.py")
    a = src.index("def stream_one_click_workflow(")
    body = src[a:src.index("subprocess.Popen(cmd", a)]
    assert "bringup_runner.is_busy()" in body and "bringup_runner.kill()" in body


def _idle_gate():
    import sys
    sys.path.insert(0, os.path.join(ROOT, "scripts"))
    from cmd_idle import IdleGate
    return IdleGate


def test_zero_is_said_for_the_idle_time_then_nothing_until_the_command_moves():
    """A publisher held at zero fought every other driver of /cmd_vel: Nav2's commands reached a robot
    interleaved with twice as many zeros, and it walked a sixth of what it was told for hours (2026-11-08)."""
    g = _idle_gate()(1.0)
    assert g.publish((0.2, 0, 0), 0.0)
    assert all(g.publish((0, 0, 0), t) for t in (0.05, 0.5, 1.0))   # the stop is said...
    assert not g.publish((0, 0, 0), 1.06) and g.quiet                # ...then nothing
    assert not g.publish((0, 0, 0), 30.0)
    assert g.publish((0, 0, 0.3), 31.0) and not g.quiet              # it moves: published again
    assert g.publish((0, 0, 0), 31.5) and not g.publish((0, 0, 0), 32.6)


def test_idle_zero_keeps_publishing_zero():
    g = _idle_gate()(0)
    assert all(g.publish((0, 0, 0), t) for t in (0.0, 5.0, 500.0))


def test_the_publisher_goes_quiet_at_zero():
    src = _read("scripts", "gamepad_publisher.py")
    assert "from cmd_idle import IdleGate" in src and "--idle-timeout" in src
    a = src.index("    def tick(self):")
    body = src[a:src.index("\n    def ", a + 10)]
    assert "self.idle.publish(" in body and body.index("self.idle.publish(") < body.index("self.pub.publish(")


def test_a_gamepad_on_this_computer_drives_only_while_its_deadman_is_held():
    js = _read("web", "frontend", "app-workflow.js")
    assert "navigator.getGamepads" in js
    a = js.index("function pollPad()")
    body = js[a:js.index("setInterval(pollPad", a)]
    assert "const enabled = held(PAD_LB);" in body
    assert "pressed(PAD_LB) && !vgpRunning" in body   # LB starts it: a BT pad's Start never reached Chrome
    assert "PAD_STOPS = [0, 1, 2, 3, 8]" in js        # stop by position: pads disagree on the letters
    for axis in ("lx = enabled ?", "ly = enabled ?", "az: enabled ?"):
        assert axis in body, axis
    t = js[js.index("  function twist()"):js.index("  function render()")]
    assert "hwPad && hwPad.enabled" in t and "linear_y: hwPad.ly" in t
    html = _read("web", "frontend", "index.html")
    assert 'id="vgp-hw"' in html and 'id="vgp-ly"' in html
