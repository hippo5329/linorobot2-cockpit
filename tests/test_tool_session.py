"""The cockpit's hold on the board's tool (web/backend/tool_session.py), with a fake link.

The 2026-10-13 web-designer review: the Hardware Tests tab could start a motor and not stop
it (its Stop closed a reader), it stranded robots in a test by writing the env's `app`, and a
browser must never hold the OTA password. The session these tests drive is what the tab now
talks to. No hardware, no FastAPI: a fake link and a fake clock.
"""
import os
import sys

import pytest

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO_ROOT, "web", "backend"))

import tool_session as ts  # noqa: E402


class FakeLink:
    def __init__(self):
        self.sent = []
        self.inbox = []
        self.closed = False

    def write(self, data):
        self.sent.append(data)

    def read(self):
        return self.inbox.pop(0) if self.inbox else b""

    def close(self):
        self.closed = True


class Clock:
    def __init__(self):
        self.t = 1000.0

    def __call__(self):
        return self.t


def make(kind="serial"):
    link, clock, grams = FakeLink(), Clock(), []
    s = ts.ToolSession("robot1", link, kind, ip="192.0.2.7" if kind == "wifi" else None,
                       clock=clock, send_datagram=lambda ip, port, d: grams.append((ip, port, d)))
    return s, link, clock, grams


# ---------------------------------------------------------------- the grammar
@pytest.mark.parametrize("line", [
    "status", "help", "i2c", "i2c 5", "sensors", "enc", "motor 1 30", "motor all -25 0",
    "motor 2 100 3600", "motor", "motors", "motors 40", "acc", "acc 500", "acc 500 50", "adc",
    "set tm_pct -40", "set ta_ms 400", "set tm_secs", "get", "get tm_pct", "base", "stop", "s",
])
def test_the_shells_own_lines_pass(line):
    assert ts.validate(line) == " ".join(line.split())


@pytest.mark.parametrize("line,why", [
    ("", "empty"), ("motor 1 0", "0 turns nothing"), ("motor 5 30", "outside"),
    ("motor 1 101", "outside"), ("motor 1 30 -1", "outside"), ("motors 0", "outside"),
    ("acc 200", "outside"), ("acc 500 5", "outside"), ("set pwm_bits 16", "not a test setting"),
    ("set sim_wheel 0", "not a test setting"), ("set ota_password x", "not a test setting"),
    ("set tm_pct 500", "outside"), ("reboot now", "not a tool command"), ("rm -rf /", "not a tool command"),
    ("status\r\nmotor 1 100", "plain text"), ("motor 1 30\x00", "plain text"), ("i2c x", "whole number"),
    ("x" * 65, "plain text"), ("get ../../etc", "get [key]"),
])
def test_anything_else_is_refused_before_a_byte_is_sent(line, why):
    s, link, _, _ = make()
    with pytest.raises(ts.Refused) as e:
        s.send(line)
    assert why in str(e.value)
    assert link.sent == []


# ---------------------------------------------------------------- stop
def test_stop_on_serial_is_the_shells_stop():
    s, link, _, grams = make("serial")
    s.stop()
    assert link.sent == [b"stop\r\n"] and grams == []


def test_stop_on_wifi_also_sends_lino_stop():
    """lino-stop needs no login and reaches the board when the terminal does not."""
    s, link, _, grams = make("wifi")
    s.send("stop")
    assert link.sent == [b"stop\r\n"]
    assert grams == [("192.0.2.7", 3233, b"lino-stop")]


# ---------------------------------------------------------------- liveness
def test_the_keep_alive_goes_only_while_a_page_watches():
    s, link, clock, _ = make()
    s.tick()
    assert b"\0" not in link.sent, "nobody watches: no keep-alive"
    wid = s.watch()
    s.tick()
    assert link.sent.count(b"\0") == 1
    clock.t += 1.1
    s.beat(wid)
    s.tick()
    assert link.sent.count(b"\0") == 2
    clock.t += ts.HEARTBEAT_S + 0.5          # the page stopped beating (a closed lid)
    s.tick()
    assert link.sent.count(b"\0") == 2, "a silent page must not keep a drive alive"


def test_the_last_page_going_away_stops_a_running_command():
    s, link, clock, grams = make("wifi")
    wid = s.watch()
    s.send("motor 1 30 0")
    assert s.running == "motor"
    clock.t += ts.HEARTBEAT_S + 0.5
    s.tick()
    assert b"stop\r\n" in link.sent and grams == [("192.0.2.7", 3233, b"lino-stop")]
    s.tick()
    assert link.sent.count(b"stop\r\n") == 1, "once, not every tick"
    s.unwatch(wid)


def test_no_stop_when_nothing_runs():
    s, link, clock, _ = make()
    s.watch()
    clock.t += 10
    s.tick()
    assert b"stop\r\n" not in link.sent


def test_the_prompt_or_a_stop_line_ends_the_running_command():
    s, _, _, _ = make()
    s.send("motors 25")
    assert s.running == "motors"
    s.feed(b"MOTOR1 FWD RPM 1 2 3 4\n[tool] stopped: you typed stop -- motors off\n")
    assert s.running is None
    s.send("sensors")
    s.feed(b"ACC none (no IMU)\ntool> ")
    assert s.running is None


# ---------------------------------------------------------------- fan-out
def test_every_page_sees_the_backlog_then_the_live_lines():
    s, _, _, _ = make()
    s.feed(b"[app] tool\n[tool] ready -- 'help' lists the commands\ntool> ")
    a = s.watch()
    assert s.drain(a) == ["[app] tool", "[tool] ready -- 'help' lists the commands", "tool> "]
    b = s.watch()
    s.feed(b"ENC  m1 0 (+0)\n")
    assert s.drain(a) == ["ENC  m1 0 (+0)"]
    assert s.drain(b)[-1] == "ENC  m1 0 (+0)"


# ---------------------------------------------------------------- leaving
def test_leaving_is_stop_then_base_and_names_an_env_that_boots_a_tool():
    s, link, _, _ = make()
    s.feed(b"[fw] linorobot2_hardware app=test_motors distro=jazzy\n")
    assert s.needs_env_base(), "an older cockpit's app=test_motors: every power-on runs it"
    s.leave()
    assert link.sent == [b"stop\r\n", b"base\r\n"]
    s2, _, _, _ = make()
    s2.feed(b"[fw] linorobot2_hardware app=base distro=jazzy\n")
    assert not s2.needs_env_base()


def test_one_session_per_robot():
    reg = ts.Registry()
    s1, _, _, _ = make()
    reg.add(s1, start_threads=False)
    s2, _, _, _ = make()
    with pytest.raises(RuntimeError):
        reg.add(s2, start_threads=False)
    reg.remove("robot1")
    assert s1.closed
    reg.add(s2, start_threads=False)
    assert reg.get("robot1") is s2


def test_a_terminal_the_board_hung_up_closes_the_session():
    s, link, _, _ = make("wifi")
    link.closed = True
    s.pump()
    assert s.closed


def test_the_description_never_carries_a_password():
    s, _, _, _ = make("wifi")
    s.feed(b"nonce 1a2b3c4d\nlogin: ok\ntool> ")
    d = repr(s.describe())
    assert "password" not in d.lower()
