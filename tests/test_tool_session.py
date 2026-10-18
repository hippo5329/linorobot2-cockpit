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
sys.path.insert(0, os.path.join(REPO_ROOT, "scripts"))


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
    "set m1_inv 1", "set m4_enc_inv 0", "set m2_inv -",
])
def test_the_shells_own_lines_pass(line):
    assert ts.validate(line) == " ".join(line.split())


@pytest.mark.parametrize("line,why", [
    ("", "empty"), ("motor 1 0", "0 turns nothing"), ("motor 5 30", "outside"),
    ("motor 1 101", "outside"), ("motor 1 30 -1", "outside"), ("motors 0", "outside"),
    ("acc 200", "outside"), ("acc 500 5", "outside"), ("set pwm_bits 16", "not a test setting"),
    ("set sim_wheel 0", "not a test setting"), ("set ota_password x", "not a test setting"),
    ("set tm_pct 500", "outside"), ("set m1_inv", "an invert"), ("set m1_inv 2", "an invert"),
    ("set m5_inv 1", "not a test setting"), ("set m1_invv 1", "not a test setting"), ("reboot now", "not a tool command"), ("rm -rf /", "not a tool command"),
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
    s.feed(b"MOTOR1 FWD RPM 1 2 3 4\nstopped: motors 25 (you typed stop -- motors off)\n")
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


def test_ramp_is_in_the_grammar_and_counts_as_a_drive():
    from tool_session import validate, Refused, RUNNING, DRIVES
    assert validate("ramp 2") == "ramp 2"
    assert validate("ramp 1 60") == "ramp 1 60"
    for bad in ("ramp", "ramp 5", "ramp 1 3", "ramp 1 60 9"):
        try:
            validate(bad)
        except Refused:
            continue
        raise AssertionError(bad)
    assert "ramp" in RUNNING and "ramp" in DRIVES


def test_the_keep_alive_beats_the_acc_deadman():
    """The board stops an acc drive whose terminal is quiet for 2 s: send more often than that."""
    import tool_session
    assert tool_session.KEEPALIVE_S <= 0.5


# ---------------------------------------------------------------- command and response
# user, 2026-10-14: "when we send command to tool, tool should response" -- the board answers
# every line in words, "ok: <line> (msg)" or "fail: <line> -- why", and the cockpit waits for it;
# user, same day: "more human like, eg, done ok fail".
def _ack(line, ok=True, msg="running"):
    return ((f"ok: {line} ({msg})" if ok else f"fail: {line} -- {msg}") + "\n").encode()


def test_the_boards_answer_reaches_the_sender():
    s, link, clock, _ = make()
    s.send("motor 1 30 3")
    seq = s.last_seq
    s.feed(_ack("motor 1 30 3"))
    a = s.wait_answer(seq, timeout=0.1)
    assert a["ok"] is True and a["msg"] == "running" and a["line"] == "motor 1 30 3"
    assert s.running == "motor" and s.describe()["answers"] is True


def test_a_refusal_is_the_senders_and_nothing_runs():
    s, link, clock, _ = make()
    s.send("sensors 10")
    s.send("motor 2 40")
    seq = s.last_seq
    s.feed(_ack("sensors 10") + _ack("motor 2 40", ok=False, msg="busy: sensors is running -- stop first"))
    a = s.wait_answer(seq, timeout=0.1)
    assert a["ok"] is False and "busy" in a["msg"]


def test_silence_is_said_in_the_terminal():
    s, link, clock, _ = make()
    wid = s.watch()
    s.send("status")
    seq = s.last_seq
    clock.t += 2.5
    s.tick()
    a = s.wait_answer(seq, timeout=0.1)
    assert a["ok"] is None
    said = [ln for ln in s.drain(wid) if ln.startswith("no answer: status")]
    assert said and "predate command answers" in said[0]      # no answer ever seen: old firmware?


def test_an_answer_nobody_waits_for_is_ignored():
    s, link, clock, _ = make()
    s.feed(_ack("stop", msg="idle, motors off"))                # an internal stop
    assert s.describe()["last_answer"] is None



def test_an_internal_stop_answer_is_not_a_waiting_commands():
    """The session's own stop (the last page went away) is answered `ok: stop`; that answer
    was handed to the oldest line waiting -- a user's motor command -- as its own."""
    s, link, clock, _ = make()
    sent, seq = s.send("motor 1 30 3")
    assert sent == "motor 1 30 3"
    s.stop()                                                     # internal: nothing waits for it
    s.feed(_ack("stop", msg="idle, motors off"))
    assert s.wait_answer(seq, timeout=0.05) is None, "the motor's answer has not come yet"
    s.feed(_ack("motor 1 30 3"))
    assert s.wait_answer(seq, timeout=0.05)["line"] == "motor 1 30 3"


def test_send_returns_its_own_seq():
    """The route waits on the seq send() returns, not last_seq read afterwards (another
    request's send may have moved it)."""
    s, link, clock, _ = make()
    _, a = s.send("status")
    _, b = s.send("sensors 10")
    assert b == a + 1 and s.last_seq == b
    _, c = s.send("s")                                           # goes out as `stop`, answered `stop`
    s.feed(_ack("stop", msg="motors off"))
    assert s.wait_answer(c, timeout=0.05)["ok"] is True

def test_lino_tool_reads_the_answer_to_its_own_line():
    import lino_tool
    import tool_answer
    assert tool_answer.parse("ok: i2c (done)") == {"kind": "ok", "line": "i2c", "ok": True, "msg": "done"}
    assert tool_answer.parse("ok: status") == {"kind": "ok", "line": "status", "ok": True, "msg": ""}
    assert tool_answer.parse("fail: motor 3 40 -- this robot has 2 wheels: motor 1 or 2, or all") == {
        "kind": "fail", "line": "motor 3 40", "ok": False, "msg": "this robot has 2 wheels: motor 1 or 2, or all"}
    assert tool_answer.parse("done: motor 1 40 3")["kind"] == "done"
    assert tool_answer.parse("stopped: motors 25 (lino-stop -- motors off)")["msg"] == "lino-stop -- motors off"
    assert tool_answer.parse("MOTOR 1 FWD  RPM 45.2") is None
    assert tool_answer.words(tool_answer.parse("fail: i2c -- no pins")) == "fail: i2c -- no pins"

    class L:
        def __init__(self):
            self.out, self.inbox, self.closed = [], [_ack("status", msg="done"), _ack("i2c", ok=False, msg="no pins")], False

        def write(self, d):
            self.out.append(d)

        def read(self):
            return self.inbox.pop(0) if self.inbox else b""

        def keepalive(self):
            pass
    got, _ = lino_tool.answer(L(), "i2c", timeout=0.5)
    assert got["ok"] is False and got["msg"] == "no pins"         # the earlier line's answer skipped


def test_the_firmware_answers_every_path():
    src = open(os.path.join(REPO_ROOT, "firmware", "src", "tools", "tools.cpp")).read()
    for form in ('"fail: %s -- %s\\n"', '"ok: %s (%s)\\n"', '"ok: %s\\n"', '"done: %s\\n"', '"\\nstopped: %s (%s%s)\\n"'):
        assert form in src, form
    body = src[src.index("static void handleLine(char *line, bool busy)\n{"):src.index("void toolSetup(void)")]
    # every refusal the shell prints is answered first
    for marker in ("unknown command", "busy: %s is running -- 'stop' first", "is read at boot: set %s <value>", "tests %s, and this robot",
                   "'adc' would drive"):
        at = body.index(marker)
        assert "ack(false" in body[max(0, at - 260):at], marker
    assert "ack(true, \"running\");" in src           # startJob: a running command answers at its start
    assert src.count("answerLine(line,") == 2          # console, terminal (no boot command: base only)


def test_a_command_that_ends_without_its_work_says_fail():
    """test_acc on simulated wheels, or halted on a wheel turning against its encoder, ends
    `fail: <command> -- why`, not `done` (the 2026-10-14 Sim MCU walk: `done: acc 500 30` after
    a refusal read as success)."""
    s, link, clock, _ = make()
    s.send("acc 500 30")
    s.feed(_ack("acc 500 30"))
    assert s.running == "acc"
    s.feed(b"fail: acc 500 30 -- the wheels are simulated (sim_wheel=1): nothing to measure\n")
    assert s.running is None
    acc = open(os.path.join(REPO_ROOT, "firmware", "src", "tools", "test_acc.cpp")).read()
    assert 'toolJobFail("the wheels are simulated (sim_wheel=1): nothing to measure");' in acc
    halt = acc[acc.index("static void halt(const char *why)"):acc.index("static float imu_heading")]
    assert "if (why == toolStopReason()) toolJobStopped(why);" in halt and "else toolJobFail(why);" in halt
    tools = open(os.path.join(REPO_ROOT, "firmware", "src", "tools", "tools.cpp")).read()
    assert tools.count("if (job_ended) return;") == 2         # one end answer per command
