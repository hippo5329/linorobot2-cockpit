"""test_acc on a robot (test_acc_run.py, robot_bringup.py acc, the 1-Click's --test-acc).

The board boots base only (user, 2026-10-16): test_acc is the tool's `acc`, and the tool is
entered by its boot escape, through scripts/lino_tool.py, as the web UI's Hardware Tests tab
enters it -- never by an env write. On a serial robot (the UNO Q, an ESP32 or an RP2 on USB)
the agent on the tty is paused, the board restarted and `tool` typed into its window, `acc`
sent, its lines read, and `stop` then `base` end it whatever happened. On Wi-Fi, "lino-tool"
and the TCP terminal, with "lino-stop" in the cleanup too.
"""
import os
import sys

import pytest

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO_ROOT, "scripts"))

import lino_tool  # noqa: E402
import test_acc_run as tar  # noqa: E402

BRINGUP = open(os.path.join(REPO_ROOT, "scripts", "robot_bringup.py")).read()
SOURCE = open(os.path.join(REPO_ROOT, "scripts", "test_acc_run.py")).read()

RUN = [b"ok: acc (running)\r\nIDENT gains kp=0.6\r\nIDENT pl",
       b"ant wheel=1 steady_rpm=168.3\r\n", b"MAX VEL   0.21   0.01 m/s    0.04 rad/s\r\n",
       b"[test_acc] done -- 'base' to leave the tool\r\ndone: acc\r\ntool> "]


class FakeLink:
    """A board in the tool, as lino_tool's SerialLink / TcpLink show it."""

    def __init__(self, chunks=(), calls=None, boom=False, answers_base=True, reboots=True):
        self.chunks, self.boom = list(chunks), boom
        self.calls = calls if calls is not None else []
        self.answers_base, self.reboots = answers_base, reboots
        self.s, self.reopened, self.closed = object(), 0, False
        self.out = []

    def write(self, data):
        self.calls.append(("write", data))
        if data == b"acc\r\n":
            self.out += self.chunks
        elif data == b"base\r\n" and self.answers_base:
            self.out.append(b"ok: base (restarting into base)\r\n[tool] restarting into base\r\n")
        elif data == tar.CONSOLE_STOP:
            self.out.append(b"ok: stop (idle, motors off)\r\ntool> ")

    def read(self):
        if self.boom and ("write", b"acc\r\n") in self.calls and not self.out:
            raise KeyboardInterrupt
        return self.out.pop(0) if self.out else b""

    def keepalive(self):
        self.kept = getattr(self, "kept", None) or len(self.calls) + 1   # the first one's place


    def reboot(self, release=True):
        self.calls.append(("reboot", release))
        return self.reboots

    def _reopen(self):
        self.reopened += 1
        return True

    def close(self):
        pass


def _args(**kw):
    import argparse
    d = dict(params="c.yaml", env="unoq", port="/dev/ttyHS1", baud=4000000, controller="unoq",
             ip="", serial=True, console_baud=0, timeout=5.0, transcript="")
    d.update(kw)
    return argparse.Namespace(**d)


def _wire(monkeypatch, link, calls, holders=(4242,), paused=(4242,), agents=(), enters=True):
    import flash_mcu
    seen = {}

    def serial_link(port, baud, env):
        seen.update(port=port, baud=baud, env=env)
        return link
    monkeypatch.setattr(lino_tool, "SerialLink", serial_link)
    monkeypatch.setattr(lino_tool, "catch_window",
                        lambda lk, seconds: calls.append("catch") or enters)
    monkeypatch.setattr(tar, "pause_holders", lambda port: calls.append("pause") or list(paused))
    monkeypatch.setattr(flash_mcu, "_holders_of", lambda port: list(holders))
    monkeypatch.setattr(tar, "agents_on", lambda port: list(agents))
    monkeypatch.setattr(tar, "resume", lambda pids: calls.append(("resume", tuple(pids))))
    return seen


def _flow(calls):
    """The calls, with the writes named: acc, stop, base."""
    names = {b"acc\r\n": "acc", tar.CONSOLE_STOP: "stop", b"base\r\n": "base"}
    out = []
    for c in calls:
        if isinstance(c, tuple) and c[0] == "write":
            out.append(names.get(c[1], c[1]))
        else:
            out.append(c)
    return out


def test_console_lines_are_split_across_reads():
    link = FakeLink()
    link.out = list(RUN)
    got = tar.ConsoleSource(link, "fake").lines(0.3)
    assert "IDENT plant wheel=1 steady_rpm=168.3" in got
    assert tar.ended(got).startswith("[test_acc] done")


def test_a_silent_tty_is_reopened():
    src = tar.ConsoleSource(FakeLink(), "fake")
    src.heard -= tar.SERIAL_QUIET_S + 1
    src.lines(0.1)
    assert src.link.reopened == 1


def test_the_serial_run_enters_by_the_escape_runs_acc_and_ends_on_base(monkeypatch, tmp_path):
    calls = []
    _wire(monkeypatch, FakeLink(RUN, calls), calls)
    rc = tar.main(["--params", "c.yaml", "--env", "unoq", "--port", "/dev/ttyHS1", "--baud", "4000000",
                   "--controller", "unoq", "--serial", "--timeout", "5",
                   "--transcript", str(tmp_path / "t.txt")])
    assert rc == 0
    # the agent paused first, and the reboot leaves it alone (a SIGINT would kill it on SIGCONT)
    assert _flow(calls) == ["pause", ("reboot", False), "catch", "acc", "stop", "base",
                            ("resume", (4242,))]
    assert "IDENT plant wheel=1" in (tmp_path / "t.txt").read_text()


def test_a_keep_alive_goes_ahead_of_acc_so_the_tools_deadman_is_armed(monkeypatch):
    calls = []
    link = FakeLink(RUN, calls)
    _wire(monkeypatch, link, calls)
    assert tar.run_serial(_args()) == 0
    # the first keep-alive went out with N calls made, and `acc` is a later call
    assert calls.index(("write", b"acc\r\n")) >= link.kept - 1


def test_it_never_writes_the_env_and_enters_through_lino_tool():
    # Only base boots (user, 2026-10-16): no app= other than base, and no env write at all.
    assert "--env-only" not in SOURCE and "subprocess" not in SOURCE
    assert '"--app"' not in SOURCE and "write_app" not in SOURCE
    import re
    assert not re.search(r"app=(?!base)\w", SOURCE)
    assert "lino_tool.SerialLink(" in SOURCE and "lino_tool.catch_window(" in SOURCE
    assert "lino_tool.request_tool_udp(" in SOURCE and "lino_tool.TcpLink(" in SOURCE


def test_stop_and_base_are_sent_and_the_agent_let_go_whatever_happens(monkeypatch):
    calls = []
    _wire(monkeypatch, FakeLink([], calls, boom=True), calls)
    with pytest.raises(KeyboardInterrupt):
        tar.run_serial(_args())
    assert _flow(calls)[-3:] == ["stop", "base", ("resume", (4242,))]


def test_no_end_line_fails_after_base(monkeypatch):
    calls = []
    _wire(monkeypatch, FakeLink(RUN[:2], calls), calls)
    assert tar.run_serial(_args(timeout=0.5)) == 1
    assert _flow(calls)[-2:] == ["base", ("resume", (4242,))]


def test_a_refused_acc_ends_the_run_and_still_leaves(monkeypatch, capsys):
    calls = []
    refused = [b"fail: acc -- busy: sensors is running -- stop first\r\n"]
    _wire(monkeypatch, FakeLink(refused, calls), calls)
    assert tar.run_serial(_args()) == 1
    assert "TEST_ACC_FAILED: fail: acc -- busy" in capsys.readouterr().out
    assert _flow(calls)[-3:] == ["stop", "base", ("resume", (4242,))]


def test_a_board_that_does_not_enter_the_tool_moves_nothing(monkeypatch, capsys):
    calls = []
    _wire(monkeypatch, FakeLink(RUN, calls), calls, enters=False)
    assert tar.run_serial(_args()) == 1
    flow = _flow(calls)
    assert "acc" not in flow and "base" not in flow       # it booted base: nothing to leave
    assert flow.count("catch") == 2                       # once more on a fresh open
    assert flow[-1] == ("resume", (4242,))
    out = capsys.readouterr().out
    assert "did not enter the tool by its boot escape" in out and "-- nothing moved" in out


def test_a_tool_that_does_not_answer_base_is_reset_into_base(monkeypatch):
    calls = []
    monkeypatch.setattr(tar, "LEAVE_S", 0.2)
    _wire(monkeypatch, FakeLink(RUN, calls, answers_base=False), calls)
    assert tar.run_serial(_args()) == 0
    flow = _flow(calls)
    assert flow[flow.index("base") + 1] == ("reboot", False)


def test_a_board_left_in_the_tool_fails_the_run(monkeypatch, capsys):
    calls = []
    monkeypatch.setattr(tar, "LEAVE_S", 0.2)
    link = FakeLink(RUN, calls, answers_base=False)
    # the entry's reboot works, the cleanup's reset does not
    link.reboot = lambda release=True: calls.append(("reboot", release)) or len(
        [c for c in calls if isinstance(c, tuple) and c[0] == "reboot"]) == 1
    _wire(monkeypatch, link, calls)
    assert tar.run_serial(_args()) == 1
    assert "the board did not restart into base" in capsys.readouterr().out


def test_wifi_still_needs_an_ip():
    import pytest
    with pytest.raises(SystemExit):
        tar.main(["--params", "c", "--env", "esp32", "--port", "p", "--baud", "1", "--controller", "c"])


def test_the_wifi_run_opens_the_tool_terminal_and_stops_with_lino_stop(monkeypatch, capsys):
    calls = []
    link = FakeLink(RUN, calls)
    monkeypatch.setattr(tar, "_ota", lambda a: ("pw", {"ping_port": "3233", "term_port": "2323"}))
    monkeypatch.setattr(lino_tool, "request_tool_udp",
                        lambda ip, port, password: calls.append(("lino-tool", ip, port)) or True)

    class Tcp:
        def __new__(cls, ip, port, password):
            calls.append(("terminal", ip, port))
            link.open = lambda timeout=30.0: True
            link.port = port
            return link
    monkeypatch.setattr(lino_tool, "TcpLink", Tcp)
    monkeypatch.setattr(tar, "stop_board", lambda ip: calls.append(("lino-stop", ip)) or True)
    assert tar.run_wifi(_args(serial=False, ip="10.9.9.9", env="esp32")) == 0
    assert _flow(calls) == [("lino-tool", "10.9.9.9", 3233), ("terminal", "10.9.9.9", 2323), "acc",
                            ("lino-stop", "10.9.9.9"), "stop", "base"]


def test_a_wifi_board_without_an_ota_password_is_not_touched(monkeypatch, capsys):
    monkeypatch.setattr(tar, "_ota", lambda a: ("", {}))
    monkeypatch.setattr(lino_tool, "request_tool_udp", lambda *a: pytest.fail("touched the board"))
    assert tar.run_wifi(_args(serial=False, ip="10.9.9.9", env="esp32")) == 1
    assert "nothing moved" in capsys.readouterr().out


def test_the_acc_step_no_longer_refuses_a_serial_robot():
    assert "this robot is on serial" not in BRINGUP
    assert '(["--serial", "--console-baud", str(baud)] if on_serial' in BRINGUP
    assert "found = {} if on_serial else ocp.wireless_board(" in BRINGUP


def test_the_console_is_read_at_the_micro_ros_rate(monkeypatch):
    calls = []
    seen = _wire(monkeypatch, FakeLink(RUN, calls), calls, holders=(), paused=())
    tar.run_serial(_args(env="unoq", baud=921600, console_baud=4000000))
    assert seen["baud"] == 4000000


def test_the_acc_step_writes_at_the_upload_rate_and_reads_at_the_runtime_one():
    assert '"--baud", str(min(baud, ocp.FLASH_BAUD_CEILING))' in BRINGUP
    assert '["--serial", "--console-baud", str(baud)]' in BRINGUP


OCP = open(os.path.join(REPO_ROOT, "scripts", "one_click_pipeline.py")).read()


def test_the_one_click_measures_a_serial_robot_too():
    # it skipped every serial robot with "--test-acc needs ... a real Wi-Fi robot"
    assert "do_test_acc = args.test_acc and not args.map and (sim_mcu or is_real)" in OCP
    assert "a real Wi-Fi robot" not in OCP
    assert 'on_serial = not sim and not mcu_env.transport_is_wifi(controller_cfg.get("transport"))' in OCP
    assert '["--serial", "--console-baud", str(int(controller_cfg.get("baudrate", 921600)))]' in OCP


def test_the_web_ui_measures_by_default():
    html = open(os.path.join(REPO_ROOT, "web", "frontend", "index.html")).read()
    assert '<input type="checkbox" id="cockpit-test-acc" checked>' in html


def test_micro_ros_is_proved_back_by_data_after_each_env_write():
    # a 5 s `ros2 topic info` timed out on every try on a slow robot computer under SLAM and
    # Nav2, and a board that was back read "did not come back" for 120 s
    import re as _re
    for msg in ("after base", "after the tuned env"):
        i = OCP.index(f'"test_acc: micro-ROS did not come back {msg}"')
        assert 'require_message="header.stamp"' in OCP[i - 300:i], msg
    assert "timeout 15 ros2 topic info --no-daemon {topic_name}" in OCP
    assert 'wait_for_topic("/odom/unfiltered", timeout_sec=120, distro=distro, require_message="header.stamp")' in BRINGUP


# ---- the cleanup always runs, the tool stops at once, and a run never starts unpaused ----

def test_an_agent_that_could_not_be_paused_refuses_before_the_board_is_touched(monkeypatch, capsys):
    calls = []
    _wire(monkeypatch, FakeLink(RUN, calls), calls, holders=(4242,), paused=())
    assert tar.run_serial(_args()) == 1
    assert not any(isinstance(c, tuple) and c[0] in ("reboot", "write") for c in calls)
    out = capsys.readouterr().out
    assert "TEST_ACC_REFUSED: pid 4242 holds /dev/ttyHS1 and could not be paused" in out


def test_an_agent_this_user_cannot_see_refuses_too(monkeypatch, capsys):
    calls = []
    _wire(monkeypatch, FakeLink(RUN, calls), calls, holders=(), paused=(), agents=(777,))
    assert tar.run_serial(_args()) == 1
    assert "acc" not in _flow(calls)
    assert "micro-ROS agent (pid 777) runs on /dev/ttyHS1" in capsys.readouterr().out


def test_no_agent_at_all_runs(monkeypatch):
    calls = []
    _wire(monkeypatch, FakeLink(RUN, calls), calls, holders=(), paused=(), agents=())
    assert tar.run_serial(_args()) == 0 and "acc" in _flow(calls)


def test_an_agent_is_counted_only_when_it_names_this_tty(monkeypatch, tmp_path):
    proc = tmp_path / "proc"
    for pid, argv in ((11, ["micro_ros_agent", "serial", "--dev", "/dev/ttyHS1"]),
                      (12, ["micro_ros_agent", "udp4", "--port", "8888"]),
                      (13, ["bash", "-c", "cat /dev/ttyHS1"])):
        (proc / str(pid)).mkdir(parents=True)
        (proc / str(pid) / "cmdline").write_bytes("\0".join(argv).encode())
    real_listdir, real_open = os.listdir, open

    def listdir(p):
        return real_listdir(proc) if p == "/proc" else real_listdir(p)

    def fake_open(p, *a, **k):
        if isinstance(p, str) and p.startswith("/proc/") and p.endswith("/cmdline"):
            p = str(proc / p.split("/")[2] / "cmdline")
        return real_open(p, *a, **k)
    monkeypatch.setattr(tar.os, "listdir", listdir)
    monkeypatch.setattr("builtins.open", fake_open)
    monkeypatch.setattr(tar, "_ns_depth", lambda pid: 1)
    assert tar.agents_on("/dev/ttyHS1") == [11]


def _with_handlers(fn):
    import signal
    old = {s: signal.getsignal(s) for s in (signal.SIGTERM, signal.SIGHUP)}
    try:
        return fn()
    finally:
        for s, h in old.items():
            signal.signal(s, h)


def test_sigterm_mid_run_still_stops_sends_base_and_lets_the_agent_go(monkeypatch, capsys):
    import signal
    calls = []

    class Killed(FakeLink):
        def read(self):
            if ("write", b"acc\r\n") in self.calls and not self.out:
                os.kill(os.getpid(), signal.SIGTERM)    # the 1-Click's timeout, the UI's cancel
            return super().read()
    _wire(monkeypatch, Killed([b"ok: acc (running)\r\n"], calls), calls)
    argv = ["--params", "c.yaml", "--env", "unoq", "--port", "/dev/ttyHS1", "--baud", "4000000",
            "--controller", "unoq", "--serial", "--timeout", "30"]
    assert _with_handlers(lambda: tar.main(argv)) == 1
    flow = _flow(calls)
    assert flow[:4] == ["pause", ("reboot", False), "catch", "acc"]
    assert flow[-3:] == ["stop", "base", ("resume", (4242,))]
    assert "TEST_ACC_FAILED: interrupted" in capsys.readouterr().out


def test_a_second_signal_waits_for_the_cleanup(monkeypatch):
    import signal
    calls = []

    class Again(FakeLink):
        def write(self, data):
            super().write(data)
            if data == b"base\r\n":
                os.kill(os.getpid(), signal.SIGTERM)    # again, while base is being sent
                self.calls.append("base done")
    _wire(monkeypatch, Again([], calls, boom=True), calls)

    def run():
        tar.install_signal_handlers()
        try:
            tar.run_serial(_args())
        except KeyboardInterrupt:
            pass
    _with_handlers(run)
    assert _flow(calls)[-3:] == ["base", "base done", ("resume", (4242,))]


def test_resume_lets_each_pid_go_once(monkeypatch):
    sent = []
    monkeypatch.setattr(tar.os, "kill", lambda pid, sig: sent.append(pid))
    pids = [5, 6]
    tar.resume(pids)
    tar.resume(pids)
    assert sent == [5, 6] and pids == []


def test_the_ros2_run_launcher_naming_the_tty_is_not_a_hidden_agent(monkeypatch):
    # on a robot: pid 472 = `ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyHS1`
    # (holds nothing), pid 586 = the agent itself, paused. The launcher refused every run.
    import flash_mcu
    monkeypatch.setattr(flash_mcu, "_holders_of", lambda port: [586])
    monkeypatch.setattr(tar, "agents_on", lambda port: [472, 586])
    monkeypatch.setattr(tar, "_fds_readable", lambda pid: True)
    assert tar.pause_refusal("/dev/ttyHS1", [586]) == ""
    # an agent whose fds cannot be read is still refused: it may hold the tty unseen
    monkeypatch.setattr(tar, "_fds_readable", lambda pid: pid != 472)
    assert "cannot see its open files" in tar.pause_refusal("/dev/ttyHS1", [586])

