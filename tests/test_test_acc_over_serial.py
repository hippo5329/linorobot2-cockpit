"""test_acc on a serial robot (test_acc_run.py --serial, robot_bringup.py acc).

The bring-up card's acc step refused every serial robot: a Wi-Fi board sends test_acc's
results by syslog, and nothing read them anywhere else. The UNO Q's STM32 has no radio, but
while a tool runs it prints on its micro-ROS tty; the agent there is paused, the tty read,
and app=base -- which restarts the board -- is the stop.
"""
import os
import sys

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO_ROOT, "scripts"))

import test_acc_run as tar  # noqa: E402

BRINGUP = open(os.path.join(REPO_ROOT, "scripts", "robot_bringup.py")).read()

RUN = [b"[fw] linorobot2_hardware app=test_acc\r\nIDENT gains kp=0.6\r\nIDENT pl",
       b"ant wheel=1 steady_rpm=168.3\r\n", b"MAX VEL   0.21   0.01 m/s    0.04 rad/s\r\n",
       b"[test_acc] done -- write app=base to leave\r\n"]


class FakeLink:
    def __init__(self, chunks, boom=False):
        self.chunks, self.boom, self.s, self.reopened = list(chunks), boom, None, 0

    def open(self, timeout=15.0):
        self.s = object()
        return True

    def read(self):
        if self.boom and not self.chunks:
            raise KeyboardInterrupt
        return self.chunks.pop(0) if self.chunks else b""

    def _reopen(self):
        self.reopened += 1
        return True


def _source(chunks, boom=False):
    src = tar.SerialSource.__new__(tar.SerialSource)
    src.link, src.buf, src.where = FakeLink(chunks, boom), b"", "fake"
    src.heard = tar.time.time()
    src.link.open()
    return src


def _args(**kw):
    import argparse
    d = dict(params="c.yaml", env="unoq", port="/dev/ttyHS1", baud=4000000, controller="unoq",
             ip="", serial=True, timeout=5.0, transcript="")
    d.update(kw)
    return argparse.Namespace(**d)


def test_console_lines_are_split_across_reads():
    got = _source(RUN).lines(0.3)
    assert "IDENT plant wheel=1 steady_rpm=168.3" in got
    assert tar.ended(got).startswith("[test_acc] done")


def test_a_silent_tty_is_reopened():
    src = _source([])
    src.heard -= tar.SERIAL_QUIET_S + 1
    src.lines(0.1)
    assert src.link.reopened == 1


def _wire(monkeypatch, src, calls):
    monkeypatch.setattr(tar, "SerialSource", lambda port, baud, env: src)
    monkeypatch.setattr(tar, "pause_holders", lambda port: calls.append("pause") or [4242])
    monkeypatch.setattr(tar, "resume", lambda pids: calls.append(("resume", tuple(pids))))
    monkeypatch.setattr(tar, "write_app", lambda a, app: calls.append(app) or True)
    src.open = lambda: True


def test_the_serial_run_pauses_the_agent_reads_and_ends_on_base(monkeypatch, tmp_path):
    calls = []
    _wire(monkeypatch, _source(RUN), calls)
    rc = tar.main(["--params", "c.yaml", "--env", "unoq", "--port", "/dev/ttyHS1", "--baud", "4000000",
                   "--controller", "unoq", "--serial", "--timeout", "5",
                   "--transcript", str(tmp_path / "t.txt")])
    assert rc == 0
    assert calls == ["pause", "test_acc", "base", ("resume", (4242,))]
    assert "IDENT plant wheel=1" in (tmp_path / "t.txt").read_text()


def test_base_is_written_and_the_agent_let_go_whatever_happens(monkeypatch):
    calls = []
    _wire(monkeypatch, _source([], boom=True), calls)
    try:
        tar.run_serial(_args())
    except KeyboardInterrupt:
        pass
    assert calls == ["pause", "test_acc", "base", ("resume", (4242,))]


def test_no_end_line_fails_after_base(monkeypatch):
    calls = []
    _wire(monkeypatch, _source(RUN[:2]), calls)
    assert tar.run_serial(_args(timeout=0.5)) == 1
    assert calls[-2:] == ["base", ("resume", (4242,))]


def test_wifi_still_needs_an_ip():
    import pytest
    with pytest.raises(SystemExit):
        tar.main(["--params", "c", "--env", "esp32", "--port", "p", "--baud", "1", "--controller", "c"])


def test_the_acc_step_no_longer_refuses_a_serial_robot():
    assert "this robot is on serial" not in BRINGUP
    assert '(["--serial"] if on_serial else ["--ip", found["ip"]])' in BRINGUP
    assert "found = {} if on_serial else ocp.wireless_board(" in BRINGUP
