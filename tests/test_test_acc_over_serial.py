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
             ip="", serial=True, console_baud=0, timeout=5.0, transcript="")
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
    assert '(["--serial", "--console-baud", str(baud)] if on_serial' in BRINGUP
    assert "found = {} if on_serial else ocp.wireless_board(" in BRINGUP


def test_an_esp32_or_rp2_reader_lets_go_of_the_tty_for_each_env_write(monkeypatch):
    # esptool and an RP2's 1200-baud touch write through the console's own tty; the UNO Q's
    # env goes over SWD, so only there does the reader keep it open
    calls, src = [], _source(RUN)
    _wire(monkeypatch, src, calls)
    src.open = lambda timeout=15.0: calls.append("open") or True
    src.close = lambda: calls.append("close")
    assert tar.run_serial(_args(env="esp32", port="/dev/ttyUSB0", baud=921600, console_baud=1500000)) == 0
    i = calls.index("test_acc")
    assert calls[:i] == ["pause", "open", "close"]
    assert calls[i + 1] == "open"
    j = calls.index("base")
    assert calls[j - 1] == "close" and calls[-1] == ("resume", (4242,))
    assert not tar.env_over_tty("unoq") and tar.env_over_tty("esp32") and tar.env_over_tty("pico2w")


def test_the_console_is_read_at_the_micro_ros_rate(monkeypatch):
    seen, src = {}, _source(RUN)
    monkeypatch.setattr(tar, "SerialSource", lambda port, baud, env: seen.update(baud=baud) or src)
    monkeypatch.setattr(tar, "pause_holders", lambda port: [])
    monkeypatch.setattr(tar, "write_app", lambda a, app: True)
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
    for msg in ("after app=base", "after the tuned env"):
        i = OCP.index(f'"test_acc: micro-ROS did not come back {msg}"')
        assert 'require_message="header.stamp"' in OCP[i - 300:i], msg
    assert "timeout 15 ros2 topic info --no-daemon {topic_name}" in OCP
    assert 'wait_for_topic("/odom/unfiltered", timeout_sec=120, distro=distro, require_message="header.stamp")' in BRINGUP
