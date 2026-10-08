#!/usr/bin/env python3
"""test_acc on a Wi-Fi robot, from start to "app=base" again -- the 1-Click's --test-acc step.

    test_acc_run.py --params CFG --env PIO_ENV --port PORT --baud B --controller NAME --ip BOARD_IP
                    [--timeout 300]

test_acc is the firmware's drivetrain tool: the PID identification (dead zone, open-loop
step, closed-loop steps, one wheel at a time) and then twelve 1 s full-power runs,
straight and spinning. THE ROBOT MOVES, at full power, so the caller checks the room
first (clearance.py, 1.0 m all round).

  1  the board's syslog must be reaching us: its console is teed to syslog while a tool
     runs, and that is the only place the results go. When the cockpit backend holds
     the syslog port its day log (logs/syslog_YYYYMMDD.log) carries the same lines, and
     is followed instead. No line from the board in 30 s: refused, nothing moves.
  2  the env block with app=test_acc, over the air (flash_mcu.py --env-only)
  3  its lines until "[test_acc] done", "STOPPED" or "test_acc refused", --timeout at most
  4  ALWAYS, whatever happened: "lino-stop" to the ping port until the board acknowledges
     (the firmware latches it and halts the motors), then the env with app=base
Prints the results (IDENT gains / plant / dead zone, MAX VEL / ACC) and TEST_ACC_DONE or
TEST_ACC_FAILED. Exit 0 when test_acc finished.
"""
import argparse
import os
import re
import socket
import subprocess
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import wifi_boards  # noqa: E402

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
END_RE = re.compile(r"\[test_acc\] done|test_acc refused|\[test_acc\] STOPPED")
RESULT_RE = re.compile(r"IDENT (gains|robot|plant|deadzone|done)|MAX VEL|MIN VEL|MAX ACC|MIN ACC|"
                       r"IMU ACC|time to|distance to stop|HOME |refused|STOPPED")
SYSLOG_WAIT_S = 30
STOP_TRIES = 20


class SyslogSource:
    """One board's syslog lines: from the syslog port when it is free, else from the
    backend's day log (which the backend writes as it receives)."""

    def __init__(self, ip: str, port: int = None, log_dir: str = None):
        self.ip = ip
        self.tag = f"] [{ip}:"
        self.log_dir = log_dir or os.path.join(REPO_ROOT, "logs")
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self.sock.bind(("0.0.0.0", port or wifi_boards.syslog_port()))
        except OSError:
            self.sock.close()
            self.sock = None
        self.fh, self.path = None, None
        if self.sock is None:
            self._open_day_log(at_end=True)

    @property
    def where(self) -> str:
        return "the syslog port" if self.sock else f"the cockpit's log {self.path}"

    def _open_day_log(self, at_end: bool):
        self.path = os.path.join(self.log_dir, time.strftime("syslog_%Y%m%d.log"))
        if os.path.exists(self.path):
            self.fh = open(self.path, encoding="utf-8", errors="replace")
            if at_end:
                self.fh.seek(0, os.SEEK_END)

    def lines(self, timeout: float) -> list:
        """The board's lines that arrive within `timeout` seconds (header dropped)."""
        out, end = [], time.time() + timeout
        while time.time() < end:
            if self.sock:
                self.sock.settimeout(max(0.05, end - time.time()))
                try:
                    data, addr = self.sock.recvfrom(4096)
                except socket.timeout:
                    break
                text = data.decode("utf-8", errors="replace")
                wifi_boards.record_banner(text, addr[0])
                if addr[0] == self.ip:
                    out.append(wifi_boards._clean(text))
                continue
            if self.fh is None or os.path.basename(self.path) != time.strftime("syslog_%Y%m%d.log"):
                self._open_day_log(at_end=False)       # the file appeared, or a new day began
            line = self.fh.readline() if self.fh else ""
            if not line:
                time.sleep(0.2)
                continue
            if self.tag in line:
                out.append(wifi_boards._clean(line.split("] ", 2)[-1]))
        return out

    def close(self):
        if self.sock:
            self.sock.close()
        if self.fh:
            self.fh.close()


def stop_board(ip: str, port: int = wifi_boards.PING_PORT, tries: int = STOP_TRIES) -> bool:
    """"lino-stop" until the board answers "lino-stop ok": the firmware latches it and halts
    the motors at once, whatever tool is running."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.settimeout(0.25)
    try:
        for i in range(tries):
            s.sendto(b"lino-stop", (ip, port))
            try:
                data, _ = s.recvfrom(512)
            except socket.timeout:
                continue
            if data.decode(errors="replace").strip().startswith("lino-stop ok"):
                print(f"  lino-stop acknowledged after {i + 1} datagram(s)")
                return True
    finally:
        s.close()
    print(f"  ⚠️ lino-stop: no acknowledgement in {tries} datagrams")
    return False


def write_app(a, app: str) -> bool:
    argv = [sys.executable, "-u", os.path.join(REPO_ROOT, "scripts", "flash_mcu.py"), "--env-only",
            "--env", a.env, "--port", a.port, "--baud", str(a.baud), "--params", a.params,
            "--firmware-name", a.controller, "--app", app, "--total-timeout", "300"]
    res = subprocess.run(argv, capture_output=True, text=True, timeout=400)
    for line in (res.stdout + res.stderr).splitlines():
        if re.search(r"env block|Board reports|❌", line):
            print(f"    | {line.strip()}")
    return res.returncode == 0


def ended(lines: list) -> str:
    """The line that ends a test_acc run, or ''."""
    return next((ln for ln in lines if END_RE.search(ln)), "")


def results(lines: list) -> list:
    return [ln for ln in lines if RESULT_RE.search(ln)]


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="test_acc on a Wi-Fi robot, then app=base again")
    ap.add_argument("--params", required=True)
    ap.add_argument("--env", required=True, help="the PlatformIO env (flash_mcu.py --env)")
    ap.add_argument("--port", required=True, help="the configured tty (absent = over the air)")
    ap.add_argument("--baud", type=int, required=True)
    ap.add_argument("--controller", required=True)
    ap.add_argument("--ip", required=True, help="the board's address (its syslog source)")
    ap.add_argument("--timeout", type=float, default=300.0)
    ap.add_argument("--transcript", default="", help="write every line the board sent here "
                    "(drivetrain_report.py --from-test-acc reads it)")
    a = ap.parse_args(argv)

    src = SyslogSource(a.ip)
    print(f"  the board's syslog: {src.where}")
    try:
        if not src.lines(SYSLOG_WAIT_S) and not (wifi_boards.ping(a.ip, timeout=1.5) and src.lines(10)):
            print(f"TEST_ACC_FAILED: no syslog line from {a.ip} in {SYSLOG_WAIT_S} s -- the results "
                  f"would be lost, so nothing moves")
            return 1
        print("  app=test_acc over the air -- THE ROBOT MOVES (full power runs)")
        t0 = time.time()
        got = []
        try:
            if not write_app(a, "test_acc"):
                print("TEST_ACC_FAILED: the env write with app=test_acc failed")
                return 1
            last = 0
            while time.time() - t0 < a.timeout:
                got += src.lines(2.0)
                if ended(got):
                    break
                el = int(time.time() - t0)
                if el // 20 != last // 20:
                    print(f"  {el} s: {sum('IDENT' in ln for ln in got)} IDENT, "
                          f"{sum('MAX PWM' in ln for ln in got)} runs", flush=True)
                last = el
        finally:
            stop_board(a.ip)
            print("  app=base back over the air")
            base_ok = write_app(a, "base")
        end = ended(got)
        if a.transcript:
            with open(a.transcript, "w", encoding="utf-8") as fh:
                fh.write("\n".join(got) + "\n")
            print(f"  transcript: {len(got)} lines -> {a.transcript}")
        for ln in results(got):
            print(f"  | {ln}")
        if not base_ok:
            print("TEST_ACC_FAILED: app=base was not written back -- the board is still in the tool")
            return 1
        if end and "[test_acc] done" in end:
            print(f"TEST_ACC_DONE after {time.time() - t0:.0f} s")
            return 0
        print(f"TEST_ACC_FAILED: {end or f'no end line in {a.timeout:.0f} s'}")
        return 1
    finally:
        src.close()


if __name__ == "__main__":
    sys.exit(main())
