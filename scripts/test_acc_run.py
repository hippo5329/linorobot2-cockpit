#!/usr/bin/env python3
"""test_acc on a robot, from start to "app=base" again -- the 1-Click's --test-acc step.

    test_acc_run.py --params CFG --env PIO_ENV --port PORT --baud B --controller NAME --ip BOARD_IP
                    [--timeout 300]
    test_acc_run.py ... --serial [--console-baud B2]
                                          a serial robot: the console is PORT at B2 (no --ip)

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

A serial robot (--serial: any board on a serial transport -- the UNO Q, an ESP32 or an RP2 on
USB) has no syslog and no ping port. While a tool runs its console is the micro-ROS tty
itself, at the micro-ROS rate (--console-baud; --baud stays the env write's), so the agent
on that tty is paused (SIGSTOP: a dead one would be respawned by the launch onto the tty and
eat the lines) and the port is read here. An agent that cannot be paused (another user's
process), or one this user cannot see holding the tty, refuses the run before app=test_acc
is written. The stop is the tool's own `stop` on the console, at once, then the env write
with app=base, which restarts the board; then the agent is let go (SIGCONT), and the board
opens a new session with it. SIGTERM and SIGHUP run the same cleanup as any other end. The UNO Q's env goes over SWD, so the tty stays open across it; an ESP32's goes
through esptool and an RP2's through BOOTSEL on that same tty, so the reader lets go of it
for the write and opens it again after (the RP2's port re-enumerates).
"""
import argparse
import contextlib
import os
import re
import signal
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
# Longer than the base app's quietest syslog: a board at rest sends its [fw] banner once a
# minute and little else, and a 30 s window that opened just after one heard nothing (a tracked robot).
SYSLOG_WAIT_S = 65
STOP_TRIES = 20
# test_acc prints a line at least every few seconds; this long without one, the tty is reopened.
SERIAL_QUIET_S = 15.0
# The tool's own stop on its console (firmware/src/tools/tools.cpp handleLine: `stop`, `s` or
# an empty line; CR LF is one line end). The empty line too: a half line already in the
# board's buffer (a reopen mid-burst) would otherwise swallow the `stop` into one bad command.
CONSOLE_STOP = b"stop\r\n\r\n"


# A SIGTERM (the 1-Click's timeout, the web UI's cancel) or a SIGHUP (the ssh session went
# away) ended the process without its `finally`: the agent stayed SIGSTOPped on the tty and the
# board booted into test_acc again. Both become the KeyboardInterrupt every `finally` here
# handles -- except while that cleanup itself runs: a second signal then only waits for it.
_cleaning = {"depth": 0, "pending": None}


def _bail(signum, frame):
    if _cleaning["depth"]:
        _cleaning["pending"] = signum
        return
    raise KeyboardInterrupt


def install_signal_handlers():
    for sig in (signal.SIGHUP, signal.SIGTERM):
        signal.signal(sig, _bail)


@contextlib.contextmanager
def cleaning():
    """The stop, app=base and the agent's SIGCONT: not to be cut short by another signal."""
    _cleaning["depth"] += 1
    try:
        yield
    finally:
        _cleaning["depth"] -= 1


def heard(src, seconds: float) -> bool:
    """True at the board's first syslog line, without waiting out the rest of `seconds`."""
    end = time.time() + seconds
    while time.time() < end:
        if src.lines(min(2.0, max(0.05, end - time.time()))):
            return True
    return False


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


class SerialSource:
    """The board's console lines from its tty (a serial robot's test_acc)."""

    def __init__(self, port: str, baud: int, env: str):
        import lino_tool
        self.link = lino_tool.SerialLink(port, baud, env)
        self.buf = b""
        self.where = f"the console on {port}"
        self.heard = time.time()

    def open(self, timeout: float = 15.0) -> bool:
        self.heard = time.time()
        return self.link.open(timeout)

    def lines(self, timeout: float) -> list:
        out, end = [], time.time() + timeout
        while time.time() < end:
            data = self.link.read() if self.link.s else b""
            if not data:
                if time.time() - self.heard > SERIAL_QUIET_S:
                    # The UNO Q's ttyHS1 can stop delivering mid-burst while the board has sent
                    # everything (2026-10-14); a reopen brings the bytes back.
                    self.heard = time.time()
                    self.link._reopen()
                time.sleep(0.05)
                continue
            self.heard = time.time()
            self.buf += data
            *done, self.buf = self.buf.split(b"\n")
            out += [ln.decode("utf-8", errors="replace").strip("\r") for ln in done]
        return [ln for ln in out if ln.strip()]

    def stop_tool(self) -> bool:
        """The tool's fast stop on its own console: the motors off at once, not when the env
        write with app=base (up to 300 s) restarts the board. Opens the tty for it when the
        reader had let go of it (an ESP32's or an RP2's env write). True when it was sent."""
        try:
            if not self.link.s and not self.open(timeout=3.0):
                return False
            self.link.write(CONSOLE_STOP)
            print("  stop sent on the console (the tool halts its motors)")
            return True
        except Exception as exc:
            print(f"  ⚠️ stop on the console not sent ({type(exc).__name__})")
            return False

    def close(self):
        if self.link.s:
            try:
                self.link.s.close()
            except Exception:
                pass
            self.link.s = None


def pause_holders(port: str) -> list:
    """SIGSTOP whatever holds the tty (the micro-ROS agent): stopped, it keeps the tty open
    but reads nothing, and the launch respawns only a process that died."""
    import signal
    import flash_mcu
    paused = []
    for pid in flash_mcu._holders_of(port):
        try:
            os.kill(pid, signal.SIGSTOP)
            paused.append(pid)
            print(f"  pid {pid} holds {port}: paused for the run")
        except (ProcessLookupError, PermissionError) as exc:
            print(f"  pid {pid} holds {port}: not paused ({type(exc).__name__})")
    return paused


def _same_tty(token: str, port: str) -> bool:
    try:
        return os.path.realpath(token) == os.path.realpath(port)
    except OSError:
        return token == port


def _ns_depth(pid) -> int:
    """How many pid namespaces deep `pid` sits, seen from here (/proc/PID/status NSpid, which
    anyone may read); 0 when unknown."""
    try:
        with open(f"/proc/{pid}/status") as fh:
            for line in fh:
                if line.startswith("NSpid:"):
                    return len(line.split()) - 1
    except OSError:
        pass
    return 0


def agents_on(port: str) -> list:
    """Pids of micro-ROS agents whose command line names this tty. A process's command line
    is readable by anyone; its fds (flash_mcu._holders_of) only by its own user or root. Only
    agents in this pid namespace count: a container's agent on its own /dev/ttyUSB0 names a
    different board."""
    out = []
    try:
        entries = os.listdir("/proc")
    except OSError:
        return out
    depth = _ns_depth("self")
    for d in entries:
        if not d.isdigit() or int(d) == os.getpid() or _ns_depth(d) != depth:
            continue
        try:
            with open(f"/proc/{d}/cmdline", "rb") as fh:
                argv = fh.read().decode("utf-8", errors="replace").split("\0")
        except OSError:
            continue
        if any("micro_ros_agent" in t for t in argv) and \
                any(t.startswith("/dev/") and _same_tty(t, port) for t in argv):
            out.append(int(d))
    return out


def pause_refusal(port: str, paused: list) -> str:
    """Why test_acc must not start on this tty, or ''. A holder left running reads the tool's
    lines as micro-ROS frames (the results are lost) and keeps writing to the board; and when
    no holder shows at all while an agent names the tty, its fds are hidden from this user --
    it was never paused, only not seen."""
    import flash_mcu
    left = [p for p in flash_mcu._holders_of(port) if p not in paused]
    if left:
        return (f"pid {', '.join(map(str, left))} holds {port} and could not be paused -- run this "
                f"as the user that started the stack (or root), or stop the stack first")
    hidden = [p for p in agents_on(port) if p not in paused]
    if hidden:
        return (f"a micro-ROS agent (pid {', '.join(map(str, hidden))}) runs on {port} but this user "
                f"cannot see its open files, so it cannot be paused -- run this as the user that "
                f"started the stack (or root), or stop the stack first")
    return ""


def resume(pids: list):
    """SIGCONT every paused pid, once: the list is emptied, so a second call does nothing."""
    import signal
    done = list(pids)
    del pids[:]
    for pid in done:
        try:
            os.kill(pid, signal.SIGCONT)
        except (ProcessLookupError, PermissionError):
            pass
    if done:
        print(f"  {len(done)} paused process(es) let go: the agent takes the tty back")


def env_over_tty(env: str) -> bool:
    """True when the env write goes through the console's own tty (esptool, an RP2's BOOTSEL),
    False for the UNO Q, whose env goes over SWD."""
    return not env.startswith("unoq")


def run_serial(a) -> int:
    """test_acc with its console on the tty: no syslog, no ping port; app=base is the stop."""
    paused = list(pause_holders(a.port))
    src = SerialSource(a.port, a.console_baud or a.baud, a.env)
    shared = env_over_tty(a.env)
    got, t0, base_ok = [], time.time(), False
    try:
        why = pause_refusal(a.port, paused)
        if why:
            print(f"TEST_ACC_REFUSED: {why} -- nothing moved")
            return 1
        if not src.open():
            print(f"TEST_ACC_FAILED: {a.port} did not open -- nothing moved")
            return 1
        print(f"  the board's console: {src.where} at {a.console_baud or a.baud} baud")
        print("  app=test_acc -- THE ROBOT MOVES (full power runs)")
        try:
            if shared:
                src.close()      # esptool / the 1200-baud touch need the tty to themselves
            if not write_app(a, "test_acc"):
                print("TEST_ACC_FAILED: the env write with app=test_acc failed")
                return 1
            if shared and not src.open(timeout=30.0):
                print(f"TEST_ACC_FAILED: {a.port} did not come back after the env write")
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
            with cleaning():
                src.stop_tool()
                src.close()
                print("  app=base (the board restarts: the motors stop)")
                base_ok = write_app(a, "base")
    finally:
        with cleaning():
            src.close()
            resume(paused)
    return report(a, got, base_ok, t0)


def report(a, got: list, base_ok: bool, t0: float) -> int:
    end = ended(got)
    if a.transcript:
        # A transcript that cannot be written must not take the run's verdict with it.
        try:
            os.makedirs(os.path.dirname(a.transcript) or ".", exist_ok=True)
            with open(a.transcript, "w", encoding="utf-8") as fh:
                fh.write("\n".join(got) + "\n")
            print(f"  transcript: {len(got)} lines -> {a.transcript}")
        except OSError as e:
            print(f"  transcript not written ({e}); the results follow")
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


def write_app(a, app: str) -> bool:
    """The env block with this app (flash_mcu.py --env-only). app=base runs in a session of its
    own: the callers stop a step by signalling its whole process group, and the write that
    takes the board out of the tool must not die with the rest of it."""
    argv = [sys.executable, "-u", os.path.join(REPO_ROOT, "scripts", "flash_mcu.py"), "--env-only",
            "--env", a.env, "--port", a.port, "--baud", str(a.baud), "--params", a.params,
            "--firmware-name", a.controller, "--app", app, "--total-timeout", "300"]
    res = subprocess.run(argv, capture_output=True, text=True, timeout=400,
                         start_new_session=(app == "base"))
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
    ap = argparse.ArgumentParser(description="test_acc on a robot, then app=base again")
    ap.add_argument("--params", required=True)
    ap.add_argument("--env", required=True, help="the PlatformIO env (flash_mcu.py --env)")
    ap.add_argument("--port", required=True, help="the configured tty (absent = over the air)")
    ap.add_argument("--baud", type=int, required=True)
    ap.add_argument("--controller", required=True)
    ap.add_argument("--ip", default="", help="the board's address (its syslog source)")
    ap.add_argument("--serial", action="store_true",
                    help="a serial robot: the console is --port, the agent there is paused")
    ap.add_argument("--console-baud", type=int, default=0,
                    help="--serial: the console's rate, the micro-ROS one (default: --baud)")
    ap.add_argument("--timeout", type=float, default=300.0)
    ap.add_argument("--transcript", default="", help="write every line the board sent here "
                    "(drivetrain_report.py --from-test-acc reads it)")
    a = ap.parse_args(argv)
    if not a.serial and not a.ip:
        ap.error("--ip is required without --serial")
    install_signal_handlers()
    try:
        return run_serial(a) if a.serial else run_wifi(a)
    except KeyboardInterrupt:
        # every `finally` above has run: the stop, app=base, the agent let go
        print("TEST_ACC_FAILED: interrupted (a signal or ^C) -- the stop and app=base were sent")
        return 1


def run_wifi(a) -> int:
    """test_acc over the air: the results by syslog, the stop by lino-stop, then app=base."""
    src = SyslogSource(a.ip)
    print(f"  the board's syslog: {src.where}")
    try:
        if not heard(src, SYSLOG_WAIT_S) and not (wifi_boards.ping(a.ip, timeout=1.5) and heard(src, 10)):
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
            with cleaning():
                stop_board(a.ip)
                print("  app=base back over the air")
                base_ok = write_app(a, "base")
        return report(a, got, base_ok, t0)
    finally:
        with cleaning():
            src.close()


if __name__ == "__main__":
    sys.exit(main())
