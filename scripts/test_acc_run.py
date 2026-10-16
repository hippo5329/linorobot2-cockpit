#!/usr/bin/env python3
"""test_acc on a robot, from start to base again -- the 1-Click's --test-acc step.

    test_acc_run.py --params CFG --env PIO_ENV --port PORT --baud B --controller NAME --ip BOARD_IP
                    [--timeout 300]
    test_acc_run.py ... --serial [--console-baud B2]
                                          a serial robot: the console is PORT at B2 (no --ip)

test_acc is the tool's `acc` command: the drivetrain's PID identification (dead zone,
open-loop step, closed-loop steps, one wheel at a time) and then twelve 1 s full-power runs,
straight and spinning. THE ROBOT MOVES, at full power, so the caller checks the room first
(clearance.py, 1.0 m all round).

The board boots base only (user, 2026-10-16: "keep only base with tool escape ... we will use
tool escape to do all these tasks"), so this enters the tool the way the web UI's Hardware
Tests tab does, through scripts/lino_tool.py, and writes nothing to the board's env:

  1  into the tool: on Wi-Fi "lino-tool" to the ping port (answered with the OTA password's
     challenge) and the tool's TCP terminal, logged in the same way; on a serial robot the
     board restarted on its tty (lino_tool SerialLink.reboot: EN on an ESP32, the 1200-baud
     touch and picotool on an RP2, an SWD reset on the UNO Q) and `tool` typed into its boot
     window (catch_window)
  2  `acc`, and its lines on that link until "[test_acc] done", "STOPPED", "test_acc refused"
     or the tool's own fail / stopped answer, --timeout at most. A keep-alive goes out every
     250 ms: the tool stops the drive 2 s after this end of the link goes quiet
  3  ALWAYS, whatever happened: on Wi-Fi "lino-stop" to the ping port until the board
     acknowledges (the firmware latches it and halts the motors); `stop` on the link; then
     `base`, until the board says "restarting into base". A serial board that does not say it
     is restarted by its reset (step 1's), which boots base too
Prints the results (IDENT gains / plant / dead zone, MAX VEL / ACC) and TEST_ACC_DONE or
TEST_ACC_FAILED. Exit 0 when test_acc finished.

A serial robot (--serial: any board on a serial transport -- the UNO Q, an ESP32 or an RP2 on
USB): the tool's console is the micro-ROS tty itself, at the micro-ROS rate (--console-baud),
so the agent on that tty is paused (SIGSTOP: a dead one would be respawned by the launch onto
the tty and eat the lines) and the port is read here. An agent that cannot be paused (another
user's process), or one this user cannot see holding the tty, refuses the run before the
board is touched. After `base` the agent is let go (SIGCONT), and the board opens a new
session with it. SIGTERM and SIGHUP run the same cleanup as any other end.
"""
import argparse
import contextlib
import os
import re
import signal
import socket
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import wifi_boards  # noqa: E402

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
# test_acc's own end lines, and the tool's end answers to `acc` (tools.cpp: `fail: acc -- why`
# when it halts on a fault or is refused, `stopped: acc (why)` when it is stopped).
END_RE = re.compile(r"\[test_acc\] done|test_acc refused|\[test_acc\] STOPPED|^(fail|stopped): acc\b")
RESULT_RE = re.compile(r"IDENT (gains|robot|plant|deadzone|done)|MAX VEL|MIN VEL|MAX ACC|MIN ACC|"
                       r"IMU ACC|time to|distance to stop|BATTERY |HOME |refused|STOPPED")
STOP_TRIES = 20
# test_acc prints a line at least every few seconds; this long without one, the tty is reopened.
SERIAL_QUIET_S = 15.0
# The tool's own stop on its console (firmware/src/tools/tools.cpp handleLine: `stop`, `s` or
# an empty line; CR LF is one line end). The empty line too: a half line already in the
# board's buffer (a reopen mid-burst) would otherwise swallow the `stop` into one bad command.
CONSOLE_STOP = b"stop\r\n\r\n"
# To catch the 3 s boot window (an RP2 takes ~2 s to come back): routes_tool.WINDOW_S.
WINDOW_S = 15.0
# `base` answers "ok: base (restarting into base)" and prints "[tool] restarting into base".
LEFT = b"restarting into base"
LEAVE_S = 10.0


# A SIGTERM (the 1-Click's timeout, the web UI's cancel) or a SIGHUP (the ssh session went
# away) ended the process without its `finally`: the agent stayed SIGSTOPped on the tty and the
# board stayed in the tool, out of micro-ROS. Both become the KeyboardInterrupt every `finally` here
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
    """The stop, `base` and the agent's SIGCONT: not to be cut short by another signal."""
    _cleaning["depth"] += 1
    try:
        yield
    finally:
        _cleaning["depth"] -= 1


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


class ConsoleSource:
    """The tool's lines from its link (lino_tool SerialLink or TcpLink), with the keep-alive
    its deadman wants."""

    def __init__(self, link, where: str):
        self.link = link
        self.buf = b""
        self.where = where
        self.heard = time.time()

    def feed(self, data: bytes) -> list:
        self.buf += data
        *done, self.buf = self.buf.split(b"\n")
        out = [ln.decode("utf-8", errors="replace").strip("\r") for ln in done]
        return [ln for ln in out if ln.strip()]

    @property
    def closed(self) -> bool:
        return bool(getattr(self.link, "closed", False))

    def lines(self, timeout: float) -> list:
        out, end = [], time.time() + timeout
        while time.time() < end and not self.closed:
            self.link.keepalive()
            data = self.link.read()
            if not data:
                if time.time() - self.heard > SERIAL_QUIET_S and hasattr(self.link, "_reopen"):
                    # The UNO Q's ttyHS1 can stop delivering mid-burst while the board has sent
                    # everything (2026-10-14); a reopen brings the bytes back.
                    self.heard = time.time()
                    self.link._reopen()
                time.sleep(0.02)
                continue
            self.heard = time.time()
            new = self.feed(data)
            out += new
            if ended(new):
                break                     # the run's end: no need to wait out the rest
        return out


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


def _fds_readable(pid: int) -> bool:
    """Whether this user can list a process's open files (/proc/<pid>/fd)."""
    try:
        os.listdir(f"/proc/{pid}/fd")
        return True
    except OSError:
        return False


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
    # Hidden = an agent naming the tty whose fds this user cannot read. One whose fds ARE
    # readable and hold no such tty is not on it: `ros2 run micro_ros_agent ... --dev <tty>`,
    # the launcher whose child is the agent -- counted as hidden, it refused every serial run.
    hidden = [p for p in agents_on(port) if p not in paused and not _fds_readable(p)]
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


def enter_serial(a, held: dict):
    """The tool on the board's tty, entered as the web UI enters it (routes_tool._open_serial,
    lino_tool.connect): the board restarted, `tool` typed into its boot window. Once more on a
    fresh open when no prompt comes (the UNO Q's ttyHS1 has dropped the tool's start-up burst).
    The link goes into `held` as soon as it exists, so the cleanup reaches it whatever stops
    this. The agent is already paused: the reboot leaves it alone (release=False)."""
    import lino_tool
    baud = a.console_baud or a.baud
    for attempt in (1, 2):
        link = lino_tool.SerialLink(a.port, baud, a.env)
        held["link"] = link
        if link.reboot(release=False) and lino_tool.catch_window(link, WINDOW_S):
            held["entered"] = True
            return link
        link.close()
        held.pop("link", None)            # not in the tool: the board boots base
        if attempt == 1:
            print("  no prompt: rebooting the board once more")
    return None


def enter_wifi(a, held: dict):
    """The tool over the air, entered as the web UI enters it (routes_tool._open_wifi):
    "lino-tool" to the ping port with the OTA password's answer, then the TCP terminal."""
    import lino_tool
    password, env = _ota(a)
    if not password:
        print("  this Wi-Fi robot has no OTA password: the board refuses the tool over the network")
        return None
    ping_port = int(env.get("ping_port") or wifi_boards.PING_PORT)
    if not lino_tool.request_tool_udp(a.ip, ping_port, password):
        return None
    # The board restarts into the tool now, whether or not its terminal is reached.
    held["entered"] = True
    link = lino_tool.TcpLink(a.ip, int(env.get("term_port") or 2323), password)
    held["link"] = link
    if not link.open():
        print(f"  no tool terminal at {a.ip}:{link.port}")
        return None
    return link


def _ota(a):
    """The robot's OTA password and env values, from its config and secrets (never argv)."""
    import cockpit_paths
    import mcu_env
    env = mcu_env.env_from_config(os.path.abspath(a.params), cockpit_paths.secrets_path())
    return str(env.get("ota_password") or ""), env


def run_acc(a, src, got: list, t0: float) -> None:
    """`acc`, then its lines until it ends or --timeout. The tool's settings (ta_ms, ta_pct)
    are the board's env's, as they were when an env write started test_acc."""
    import lino_tool
    print("  acc -- THE ROBOT MOVES (full power runs)")
    # A keep-alive ahead of the command: the tool arms its console deadman for a drive only
    # when that console has sent one (tools.cpp startJob), so this end going quiet -- this
    # script killed, its ssh gone -- stops the robot 2 s later.
    src.link.keepalive()
    ans, buf = lino_tool.answer(src.link, "acc")
    got += src.feed(buf)
    if ans is not None and not ans.get("ok"):
        return                            # its `fail: acc -- why` line is in `got`: the end
    last = 0
    while time.time() - t0 < a.timeout and not ended(got):
        got += src.lines(2.0)
        if src.closed:
            print("  the board hung up")
            break
        el = int(time.time() - t0)
        if el // 20 != last // 20:
            print(f"  {el} s: {sum('IDENT' in ln for ln in got)} IDENT, "
                  f"{sum('MAX PWM' in ln for ln in got)} runs", flush=True)
        last = el


def leave_tool(a, held: dict, serial: bool) -> bool:
    """The motors off and the board back in base, whatever happened: `stop`, then `base`
    until it says it restarts. True when it did (or, on serial, its reset did it)."""
    import lino_tool
    link = held.get("link")
    entered = held.get("entered", False)
    if link is None:
        # Never reached; or, on Wi-Fi, restarted into the tool with no terminal to say base.
        return not entered
    try:
        if not getattr(link, "closed", False) and getattr(link, "s", None) is not None:
            link.write(CONSOLE_STOP)
            print("  stop sent to the tool (it halts its motors)")
            lino_tool.pump(link, 1.0, until=(lino_tool.PROMPT,))
            print("  base (the board restarts into micro-ROS)")
            link.write(b"base\r\n")
            if LEFT in lino_tool.pump(link, LEAVE_S if entered else 1.0, until=(LEFT,)):
                return True
    except Exception as exc:
        print(f"  the tool's stop/base not sent ({type(exc).__name__})")
    if not entered:
        return True                       # it never entered the tool: it boots base anyway
    if serial:
        # No word from the tool: its reset (EN, the 1200-baud touch and picotool, SWD) stops
        # the motors, and the board boots base -- the restart flag is set by the tool only.
        print("  no answer to base: resetting the board (it boots base)")
        try:
            link.close()
            return bool(link.reboot(release=False))
        except Exception as exc:
            print(f"  the reset failed ({type(exc).__name__})")
    return False


def _close(held: dict) -> None:
    link = held.get("link")
    if link is not None:
        try:
            link.close()
        except Exception:
            pass


def run_serial(a) -> int:
    """test_acc with its console on the tty: no syslog, no ping port; the agent paused."""
    paused = list(pause_holders(a.port))
    held, got, t0, left = {}, [], time.time(), False
    try:
        why = pause_refusal(a.port, paused)
        if why:
            print(f"TEST_ACC_REFUSED: {why} -- nothing moved")
            return 1
        print(f"  the board's console: {a.port} at {a.console_baud or a.baud} baud")
        try:
            link = enter_serial(a, held)
            if link is None:
                print("TEST_ACC_FAILED: the board did not enter the tool by its boot escape "
                      "(is tool_wait 0, or is its firmware older?) -- nothing moved")
                return 1
            t0 = time.time()
            run_acc(a, ConsoleSource(link, f"the console on {a.port}"), got, t0)
        finally:
            with cleaning():
                left = leave_tool(a, held, serial=True)
                _close(held)
    finally:
        with cleaning():
            _close(held)
            resume(paused)
    return report(a, got, left, t0)


def run_wifi(a) -> int:
    """test_acc over the air: the tool's terminal, the stop by lino-stop as well, then base."""
    held, got, t0, left, link = {}, [], time.time(), False, None
    try:
        link = enter_wifi(a, held)
        if link is not None:
            t0 = time.time()
            run_acc(a, ConsoleSource(link, f"the tool terminal at {a.ip}"), got, t0)
    finally:
        with cleaning():
            if held.get("entered"):
                stop_board(a.ip)
            left = leave_tool(a, held, serial=False)
            _close(held)
    if link is None:
        print(f"TEST_ACC_FAILED: {a.ip} did not open its tool (lino-tool and the terminal) -- "
              + ("nothing moved" if left else "nothing moved, but the board restarted into the "
                 "tool and could not be told base: power-cycle it, or open the tool and press "
                 "Back to robot"))
        return 1
    return report(a, got, left, t0)


def report(a, got: list, left: bool, t0: float) -> int:
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
    if not left:
        print("TEST_ACC_FAILED: the board did not restart into base -- it may still be in the tool")
        return 1
    if end and "[test_acc] done" in end:
        print(f"TEST_ACC_DONE after {time.time() - t0:.0f} s")
        return 0
    print(f"TEST_ACC_FAILED: {end or f'no end line in {a.timeout:.0f} s'}")
    return 1


def ended(lines: list) -> str:
    """The line that ends a test_acc run, or ''."""
    return next((ln for ln in lines if END_RE.search(ln)), "")


def results(lines: list) -> list:
    return [ln for ln in lines if RESULT_RE.search(ln)]


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="test_acc on a robot, then base again")
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
        # every `finally` above has run: the stop, `base`, the agent let go
        print("TEST_ACC_FAILED: interrupted (a signal or ^C) -- the stop and base were sent")
        return 1


if __name__ == "__main__":
    sys.exit(main())
