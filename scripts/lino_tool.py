#!/usr/bin/env python3
"""lino_tool.py -- the board's tool from the host: enter it, run commands, leave.

The firmware has one mode (2026-10-12): `base` boots, waits `tool_wait` ms (3 s) for the
word `tool` and Enter, then starts micro-ROS. Typed in time, the board stays in the tool, a
command shell on its console. This script gets it there and drives it:

    # USB: reboot the board, catch its window, open a terminal (Ctrl-] leaves)
    python3 scripts/lino_tool.py --port /dev/ttyACM0 --env pico2

    # commands, then back to base; `sleep N` waits, `stop` stops the running command
    python3 scripts/lino_tool.py --port /dev/ttyUSB0 --env esp32 --baud 1500000 \\
        -c "i2c" -c "motor 1 30" -c "sleep 5" -c "stop" --base

    # Wi-Fi: "lino-tool" to the ping port, answered with a nonce, and the reply is
    # md5(password:nonce), as OTA does; then the TCP terminal, logged in the same way --
    # the OTA password never crosses the network
    python3 scripts/lino_tool.py --params robot_config.yaml -c "sensors" -c "sleep 3" -c stop

The commands (the tool's `help`): i2c [secs], sensors, motor <1-4|all> <pct> [secs],
motors [pct], acc [ms], adc, set <key> <value>, get [key], status, base. `stop`, `s` or an
empty line stops the running one. On exit this script sends `stop`: a wheel it started is
never left turning. On the Wi-Fi terminal it sends a keep-alive every second; the board
stops a drive started there when the terminal goes quiet for 3 s.

Every command gets the board's answer in words -- `ok: motor 1 30 (running)`, `fail: motor 3
30 -- <why>`, and for a running one `done: ...` or `stopped: ... (why)` when it ends; no answer
in 2 s is said too.
Exit 0 when the tool was entered and the board took every command; 3 when it refused one or
did not answer; the transcript is stdout.
"""
import argparse
import hashlib
import os
import socket
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import tool_answer as tool_words  # noqa: E402  (tool_answer() below is the login hash)

PROMPT = b"tool> "
ENTERED = (b"[app] tool", b"[tool] ready")
# Commands that keep running until `stop` (or their own time), and the lines that end
# the long blocking ones.
RUNNING = ("sensors", "motor", "motors", "enc", "ramp")
# The long blocking commands end where every command ends: at the prompt, which every path
# through the shell prints (a refusal, a usage line, `done`, a stop). Their own words go to
# syslog as well as the console, and differ by path; the prompt does not.
LONG = {"acc": 900.0, "adc": 300.0}


def log(msg):
    print(f"[lino_tool] {msg}", flush=True)


def tool_answer(password: str, nonce: str) -> str:
    """md5(password:nonce): the ping port's lino-tool and the terminal's login alike."""
    return hashlib.md5(f"{password}:{nonce}".encode()).hexdigest()


# ---------------------------------------------------------------- serial
def release_port(port: str) -> None:
    """Stop only what holds THIS port (a micro-ROS agent on it), found through /proc: a
    host with two robots keeps the other's agent. Restart the stack afterwards."""
    import signal
    try:
        target = os.path.realpath(port)
    except OSError:
        return
    me = os.getpid()
    for pid in os.listdir("/proc"):
        if not pid.isdigit() or int(pid) == me:
            continue
        fd_dir = f"/proc/{pid}/fd"
        try:
            for fd in os.listdir(fd_dir):
                if os.path.realpath(os.path.join(fd_dir, fd)) == target:
                    log(f"stopping pid {pid}, which holds {port} (restart the robot stack after)")
                    os.kill(int(pid), signal.SIGINT)
                    time.sleep(1.0)
                    break
        except (PermissionError, FileNotFoundError, ProcessLookupError):
            continue


# ---------------------------------------------------------------- serial (the link)
class SerialLink:
    def __init__(self, port: str, baud: int, env: str):
        try:
            import serial
        except ImportError:
            raise SystemExit("[lino_tool] pyserial is missing: run this inside the cockpit "
                             "container (docker compose exec -it cockpit python3 "
                             "scripts/lino_tool.py ...), or pip install pyserial")
        self.port, self.baud, self.env = port, baud, env
        self.serial = serial
        self.s = None
        self.last_keep = 0.0

    def _new(self):
        # An ESP32's auto-reset circuit reads DTR/RTS as EN/IO0: open with both released,
        # or the open itself can land it in the ROM's download mode (mcu_probe.py). An RP2
        # (and the UNO Q's UART) is the other way round: its USB port drops every byte it
        # writes while DTR is low -- measured 2026-10-12 on a Pico 2, which entered the tool
        # on `tool` and answered into nothing -- so DTR is asserted there.
        s = self.serial.Serial()
        s.port, s.baudrate, s.timeout = self.port, self.baud, 0.05
        # Open with both lines high, as the kernel raises them: that resets nothing. pyserial
        # applies DTR before RTS, and clearing them in that order passes through DTR low /
        # RTS high -- the reset on an ESP32-S3's USB-Serial/JTAG (measured 2026-10-12: every
        # reopen restarted the chip, rst:0x15, and the tool window never came). So RTS first.
        s.open()
        if self.env.startswith("esp32"):
            s.rts = False
            s.dtr = False
        return s

    def open(self, timeout: float = 15.0) -> bool:
        end = time.time() + timeout
        while time.time() < end:
            try:
                self.s = self._new()
                return True
            except Exception:
                time.sleep(0.1)       # an RP2's port is re-enumerating after the reboot
        return False

    def reboot(self) -> bool:
        """Restart the board so its tool window opens. An ESP32: EN, pulsed on this
        handle. An RP2: the 1200-baud touch into BOOTSEL, then `picotool reboot` into the
        application, both aimed at THIS port's board. A USB reset is no reboot there:
        measured 2026-10-12 on a running Pico 2, the port re-enumerated and no banner
        followed, and the firmware has no picotool reset interface for a running board."""
        import flash_mcu
        release_port(self.port)                           # its micro-ROS agent lets go
        if self.env.startswith("esp32"):
            if not self.open():
                return False
            self.s.rts = True             # EN low
            time.sleep(0.1)
            self.s.rts = False
            # An S3 on its own USB (USB-Serial/JTAG) re-enumerates on that reset: this
            # handle is then dead. Reopen; a bridge's port simply opens again.
            self.s.close()
            time.sleep(0.3)
            return self.open()
        if self.env.startswith("unoq"):
            # The STM32 behind the UNO Q's ttyHS1: its boot lines, then an SWD reset, the
            # flasher's own path (unoq_swd.py). The agent must not hold ttyHS1, nor
            # arduino-router the lines.
            import unoq_swd
            try:
                unoq_swd.set_boot_lines()
                unoq_swd.run(["reset_config srst_only srst_push_pull", "init", "reset run", "shutdown"],
                             timeout=60)
            except unoq_swd.SwdError as exc:
                log(f"UNO Q reset: {exc}")
                return False
            return self.open()
        if self.env.startswith(("pico", "xrp")):
            flash_mcu.remember_usb_path(self.port)        # picotool aims at this board only
            if not flash_mcu.pulse_1200_baud(self.port):
                return False
            if not flash_mcu.rp2_reboot_into_app(self.env):
                return False
            return self.open()
        log(f"no reboot for env '{self.env}': give one of esp32*, esp32s3*, pico*, pico2*, xrp*, unoq*")
        return False

    def _reopen(self) -> bool:
        # The port went away under us: an ESP32-S3 on its own USB re-enumerates more than
        # once while it boots (the ROM, then the application) -- measured 2026-10-12 on an
        # S3, which printed its ROM banner on the reopened port and then dropped it again.
        try:
            self.s.close()
        except Exception:
            pass
        time.sleep(0.2)
        return self.open(10.0)

    def write(self, data: bytes):
        try:
            self.s.write(data)
        except Exception:
            if self._reopen():
                self.s.write(data)

    def read(self) -> bytes:
        try:
            return self.s.read(4096)
        except Exception:
            self._reopen()
            return b""

    def keepalive(self):
        # A NUL a second: the tool then stops a drive started from here if this script dies
        # (an ssh session that drops kills it before its `finally` can send `stop`).
        if time.time() - self.last_keep >= 0.25:
            self.last_keep = time.time()
            try:
                self.s.write(b"\0")
            except Exception:
                pass

    def close(self):
        if self.s:
            self.s.close()


# ---------------------------------------------------------------- Wi-Fi
def request_tool_udp(ip: str, ping_port: int, password: str, timeout: float = 2.0) -> bool:
    """"lino-tool", and the nonce's answer. A board without an OTA password refuses."""
    sk = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sk.settimeout(timeout)
    try:
        sk.sendto(b"lino-tool", (ip, ping_port))
        reply = sk.recvfrom(512)[0].decode(errors="replace")
        if reply.startswith("lino-tool nonce "):
            if not password:
                log("the board asks for its OTA password and none was given (--params or --password)")
                return False
            nonce = reply.split()[2]
            sk.sendto(f"lino-tool {tool_answer(password, nonce)}".encode(), (ip, ping_port))
            reply = sk.recvfrom(512)[0].decode(errors="replace")
        log(f"{ip}: {reply.split(' linorobot2')[0].strip()}")
        return reply.startswith("lino-tool ok")
    except socket.timeout:
        log(f"{ip}:{ping_port}: no answer to lino-tool")
        return False
    finally:
        sk.close()


class TcpLink:
    def __init__(self, ip: str, port: int, password: str):
        self.ip, self.port, self.password = ip, port, password
        self.s = None
        self.closed = False
        self.last_keep = 0.0

    def open(self, timeout: float = 30.0) -> bool:
        end = time.time() + timeout
        while time.time() < end:      # the board is restarting into the tool
            try:
                self.s = socket.create_connection((self.ip, self.port), timeout=2)
                self.s.settimeout(0.05)
                break
            except OSError:
                time.sleep(0.5)
        if not self.s:
            return False
        got = self._until((b"login: ", b"busy:", b"locked:"), 5)
        if b"busy:" in got or b"locked:" in got:
            log(got.decode(errors="replace").strip())
            return False
        nonce = None
        for line in got.decode(errors="replace").splitlines():
            if line.startswith("nonce "):
                nonce = line.split()[1]
        if not nonce or b"login: " not in got:
            log("the terminal did not offer a login")
            return False
        if not self.password:
            log("the terminal asks for the OTA password's answer and none was given")
            return False
        self.s.sendall(tool_answer(self.password, nonce).encode() + b"\n")
        got = self._until((PROMPT, b"wrong answer", b"too slow"), 5)
        if PROMPT not in got:
            log("the terminal refused the login")
            return False
        sys.stdout.write(got.split(b"ok\r\n", 1)[-1].decode(errors="replace"))
        return True

    def _until(self, endings, timeout):
        buf = b""
        end = time.time() + timeout
        while time.time() < end and not self.closed:
            buf += self.read()
            if any(e in buf for e in endings):
                return buf
        return buf

    def write(self, data: bytes):
        self.s.sendall(data)

    def read(self) -> bytes:
        try:
            d = self.s.recv(4096)
            if d == b"":
                self.closed = True    # the board hung up
            return d
        except (socket.timeout, BlockingIOError):
            return b""
        except OSError:
            self.closed = True
            return b""

    def keepalive(self):
        # A NUL a second: the board stops a drive started here when the terminal goes quiet.
        if time.time() - self.last_keep >= 0.25 and not self.closed:
            self.last_keep = time.time()
            try:
                self.s.sendall(b"\0")
            except OSError:
                self.closed = True

    def close(self):
        if self.s:
            self.s.close()


# ---------------------------------------------------------------- driving it
def pump(link, seconds: float, until=None) -> bytes:
    """Copy the board's output for `seconds`, or until one of `until` appears."""
    buf = b""
    end = time.time() + seconds
    while time.time() < end:
        link.keepalive()
        d = link.read()
        if d:
            buf += d
            sys.stdout.write(d.decode(errors="replace"))
            sys.stdout.flush()
            if until and any(u in buf for u in until):
                break
        else:
            if getattr(link, "closed", False):
                log("the board hung up")
                break
            time.sleep(0.02)
    return buf


ACK_TIMEOUT_S = tool_words.TIMEOUT_S
# Whether this board answers commands at all: firmware before 2026-10-14 does not, and its
# silence is then not a fault.
ANSWERS_SEEN = [False]


def answer(link, line: str, timeout: float = ACK_TIMEOUT_S):
    """Send one line and wait for the board's answer to it (scripts/tool_answer.py): every
    line the shell receives is answered `ok:` or `fail:`. Returns (answer or None, the output
    read meanwhile, which is also printed)."""
    link.write(line.encode() + b"\r\n")
    buf = b""
    end = time.time() + timeout
    while time.time() < end:
        link.keepalive()
        d = link.read()
        if not d:
            if getattr(link, "closed", False):
                break
            time.sleep(0.02)
            continue
        buf += d
        sys.stdout.write(d.decode(errors="replace"))
        sys.stdout.flush()
        for raw in buf.split(b"\n")[:-1]:
            a = tool_words.parse(raw.decode(errors="replace").replace("\r", ""))
            if a and a["kind"] in ("ok", "fail"):
                ANSWERS_SEEN[0] = True
                if a["line"] == line.strip():
                    return a, buf
    return None, buf


def say_answer(line: str, a) -> None:
    if a is None and not ANSWERS_SEEN[0]:
        log(f"'{line}': sent (this firmware does not answer commands: flash the MCU for answers)")
    elif a is None:
        log(f"'{line}': NO ANSWER from the board in {ACK_TIMEOUT_S:.0f} s (the link is down?)")
    else:
        log(tool_words.words(a))


def catch_window(link, seconds: float) -> bool:
    """Type `tool` until the board says it is in the tool (its window is 3 s)."""
    buf = b""
    end = time.time() + seconds
    nxt = 0.0
    while time.time() < end:
        if time.time() >= nxt:
            link.write(b"tool\r\n")
            nxt = time.time() + 0.25
        d = link.read()
        if d:
            buf += d
            sys.stdout.write(d.decode(errors="replace"))
            sys.stdout.flush()
            if any(e in buf for e in ENTERED):
                pump(link, 4, until=(PROMPT,))
                return True
    return False


def run_commands(link, commands, answers: list = None) -> bool:
    """Send each and wait for its answer; returns whether something that drives may still
    be running. Each command's answer ({line, ok, msg} or None) is appended to `answers`."""
    driving = False
    for cmd in commands:
        words = cmd.split()
        if words[:1] == ["sleep"]:
            if len(words) != 2:
                log("sleep <seconds>")
                continue
            pump(link, float(words[1]))
            continue
        a, got = answer(link, cmd)
        say_answer(cmd, a)
        if answers is not None:
            answers.append(a)
        head = words[0] if words else ""
        if a is not None and not a.get("ok"):
            pump(link, 0.5, until=(PROMPT,))     # refused: nothing started
            continue
        if not words or head in ("stop", "s"):
            driving = False
            pump(link, 2.0, until=(PROMPT,))
        elif head in LONG:
            got = pump(link, LONG[head], until=(PROMPT,))
            # Still driving unless the prompt came back: then the `finally` stops it.
            driving = head == "acc" and PROMPT not in got
        elif head in RUNNING or (head == "i2c" and len(words) > 1):
            driving = driving or head in ("motor", "motors")
            pump(link, 2.0)
        else:
            pump(link, 30.0, until=(PROMPT,))
    return driving


def interactive(link) -> None:
    """stdin to the board, the board to stdout. Ctrl-] leaves."""
    import select
    import termios
    import tty
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    log("terminal: Ctrl-] leaves")
    try:
        tty.setcbreak(fd)
        while not getattr(link, "closed", False):
            link.keepalive()
            r, _, _ = select.select([fd], [], [], 0.02)
            if r:
                ch = os.read(fd, 64)
                if b"\x1d" in ch:
                    break
                link.write(ch.replace(b"\n", b"\r"))
            d = link.read()
            if d:
                sys.stdout.write(d.decode(errors="replace"))
                sys.stdout.flush()
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def connect(port, env, baud, params_path, ip, password, env_vals, no_reboot=False, window=15.0):
    """The board's tool, entered and at its prompt: a SerialLink on `port` (rebooted into the
    window unless no_reboot), else a TcpLink to `ip` -- or, without one, to the address
    `params_path`'s robot answers pings at. None (with the reason logged) when not reached.
    robot_bringup.py drives the tool through this too."""
    if port:
        if not env and not no_reboot:
            log("--env is needed to reboot the board over USB (esp32, esp32s3, pico, pico2 ...)")
            return None
        link = SerialLink(port, baud, env or "")
        if no_reboot:
            if not link.open():
                log(f"cannot open {port}")
                return None
            link.write(b"\r\n")
            pump(link, 1, until=(PROMPT,))
        else:
            log(f"rebooting the board on {port} into its tool window")
            if not link.reboot():
                log("the reboot failed: nothing to catch")
                link.close()
                return None
            if not catch_window(link, window):
                log("the board did not enter the tool (is tool_wait 0, or is this firmware older?)")
                link.close()
                return None
        return link
    if not ip and params_path:
        import mcu_env
        import wifi_boards
        params = mcu_env.load_yaml(params_path)
        robot = mcu_env.robot_name_of(params, params_path)
        _, entry = wifi_boards.find_robot_board(
            robot, port=int(env_vals.get("syslog_port") or wifi_boards.syslog_port()),
            ping_port=int(env_vals.get("ping_port") or wifi_boards.PING_PORT),
            mdns_name=mcu_env.robot_mdns_name(params, params_path),
            pinned_ip=mcu_env.robot_ip_for(params))
        ip = entry and entry.get("ip")
    if not ip:
        log("no board address: give --port, --ip, or --params of a robot this computer knows")
        return None
    ping_port = int(env_vals.get("ping_port") or 3233)
    if not no_reboot and not request_tool_udp(ip, ping_port, password):
        return None
    link = TcpLink(ip, int(env_vals.get("term_port") or 2323), password)
    if not link.open():
        log(f"no terminal at {ip}:{link.port}")
        return None
    link.ip, link.ping_port = ip, ping_port
    return link


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--port", help="the board's USB serial port (else Wi-Fi)")
    ap.add_argument("--env", help="PlatformIO env, for the reboot over USB: esp32, esp32s3, pico, pico2 ...")
    ap.add_argument("--baud", type=int, default=None, help="serial rate (default: the config's, else 921600)")
    ap.add_argument("--params", help="the robot config: baud, OTA password, ports, and on Wi-Fi its address")
    ap.add_argument("--ip", help="the board's address on Wi-Fi (else found from --params)")
    ap.add_argument("--password", default=None,
                    help="OTA password (default: the robot's secrets; on the command line it shows in ps)")
    ap.add_argument("--no-reboot", action="store_true", help="the board is already in the tool")
    ap.add_argument("-c", "--command", action="append", default=[], help="a tool command; repeat")
    ap.add_argument("--base", action="store_true", help="leave the tool when done (restart into base)")
    ap.add_argument("--window", type=float, default=15.0, help="seconds to catch the tool window")
    ap.add_argument("--stop", action="store_true",
                    help="stop the board's motors now and leave: lino-stop on Wi-Fi (no password), "
                         "`stop` on the console")
    args = ap.parse_args()

    env_vals = {}
    if args.params:
        import cockpit_paths
        import mcu_env
        env_vals = mcu_env.env_from_config(os.path.abspath(args.params), cockpit_paths.secrets_path())
    baud = args.baud or int(env_vals.get("baud") or 921600)
    if not args.env and args.params:
        import mcu_env
        tgt = (mcu_env.load_yaml(os.path.abspath(args.params)).get("base_controller") or {})
        args.env = str(tgt.get("pio_env") or tgt.get("mcu") or "")
    if args.port and args.port.endswith("ttyHS1") and not args.env:
        args.env = "unoq"

    # A SIGHUP (the ssh session that ran this went away) or a SIGTERM must still stop the
    # wheels: turn them into the KeyboardInterrupt the `finally` below handles.
    import signal
    def _bail(signum, frame):
        raise KeyboardInterrupt
    for sig in (signal.SIGHUP, signal.SIGTERM):
        signal.signal(sig, _bail)

    if args.stop:
        if args.port:
            link = SerialLink(args.port, baud, args.env or "")
            if not link.open(3):
                log(f"cannot open {args.port}")
                return 1
            link.write(b"stop\r\n")
            link.close()
            log("sent stop on the console")
            return 0
        ip = args.ip
        if not ip:
            log("--stop needs --port or --ip")
            return 2
        sk = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sk.settimeout(1.0)
        sk.sendto(b"lino-stop", (ip, int(env_vals.get("ping_port") or 3233)))
        try:
            log(sk.recvfrom(512)[0].decode(errors="replace").split(" linorobot2")[0])
        except socket.timeout:
            log("no answer to lino-stop")
            return 1
        return 0
    password = args.password if args.password is not None else str(env_vals.get("ota_password") or "")

    link = connect(args.port, args.env, baud, os.path.abspath(args.params) if args.params else None,
                   args.ip, password, env_vals, no_reboot=args.no_reboot, window=args.window)
    if link is None:
        return 1

    driving = True
    answers = []
    try:
        if args.command:
            driving = run_commands(link, args.command, answers)
        elif sys.stdin.isatty():
            interactive(link)
        if args.base:
            link.write(b"stop\r\n")
            pump(link, 0.5)
            link.write(b"base\r\n")
            pump(link, 3, until=(b"restarting into base",))
            driving = False
    finally:
        # Never leave a wheel turning: a closed serial port stops nothing on the board.
        if driving and not getattr(link, "closed", False):
            try:
                link.write(b"stop\r\n")
                pump(link, 1.0, until=(PROMPT,))
            except Exception:
                pass
        link.close()
    # A command the board refused, or never answered, is a failed run.
    refused = any(a is not None and not a.get("ok") for a in answers)
    silent = ANSWERS_SEEN[0] and any(a is None for a in answers)
    return 3 if refused or silent else 0


if __name__ == "__main__":
    sys.exit(main())
