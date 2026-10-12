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

    # Wi-Fi: "lino-tool" to the ping port (answered with a nonce when the board has an
    # OTA password: the reply is md5(password:nonce), as OTA does), then the TCP terminal
    python3 scripts/lino_tool.py --params robot_config.yaml -c "sensors" -c "sleep 3" -c stop

The commands (the tool's `help`): i2c [secs], sensors, motor <1-4|all> <pct> [secs],
motors [pct], acc [ms], adc, set <key> <value>, get [key], status, base. `stop`, `s` or an
empty line stops the running one.

Exit 0 when the tool was entered and every command was sent; the transcript is stdout.
"""
import argparse
import hashlib
import os
import socket
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

PROMPT = b"tool> "
ENTERED = (b"[app] tool", b"[tool] ready")


def log(msg):
    print(f"[lino_tool] {msg}", flush=True)


# ---------------------------------------------------------------- serial
def reboot_serial(env: str, port: str, baud: int) -> bool:
    """Restart the board so its tool window opens: EN on an ESP32's auto-reset circuit,
    `picotool reboot -f` on an RP2 (its USB reset interface), the USB reset otherwise."""
    import flash_mcu
    if env.startswith("esp32"):
        import mcu_probe
        return mcu_probe.esp32_boot_app(port, baud)
    if env.startswith("pico") or env.startswith("xrp"):
        if flash_mcu.rp2_reboot_into_app(env):
            return True
    return flash_mcu.usb_reset_target(port)


class SerialLink:
    def __init__(self, port: str, baud: int):
        import serial
        self.port, self.baud = port, baud
        self.s = None
        self.serial = serial

    def open(self, timeout: float = 15.0) -> bool:
        end = time.time() + timeout
        while time.time() < end:
            try:
                self.s = self.serial.Serial(self.port, self.baud, timeout=0.05)
                return True
            except Exception:
                time.sleep(0.1)       # an RP2's port is re-enumerating after the reboot
        return False

    def write(self, data: bytes):
        self.s.write(data)

    def read(self) -> bytes:
        try:
            return self.s.read(4096)
        except Exception:
            return b""

    def close(self):
        if self.s:
            self.s.close()


# ---------------------------------------------------------------- Wi-Fi
def tool_answer(password: str, nonce: str) -> str:
    return hashlib.md5(f"{password}:{nonce}".encode()).hexdigest()


def request_tool_udp(ip: str, ping_port: int, password: str, timeout: float = 2.0) -> bool:
    """"lino-tool", and the nonce's answer when the board asks for one."""
    sk = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sk.settimeout(timeout)
    try:
        sk.sendto(b"lino-tool", (ip, ping_port))
        reply = sk.recvfrom(512)[0].decode(errors="replace")
        if reply.startswith("lino-tool nonce "):
            if not password:
                log("the board has an OTA password and none was given (--password or --params)")
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
        got = self._until((b"password: ", PROMPT), 5)
        if got.endswith(b"password: "):
            self.s.sendall((self.password or "").encode() + b"\n")
            got = self._until((PROMPT, b"wrong password"), 5)
            if b"wrong password" in got or not got.endswith(PROMPT):
                log("the terminal refused the password")
                return False
        sys.stdout.write(got.decode(errors="replace"))
        return True

    def _until(self, endings, timeout):
        buf = b""
        end = time.time() + timeout
        while time.time() < end:
            buf += self.read()
            if any(buf.endswith(e) or e in buf for e in endings):
                return buf
        return buf

    def write(self, data: bytes):
        self.s.sendall(data)

    def read(self) -> bytes:
        try:
            d = self.s.recv(4096)
            return d if d else b""
        except (socket.timeout, BlockingIOError):
            return b""
        except OSError:
            return b""

    def close(self):
        if self.s:
            self.s.close()


# ---------------------------------------------------------------- driving it
def pump(link, seconds: float, out=sys.stdout, until=None) -> bytes:
    """Copy the board's output for `seconds` (or until a byte string appears)."""
    buf = b""
    end = time.time() + seconds
    while time.time() < end:
        d = link.read()
        if d:
            buf += d
            out.write(d.decode(errors="replace"))
            out.flush()
            if until and until in buf:
                break
        else:
            time.sleep(0.02)
    return buf


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
                pump(link, 4, until=PROMPT)
                return True
    return False


def run_commands(link, commands) -> None:
    for cmd in commands:
        if cmd.startswith("sleep "):
            pump(link, float(cmd.split()[1]))
            continue
        link.write(cmd.encode() + b"\r\n")
        # A command that ends prints the prompt again; one that runs (sensors, motor,
        # motors, i2c <secs>) does not, and the next entry is usually `sleep` or `stop`.
        pump(link, 2.0 if cmd.split()[0] in ("sensors", "motor", "motors") else 30.0,
             until=PROMPT)


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
        while True:
            r, _, _ = select.select([fd], [], [], 0.02)
            if r:
                ch = os.read(fd, 64)
                if b"\x1d" in ch:
                    break
                link.write(ch.replace(b"\n", b"\r\n"))
            d = link.read()
            if d:
                sys.stdout.write(d.decode(errors="replace"))
                sys.stdout.flush()
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--port", help="the board's USB serial port (else Wi-Fi)")
    ap.add_argument("--env", default="pico2", help="PlatformIO env: pico, pico2, esp32, esp32s3 ...")
    ap.add_argument("--baud", type=int, default=None, help="serial rate (default: the config's, else 921600)")
    ap.add_argument("--params", help="the robot config: baud, OTA password, ports, and on Wi-Fi its address")
    ap.add_argument("--ip", help="the board's address on Wi-Fi (else found from --params)")
    ap.add_argument("--password", default=None, help="OTA password (default: the robot's secrets)")
    ap.add_argument("--no-reboot", action="store_true", help="the board is already in the tool")
    ap.add_argument("-c", "--command", action="append", default=[], help="a tool command; repeat")
    ap.add_argument("--base", action="store_true", help="leave the tool when done (restart into base)")
    ap.add_argument("--window", type=float, default=15.0, help="seconds to catch the tool window")
    args = ap.parse_args()

    env_vals = {}
    if args.params:
        import cockpit_paths
        import mcu_env
        env_vals = mcu_env.env_from_config(os.path.abspath(args.params), cockpit_paths.secrets_path())
    baud = args.baud or int(env_vals.get("baud") or 921600)
    password = args.password if args.password is not None else str(env_vals.get("ota_password") or "")

    if args.port:
        link = SerialLink(args.port, baud)
        if not args.no_reboot:
            log(f"rebooting the board on {args.port} into its tool window")
            reboot_serial(args.env, args.port, baud)
        if not link.open():
            log(f"cannot open {args.port}")
            return 1
        if not args.no_reboot and not catch_window(link, args.window):
            log("the board did not enter the tool (is tool_wait 0, or is this firmware older?)")
            link.close()
            return 1
        if args.no_reboot:
            link.write(b"\r\n")
            pump(link, 1, until=PROMPT)
    else:
        ip = args.ip
        if not ip and args.params:
            import mcu_env
            import wifi_boards
            params = mcu_env.load_yaml(os.path.abspath(args.params))
            robot = mcu_env.robot_name_of(params, os.path.abspath(args.params))
            _, entry = wifi_boards.find_robot_board(
                robot, port=int(env_vals.get("syslog_port") or wifi_boards.syslog_port()),
                ping_port=int(env_vals.get("ping_port") or wifi_boards.PING_PORT),
                mdns_name=mcu_env.robot_mdns_name(params, os.path.abspath(args.params)),
                pinned_ip=mcu_env.robot_ip_for(params))
            ip = entry and entry.get("ip")
        if not ip:
            log("no board address: give --port, --ip, or --params of a robot this computer knows")
            return 1
        ping_port = int(env_vals.get("ping_port") or 3233)
        if not args.no_reboot and not request_tool_udp(ip, ping_port, password):
            return 1
        link = TcpLink(ip, int(env_vals.get("term_port") or 2323), password)
        if not link.open():
            log(f"no terminal at {ip}:{link.port}")
            return 1

    try:
        if args.command:
            run_commands(link, args.command)
        elif sys.stdin.isatty():
            interactive(link)
        if args.base:
            link.write(b"stop\r\n")
            time.sleep(0.3)
            link.write(b"base\r\n")
            pump(link, 2, until=b"restarting into base")
    finally:
        link.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
