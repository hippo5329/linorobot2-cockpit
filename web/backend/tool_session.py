"""The board's tool, held by the cockpit for the browser (2026-10-13, the web-designer review).

The firmware has one mode: `base` waits 3 s at boot for `tool` and then starts micro-ROS;
typed in time, the board stays in the tool, a command shell (firmware/src/tools). This module
holds ONE session per robot -- the serial link, or the Wi-Fi terminal behind its challenge
login -- and fans its output out to every browser tab that watches it.

Rules it keeps, each from the review:

  * The OTA password stays here. It is read from the robot's secrets on the server, used to
    answer the board's nonces, and appears in no response, no log line and no command line.
  * Commands are validated against the shell's grammar before a byte reaches the board: a
    known first word, integers in the firmware's own ranges, `set` limited to the tool's
    test settings, no line breaks or NULs inside a line.
  * Liveness crosses two hops. The board stops a drive started from a terminal that goes
    quiet for 3 s; this session sends it a keep-alive only while at least one browser is
    watching (a stream with a heartbeat in the last 3 s). When the last browser goes away
    while a command runs, the session sends `stop` -- and on Wi-Fi the `lino-stop` datagram
    too -- itself.
  * Entering the tool writes nothing: the board is restarted and catches its boot window.
    Leaving is `base`. The firmware boots base whatever its env's `app` says; a board whose
    banner names another app is an older firmware booting a tool from an env an older
    cockpit wrote, and `env_base_command()` is the env-only write that fixes it.

No FastAPI here: routes_tool.py wires it to HTTP, and the tests drive it with a fake link.
"""
import collections
import os
import sys
import re
import socket
import threading
import time
from typing import Callable, Dict, List, Optional

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))), "scripts"))
import tool_answer  # noqa: E402

PROMPT = "tool> "
ENTERED = ("[app] tool", "[tool] ready")
# "[app] tool" is printed before the tool sets up its hardware (tool_hw::init: a bus scan on a
# board, the models on the Sim MCU); only "[tool] ready" and the prompt say it reads lines. A
# page that sent its first command on "[app] tool" lost the answer to the 2 s window (the
# bring-up card's sensors step on the lyrical Sim MCU, 2026-10-16). lino_tool waits the same.
READY = "[tool] ready"
READY_GRACE_S = 30.0       # lino_tool.TOOL_READY_S: a firmware without the line is let in after it


def is_ready_line(ln: str) -> bool:
    return READY in ln or ln.endswith(PROMPT)
HEARTBEAT_S = 3.0          # a browser stream counts as watching for this long after a beat
KEEPALIVE_S = 0.25         # the NUL the board's deadman wants (2 s under acc, 3 s otherwise)
BACKLOG = 400              # lines a newly opened tab is shown

# ---------------------------------------------------------------------------
# The shell's grammar (firmware/src/tools/tools.cpp handleLine), as ranges.
SET_KEYS = {
    "tm_pct": (-100, 100),
    "tm_only": (1, 5),
    "tm_secs": (0, 3600),
    "ta_ms": (300, 1000),
    "ta_pct": (10, 100),
}
# An invert, written to the board's own env and in use at once (tools.cpp saveKey; user,
# 2026-10-18: "when we test motor, we want to inverse motor ... without host esptool/
# picotool/ota", "make inverts live, no reboot"). 1 on, 0 off, - back to the config's.
# Every other key `set` writes restarts the board: that stays on lino_tool.py's console.
INVERT_KEY = re.compile(r"^m[1-4]_(inv|enc_inv)$")
# Commands that keep running until `stop` (or their own end), and the drives among them.
# The board answers every line it receives, in words (scripts/tool_answer.py):
#   ok: motor 1 40 3 (running)   fail: motor 3 40 -- why   done: ...   stopped: ... (why)
ACK_TIMEOUT_S = tool_answer.TIMEOUT_S
RUNNING = {"sensors", "enc", "motor", "motors", "ramp", "acc", "adc", "i2c"}
DRIVES = {"motor", "motors", "ramp", "acc"}
_LINE_OK = re.compile(r"^[ -~]{0,64}$")       # printable ASCII, no CR/LF/NUL, short


class Refused(ValueError):
    """A line the shell's grammar does not allow. The message says why, for the page."""


def _int(word: str, lo: int, hi: int, what: str) -> int:
    try:
        n = int(word)
    except ValueError:
        raise Refused(f"{what}: '{word}' is not a whole number")
    if n < lo or n > hi:
        raise Refused(f"{what}: {n} is outside {lo}..{hi}")
    return n


def validate(line: str) -> str:
    """The line as it will be sent, or Refused. Mirrors the firmware's own checks so a
    page never sends what the board would refuse -- and never sends what it should not."""
    if not isinstance(line, str) or not _LINE_OK.match(line):
        raise Refused("one line of plain text, at most 64 characters")
    words = line.split()
    if not words:
        raise Refused("an empty line stops -- use Stop")
    cmd, args = words[0].lower(), words[1:]
    if cmd in ("stop", "s", "status", "help", "base", "enc", "adc"):
        if args:
            raise Refused(f"{cmd} takes no arguments")
    elif cmd == "sensors":
        if len(args) > 1:
            raise Refused("sensors [hz]")
        if args:
            _int(args[0], 1, 20, "sensors hz")
    elif cmd == "i2c":
        if len(args) > 1:
            raise Refused("i2c [secs]")
        if args:
            _int(args[0], 1, 3600, "i2c secs")
    elif cmd == "motor":
        if args:
            if len(args) not in (2, 3):
                raise Refused("motor <1-4|all> <pct -100..100, not 0> [secs]")
            if args[0] != "all":
                _int(args[0], 1, 4, "wheel")
            if _int(args[1], -100, 100, "power %") == 0:
                raise Refused("power %: 0 turns nothing -- use Stop")
            if len(args) == 3:
                _int(args[2], 0, 3600, "secs")
    elif cmd == "ramp":
        if len(args) not in (1, 2):
            raise Refused("ramp <wheel> [max pct 5..100]")
        _int(args[0], 1, 4, "wheel")
        if len(args) == 2:
            _int(args[1], 5, 100, "max power %")
    elif cmd == "motors":
        if len(args) > 1:
            raise Refused("motors [pct]")
        if args:
            _int(args[0], 1, 100, "power %")
    elif cmd == "acc":
        if len(args) > 2:
            raise Refused("acc [ms] [pct]")
        if args:
            _int(args[0], 300, 1000, "run phase ms")
        if len(args) == 2:
            _int(args[1], 10, 100, "power cap %")
    elif cmd == "set":
        if not args or len(args) > 2:
            raise Refused("set <key> <value>")
        key = args[0]
        if INVERT_KEY.match(key):
            if len(args) != 2 or args[1] not in ("0", "1", "-"):
                raise Refused(f"set {key} 1|0|- (an invert: written to the board's env, in use at once)")
            return " ".join([cmd] + args)
        if key not in SET_KEYS:
            raise Refused(f"set: {key} is not a test setting ({', '.join(sorted(SET_KEYS))}) "
                          "or an invert (m1_inv .. m4_enc_inv)")
        if len(args) == 2:
            lo, hi = SET_KEYS[key]
            _int(args[1], lo, hi, key)
    elif cmd == "get":
        if len(args) > 1 or (args and not re.match(r"^[a-z0-9_]{1,23}$", args[0])):
            raise Refused("get [key]")
    else:
        raise Refused(f"'{cmd}' is not a tool command")
    return " ".join([cmd] + args)


# ---------------------------------------------------------------------------
class ToolSession:
    """One robot's tool, open. `link` has write(bytes), read() -> bytes (non-blocking or
    with a short timeout) and close(); lino_tool.py's SerialLink and TcpLink are two."""

    def __init__(self, robot: str, link, kind: str, ip: Optional[str] = None,
                 ping_port: int = 3233, clock: Callable[[], float] = time.monotonic,
                 send_datagram: Optional[Callable[[str, int, bytes], None]] = None):
        self.robot = robot
        self.link = link
        self.kind = kind                    # "serial" | "wifi"
        self.ip = ip
        self.ping_port = ping_port
        self._clock = clock
        self._send_datagram = send_datagram or _udp_send
        self._lock = threading.RLock()
        self._lines = collections.deque(maxlen=BACKLOG)
        self._partial = ""
        self._watchers: Dict[int, dict] = {}
        self._next_id = 1
        self._last_keepalive = 0.0
        self.running: Optional[str] = None  # the command running, by its first word
        self.banner_app: Optional[str] = None
        self.closed = False
        self._stopped_unwatched = False
        # Command and response: every line sent waits for the board's answer (ok / fail).
        self._seq = 0
        self.last_seq = 0
        self._pending: List[dict] = []
        self._answers: Dict[int, dict] = {}
        self._answered = threading.Condition(self._lock)
        self.acks_seen = False
        self.last_answer: Optional[dict] = None

    # -- public state, never the password ----------------------------------
    def describe(self) -> dict:
        with self._lock:
            return {"robot": self.robot, "kind": self.kind, "running": self.running,
                    "watchers": len(self._watchers), "closed": self.closed,
                    "env_app": self.banner_app, "last_answer": self.last_answer,
                    "answers": self.acks_seen}

    # -- the board's output --------------------------------------------------
    def feed(self, data: bytes) -> List[str]:
        """Bytes from the board: complete lines go to the backlog and every watcher. A
        prompt (no newline after it) counts as a line, so a page sees it at once."""
        text = self._partial + data.decode("utf-8", errors="replace").replace("\r", "")
        out = []
        while True:
            nl = text.find("\n")
            pr = text.find(PROMPT)
            if nl < 0 and pr < 0:
                break
            if pr >= 0 and (nl < 0 or pr < nl):
                cut = pr + len(PROMPT)
            else:
                cut = nl + 1
            out.append(text[:cut].rstrip("\n"))
            text = text[cut:]
        self._partial = text[-512:]
        with self._lock:
            for ln in out:
                self._note(ln)
                self._lines.append(ln)
                for w in self._watchers.values():
                    w["queue"].append(ln)
        return out

    def _note(self, ln: str) -> None:
        a = tool_answer.parse(ln)
        if a and a["kind"] in ("ok", "fail"):
            self._on_ack(a)
        m = re.search(r"linorobot2_hardware app=(\S+)", ln)
        if m:
            self.banner_app = m.group(1)
        if a and a["kind"] == "fail" and a["line"].split()[:1] == [self.running]:
            self.running = None             # refused at its start, or ended without its work
        if ln.endswith(PROMPT) or (a and a["kind"] in ("done", "stopped")) or "[tool] stopped" in ln \
                or "[test_acc] STOPPED" in ln or re.search(r"\[tool\] \w+ done", ln):
            self.running = None

    def _on_ack(self, a: dict) -> None:
        """The board's answer to a line: matched to the oldest line sent with that text (the
        shell answers in order). An answer that matches no line waiting is nobody's: the
        session's own `stop` (a page went away) or `base`. It used to fall back to the oldest
        line waiting, so the answer to an internal stop was handed to a user's motor command
        as its own. Lock held (feed)."""
        self.acks_seen = True
        p = next((x for x in self._pending if x["line"] == a.get("line")), None)
        ans = {"kind": a["kind"], "line": a["line"], "ok": a["ok"], "msg": a["msg"]}
        if p is None:
            return                          # an internal stop or leave: nobody waits
        self._pending.remove(p)
        ans["seq"] = p["seq"]
        self._answers[p["seq"]] = ans
        self.last_answer = ans
        if not ans["ok"] and self.running == p["line"].split()[0]:
            self.running = None             # refused: nothing runs
        self._answered.notify_all()

    def wait_answer(self, seq: int, timeout: float = None) -> Optional[dict]:
        """The board's answer to line `seq`: {ok, msg, line}; {ok: None} when none came in
        time (ACK_TIMEOUT_S, measured from the send)."""
        timeout = ACK_TIMEOUT_S if timeout is None else timeout
        end = time.monotonic() + timeout
        with self._answered:
            while seq not in self._answers and not self.closed:
                left = end - time.monotonic()
                if left <= 0:
                    break
                self._answered.wait(left)
            return self._answers.get(seq)

    def _expire(self) -> None:
        """Lines with no answer in ACK_TIMEOUT_S: said so in the terminal, and answered
        {ok: None} for whoever waits."""
        now = self._clock()
        with self._lock:
            for p in [x for x in self._pending if now - x["at"] > ACK_TIMEOUT_S]:
                self._pending.remove(p)
                why = (f"no answer: {p['line']} (nothing from the board in {ACK_TIMEOUT_S:.0f} s"
                       + (" -- check the link)" if self.acks_seen else " -- its firmware may predate "
                          "command answers: flash the MCU; or the link is down)"))
                ans = {"seq": p["seq"], "kind": "none", "line": p["line"], "ok": None, "msg": "no answer"}
                self._answers[p["seq"]] = ans
                self.last_answer = ans
                self._lines.append(why)
                for w in self._watchers.values():
                    w["queue"].append(why)
                self._answered.notify_all()

    def pump(self) -> None:
        """One read from the link (the reader thread calls this in a loop)."""
        d = self.link.read()
        if d:
            self.feed(d)
        if getattr(self.link, "closed", False) and not self.closed:
            self.feed(b"[cockpit] the board closed the terminal\n")
            self.close()

    # -- watchers (browser streams) -------------------------------------------
    def watch(self) -> int:
        with self._lock:
            wid = self._next_id
            self._next_id += 1
            self._watchers[wid] = {"queue": collections.deque(list(self._lines)),
                                   "beat": self._clock()}
            self._stopped_unwatched = False
            return wid

    def beat(self, wid: int) -> bool:
        with self._lock:
            w = self._watchers.get(wid)
            if not w:
                return False
            w["beat"] = self._clock()
            return True

    def unwatch(self, wid: int) -> None:
        with self._lock:
            self._watchers.pop(wid, None)

    def drain(self, wid: int) -> List[str]:
        with self._lock:
            w = self._watchers.get(wid)
            if not w:
                return []
            q = w["queue"]
            out = list(q)
            q.clear()
            return out

    def watched(self) -> bool:
        now = self._clock()
        with self._lock:
            return any(now - w["beat"] <= HEARTBEAT_S for w in self._watchers.values())

    # -- liveness ---------------------------------------------------------------
    def tick(self) -> None:
        """Once a second: the board's keep-alive while a browser watches; a stop when the
        last one went away with a command running; lines the board never answered."""
        self._expire()
        now = self._clock()
        if self.watched():
            self._stopped_unwatched = False
            if now - self._last_keepalive >= KEEPALIVE_S:
                self._last_keepalive = now
                self._write(b"\0")
        elif self.running and not self._stopped_unwatched:
            self._stopped_unwatched = True
            self.stop()

    # -- commands ---------------------------------------------------------------
    def send(self, line: str):
        """Send one validated line: (the line as sent, its seq for wait_answer). The seq is
        returned rather than read back from last_seq afterwards, which another request's
        send() may already have moved on."""
        clean = validate(line)
        word = clean.split()[0]
        # `s` goes out as `stop` (stop() below), so the board answers `stop`.
        waits_for = "stop" if word in ("stop", "s") else clean
        with self._lock:
            self._seq += 1
            seq = self._seq
            self.last_seq = seq
            self._pending.append({"seq": seq, "line": waits_for, "at": self._clock()})
            del self._pending[:-32]
            if len(self._answers) > 64:
                for k in sorted(self._answers)[:-32]:
                    self._answers.pop(k, None)
        if word in ("stop", "s"):
            self.stop()
            return clean, seq
        self._write(clean.encode() + b"\r\n")
        if word in RUNNING:
            self.running = word
        return clean, seq

    def stop(self) -> None:
        """`stop` on the shell, and on Wi-Fi the ping port's lino-stop as well: it needs no
        login and reaches the board even when the terminal does not."""
        self._write(b"stop\r\n")
        if self.kind == "wifi" and self.ip:
            try:
                self._send_datagram(self.ip, self.ping_port, b"lino-stop")
            except OSError:
                pass

    def leave(self) -> None:
        """Back to the robot: stop anything running, then `base` (a restart into micro-ROS)."""
        self._write(b"stop\r\n")
        self._write(b"base\r\n")

    def close(self) -> None:
        with self._lock:
            self.closed = True
            self._watchers.clear()
        try:
            self.link.close()
        except Exception:
            pass

    def needs_env_base(self) -> bool:
        """The board's banner names an app other than base: an older firmware that boots
        the tool from its env (an older cockpit wrote a tool's name into `app`) would boot it
        again at every power-on until `app=base` is written."""
        return bool(self.banner_app) and self.banner_app != "base"

    def _write(self, data: bytes) -> None:
        with self._lock:
            if self.closed:
                return
            try:
                self.link.write(data)
            except Exception:
                pass


def _udp_send(ip: str, port: int, data: bytes) -> None:
    sk = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sk.sendto(data, (ip, port))
    finally:
        sk.close()


# ---------------------------------------------------------------------------
# The registry: one session per robot.
class Registry:
    def __init__(self):
        self._lock = threading.Lock()
        self._sessions: Dict[str, ToolSession] = {}
        self._threads: Dict[str, threading.Thread] = {}

    def get(self, robot: str) -> Optional[ToolSession]:
        with self._lock:
            s = self._sessions.get(robot)
            return s if s and not s.closed else None

    def add(self, session: ToolSession, start_threads: bool = True) -> ToolSession:
        with self._lock:
            old = self._sessions.get(session.robot)
            if old and not old.closed:
                raise RuntimeError(f"a tool session for {session.robot} is already open")
            self._sessions[session.robot] = session
        if start_threads:
            t = threading.Thread(target=_serve, args=(session,), daemon=True,
                                 name=f"tool-{session.robot}")
            self._threads[session.robot] = t
            t.start()
        return session

    def open_robots(self) -> List[str]:
        """The robots with a session open: the 1-Click refuses while any is."""
        with self._lock:
            return sorted(r for r, s in self._sessions.items() if not s.closed)

    def remove(self, robot: str) -> None:
        with self._lock:
            s = self._sessions.pop(robot, None)
        if s:
            s.close()


def _serve(session: ToolSession) -> None:
    """Read the board, and tick the liveness rules, until the session closes."""
    last_tick = 0.0
    while not session.closed:
        try:
            session.pump()
        except Exception:
            time.sleep(0.05)
        now = time.monotonic()
        if now - last_tick >= 0.25:
            last_tick = now
            session.tick()
        time.sleep(0.01)


REGISTRY = Registry()
