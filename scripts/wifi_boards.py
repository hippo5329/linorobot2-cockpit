#!/usr/bin/env python3
"""Where the Wi-Fi robots are: boards heard on syslog, and which robot each one is.

A Wi-Fi robot (ESP32 / ESP32-S3 on the udp4 transport) is flashed over USB once and
then leaves the computer. From then on it has no tty, and the only things that say
it exists are what it sends: its syslog lines -- among them the firmware banner,
every 2 s for the first 30 s after a boot and every 60 s after that -- and its
micro-ROS session with the agent -- and what it answers when asked: a UDP datagram
"lino?" to its ping port (3233) is answered with the same banner line (ping()). This
file keeps what is heard in one place:

    <config dir>/state/wifi_boards.json
      boards: { <uid>: {ip, seen, app, distro, git, built, envota, envcrc} }
      robots: { <robot name>: {uid, env, since} }

`boards` is written by whoever hears a banner -- the cockpit's syslog receiver
(web/backend/syslog_manager.py) or, when the cockpit is not running, listen() here.
`robots` is written by flash_mcu.py when it flashes a Wi-Fi robot over USB: that is
the one moment the host knows for certain which board a robot is (the banner came
back over the cable), and every later write over the air checks the board it is
about to write against it.

The state directory, not $HOME: the pipeline runs as container-root and the
backend as the container user, and both must see the same file (cockpit_paths).
"""
import json
import os
import re
import socket
import struct
import tempfile
import time

import cockpit_paths
import mcu_probe

STATE_NAME = "wifi_boards.json"
# Present = a banner within this long. The board repeats it every 60 s.
FRESH_S = 150
_EXTRA_RE = {
    "envota": re.compile(r"\benvota=(\d)"),
    "envcrc": re.compile(r"\benvcrc=([0-9A-Fa-f]{8})"),
}


def state_path() -> str:
    return os.path.join(cockpit_paths.state_dir(), STATE_NAME)


def load() -> dict:
    try:
        with open(state_path(), encoding="utf-8") as fh:
            data = json.load(fh)
    except (OSError, ValueError):
        data = {}
    data.setdefault("boards", {})
    data.setdefault("robots", {})
    return data


def _save(data: dict) -> None:
    """Atomically: two writers (the backend and a pipeline) may race, and a
    half-written file would lose every robot's identity."""
    path = state_path()
    fd, tmp = tempfile.mkstemp(dir=os.path.dirname(path), prefix=".wifi_boards.")
    with os.fdopen(fd, "w", encoding="utf-8") as fh:
        json.dump(data, fh, indent=1, sort_keys=True)
    os.chmod(tmp, 0o664)
    os.replace(tmp, path)


def parse(line: str) -> dict:
    """The banner fields in one syslog line, or {} when it carries no banner (or
    no identity, which an ESP32 always prints and which everything here keys on)."""
    b = mcu_probe.parse_banner(line)
    if not b or b.get("id_kind") != "uid" or not b.get("board_id"):
        return {}
    out = {k: b.get(k) for k in ("app", "distro", "git", "built")}
    out["uid"] = b["board_id"].upper()
    for k, rx in _EXTRA_RE.items():
        m = rx.search(line)
        out[k] = m.group(1).upper() if m else None
    out["envota"] = out["envota"] == "1"
    return out


def record_banner(line: str, ip: str, now: float = None) -> dict:
    """Note a banner heard from `ip`. Returns the parsed banner ({} if none)."""
    b = parse(line)
    if not b:
        return {}
    data = load()
    entry = dict(b)
    entry.pop("uid")
    entry.update(ip=ip, seen=now or time.time())
    data["boards"][b["uid"]] = entry
    _save(data)
    return b


def remember_robot(robot: str, uid: str, env: str, ip: str = None) -> None:
    """Robot `robot` is the board `uid` (heard over USB at its flash), last at `ip`
    when its banner named one (`ip=`, once its Wi-Fi is up)."""
    if not robot or not uid:
        return
    data = load()
    entry = {"uid": uid.upper(), "env": env, "since": time.time()}
    if ip:
        entry["ip"] = ip
    data["robots"][robot] = entry
    _save(data)


def robot_ip(robot: str) -> str:
    """Where the robot was last: its board's latest address, else the one its
    USB flash recorded."""
    r = load()["robots"].get(robot) or {}
    return board(r.get("uid", "")).get("ip") or r.get("ip") or ""


# ---------------------------------------------------------------------- mDNS

def _dns_name(name: str) -> bytes:
    return b"".join(bytes([len(p)]) + p.encode() for p in name.split(".") if p) + b"\0"


def _read_name(pkt: bytes, off: int) -> tuple:
    """(name, offset after it) for a DNS name at `off`, following compression."""
    labels, jumped, end = [], False, off
    for _ in range(64):
        n = pkt[off]
        if n == 0:
            off += 1
            break
        if n & 0xC0 == 0xC0:                      # a pointer
            if not jumped:
                end = off + 2
            off, jumped = ((n & 0x3F) << 8) | pkt[off + 1], True
            continue
        labels.append(pkt[off + 1:off + 1 + n].decode(errors="replace"))
        off += 1 + n
    return ".".join(labels).lower(), (end if jumped else off)


def parse_mdns_a(pkt: bytes, host: str) -> str:
    """The IPv4 address an mDNS response gives for `host`, or ""."""
    try:
        _, _, qd, an, ns, ar = struct.unpack(">HHHHHH", pkt[:12])
        off = 12
        for _ in range(qd):
            _, off = _read_name(pkt, off)
            off += 4
        for _ in range(an + ns + ar):
            name, off = _read_name(pkt, off)
            rtype, _, _, rdlen = struct.unpack(">HHIH", pkt[off:off + 10])
            off += 10
            if rtype == 1 and rdlen == 4 and name == host.lower():
                return socket.inet_ntoa(pkt[off:off + 4])
            off += rdlen
    except (struct.error, IndexError):
        pass
    return ""


def mdns_resolve(label: str, timeout: float = 1.5) -> str:
    """`<label>.local` -> IPv4, asked over mDNS directly (no avahi needed in the
    container). A one-shot query from an ordinary port: responders answer it by
    unicast, so no multicast group is joined. "" when nothing answered."""
    if not label:
        return ""
    host = f"{label}.local"
    query = struct.pack(">HHHHHH", 0, 0, 1, 0, 0, 0) + _dns_name(host) + struct.pack(">HH", 1, 1)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 255)
    try:
        deadline = time.time() + timeout
        for _ in range(2):
            sock.sendto(query, ("224.0.0.251", 5353))
            round_end = min(deadline, time.time() + timeout / 2)
            while time.time() < round_end:
                sock.settimeout(max(0.05, round_end - time.time()))
                try:
                    pkt, _ = sock.recvfrom(1500)
                except (socket.timeout, OSError):
                    break
                ip = parse_mdns_a(pkt, host)
                if ip:
                    return ip
    finally:
        sock.close()
    return ""


def robot_uid(robot: str) -> str:
    return (load()["robots"].get(robot) or {}).get("uid") or ""


def board(uid: str) -> dict:
    return dict(load()["boards"].get((uid or "").upper()) or {})


def fresh(entry: dict, now: float = None, within: float = FRESH_S) -> bool:
    return bool(entry) and (now or time.time()) - float(entry.get("seen") or 0) <= within


def heard_boards(within: float = FRESH_S) -> dict:
    """{uid: entry} for every board heard recently -- what a "which board?" error lists."""
    now = time.time()
    return {u: e for u, e in load()["boards"].items() if fresh(e, now, within)}


def syslog_port() -> int:
    """The port boards send to and the cockpit's syslog server listens on: the
    shared telemetry.syslog_port, else 5140 -- resolved the one way the board's env
    and the backend both resolve it (mcu_env.resolve_syslog_port). The cockpit starts
    its server at startup, so when it runs it holds this port and records every
    banner itself; listen() here is for a run without the cockpit."""
    import mcu_env
    sec = mcu_env.load_yaml(cockpit_paths.secrets_path())
    return int(mcu_env.resolve_syslog_port((sec.get("telemetry") or {}).get("syslog_port")))


def listen(seconds: float, port: int = None) -> int:
    """Listen for banners ourselves for up to `seconds`, recording what is heard.
    For a pipeline or flasher run with no cockpit backend holding the syslog port.
    Returns how many banners were heard; 0 at once when the port is taken (the
    backend is running and records them itself)."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.bind(("0.0.0.0", port or syslog_port()))
    except OSError:
        sock.close()
        return 0
    heard = 0
    deadline = time.time() + seconds
    try:
        while time.time() < deadline:
            sock.settimeout(max(0.1, deadline - time.time()))
            try:
                data, addr = sock.recvfrom(4096)
            except socket.timeout:
                break
            if record_banner(data.decode("utf-8", errors="replace"), addr[0]):
                heard += 1
    finally:
        sock.close()
    return heard


PING_PORT = 3233


def ping(ip: str = None, port: int = PING_PORT, timeout: float = 1.0) -> dict:
    """Ask for banners: unicast to `ip`, or broadcast on the local network when `ip`
    is None. Records every answer and returns {uid: entry} for the boards that
    answered. A board answers from its control loop in milliseconds; the timeout is
    split over two rounds, because the radio drops the odd datagram."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    answered = {}
    try:
        for _ in range(2):
            try:
                sock.sendto(b"lino?\n", (ip or "255.255.255.255", port))
            except OSError:
                break
            round_end = time.time() + timeout / 2
            while True:
                left = round_end - time.time()
                if left <= 0:
                    break
                sock.settimeout(left)
                try:
                    data, addr = sock.recvfrom(512)
                except (socket.timeout, OSError):
                    break
                b = record_banner(data.decode("utf-8", errors="replace"), addr[0])
                if b:
                    answered[b["uid"]] = board(b["uid"])
            if answered and ip:
                break           # a unicast has its answer; a broadcast keeps collecting
    finally:
        sock.close()
    return answered


def wait_for(uid: str, accept, timeout: float, since: float = 0.0, port: int = None,
             ping_port: int = PING_PORT) -> dict:
    """Wait until board `uid` sends a banner newer than `since` that `accept(entry)`
    takes. Reads the shared file (the backend writes it) and, when the backend is
    not running, listens itself. Returns the entry, or {} on timeout."""
    deadline = time.time() + timeout
    while True:
        entry = board(uid)
        if entry and float(entry.get("seen") or 0) > since and accept(entry):
            return entry
        left = deadline - time.time()
        if left <= 0:
            return {}
        # Ask: the board answers as soon as it is back on the network, where a
        # syslog banner could be a minute away. Its address can change across a
        # reboot (DHCP), so a broadcast when the last one does not answer.
        if not ping(entry.get("ip") if entry else None, ping_port, timeout=1.0) and entry:
            ping(None, ping_port, timeout=1.0)
        if not listen(min(1.0, max(0.0, deadline - time.time())), port):
            time.sleep(min(0.5, max(0.0, deadline - time.time())))


def find_robot_board(robot: str, within: float = FRESH_S, port: int = None,
                     ping_port: int = PING_PORT, mdns_name: str = None,
                     pinned_ip: str = None) -> tuple:
    """(uid, entry) of robot `robot`'s board, asked fresh: a ping to the address it
    was last at, then to `<robot>.local` (mDNS), then a broadcast, then a few seconds
    of syslog -- only an answer given now, as this uid, counts. (uid, {}) when the
    board is known but did not answer; ("", {}) when this host never flashed it."""
    uid = robot_uid(robot)
    if not uid:
        return "", {}
    t0 = time.time()
    # Where it was last, then where its name says it is, then anyone who answers.
    # The robot IP its config pins (base_controller.robot_ip), where it was last, where its
    # name says it is (mDNS), then anyone who answers.
    tried = set()
    for ip in (pinned_ip, robot_ip(robot)):
        if ip and ip not in tried:
            tried.add(ip)
            if uid in ping(ip, ping_port):
                return uid, board(uid)
    import mcu_env
    named = mdns_resolve(mcu_env.mdns_hostname(mdns_name or robot))
    if named and named not in tried and uid in ping(named, ping_port):
        return uid, board(uid)
    if uid in ping(None, ping_port):
        return uid, board(uid)
    # Nothing answered. Only a banner heard from now on counts -- a record from
    # before the question is not evidence that the board is there: a parked board
    # (radio off) was "heard 62 s ago" and the write then went to a silent address,
    # and a stale address can belong to ANOTHER board by now. A short listen catches
    # a board that is just booting (it sends its banner every 2 s for 30 s).
    listen(6.0, port)
    entry = board(uid)
    return uid, (entry if float(entry.get("seen") or 0) >= t0 else {})


# ------------------------------------------------------------------ the terminal

_SYSLOG_HEAD = re.compile(r"^(?:<\d+>)?1 - \S+ \S+ - - - ﻿?")


def _clean(text: str) -> str:
    """The message of an RFC 5424 line as the firmware formats it, header dropped."""
    return _SYSLOG_HEAD.sub("", text.strip())


def follow(ip: str, port: int = None, log_dir: str = None, out=None):
    """Stream board `ip`'s syslog lines until interrupted -- a diagnostic tool's
    console on a robot that is off the USB cable (the firmware tees a tool's every
    line to syslog). Listens on the syslog port itself; when the cockpit backend
    holds that port, follows the backend's day log instead, which is the same lines."""
    import sys
    out = out or sys.stdout
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.bind(("0.0.0.0", port or syslog_port()))
    except OSError:
        sock.close()
        sock = None
    if sock:
        while True:
            data, addr = sock.recvfrom(4096)
            text = data.decode("utf-8", errors="replace")
            record_banner(text, addr[0])
            if addr[0] == ip:
                print(_clean(text), file=out, flush=True)
    log_dir = log_dir or os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "logs")
    tag = f"] [{ip}:"
    fh, path = None, None
    while True:
        want = os.path.join(log_dir, time.strftime("syslog_%Y%m%d.log"))
        if want != path:                     # a new day's file
            if fh:
                fh.close()
            path = want
            fh = open(path, encoding="utf-8", errors="replace") if os.path.exists(path) else None
            if fh:
                fh.seek(0, os.SEEK_END)
        line = fh.readline() if fh else ""
        if not line:
            time.sleep(0.2)
            if not fh and os.path.exists(path):
                path = None
            continue
        if tag in line:
            print(_clean(line.split("] ", 2)[-1]), file=out, flush=True)


def main() -> int:
    import argparse
    import sys
    import mcu_env
    ap = argparse.ArgumentParser(description="Wi-Fi robots off the USB cable: ask, list, follow.")
    sub = ap.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("ping", help="ask for banners (broadcast, or one address)")
    p.add_argument("ip", nargs="?")
    p.add_argument("--port", type=int, default=PING_PORT)
    m = sub.add_parser("monitor", help="stream a robot's syslog (a tool's console)")
    m.add_argument("params", help="the robot config (its name finds the board)")
    a = ap.parse_args()
    if a.cmd == "ping":
        found = ping(a.ip, a.port, timeout=1.5)
        robots = {v.get("uid"): k for k, v in load()["robots"].items()}
        for uid, e in sorted(found.items()):
            print(f"{uid}  {e.get('ip'):15}  app={e.get('app')} distro={e.get('distro')} "
                  f"git={e.get('git')} envota={int(bool(e.get('envota')))} "
                  f"robot={robots.get(uid, '-')}")
        if not found:
            print("no board answered")
        return 0 if found else 1
    params = mcu_env.load_yaml(a.params)
    robot = mcu_env.robot_name_of(params, a.params)
    env = mcu_env.env_from_config(a.params, cockpit_paths.secrets_path())
    uid, entry = find_robot_board(robot, port=int(env.get("syslog_port") or syslog_port()),
                                  ping_port=int(env.get("ping_port") or PING_PORT),
                                  mdns_name=mcu_env.robot_mdns_name(params, a.params),
                                  pinned_ip=mcu_env.robot_ip_for(params))
    if not entry:
        print(f"❌ '{robot}' is not on USB and its board "
              + (f"(uid={uid}) did not answer on Wi-Fi." if uid else
                 "is unknown here: flash it once over USB first."))
        return 1
    print(f"=== {robot}: board uid={uid} at {entry['ip']}, app={entry.get('app')} -- "
          f"its syslog, live (Ctrl-C to stop) ===", flush=True)
    try:
        follow(entry["ip"], int(env.get("syslog_port") or syslog_port()))
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
