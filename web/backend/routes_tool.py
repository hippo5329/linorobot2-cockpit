"""Linorobot2 Cockpit routes -- the board's tool, for the Hardware Tests tab.

/api/tool/open      enter the tool (the board restarts and catches its boot window; no env
                    write) and hold one session for the active robot
/api/tool/stream    its output, to every open tab (server-sent events)
/api/tool/beat      a tab says it is still watching (once a second)
/api/tool/cmd       one command line, validated against the shell's grammar, and the board's
                    answer to it (ok / fail and why, or none in 2 s)
/api/tool/stop      `stop`, and on Wi-Fi the ping port's lino-stop as well
/api/tool/close     `base`, then close; and, when the board's env still names a tool, the
                    handle of the env-only write that makes it boot the robot again
/api/tool/env_base  that write on its own, for a board an older cockpit left in a tool
/api/tool/state     what the session is doing

The OTA password is read here from the robot's secrets and used here; it is in no response.
See tool_session.py for the rules.
"""
import json
import os
import re
import shlex
import time

from fastapi import HTTPException, Request
from fastapi.responses import StreamingResponse

from core import (
    REPO_ROOT,
    actions,
    agent_runner,
    app,
    get_active_params_path,
    json_body,
    mcu_identity,
    release_agent_port,
)
import cockpit_paths  # noqa: E402  (scripts/, via core's sys.path)
import tool_session  # noqa: E402

WINDOW_S = 15.0          # to catch the 3 s boot window (an RP2 takes ~2 s to come back)


class Unavailable(Exception):
    """The tool cannot be opened for this robot; the message is for the page."""


def _robot():
    """The active robot's facts the tool needs, from its config and secrets. The password
    stays in the returned dict, which never leaves this module."""
    import mcu_env
    params_path = get_active_params_path()
    params = mcu_env.load_yaml(params_path)
    tgt = params.get("base_controller") or {}
    robot = mcu_env.robot_name_of(params, params_path)
    env_vals = mcu_env.env_from_config(params_path, cockpit_paths.secrets_path())
    pio_env = str(tgt.get("pio_env") or tgt.get("mcu") or "")
    return {
        "params_path": params_path, "params": params, "tgt": tgt, "robot": robot,
        "env": env_vals, "pio_env": pio_env,
        "wifi": mcu_env.transport_is_wifi(tgt.get("transport")),
        "sim": (str(tgt.get("name") or "").lower() == "sim"
                or str(tgt.get("mcu") or "").lower() in ("host", "sim") or robot == "bare_sim"),
    }


class SimLink:
    """The Sim MCU, started for the tool: the host firmware booting straight into the tool
    (app=tool), its console -- stdin and stdout -- as the session's link. The firmware is the
    same code a board runs, so the panel can be learned and walked with no board: the tool
    refuses what the Sim MCU simulates (its wheels, its sensors) in its own words.

    `base` on this link ends the process rather than restarting it into micro-ROS: a second
    Sim MCU on the agent's port beside a running stack would be a second robot.
    COCKPIT_TOOL_SIM_WHEELS_REAL=1 (a walkthrough cell, never a robot) makes the wheels count
    as real so the drive commands run -- the host's motors drive nothing and read 0."""

    def __init__(self, params_path: str):
        import subprocess
        import tempfile
        import host_firmware
        import mcu_env
        binary = host_firmware.binary()
        if not binary:
            raise Unavailable("This cockpit has no Sim MCU build to run the tool on.")
        e = host_firmware.env(params_path)
        e.update({"app": "tool", "tool_wait": "0", "term_port": "0"})
        if os.environ.get("COCKPIT_TOOL_SIM_WHEELS_REAL") == "1":
            e["sim_wheel"] = "0"
        self._dir = tempfile.mkdtemp(prefix="lino_tool_sim_")
        path = os.path.join(self._dir, "env.bin")
        with open(path, "wb") as fh:
            fh.write(mcu_env.encode(e))
        penv = dict(os.environ, LINO_ENV_BIN=path)
        penv.pop("LINO_NEXT_BOOT", None)
        # Exactly as bringup.launch.py starts it: the client workspace sourced for this
        # process only (the binary links librclc and rmw_microxrcedds from there -- run bare
        # it died, "librclc.so: cannot open shared object file"), and RMW_IMPLEMENTATION
        # said, or rcl refuses the image's container-wide Fast DDS and exits 102.
        # The launcher runs inside a sourced ROS; the backend does not, so ROS first
        # (librcutils and the rest -- "librcutils.so: cannot open shared object file").
        cmd = host_firmware.shell_command(binary)
        self.p = subprocess.Popen(["bash", "-c", cmd], stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                  stderr=subprocess.STDOUT, env=penv)
        os.set_blocking(self.p.stdout.fileno(), False)
        self.closed = False

    def write(self, data: bytes):
        if self.closed:
            return
        if data.strip().lower() in (b"base", b"exit", b"reboot"):
            self.close()
            return
        try:
            self.p.stdin.write(data.replace(b"\r\n", b"\n").replace(b"\r", b"\n"))
            self.p.stdin.flush()
        except (BrokenPipeError, OSError):
            self.close()

    def read(self) -> bytes:
        if self.closed:
            return b""
        try:
            d = os.read(self.p.stdout.fileno(), 4096)
        except BlockingIOError:
            time.sleep(0.02)
            return b""
        except OSError:
            d = b""
        if not d and self.p.poll() is not None:
            self.closed = True
        return d

    def keepalive(self):
        pass

    def close(self):
        if self.closed:
            return
        self.closed = True
        try:
            self.p.terminate()
            self.p.wait(3)
        except Exception:
            try:
                self.p.kill()
            except Exception:
                pass


def _open_sim(r: dict):
    link = SimLink(r["params_path"])
    session = tool_session.ToolSession(r["robot"], link, "sim")
    end = time.monotonic() + 10
    seen = 0
    while time.monotonic() < end:
        session.pump()
        lines = list(session._lines)[seen:]
        seen += len(lines)
        if any(any(e in ln for e in tool_session.ENTERED) for ln in lines):
            return session
    link.close()
    raise Unavailable("The Sim MCU did not start its tool (see the server log).")


def _refusal(r: dict):
    """Why the tool cannot be opened here, in words for the page; None when it can. The Sim
    MCU can: its tool refuses what it simulates, in its own words."""
    if mcu_identity.env_family(r["pio_env"]) == "unoq" and not os.path.exists("/dev/ttyHS1"):
        return ("An Arduino UNO Q's tool is reached on the board itself (its STM32 on "
                "/dev/ttyHS1): open this cockpit on the UNO Q, or run "
                "scripts/lino_tool.py --port /dev/ttyHS1 --env unoq there.")
    return None


_DEVICE = re.compile(r"^/dev/[A-Za-z0-9_./-]{1,64}$")


def _serial_port(r: dict, requested: str) -> str:
    port = requested or str(r["tgt"].get("serial_port") or "")
    if mcu_identity.env_family(r["pio_env"]) == "unoq":
        port = "/dev/ttyHS1"
    if not port or not _DEVICE.match(port) or ".." in port:
        raise Unavailable("No serial port: pick the board's port on Hardware Tests.")
    if not os.path.exists(port):
        raise Unavailable(f"{port} is not here: is the board plugged in, and is this its port?")
    return port


def _catch_window(session, seconds: float = WINDOW_S) -> bool:
    """Type `tool` until the board says it is in the tool; the board's own lines go to the
    session, so the page shows the boot as it happens."""
    end = time.monotonic() + seconds
    nxt = 0.0
    seen = 0
    while time.monotonic() < end:
        if time.monotonic() >= nxt:
            session.link.write(b"tool\r\n")
            nxt = time.monotonic() + 0.25
        session.pump()
        lines = list(session._lines)[seen:]
        seen += len(lines)
        if any(any(e in ln for e in tool_session.ENTERED) for ln in lines):
            return True
    return False


def _open_serial(r: dict, port: str):
    import lino_tool
    baud = int(r["env"].get("baud") or 921600)
    env = r["pio_env"] or ""
    try:
        if agent_runner.is_busy():
            agent_runner.kill()
    except Exception as e:
        print(f"[tool] agent stop notice: {e}")
    try:
        release_agent_port(port=port)
    except Exception as e:
        print(f"[tool] release_agent_port notice: {e}")
    link = lino_tool.SerialLink(port, baud, env)
    if not link.reboot():
        raise Unavailable("The board did not restart (see the server log): is it plugged in, "
                          "and is the port right?")
    session = tool_session.ToolSession(r["robot"], link, "serial")
    if not _catch_window(session):
        link.close()
        raise Unavailable("The board did not enter the tool: its firmware may predate the tool "
                          "(flash the MCU first), or its tool_wait is 0.")
    return session


def _open_wifi(r: dict):
    import lino_tool
    import mcu_env
    import wifi_boards
    env = r["env"]
    password = str(env.get("ota_password") or "")
    if not password:
        raise Unavailable("This Wi-Fi robot has no OTA password, so the board refuses the tool "
                          "over the network. Flash it once over USB (that sets one), or open "
                          "the tool on USB.")
    params, params_path = r["params"], r["params_path"]
    _, entry = wifi_boards.find_robot_board(
        r["robot"], port=int(env.get("syslog_port") or wifi_boards.syslog_port()),
        ping_port=int(env.get("ping_port") or wifi_boards.PING_PORT),
        mdns_name=mcu_env.robot_mdns_name(params, params_path),
        pinned_ip=mcu_env.robot_ip_for(params))
    ip = entry and entry.get("ip")
    if not ip:
        raise Unavailable(f"{r['robot']} is not on the network (no answer to a ping): is it "
                          "powered, and on the Wi-Fi its env names?")
    ping_port = int(env.get("ping_port") or 3233)
    if not lino_tool.request_tool_udp(ip, ping_port, password):
        raise Unavailable("The board refused or did not answer lino-tool (see the server log).")
    link = lino_tool.TcpLink(ip, int(env.get("term_port") or 2323), password)
    if not link.open():
        raise Unavailable("The board's tool terminal did not log in (busy, locked, or not up).")
    return tool_session.ToolSession(r["robot"], link, "wifi", ip=ip, ping_port=ping_port)


def _session_or_404():
    r = _robot()
    s = tool_session.REGISTRY.get(r["robot"])
    if not s:
        raise HTTPException(status_code=404, detail="No tool session is open for this robot.")
    return s


def _env_base_command(r: dict, port: str = "") -> str:
    """The env-only write of app=base: 4 KB, no reflash. Over USB when the port is here,
    else over the air (flash_mcu decides; a Wi-Fi robot's OTA needs its password, which
    flash_mcu reads from the secrets itself)."""
    flash_script = os.path.join(REPO_ROOT, "scripts", "flash_mcu.py")
    env = mcu_identity.pio_env_for(r["pio_env"], "pico2")
    port = port or str(r["tgt"].get("serial_port") or "/dev/ttyACM0")
    baud = int(r["env"].get("baud") or 921600)
    return (f"python3 {flash_script} --env {shlex.quote(env)} --env-only "
            f"--port {shlex.quote(port)} --params {shlex.quote(r['params_path'])} "
            f"--app base --baud {baud}")


@app.get("/api/tool/state")
def api_tool_state():
    r = _robot()
    s = tool_session.REGISTRY.get(r["robot"])
    out = {"robot": r["robot"], "open": bool(s), "wifi": r["wifi"], "refusal": _refusal(r),
           "pio_env": r["pio_env"]}
    if s:
        out.update(s.describe())
    return out


@app.post("/api/tool/open")
async def api_tool_open(request: Request):
    data = await json_body(request)
    r = _robot()
    why = _refusal(r)
    if why:
        raise HTTPException(status_code=409, detail=why)
    existing = tool_session.REGISTRY.get(r["robot"])
    if existing:
        return {"ok": True, "already": True, **existing.describe()}
    try:
        if r["sim"]:
            session = await _run_blocking(_open_sim, r)
        elif r["wifi"] and not data.get("port"):
            session = await _run_blocking(_open_wifi, r)
        else:
            port = _serial_port(r, str(data.get("port") or ""))
            session = await _run_blocking(_open_serial, r, port)
    except Unavailable as e:
        raise HTTPException(status_code=409, detail=str(e))
    try:
        tool_session.REGISTRY.add(session)
    except RuntimeError as e:
        session.close()
        raise HTTPException(status_code=409, detail=str(e))
    return {"ok": True, **session.describe()}


async def _run_blocking(fn, *args):
    import asyncio
    return await asyncio.get_running_loop().run_in_executor(None, fn, *args)


@app.get("/api/tool/stream")
def api_tool_stream():
    s = _session_or_404()
    wid = s.watch()

    def gen():
        try:
            yield "data: " + json.dumps({"watch": wid, **s.describe()}) + "\n\n"
            last = time.monotonic()
            while not s.closed:
                lines = s.drain(wid)
                for ln in lines:
                    yield "data: " + json.dumps({"line": ln}) + "\n\n"
                if time.monotonic() - last >= 1.0:
                    last = time.monotonic()
                    yield "data: " + json.dumps({"state": s.describe()}) + "\n\n"
                if not lines:
                    time.sleep(0.05)
            yield "data: " + json.dumps({"closed": True}) + "\n\n"
        finally:
            s.unwatch(wid)

    return StreamingResponse(gen(), media_type="text/event-stream",
                             headers={"Cache-Control": "no-cache", "X-Accel-Buffering": "no"})


@app.post("/api/tool/beat")
async def api_tool_beat(request: Request):
    data = await json_body(request)
    s = _session_or_404()
    try:
        wid = int(data.get("watch"))
    except (TypeError, ValueError):
        raise HTTPException(status_code=400, detail="watch: the id the stream sent first")
    return {"ok": s.beat(wid)}


@app.post("/api/tool/cmd")
async def api_tool_cmd(request: Request):
    data = await json_body(request)
    s = _session_or_404()
    try:
        sent = s.send(str(data.get("line") or ""))
    except tool_session.Refused as e:
        raise HTTPException(status_code=400, detail=str(e))
    # Command and response: the board's own answer, or that none came (ACK_TIMEOUT_S).
    seq = s.last_seq
    answer = await _run_blocking(s.wait_answer, seq)
    return {"ok": True, "sent": sent, "answers": s.acks_seen,
            "answer": answer or {"seq": seq, "kind": "none", "line": sent, "ok": None,
                                 "msg": "no answer"}}


@app.post("/api/tool/stop")
def api_tool_stop():
    s = _session_or_404()
    s.stop()
    return {"ok": True}


@app.post("/api/tool/close")
def api_tool_close():
    r = _robot()
    s = tool_session.REGISTRY.get(r["robot"])
    if not s:
        return {"ok": True, "open": False}
    s.leave()
    time.sleep(1.0)
    # The Sim MCU's env is the temp file SimLink wrote for this session (app=tool): nothing
    # boots from it again, so there is no board env to set back to base.
    needs = s.needs_env_base() and s.kind != "sim"
    tool_session.REGISTRY.remove(r["robot"])
    out = {"ok": True, "open": False, "needs_env_base": needs}
    if needs:
        out["handle"] = actions.prepare(_env_base_command(r))
    return out


@app.post("/api/tool/env_base")
async def api_tool_env_base(request: Request):
    data = await json_body(request)
    r = _robot()
    port = str(data.get("port") or "")
    if port and (not _DEVICE.match(port) or ".." in port):
        raise HTTPException(status_code=400, detail="port: a /dev/ path")
    return {"handle": actions.prepare(_env_base_command(r, port))}
