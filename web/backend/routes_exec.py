"""Linorobot2 Cockpit routes -- generic exec, agent / bringup / stack / gamepad process control.

Part of the main.py split: route handlers registered on the shared `app`
as an import side effect (main.py imports this module). Shared state and
helpers come from core.py. See core.py for the split's contract.
"""
import json
import queue
import threading
from fastapi import HTTPException, Request
from fastapi.responses import StreamingResponse
from core import (
    REPO_ROOT,
    RUNNERS,
    access,
    agent_runner,
    app,
    bringup_runner,
    check_agent_port_status,
    check_bringup_health,
    gamepad_runner,
    get_controller,
    json_body,
    load_params,
    main_runner,
    release_agent_port,
    resolve_command,
    robot_stack,
)


# ==============================================================================
# Process Execution & Slots (SSE Streaming)
# ==============================================================================
@app.post("/api/exec")
async def api_exec(request: Request):
    data = await json_body(request)
    slot = data.get("slot", "main")
    runner = RUNNERS.get(slot, main_runner)
    command = resolve_command(data)
    if not command:
        raise HTTPException(status_code=400, detail="No action or command given")
    if runner.is_busy():
        raise HTTPException(status_code=409, detail=f"Slot '{slot}' is busy")

    def event_generator():
        q = queue.Queue(maxsize=500)
        runner.subscribe(q)
        th = threading.Thread(
            target=runner.start_streaming,
            args=(command, REPO_ROOT),
            daemon=True,
        )
        th.start()
        try:
            while runner.is_busy() or not q.empty():
                try:
                    ev_type, payload = q.get(timeout=1.0)
                    yield f"event: {ev_type}\ndata: {json.dumps(payload)}\n\n"
                    if ev_type == "done":
                        break
                except queue.Empty:
                    yield ": ping\n\n"
        finally:
            runner.unsubscribe(q)

    return StreamingResponse(event_generator(), media_type="text/event-stream")


@app.post("/api/kill")
async def api_kill(request: Request):
    data = await json_body(request)
    slot = data.get("slot", "main")
    runner = RUNNERS.get(slot, main_runner)
    killed = runner.kill()
    return {"status": "ok", "killed": killed, "slot": slot}


# micro-ROS Agent execution
@app.post("/api/agent/exec")
async def api_agent_exec(request: Request):
    data = await json_body(request)
    params = load_params()
    ctrl = get_controller(params)
    port = ctrl.get("serial_port", "/dev/ttyUSB0")
    baud = ctrl.get("baudrate", 1500000)
    default = f"ros2 run micro_ros_agent micro_ros_agent serial --dev {port} -b {baud}"
    command = resolve_command(data, default=default)

    if agent_runner.is_busy():
        raise HTTPException(status_code=409, detail="micro-ROS Agent slot is busy")

    def event_generator():
        q = queue.Queue(maxsize=500)
        agent_runner.subscribe(q)
        th = threading.Thread(
            target=agent_runner.start_streaming,
            args=(command, REPO_ROOT),
            daemon=True,
        )
        th.start()
        try:
            while agent_runner.is_busy() or not q.empty():
                try:
                    ev_type, payload = q.get(timeout=1.0)
                    yield f"event: {ev_type}\ndata: {json.dumps(payload)}\n\n"
                    if ev_type == "done":
                        break
                except queue.Empty:
                    yield ": ping\n\n"
        finally:
            agent_runner.unsubscribe(q)

    return StreamingResponse(event_generator(), media_type="text/event-stream")


@app.post("/api/agent/kill")
def api_agent_kill():
    killed = agent_runner.kill()
    return {"status": "ok", "killed": killed}


@app.get("/api/agent/port_check")
@app.post("/api/agent/port_check")
async def api_agent_port_check(request: Request):
    if request.method == "POST":
        data = await json_body(request)
    else:
        data = dict(request.query_params)
    port = data.get("port", "/dev/ttyUSB0")
    mode = data.get("mode", "serial")
    udp_port = int(data.get("udp_port", 8888))
    host = data.get("host")
    user = data.get("user")
    return check_agent_port_status(port=port, mode=mode, udp_port=udp_port, host=host, user=user)


@app.post("/api/agent/port_release")
async def api_agent_port_release(request: Request):
    data = await json_body(request)
    port = data.get("port", "/dev/ttyUSB0")
    mode = data.get("mode", "serial")
    udp_port = int(data.get("udp_port", 8888))
    host = data.get("host")
    user = data.get("user")
    return release_agent_port(port=port, mode=mode, udp_port=udp_port, host=host, user=user)


# Bringup execution
@app.post("/api/bringup/exec")
async def api_bringup_exec(request: Request):
    data = await json_body(request)
    command = resolve_command(data, default="ros2 launch linorobot2_cockpit bringup.launch.py")

    if bringup_runner.is_busy():
        raise HTTPException(status_code=409, detail="Bringup slot is busy")

    def event_generator():
        q = queue.Queue(maxsize=500)
        bringup_runner.subscribe(q)
        th = threading.Thread(
            target=bringup_runner.start_streaming,
            args=(command, REPO_ROOT),
            daemon=True,
        )
        th.start()
        try:
            while bringup_runner.is_busy() or not q.empty():
                try:
                    ev_type, payload = q.get(timeout=1.0)
                    yield f"event: {ev_type}\ndata: {json.dumps(payload)}\n\n"
                    if ev_type == "done":
                        break
                except queue.Empty:
                    yield ": ping\n\n"
        finally:
            bringup_runner.unsubscribe(q)

    return StreamingResponse(event_generator(), media_type="text/event-stream")


@app.post("/api/bringup/kill")
def api_bringup_kill():
    # Two ways a robot can be running. The bringup RUNNER is this backend's own
    # child, started from the Bringup tab. A 1-Click run is not: the pipeline
    # exits and leaves bringup, SLAM and Nav2 alive on purpose, so Stop has to
    # reach those too or the button lies. scripts/robot_stack.py is the record.
    killed = bringup_runner.kill()
    stopped = []
    try:
        stopped = robot_stack.stop()
    except Exception as exc:
        print(f"[api_bringup_kill] robot_stack notice: {exc}")
    return {"status": "ok", "killed": killed, "stack_stopped": stopped}


@app.get("/api/stack")
def api_stack():
    """What a 1-Click run left running, for the Stop buttons and the header."""
    try:
        entries = robot_stack.load()
    except Exception as exc:
        return {"status": "error", "detail": str(exc), "running": []}
    return {"status": "ok", "running": entries,
            "summary": robot_stack.describe()}


@app.post("/api/stack/stop")
async def api_stack_stop(request: Request):
    """Stop the whole kept-running stack, or one part of it."""
    data = await json_body(request)
    tag = (data.get("tag") or "").strip() or None
    if tag and tag not in ("bringup", "slam", "nav2"):
        raise HTTPException(status_code=400, detail=f"Unknown stack part: {tag}")
    try:
        stopped = robot_stack.stop(tag)
    except Exception as exc:
        raise HTTPException(status_code=500, detail=str(exc))
    return {"status": "ok", "stopped": stopped}


@app.get("/api/bringup/stream")
def api_bringup_stream():
    def event_generator():
        q = queue.Queue(maxsize=500)
        bringup_runner.subscribe(q)
        try:
            for line in bringup_runner.get_history():
                yield f"event: output\ndata: {json.dumps({'line': line})}\n\n"
            while True:
                try:
                    ev_type, payload = q.get(timeout=2.0)
                    yield f"event: {ev_type}\ndata: {json.dumps(payload)}\n\n"
                    if ev_type == "done":
                        break
                except queue.Empty:
                    yield ": ping\n\n"
        finally:
            bringup_runner.unsubscribe(q)

    return StreamingResponse(event_generator(), media_type="text/event-stream")


@app.get("/api/bringup/health")
def api_bringup_health(timeout: float = 4.0):
    return check_bringup_health(timeout=timeout)


# ==============================================================================
# Remote 1-Click execution — the robot half of a run driven from a workstation
#
# AGENTS.md §6: the host workstation owns the config and the compiler, the ROBOT
# computer owns the serial bus and the ROS 2 runtime. When "Robot PC Address"
# names another machine, the firmware already travelled here over HTTP; these
# four routes carry everything AFTER the flash the same way, so bringup, SLAM,
# Nav2 and the map saver run on the machine the board is plugged into rather
# than on the workstation driving the run.
#
# HTTP and not ssh, for the reason §6 gives: ssh needs keys, an account and a
# shell on the robot that a user who installed the Cockpit has not necessarily
# set up, while this supervisor is already running, already owns the bus and is
# already the thing the browser talks to. One channel, one set of failure modes.
# ==============================================================================
# ==============================================================================
# Gamepad & Virtual Teleop Publisher API
# ==============================================================================
@app.post("/api/gamepad/start")
def api_gamepad_start():
    started = gamepad_runner.start()
    return {"status": "ok", "running": started}


@app.post("/api/gamepad/cmd")
async def api_gamepad_cmd(request: Request):
    data = await json_body(request)
    lx = float(data.get("linear_x", 0.0))
    ly = float(data.get("linear_y", 0.0))
    az = float(data.get("angular_z", 0.0))
    if not gamepad_runner.is_running():
        gamepad_runner.start()
    sent = gamepad_runner.send(lx, ly, az)
    return {"sent": sent, "running": gamepad_runner.is_running()}


@app.get("/api/gamepad/stall")
def api_gamepad_stall():
    return {"stalled": False, "running": gamepad_runner.is_running()}


@app.post("/api/gamepad/kill")
def api_gamepad_kill():
    killed = gamepad_runner.kill()
    return {"killed": killed}


@app.post("/api/stream_ticket")
def api_stream_ticket():
    """Mint a one-shot, one-minute ticket for a server-sent-event stream.

    This is a POST, so the middleware requires the real token (in the header) to
    reach it; it hands back a nonce the page puts in the stream URL's ?ticket=.
    The token itself then never appears in a URL. Auth-off installs need no
    ticket, but one is returned anyway so the same frontend path works.
    """
    return {"status": "ok", "ticket": access.mint_stream_ticket()}
