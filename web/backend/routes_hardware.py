"""Linorobot2 Cockpit routes -- hardware tests, firmware, presets, one-click workflow, ROS 2 probes, maps.

Part of the main.py split: route handlers registered on the shared `app`
as an import side effect (main.py imports this module). Shared state and
helpers come from core.py. See core.py for the split's contract.
"""
import json
import os
import queue
import shlex
import subprocess
import sys
import threading
from typing import Optional
from fastapi import HTTPException, Request
from fastapi.responses import FileResponse, StreamingResponse
from core import (
    REPO_ROOT,
    actions,
    agent_runner,
    app,
    build_base_install_cmd,
    build_ros2_install_cmd,
    build_sensor_install_cmd,
    fetch_prebuilt,
    get_active_params_path,
    get_controller,
    get_controller_name,
    get_ros_exec_cmd,
    hw_detect,
    hw_flash,
    hw_list_serial_ports,
    hw_resolve_artifact,
    hw_tool_status,
    json_body,
    load_params,
    mcu_identity,
    patcher,
    pio_present,
    regenerate_firmware_headers,
    refuse_sim_flash,
    release_agent_port,
    require_header,
    ros_setup_shell,
    text_field,
)


@app.post("/api/hardware/test")
async def api_hardware_test(request: Request):
    data = await json_body(request)
    firmware = data.get("firmware") or data.get("target") or "test_sensors"
    port = data.get("port", "/dev/ttyACM0")
    baud = int(data.get("baud", 921600))
    action = data.get("action", "upload")
    # The Sensors tab fills cfg-mcu from the USB probe, which answers with a
    # BOARD ("gendrv" for a CP2102N). Flashing needs a PlatformIO env, and
    # asking for "gendrv" sent fetch_prebuilt after a release artifact that
    # does not exist -- firmware upload 404ed on the one ESP32 board this
    # project ships a reference design for.
    if action == "upload":
        refuse_sim_flash(data.get("mcu_env"))
    mcu_env = mcu_identity.pio_env_for(data.get("mcu_env"), "pico2")

    if action in ("upload", "monitor"):
        try:
            if agent_runner.is_busy():
                agent_runner.kill()
        except Exception as e:
            print(f"[api_hardware_test] agent stop notice: {e}")
        try:
            release_agent_port(port=port)
        except Exception as e:
            print(f"[api_hardware_test] release_agent_port notice: {e}")

    # One image now carries every application, so there is no per-tool directory
    # to point at: `firmware` is always the project, and which application boots
    # is the `app` key in the env partition.
    firmware_dir = "firmware"
    require_header(regenerate_firmware_headers(mcu_env))
    params_path = get_active_params_path()

    if action == "build":
        # The published robot image deliberately ships no PlatformIO -- it
        # flashes the release artifact instead -- so Build on a stock install
        # ended in a bare `bash: line 1: pio: command not found` and exit 127,
        # which says nothing about why or what to do. Answer the question the
        # user is actually asking.
        if not pio_present():
            raise HTTPException(
                status_code=400,
                detail=("PlatformIO is not installed here, and this image does not ship "
                        "it: Build compiles from source, which a robot does not need. "
                        "Use Flash MCU — it fetches the release image for this board — "
                        "or run the build in the pio container: "
                        "`docker compose run --rm pio pio run -e %s`." % mcu_env))
        cmd = f"pio run -d {firmware_dir} -e {mcu_env}"
    elif action == "upload":
        # Build and flash stay separate: pio only compiles, esptool/picotool
        # writes. With PlatformIO on this machine the image is built first;
        # without it the release image for this env is fetched and flashed.
        # One image carries base AND every diagnostic tool (firmware tools.h,
        # the "swiss knife"): the `app` env key selects which boots, so flashing
        # `adc_calibrate` writes the same base image and sets app=adc_calibrate.
        # No per-tool binary, and no toolchain needed on the robot.
        flash_script = os.path.join(REPO_ROOT, "scripts", "flash_mcu.py")
        if firmware != "base":
            # Switching to a diagnostic tool is the swiss knife's whole point: the
            # installed image already carries every tool, so this is a 4 KB env
            # write (app=<tool>) and a reboot -- no download, no reflash. Flash
            # the base firmware once; then switch blades freely. If the board
            # runs an image too old to have the tool, it boots base with a
            # message rather than the tool (firmware tools.cpp), so this is safe.
            cmd = (
                f"python3 {flash_script} --env {mcu_env} --env-only "
                f"--port {port} "
                f"--params {shlex.quote(params_path)} "
                f"--app {firmware} "
                f"--baud {baud}"
            )
        else:
            image = (f"--firmware-dir {firmware_dir} --env {mcu_env} --build"
                     if pio_present() else
                     f"--prebuilt {fetch_prebuilt.profile_for_env(mcu_env)}")
            fetch = ("" if pio_present() else
                     f"python3 {os.path.join(REPO_ROOT, 'scripts', 'fetch_prebuilt.py')} {mcu_env} && ")
            cmd = (
                f"{fetch}python3 {flash_script} {image} "
                f"--port {port} "
                f"--params {shlex.quote(params_path)} "
                f"--firmware-name {firmware} "
                f"--app {firmware} "
                f"--baud {baud}"
            )
    elif action == "monitor":
        # NOT miniterm. It builds a Console() in its constructor, which calls
        # termios.tcgetattr() on stdin, and every command here runs on a PIPE --
        # so Monitor failed with `termios.error: (25, 'Inappropriate ioctl for
        # device')` every single time, and the diagnostic applications had no
        # way to show their output at all. scripts/serial_monitor.py reads the
        # port and writes lines to stdout, which is what the SSE runner
        # forwards.
        cmd = (
            f"echo '=== [1/2] Releasing serial port {port} ===' && "
            f"lsof -ti {port} 2>/dev/null | xargs -r kill -9 2>/dev/null || true; "
            f"sleep 0.5; "
            f"echo '=== [2/2] Streaming {port} @ {baud} ===' && "
            f"python3 -u {os.path.join(REPO_ROOT, 'scripts', 'serial_monitor.py')} "
            f"{shlex.quote(port)} {baud}"
        )
    else:
        raise HTTPException(status_code=400, detail=f"Unknown action: {action}")

    return {"command": cmd, "handle": actions.prepare(cmd), "firmware": firmware, "port": port, "mcu_env": mcu_env}


# ==============================================================================
# Nav2 / EKF / SLAM Configuration & Presets (Single Source of Truth)
# ==============================================================================
@app.get("/api/presets")
def api_presets():
    return {"presets": patcher.PRESETS}


@app.post("/api/presets/apply")
async def api_presets_apply(request: Request):
    data = await json_body(request)
    preset_name = data.get("preset", "")
    if preset_name not in patcher.PRESETS:
        raise HTTPException(status_code=400, detail=f"Unknown preset '{preset_name}'")
    pinfo = patcher.PRESETS[preset_name]

    with open(get_active_params_path(), "r") as f:
        text = f.read()

    # 1. Nav2 patch
    nav2_out = patcher.patch_nav2_text(text, **patcher.nav2_kwargs(pinfo))
    # 2. EKF patch
    ekf_out = patcher.patch_ekf_text(
        nav2_out,
        base_type=pinfo["base"],
        frequency=pinfo["ekf_frequency"],
        fuse_vy=pinfo["fuse_vy"],
        fuse_imu_yaw=pinfo["fuse_imu_yaw"],
    )
    # 3. SLAM patch
    slam_out = patcher.patch_slam_text(
        ekf_out,
        resolution=pinfo["slam_resolution"],
        max_laser_range=pinfo["slam_max_range"],
    )

    with open(get_active_params_path(), "w") as f:
        f.write(slam_out)

    return {
        "status": "applied",
        "preset": preset_name,
        "label": pinfo["label"],
        "base": pinfo["base"],
    }


@app.get("/api/sensor_install_cmd")
@app.post("/api/sensor_install_cmd")
def api_sensor_install_cmd(sensor: str = "", distro: str = "jazzy", ws: str = ""):
    # A plain apt install -- no ROS sourcing or cd needed, so the handle is the
    # command as-is. The browser runs the handle instead of re-composing it.
    result = build_sensor_install_cmd(sensor, distro=distro, ws=ws)
    if result.get("command"):
        result["handle"] = actions.prepare(result["command"])
    return result


@app.get("/api/workspace/build_cmd")
def api_workspace_build_cmd(ws: str = REPO_ROOT, distro: str = "jazzy"):
    # colcon needs a sourced ROS, so the handle carries the sourcing the browser
    # used to prepend -- the page runs the handle verbatim, adding nothing.
    base_cmd = build_base_install_cmd(ws, distro=distro)
    full = f"{ros_setup_shell(distro)}; {base_cmd}"
    return {"command": base_cmd, "handle": actions.prepare(full), "workspace": ws}


@app.get("/api/ros2/install_cmd")
def api_ros2_install_cmd(distro: str = "jazzy"):
    ros2_cmd = build_ros2_install_cmd(distro)
    return {
        "distro": distro,
        "installed": True,
        "command": ros2_cmd,
        "handle": actions.prepare(ros2_cmd),
    }


# ==============================================================================
# Topic Monitor & LiDAR SSE APIs
# ==============================================================================
@app.get("/api/ros2/topics")
def api_ros2_topics(distro: str = "jazzy"):
    cmd = get_ros_exec_cmd("ros2 topic list -t", distro=distro)
    try:
        res = subprocess.run(cmd, capture_output=True, text=True, timeout=10)
        lines = res.stdout.strip().splitlines()
        topics = []
        for line in lines:
            parts = line.split()
            if len(parts) >= 2:
                topics.append({"topic": parts[0], "type": parts[1].strip("[]")})
        return {"success": True, "topics": topics}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@app.get("/api/ros2/hz_single")
def api_ros2_hz_single(topic: str, distro: str = "jazzy"):
    cmd = get_ros_exec_cmd(f"timeout 3 ros2 topic hz {shlex.quote(topic)}", distro=distro)
    try:
        res = subprocess.run(cmd, capture_output=True, text=True, timeout=8)
        rate = 0.0
        for line in res.stdout.splitlines():
            if "average rate:" in line:
                try:
                    rate = float(line.split("average rate:")[1].strip().split()[0])
                except Exception:
                    pass
        return {"success": True, "topic": topic, "hz": rate}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@app.get("/api/lidar_stream")
def api_lidar_stream(distro: str = "jazzy", topic: str = "/scan", max_points: int = 360):
    """
    SSE feed of LaserScan messages for the canvas viewer.

    scripts/scan_stream.py emits one JSON object per line; each is relayed as a
    named `scan` event (plus `status` events for "no publisher yet"), which is
    what the browser listens for. Echoing `--csv` here produced unnamed events
    carrying field-less CSV, so the viewer never drew anything.
    """
    script = os.path.join(REPO_ROOT, "scripts", "scan_stream.py")
    cmd = get_ros_exec_cmd(
        f"python3 -u {shlex.quote(script)} --topic {shlex.quote(topic)} --max-points {int(max_points)}",
        distro=distro,
    )

    def event_generator():
        proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1)
        try:
            for line in iter(proc.stdout.readline, ""):
                line = line.strip()
                if not line:
                    continue
                try:
                    payload = json.loads(line)
                    evt = payload.get("event", "scan")
                except (ValueError, AttributeError):
                    # Anything the ROS setup shell prints on the way up is not
                    # JSON; surface it as status rather than dropping it.
                    payload = {"event": "status", "level": "info", "message": line}
                    evt = "status"
                yield f"event: {evt}\ndata: {json.dumps(payload)}\n\n"
            rc = proc.wait()
            yield f"event: status\ndata: {json.dumps({'level': 'error' if rc else 'info', 'message': f'scan stream ended (exit {rc})'})}\n\n"
        finally:
            if proc.poll() is None:
                proc.terminate()
                try:
                    proc.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    proc.kill()

    return StreamingResponse(event_generator(), media_type="text/event-stream")


# ------------------------------------------------------------------------------
# Firmware Detection & Native Flashing Engine (robot-computer side)
#
# The host workstation compiles the binary (pio run) and never opens /dev; this
# machine owns the serial bus and writes it with esptool / picotool. Build and
# flash are always separate steps -- no PlatformIO uploader is ever invoked.
# Flashing is never routed through the browser: WebSerial is the debug monitor only.
# ------------------------------------------------------------------------------
@app.get("/api/firmware/tools")
def api_firmware_tools():
    """Which flashing tools this machine can actually run, and what is plugged in."""
    return {"status": "ok", "tools": hw_tool_status(), "ports": hw_list_serial_ports()}


@app.get("/api/firmware/detect")
@app.post("/api/firmware/detect")
async def api_firmware_detect(request: Request):
    """
    Identify the attached MCU via `esptool chip_id` / `picotool info`.

    micro_ros_agent is stopped for the probe and resumed afterwards, because it
    holds the tty open for as long as it runs.
    """
    if request.method == "POST":
        try:
            data = await json_body(request)
        except Exception:
            data = {}
    else:
        data = dict(request.query_params)

    params = load_params()
    ctrl = get_controller(params)
    port = text_field(data, "port") or None
    family = data.get("family", "auto")
    baud = int(data.get("baud") or 115200)
    force = str(data.get("force", "")).lower() in ("1", "true", "yes")
    mode = data.get("mode") or ctrl.get("transport", "serial")
    udp_port = int(data.get("udp_port") or ctrl.get("udp_port", 8888))

    return hw_detect(
        port=port, family=family, baud=baud, force=force,
        agent_runner=agent_runner, mode=mode, udp_port=udp_port,
    )


@app.get("/api/firmware/artifact")
def api_firmware_artifact(firmware_dir: str = "firmware", env: Optional[str] = None):
    """Locate the binary for this environment: a local build, else the prebuilt release image."""
    params = load_params()
    return hw_resolve_artifact(firmware_dir, env or get_controller_name(params, "pico2"))


@app.post("/api/firmware/flash")
async def api_firmware_flash(request: Request):
    """
    Flash the firmware natively, streaming tool output as SSE.

    Stops micro_ros_agent before touching the bus and restarts it afterwards in
    the same execution mode it was running in.
    """
    try:
        data = await json_body(request)
    except Exception:
        data = {}

    params = load_params()
    ctrl = get_controller(params)
    firmware_dir = data.get("firmware_dir") or "firmware"
    refuse_sim_flash(text_field(data, "env") or get_controller_name(params, "pico2"))
    env = mcu_identity.pio_env_for(text_field(data, "env") or get_controller_name(params, "pico2"), "pico2")
    port = text_field(data, "port") or ctrl.get("serial_port", "/dev/ttyUSB0")
    baud = int(data.get("baud") or ctrl.get("upload_baudrate") or 921600)
    chip = data.get("chip") or "auto"
    erase = bool(data.get("erase"))
    resume_agent = data.get("resume_agent", True)
    mode = data.get("mode") or ctrl.get("transport", "serial")
    udp_port = int(data.get("udp_port") or ctrl.get("udp_port", 8888))
    timeout = int(data.get("timeout") or 300)

    def event_generator():
        q = queue.Queue(maxsize=2000)
        done = {}

        def emit(line):
            try:
                q.put_nowait(line)
            except queue.Full:
                pass

        def worker():
            try:
                done["result"] = hw_flash(
                    firmware_dir=firmware_dir, env=env, port=port, baud=baud,
                    chip=chip, erase=erase, agent_runner=agent_runner, mode=mode,
                    udp_port=udp_port, resume_agent=resume_agent, emit=emit,
                    timeout=timeout,
                )
            except Exception as exc:
                done["result"] = {"status": "error", "ok": False,
                                  "error": f"{type(exc).__name__}: {exc}"}
                emit(f"[error] {type(exc).__name__}: {exc}")
            finally:
                done["finished"] = True

        threading.Thread(target=worker, daemon=True).start()

        while not done.get("finished") or not q.empty():
            try:
                line = q.get(timeout=1.0)
                yield f"event: output\ndata: {json.dumps({'line': line})}\n\n"
            except queue.Empty:
                yield ": ping\n\n"

        result = done.get("result") or {"status": "error", "ok": False,
                                        "error": "flash worker produced no result"}
        # The log was already streamed line by line; resending it would double it.
        summary = {k: v for k, v in result.items() if k != "log"}
        yield f"event: done\ndata: {json.dumps(summary)}\n\n"

    return StreamingResponse(event_generator(), media_type="text/event-stream")


@app.post("/api/firmware/generate")
def trigger_firmware_generation(controller: Optional[str] = None):
    gen_script = os.path.join(REPO_ROOT, "scripts", "gen_firmware_header.py")
    cmd = [sys.executable, gen_script]
    if controller:
        cmd.extend(["--controller", controller])
    res = subprocess.run(cmd, capture_output=True, text=True)
    if res.returncode != 0:
        raise HTTPException(status_code=500, detail=res.stderr)
    return {"success": True, "output": res.stdout}


@app.post("/api/workflow/one-click")
def trigger_one_click_workflow(controller: Optional[str] = None, explore_sec: int = 15, no_nav2: bool = False, distro: Optional[str] = None, mode: Optional[str] = None, flash_firmware: bool = False, auto_update: bool = True, firmware: str = "auto"):
    params = load_params()
    selected_controller = controller or get_controller_name(params, "gendrv")
    pipeline_script = os.path.join(REPO_ROOT, "scripts", "one_click_pipeline.py")

    d_str = distro
    if not d_str:
        if os.path.exists("/opt/ros/lyrical"):
            d_str = "lyrical"
        elif os.path.exists("/opt/ros/jazzy"):
            d_str = "jazzy"
        else:
            d_str = os.environ.get("ROS_DISTRO", "jazzy")

    cmd = [
        sys.executable, pipeline_script,
        "--controller", selected_controller,
        "--explore-sec", str(explore_sec),
        "--distro", d_str,
    ]
    if no_nav2:
        cmd.append("--no-nav2")
    if firmware in ("build", "prebuilt"):
        cmd.extend(["--firmware", firmware])
    # Two different questions, and the UI asks both.
    #   flash_firmware  force a write even over an up_to_date board.
    #   auto_update     (default on) write when the probe says the board is stale.
    # Only the negative is passed: the pipeline's own default is on, so a caller
    # that predates this parameter keeps the pipeline default rather than
    # silently pinning the old protective behaviour.
    if flash_firmware:
        cmd.append("--flash")
    if not auto_update:
        cmd.append("--no-auto-update")
    if mode:
        cmd.extend(["--mode", mode])
    res = subprocess.run(cmd, capture_output=True, text=True)
    if res.returncode != 0:
        raise HTTPException(status_code=500, detail=res.stderr or res.stdout)
    return {
        "success": True,
        "controller": selected_controller,
        "output": res.stdout,
    }


@app.get("/api/workflow/one-click/stream")
def stream_one_click_workflow(controller: Optional[str] = None, explore_sec: int = 15, no_nav2: bool = False, distro: Optional[str] = None, mode: Optional[str] = "sim", flash_firmware: bool = False, auto_update: bool = True, firmware: str = "auto", robot: Optional[str] = None):
    # `robot` is not optional decoration: without it the pipeline resolves the
    # config from --controller alone, and where two robots declare the same
    # base_controller it takes the alphabetically first one. Measured with
    # rover_pico2 selected in the UI, on a box whose config dir also holds
    # linorobot2_config.yaml (also `pico2`):
    #
    #   Requested: robot config 'linorobot2_config.yaml' -> base_controller 'pico2'
    #
    # So the run used another robot's kinematics, pins, EKF, SLAM and Nav2
    # parameters while the header displayed "Robot: rover_pico2" throughout.
    # The UI knows the robot; it simply was not sending it.
    params = load_params()
    selected_controller = controller or get_controller_name(params, "pico2")
    pipeline_script = os.path.join(REPO_ROOT, "scripts", "one_click_pipeline.py")
    m_str = mode or "sim"

    d_str = distro
    if not d_str:
        if os.path.exists("/opt/ros/lyrical"):
            d_str = "lyrical"
        elif os.path.exists("/opt/ros/jazzy"):
            d_str = "jazzy"
        else:
            d_str = os.environ.get("ROS_DISTRO", "jazzy")

    cmd = [
        sys.executable, "-u", pipeline_script,
        "--controller", selected_controller,
        "--explore-sec", str(explore_sec),
        "--mode", m_str,
        "--distro", d_str,
    ]
    if robot:
        cmd.extend(["--robot", robot])
    if no_nav2:
        cmd.append("--no-nav2")
    if firmware in ("build", "prebuilt"):
        cmd.extend(["--firmware", firmware])
    if flash_firmware:
        cmd.append("--flash")
    if not auto_update:
        cmd.append("--no-auto-update")

    def event_generator():
        start_line = (f">>> Starting One-Click Pipeline on robot: {robot or '<resolved from controller>'}, "
                      f"controller: {selected_controller} "
                      f"(distro: {d_str}, mode: {m_str}, "
                      f"auto-update: {'on' if auto_update else 'off'})")
        yield f"event: output\ndata: {json.dumps({'line': start_line})}\n\n"
        proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1)
        for line in iter(proc.stdout.readline, ""):
            if not line:
                break
            yield f"event: output\ndata: {json.dumps({'line': line.rstrip()})}\n\n"
        proc.wait()
        yield f"event: done\ndata: {json.dumps({'exit_code': proc.returncode})}\n\n"

    return StreamingResponse(event_generator(), media_type="text/event-stream")


@app.get("/api/topics/verify")
def api_verify_topics(no_scan: bool = False, distro: str = "jazzy"):
    verify_script = os.path.join(REPO_ROOT, "scripts", "verify_topics.py")
    flag = " --no-scan" if no_scan else ""
    cmd = get_ros_exec_cmd(f"python3 {verify_script} --json{flag}", distro=distro)
    try:
        res = subprocess.run(cmd, capture_output=True, text=True, timeout=20)
        parsed = None
        for line in reversed(res.stdout.splitlines()):
            line = line.strip()
            if line.startswith("{") and line.endswith("}"):
                try:
                    parsed = json.loads(line)
                    break
                except Exception:
                    pass
        return {
            "success": res.returncode == 0,
            "exit_code": res.returncode,
            "data": parsed,
            "raw": res.stdout or res.stderr,
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@app.get("/api/maps")
def list_maps():
    maps_dir = os.path.join(REPO_ROOT, "maps")
    if not os.path.isdir(maps_dir):
        return {"maps": [], "maps_dir": maps_dir}
    files = os.listdir(maps_dir)
    map_list = []
    for f in files:
        if f.endswith(".yaml"):
            base = f[:-5]
            map_list.append({
                "name": base,
                "yaml": f,
                "image": f"{base}.pgm" if f"{base}.pgm" in files else None,
            })
    # maps_dir is part of the contract: Nav2's map_server takes a filesystem
    # path (map:=/abs/path.yaml), not a URL, so the caller cannot build the
    # argument from the file names alone.
    return {"maps": map_list, "maps_dir": maps_dir}


@app.get("/api/maps/{filename}")
def get_map_file(filename: str):
    maps_dir = os.path.join(REPO_ROOT, "maps")
    file_path = os.path.join(maps_dir, filename)
    if not os.path.isfile(file_path):
        raise HTTPException(status_code=404, detail="Map file not found")
    media_type = "application/x-yaml" if filename.endswith(".yaml") else "application/octet-stream"
    return FileResponse(file_path, media_type=media_type)
