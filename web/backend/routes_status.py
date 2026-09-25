"""Linorobot2 Cockpit routes -- status, git info, serial/MCU ports, boards, wiring table, sensors listing.

Part of the main.py split: route handlers registered on the shared `app`
as an import side effect (main.py imports this module). Shared state and
helpers come from core.py. See core.py for the split's contract.
"""
import glob
import json
import os
import re
import socket
import subprocess
import yaml
from typing import Optional
from fastapi import HTTPException, Request
from fastapi.responses import StreamingResponse
from core import (
    SIM_MCU,
    active_robot_name,
    CONFIG_DIR,
    REPO_ROOT,
    access,
    agent_runner,
    app,
    bringup_runner,
    display_path,
    gamepad_runner,
    get_active_params_path,
    get_controller,
    get_git_info_dict,
    get_package_install_info,
    get_robots_list,
    get_sensor_driver_status,
    get_serial_ports_dict,
    host_lan_ip,
    json_body,
    laser_runner,
    list_dir,
    list_local_ports,
    list_robot_config_files,
    load_params,
    main_runner,
    mcu_identity,
    mcu_probe,
    nav2_stack_status,
    one_click_pipeline,
    probe_stack_liveness,
    sensor_registry,
    suggest_mcu,
    syslog_manager,
    text_field,
)


def _board_id_for_port(port: str):
    """What the board on `port` called itself at its last boot, or None.

    Read from the flash stamp rather than the bus: the VID:PID says which KIND of
    part is plugged in, the banner says WHICH ONE -- the difference that matters
    when two identical boards sit on one bench. `kind` carries the claim: "uid" is
    the silicon's own identifier (RP2350 chip info, ESP32 eFuse MAC), "flashid"
    only the external flash chip's, which is all an RP2040 has. They are never
    merged, so a replaced flash cannot read as the same chip.

    The stamp is named <env>_<port>.json and one board can have stamps under more
    than one env name (pico2 and pico2w are the same silicon), so match on the
    port and take the most recently written.
    """
    if not port:
        return None
    base = os.path.basename(port)
    try:
        stamps = glob.glob(os.path.join(mcu_probe.STAMP_DIR, f"*_{base}.json"))
    except Exception:
        return None
    for path in sorted(stamps, key=os.path.getmtime, reverse=True):
        try:
            with open(path) as fh:
                stamp = json.load(fh)
        except Exception:
            continue
        if stamp.get("board_id"):
            return {"id": stamp["board_id"],
                    "kind": stamp.get("id_kind") or "uid",
                    "confirmed": bool(stamp.get("banner_confirmed"))}
    return None


@app.get("/api/status")
def get_status(controller: Optional[str] = None):
    params = load_params()
    robot = params.get("robot", {})
    git = get_git_info_dict()

    syslog_stat = syslog_manager.get_status()
    ports_info = get_serial_ports_dict(params)

    controller_hint = (controller or "").strip()
    controller = get_controller(params)
    # Whether anything was actually found on the bus, kept separate from the
    # fallback. The two used to be indistinguishable downstream: with no board
    # detected the UI still printed "Auto-Detected MCU: Controller: pico2" --
    # a configured name presented as a probe result -- above a stale VID:PID
    # line from the previous poll. That happens routinely, because a board in
    # BOOTSEL has no tty, which is exactly when it is being flashed.
    mcu_detected = False
    detected_mcu = controller.get("name") or "pico2"
    detected_chip = "Controller: " + detected_mcu
    detected_port = ""
    for lp in ports_info.get("local_ports", []):
        hint = lp.get("mcu_hint")
        if hint:
            detected_mcu = hint
            detected_chip = lp.get("chip", hint)
            detected_port = lp.get("path", "") or lp.get("port", "")
            mcu_detected = True
            break

    # Does the bus positively contradict the configured controller? Answered
    # here rather than in the browser so there is one vid/pid table in the
    # project (scripts/mcu_identity.py) and the warning can never disagree with
    # the guard that refuses the flash.
    mcu_mismatch = None
    if mcu_detected:
        # Judge what 1-Click will ACTUALLY run. The browser passes its live
        # selection, which can differ from the config's own base_controller --
        # a Reference Build preset changes the former and not the latter -- and
        # warning about the config while the run uses something else is a false
        # alarm in one direction and silence in the other.
        want_name = (controller_hint or controller.get("name") or "")
        want = mcu_identity.env_family(want_name)
        got = mcu_identity.env_family(detected_mcu)
        for lp in ports_info.get("local_ports", []):
            if lp.get("mcu_hint") == detected_mcu:
                got = got or ""
                decisive = bool(lp.get("decisive"))
                if mcu_identity.mismatch(want, got, decisive):
                    mcu_mismatch = {
                        "expected": mcu_identity.FAMILY_LABEL.get(want, want),
                        "detected": mcu_identity.FAMILY_LABEL.get(got, got),
                        "controller": want_name,
                        "chip": detected_chip,
                    }
                break

    # What the board called ITSELF at its last boot, as recorded by the flasher
    # from the banner. This is not the VID:PID above: that says which kind of
    # part is on the bus, this says WHICH ONE -- the difference that matters when
    # two identical boards are on one bench. The key is the claim: `uid` is the
    # silicon's own id (RP2350 chip info, ESP32 eFuse MAC), `flashid` is only the
    # external flash chip's, which is all an RP2040 has to offer. Absent for a
    # board this host has never flashed, or one running an image without the field.
    board_id = _board_id_for_port(detected_port) if mcu_detected else None

    ctrl_sensors = controller.get("sensors", {}) or {}
    sim_mode_active = bool(
        str(controller.get("name") or "").lower() == SIM_MCU or
        ctrl_sensors.get("use_sim_wheel") or
        ctrl_sensors.get("use_sim_imu") or
        ctrl_sensors.get("use_sim_ld19")
    )

    # Ask the same list the bringup command itself sources, or the two disagree:
    # the container image ships its workspace at /opt/lino_ws/setup.bash, which
    # this check used to miss entirely, so the Bringup tab declared the workspace
    # unbuilt and tried to colcon build a directory that is not there.
    ws_setup = one_click_pipeline.workspace_setup()
    ws_built = bool(ws_setup)
    ws_path = (os.path.dirname(os.path.dirname(ws_setup))
               if ws_setup.endswith(os.path.join("install", "setup.bash"))
               else os.path.dirname(ws_setup)) if ws_setup else \
        os.path.abspath(os.path.join(REPO_ROOT, "..", ".."))

    liveness = probe_stack_liveness()
    agent_external = liveness["agent"]
    bringup_external = liveness["bringup"]

    controller_name = controller.get("name") or "pico2"
    active_path_rel = display_path(get_active_params_path())

    # The installed ROS wins over the configured name, for the same reason the
    # I2C bus wins over the configured IMU (§5): the YAML is a CLAIM about the
    # computer and /opt/ros is the computer. `ros_distro` is not a property of
    # the robot at all -- it belongs to whichever machine runs the stack -- so a
    # config pinning `jazzy` that is opened on a lyrical-only box was announcing
    # "Jazzy (Noble 24.04)" in the header while every node on the box came from
    # /opt/ros/lyrical, and the payload saying so carried the agent's own
    # `/opt/ros/lyrical/bin/ros2` command line two keys further down.
    #
    # That is not cosmetic: "Start 1-Click" reads this selector, so the button
    # would have run a JAZZY pipeline against a box with no jazzy installed --
    # the §12 class of bug where the UI and the stack disagree and nothing says
    # which is right. The configured value is honoured whenever it is actually
    # installed, so a machine carrying both distros still does what the config
    # asks; it is overridden only when it names a distro that is not there, and
    # the override is reported rather than silently applied.
    installed = [d for d in ("jazzy", "lyrical", "rolling")
                 if os.path.exists(os.path.join("/opt/ros", d))]
    configured = params.get("ros_distro")
    distro_override = None
    if configured and configured in installed:
        active_distro = configured
    elif installed:
        active_distro = installed[0]
        if configured:
            distro_override = (f"config says {configured}, which is not installed here; "
                               f"using {active_distro}")
    else:
        active_distro = configured or os.environ.get("ROS_DISTRO", "jazzy")

    return {
        "status": "online",
        "robot_name": robot.get("name", active_robot_name()),
        "controller": controller_name,
        "base_controller": controller,
        "robot_config_path": active_path_rel,
        "robots": get_robots_list(params),
        "detected_mcu": detected_mcu,
        "detected_chip": detected_chip,
        "mcu_detected": mcu_detected,
        # Anything flashable on the bus at all, tty or not: a board in BOOTSEL has
        # no tty, so mcu_detected alone would call it missing mid-flash. The UI
        # switches to the simulated MCU only when this is false -- the same rule
        # the 1-Click pipeline falls back on (no_board_attached).
        "board_on_bus": bool(mcu_detected or _bus_devices()),
        "mcu_mismatch": mcu_mismatch,
        "board_id": board_id,
        "host_ip": ports_info.get("host_ip", ""),
        "config_dir": display_path(CONFIG_DIR),
        "sim_mode_active": sim_mode_active,
        "ros_distro": active_distro,
        "ros_distro_configured": configured,
        "ros_distro_override": distro_override,
        "ros_distro_installed": installed,
        "supported_distros": ["jazzy", "lyrical", "rolling"],
        "ros2_installed": True,
        "workspace_path": ws_path,
        "workspace_built": ws_built,
        "agent_alive_external": agent_external,
        "bringup_alive_external": bringup_external,
        "liveness": liveness,
        "repo_root": REPO_ROOT,
        "web_dir": os.path.join(REPO_ROOT, "web"),
        "git": git,
        "git_branch": git.get("branch", ""),
        "ports": ports_info,
        "config": params,
        "syslog": syslog_stat,
        "is_rootless": syslog_manager.is_rootless,
        "agent_busy_console": agent_runner.is_busy(),
        "bringup_busy_console": bringup_runner.is_busy(),
        "main_busy": main_runner.is_busy(),
        "laser_busy": laser_runner.is_busy(),
        "gamepad_running": gamepad_runner.is_running(),
    }


@app.get("/api/gitinfo")
def api_gitinfo():
    return get_git_info_dict()


@app.get("/api/gitinfo/branch")
@app.post("/api/gitinfo/branch")
async def api_gitinfo_branch(request: Request):
    data = {}
    if request.method == "POST":
        try:
            data = await json_body(request)
        except Exception:
            pass
    branch = text_field(data, "branch") or (request.query_params.get("branch") or "").strip()
    if not branch or not re.match(r"^[A-Za-z0-9._/-]+$", branch):
        raise HTTPException(
            status_code=400,
            detail=f"Invalid branch name: '{branch}'. Use letters, numbers, dot, underscore, slash, hyphen."
        )

    def event_generator():
        try:
            current_branch = subprocess.check_output(
                ["git", "branch", "--show-current"], cwd=REPO_ROOT, text=True, stderr=subprocess.DEVNULL
            ).strip()
        except Exception:
            current_branch = ""

        if current_branch == branch:
            yield f"data: {json.dumps({'line': f'Already on branch {branch}'})}\n\n"
            yield f"data: {json.dumps({'exit_code': 0, 'branch': branch})}\n\n"
            return

        # Check if local branch exists
        check_local = subprocess.run(
            ["git", "rev-parse", "--verify", f"refs/heads/{branch}"],
            cwd=REPO_ROOT,
            capture_output=True,
            text=True,
        )
        if check_local.returncode == 0:
            cmd = ["git", "checkout", branch]
        else:
            # Check if remote branch exists
            check_remote = subprocess.run(
                ["git", "rev-parse", "--verify", f"refs/remotes/origin/{branch}"],
                cwd=REPO_ROOT,
                capture_output=True,
                text=True,
            )
            if check_remote.returncode == 0:
                cmd = ["git", "checkout", "-b", branch, "--track", f"origin/{branch}"]
            else:
                # Create a new local branch from current HEAD
                cmd = ["git", "checkout", "-b", branch]

        cmd_str = " ".join(cmd)
        yield f"data: {json.dumps({'line': f'$ {cmd_str}'})}\n\n"

        proc = subprocess.Popen(
            cmd,
            cwd=REPO_ROOT,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
        )
        has_output = False
        if proc.stdout:
            for line in iter(proc.stdout.readline, ""):
                line_str = line.rstrip("\r\n")
                if line_str:
                    has_output = True
                    yield f"data: {json.dumps({'line': line_str})}\n\n"
            proc.stdout.close()
        proc.wait()

        if proc.returncode == 0 and not has_output:
            yield f"data: {json.dumps({'line': f'Switched to branch {branch}'})}\n\n"

        yield f"data: {json.dumps({'exit_code': proc.returncode, 'branch': branch})}\n\n"

    return StreamingResponse(event_generator(), media_type="text/event-stream")


def _bus_devices() -> list:
    try:
        return mcu_identity.identify_bus()
    except Exception:
        return []


@app.get("/api/serial_ports")
def api_serial_ports():
    sp = get_serial_ports_dict()
    # Both shapes: the flat list for the chip pickers, the dicts for the rest.
    return {"ports": [p["path"] for p in sp["local_ports"]], **sp}


@app.get("/api/mcu/ports")
def api_mcu_ports():
    """The USB serial devices on this machine, and the MCU each one looks like."""
    ports = list_local_ports()
    return {
        "status": "ok",
        "host_ip": host_lan_ip(),
        "hostname": socket.gethostname(),
        "ports": ports,
        "suggested": suggest_mcu(ports),
    }


# ==============================================================================
# Sensors & Workspace Diagnostics APIs
# ==============================================================================
@app.get("/api/sensors")
def api_sensors():
    return sensor_registry()


@app.get("/api/sensors/driver_status")
def api_sensor_driver_status(sensor: str = "", ws: str = ""):
    return get_sensor_driver_status(sensor, ws=ws)


@app.get("/api/package/check")
def api_package_check(pkg: str = "", distro: str = "jazzy", ws: str = ""):
    return get_package_install_info(pkg, distro=distro, ws=ws)


@app.get("/api/nav2/stack")
def api_nav2_stack(distro: str = "jazzy", ws: Optional[str] = None):
    return nav2_stack_status(distro=distro, ws=ws)


@app.get("/api/list_dir")
def api_list_dir(path: str = "", only: str = "any", exts: str = "", start: bool = False):
    """One folder for the browser's picker, with the parent it may go up to.

    `start=true` marks `path` as where the picker OPENS -- the field's current
    value, which may be a file, or a folder outside the fence (the robot
    image's workspace is /opt/lino_ws). That opens the default folder with a
    notice rather than a 403: every picker on such a field failed to open.
    """
    target = os.path.expanduser(path or REPO_ROOT)
    notice = ""
    if start and os.path.isfile(target):
        target = os.path.dirname(target)
    if not access.path_allowed(target, CONFIG_DIR, REPO_ROOT):
        if not start:
            raise HTTPException(status_code=403, detail=f"Not a browsable location: {target}")
        notice = f"{target} is outside the folders this page may browse; showing {REPO_ROOT}."
        target = REPO_ROOT
    out = list_dir(target, only=only, exts=exts)
    here = out.get("path") or target
    parent = os.path.dirname(here)
    out["parent"] = parent if parent != here and access.path_allowed(parent, CONFIG_DIR, REPO_ROOT) else ""
    if notice:
        out["notice"] = notice
    return out


# ==============================================================================
# Boards & Config Engine Diagnostics APIs
# ==============================================================================
@app.get("/api/boards")
def api_boards():
    ini_path = os.path.join(REPO_ROOT, "firmware", "platformio.ini")
    base_ini = os.path.join(REPO_ROOT, "firmware", "common", "platformio_base.ini")
    boards = []
    seen = set()

    # This robot's controller first, then every other robot's, so the selector
    # still offers the full set of known controllers to switch to.
    for cfg_path in [get_active_params_path()] + list_robot_config_files():
        try:
            with open(cfg_path, "r") as f:
                yd = yaml.safe_load(f) or {}
        except Exception:
            continue
        c_cfg = yd.get("base_controller") or {}
        c_name = c_cfg.get("name")
        if not c_name or c_name in seen:
            continue
        seen.add(c_name)
        boards.append({
            "env": c_name,
            "board": c_cfg.get("board", c_name),
            "platform": c_cfg.get("platform", ""),
            "mcu": c_cfg.get("mcu", ""),
            "transport": c_cfg.get("transport", "serial"),
            "baudrate": c_cfg.get("baudrate", 921600),
            "description": c_cfg.get("description", c_name),
            "robot": yd.get("robot", {}).get("name", ""),
        })
    return {"status": "ok", "boards": boards}


@app.get("/api/wiring_table")
def api_wiring_table():
    """The wiring chart for the active robot, as Markdown -- the sheet you want
    at the bench, generated from the same pins the Pin Matrix holds."""
    try:
        import gen_wiring_table
    except Exception as exc:
        raise HTTPException(status_code=500, detail=f"gen_wiring_table: {exc}")
    params = load_params()
    return {"status": "ok",
            "robot": (params.get("robot") or {}).get("name", ""),
            "markdown": gen_wiring_table.render(params)}
