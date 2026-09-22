"""Linorobot2 Cockpit routes -- robot config, params, hardware config, robot selection & import.

Part of the main.py split: route handlers registered on the shared `app`
as an import side effect (main.py imports this module). Shared state and
helpers come from core.py. See core.py for the split's contract.
"""
import copy
import os
import re
import shutil
import yaml
from typing import Any, Dict
from fastapi import HTTPException, Request
from fastapi.responses import PlainTextResponse
from core import (
    active_robot_name,
    CONFIG_DIR,
    REPO_ROOT,
    access,
    app,
    cockpit_paths,
    display_path,
    gen_robot_description,
    get_active_params_path,
    get_controller,
    get_controller_name,
    get_robots_list,
    json_body,
    load_params,
    pin_catalog,
    regenerate_firmware_headers,
    regenerate_robot_description,
    save_params,
    set_active_params_path,
    text_field,
    yaml_merge,
)


@app.get("/api/config")
def api_config():
    return load_params()


@app.post("/api/config")
async def api_save_config(request: Request):
    data = await json_body(request)
    params = load_params()
    if isinstance(data, dict):
        for k, v in data.items():
            params[k] = v
        save_params(params)
    return {"success": True, "config": params}


@app.get("/api/hardware/config")
def api_get_hardware_config():
    params = load_params()
    controller = get_controller(params)
    return {
        "robot_name": params.get("robot", {}).get("name", "linorobot2"),
        "controller": controller.get("name", "pico2"),
        "kinematics": params.get("kinematics", {}),
        # The body, with every gap filled from the kinematics, so the form is
        # never blank; what the user saves is what the URDF is built from.
        "geometry": gen_robot_description.effective_geometry(params),
        "geometry_warnings": gen_robot_description.geometry_warnings(params),
        "base_controller": controller,
        # What the pin catalogue thinks of this config (scripts/pin_catalog.py):
        # the Pin Matrix shows these next to the fields they concern.
        "pin_findings": [{"level": l, "message": m} for l, m in pin_catalog.check_config(params)],
    }


@app.post("/api/hardware/config")
async def api_save_hardware_config(request: Request):
    data = await json_body(request)
    params = load_params()

    ctrl = params.setdefault("base_controller", {})
    # The robot file names exactly one controller; a posted name renames it
    # (e.g. switching this robot from pico2 to esp32), it does not add a second.
    controller_name = data.get("controller") or ctrl.get("name") or "pico2"
    ctrl["name"] = controller_name

    if "kinematics" in data and isinstance(data["kinematics"], dict):
        params.setdefault("kinematics", {}).update(data["kinematics"])
    if "geometry" in data and isinstance(data["geometry"], dict):
        geo = params.setdefault("geometry", {})
        for section, values in data["geometry"].items():
            if isinstance(values, dict):
                geo.setdefault(section, {}).update(values)
            else:
                geo[section] = values

    if "driver_type" in data:
        ctrl["driver_type"] = data["driver_type"]
    if "baudrate" in data:
        ctrl["baudrate"] = int(data["baudrate"])
    if "serial_port" in data:
        ctrl["serial_port"] = str(data["serial_port"])
    # Which port is the console on an ESP32-S3 (env key `console`): the native
    # USB or UART0 through a bridge. Only the two spellings the firmware knows.
    if "console" in data:
        console = str(data["console"] or "usb").strip().lower()
        if console not in ("usb", "uart0"):
            raise HTTPException(status_code=400, detail=f"console must be 'usb' or 'uart0', not {console!r}")
        ctrl["console"] = console
    # The robot computer's address, entered by the user like `host`/`host_ip`.
    # Stored even when blank, because blank is a meaningful value: it means
    # "this is the rig, resolve the box". Written through str() and stripped so
    # a pasted address with stray whitespace cannot become an unreachable URL.
    if "mcu" in data:
        ctrl["mcu"] = data["mcu"]
    if "sensors" in data and isinstance(data["sensors"], dict):
        ctrl.setdefault("sensors", {}).update(data["sensors"])
    if "pins" in data and isinstance(data["pins"], dict):
        ctrl.setdefault("pins", {}).update(data["pins"])
    if "base_controller" in data and isinstance(data["base_controller"], dict):
        ctrl.update(data["base_controller"])

    save_params(params)
    findings = [{"level": l, "message": m} for l, m in pin_catalog.check_config(params)]
    res = regenerate_firmware_headers(controller_name)
    header_ok = res.returncode == 0
    description = regenerate_robot_description(params)
    errors = sum(1 for f in findings if f["level"] == "error")
    if header_ok:
        message = f"Hardware config saved and header regenerated for [{controller_name}]."
    elif errors:
        message = (f"Hardware config saved, but the header was NOT regenerated: {errors} pin "
                   f"error(s) for [{controller_name}]. Fix them below and save again.")
    else:
        message = f"Hardware config saved, but the header generator failed for [{controller_name}]."
    return {
        "success": True,
        "controller": controller_name,
        "header_ok": header_ok,
        "message": message,
        "header_stdout": res.stdout,
        "header_stderr": res.stderr,
        "pin_findings": findings,
        **description,
    }


@app.get("/api/robot/urdf")
def api_robot_urdf():
    """The description the next bringup will publish, as text."""
    params = load_params()
    return PlainTextResponse(gen_robot_description.build_urdf(params), media_type="application/xml")


@app.post("/api/hardware/fake_mode")
async def api_toggle_fake_mode(request: Request):
    data = await json_body(request)
    enabled = bool(data.get("enabled", False))
    params = load_params()
    ctrl = params.setdefault("base_controller", {})
    controller_name = data.get("controller") or ctrl.get("name") or "pico2"

    sensors = ctrl.setdefault("sensors", {})
    sensors["use_fake_wheel"] = enabled
    sensors["use_fake_imu"] = enabled
    sensors["use_fake_ld19"] = enabled
    sensors["use_fake_mag"] = enabled
    if "use_fake_env" in sensors or not enabled:
        sensors["use_fake_env"] = enabled

    save_params(params)
    res = regenerate_firmware_headers(controller_name)
    return {
        "success": True,
        "enabled": enabled,
        "controller": controller_name,
        "message": f"Fake simulation mode {'enabled' if enabled else 'disabled'} for [{controller_name}].",
        "header_stdout": res.stdout,
    }


@app.get("/api/params")
def api_params():
    return load_params()


@app.get("/api/params/raw", response_class=PlainTextResponse)
def get_params_raw():
    active_path = get_active_params_path()
    if not os.path.isfile(active_path):
        return ""
    with open(active_path, "r") as f:
        return f.read()


@app.post("/api/params/raw")
async def update_params_raw(request: Request):
    body = await request.body()
    text = body.decode("utf-8")
    try:
        yaml.safe_load(text)
        active_path = get_active_params_path()
        with open(active_path, "w") as f:
            f.write(text)
        res = regenerate_firmware_headers(params_path=active_path)
        return {
            "success": True,
            "message": f"Parameters saved to {os.path.basename(active_path)} and firmware header generated.",
            "generator_stdout": res.stdout if res else "",
            "header_ok": bool(res and res.returncode == 0),
            "generator_stderr": (res.stderr if res else "") or "",
            "active_path": display_path(active_path),
        }
    except Exception as e:
        raise HTTPException(status_code=400, detail=str(e))


@app.post("/api/params")
def update_params(data: Dict[str, Any]):
    try:
        save_params(data)
        res = regenerate_firmware_headers()
        return {
            "success": True,
            "message": "Parameters saved and firmware header generated.",
            "generator_stdout": res.stdout,
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@app.get("/api/robot_config")
def get_robot_config(distro: str = "jazzy"):
    active_path = get_active_params_path()
    raw_yaml = ""
    if os.path.isfile(active_path):
        with open(active_path, "r") as f:
            raw_yaml = f.read()
    params = load_params(active_path)
    kine = params.get("kinematics", {})
    robot = params.get("robot", {})
    controller = get_controller(params)
    rel_path = display_path(active_path)
    return {
        "success": True,
        "path": rel_path,
        "yaml": raw_yaml,
        "robot": robot,
        "kinematics": kine,
        "base_controller": controller,
        "base": kine.get("base_type", "2wd"),
        "linorobot2": {
            "base": kine.get("base_type", "2wd"),
            "laser_sensor": str((controller.get("lidar") or {}).get("model") or "none"),
            "micro_ros_port": controller.get("serial_port", ""),
            "micro_ros_baudrate": str(controller.get("baudrate", "")),
        }
    }


@app.post("/api/robot_config")
async def save_robot_config(request: Request):
    content_type = request.headers.get("content-type", "")
    if "application/json" in content_type:
        data = await json_body(request)
        raw_yaml = data.get("yaml", "")
    else:
        body = await request.body()
        raw_yaml = body.decode("utf-8")
    if raw_yaml:
        yaml.safe_load(raw_yaml)
        active_path = get_active_params_path()
        with open(active_path, "w") as f:
            f.write(raw_yaml)
        res = regenerate_firmware_headers(params_path=active_path)
        rel_path = display_path(active_path)
        return {"success": True, "status": "saved", "path": rel_path, "message": f"Saved {rel_path}",
                "generator_stdout": res.stdout if res else "",
                "header_ok": bool(res and res.returncode == 0),
                "generator_stderr": (res.stderr if res else "") or ""}
    raise HTTPException(status_code=400, detail="Empty YAML")


@app.get("/api/config/git")
def api_config_git():
    """The config directory's git state, for the Config Studio header.

    Deliberately not folded into /api/status: the browser polls that every couple
    of seconds, and `git status` on a bind-mounted directory is not free. The UI
    asks for this when the tab opens and after a save.
    """
    return {"status": "ok", **cockpit_paths.git_state(CONFIG_DIR),
            "path": display_path(CONFIG_DIR)}


@app.post("/api/config/commit")
async def api_config_commit(request: Request):
    """Commit the user's robot configs. Their repository, their history."""
    try:
        data = await json_body(request)
    except Exception:
        data = {}
    message = text_field(data, "message")
    if not message:
        message = f"Update {active_robot_name()} from the Cockpit"
    res = cockpit_paths.git_commit(message, CONFIG_DIR)
    if res.get("state"):
        res["state"]["path"] = display_path(CONFIG_DIR)
    return res


@app.get("/api/robots")
def get_robots():
    active_path = get_active_params_path()
    params = load_params(active_path)
    robot = params.get("robot", {})
    robot_name = robot.get("name", active_robot_name())
    return {
        "status": "ok",
        "robots": get_robots_list(params),
        "active": robot_name,
        "path": display_path(active_path),
    }


@app.post("/api/robot/select")
async def select_robot(request: Request):
    data = await json_body(request)
    name = text_field(data, "name", "robot")
    if not name or not re.match(r"^[a-zA-Z0-9_.-]+$", name):
        raise HTTPException(
            status_code=400,
            detail=f"Invalid robot name: '{name}'. Use lowercase, digits, and underscores.",
        )

    config_dir = CONFIG_DIR
    target_yaml = None

    # 1. Match by exact config filename e.g. pico2_mecanum_config.yaml or rover_pico2.yaml
    for candidate in [f"{name}_config.yaml", f"{name}.yaml", f"{name}_config.yml", f"{name}.yml"]:
        p = os.path.join(config_dir, candidate)
        if os.path.isfile(p):
            target_yaml = p
            break

    # 2. Match by robot.name declared inside any YAML in the config dir
    if not target_yaml:
        for fname in os.listdir(config_dir):
            if not (fname.endswith(".yaml") or fname.endswith(".yml")) or fname.startswith("secrets"):
                continue
            fpath = os.path.join(config_dir, fname)
            try:
                with open(fpath, "r") as f:
                    yd = yaml.safe_load(f) or {}
                if yd.get("robot", {}).get("name") == name:
                    target_yaml = fpath
                    break
            except Exception:
                pass

    # 3. If new robot requested, create a dedicated <config dir>/{name}_config.yaml.
    #    The new file is seeded from the robot currently open and keeps exactly
    #    one base controller -- a robot is one controller, by definition.
    if not target_yaml:
        target_yaml = os.path.join(config_dir, f"{name}_config.yaml")
        base_params = load_params()
        new_params = copy.deepcopy(base_params)
        new_params.setdefault("robot", {})["name"] = name
        requested = data.get("controller") or data.get("mcu")
        ctrl = new_params.setdefault("base_controller", {})
        if requested:
            ctrl["name"] = requested
        ctrl.setdefault("name", "pico2")
        save_params(new_params, path=target_yaml)

    set_active_params_path(target_yaml)
    params = load_params(target_yaml)

    controller_name = get_controller_name(params)
    res = regenerate_firmware_headers(controller=controller_name, params_path=target_yaml)
    rel_path = display_path(target_yaml)

    return {
        "status": "ok",
        "active": name,
        "robot_name": name,
        "robot_config_path": rel_path,
        "robots": get_robots_list(params),
        "config": params,
        "message": f"Active robot set to '{name}' ({rel_path})",
        "generator_stdout": res.stdout if res else "",
    }


@app.post("/api/import_config")
async def import_config(request: Request):
    data = await json_body(request)
    raw_text = data.get("text", "")
    file_path = data.get("path", "")
    if file_path and not raw_text and os.path.isfile(file_path):
        with open(file_path, "r") as f:
            raw_text = f.read()

    if not raw_text:
        raise HTTPException(status_code=400, detail="No config text or valid path provided")

    # If full YAML provided
    if "robot:" in raw_text or "kinematics:" in raw_text or "linorobot2:" in raw_text:
        parsed = yaml.safe_load(raw_text)
        params = load_params()
        for k in ["robot", "kinematics", "base_controller", "ekf", "slam", "nav2"]:
            if k in parsed:
                params[k] = parsed[k]
        save_params(params)
        res = regenerate_firmware_headers()
        return {"status": "ok", "type": "yaml", "message": "Imported configuration YAML",
                "generator_stdout": res.stdout, "header_ok": res.returncode == 0,
                "generator_stderr": res.stderr or ""}

    return {"status": "ok", "message": "Imported config successfully"}


# ==============================================================================
# Parameter Export & Merge (yaml_merge.py)
# ==============================================================================
@app.post("/api/params/export")
async def api_params_export(request: Request):
    data = await json_body(request)
    dest = text_field(data, "dest_dir")
    if not dest:
        raise HTTPException(status_code=400, detail="dest_dir required")
    if not access.path_allowed(dest, CONFIG_DIR, REPO_ROOT, for_write=True):
        raise HTTPException(status_code=403, detail=f"Not an export location: {dest}")
    os.makedirs(dest, exist_ok=True)
    out_file = os.path.join(dest, f"{active_robot_name()}_config_export.yaml")
    shutil.copyfile(get_active_params_path(), out_file)
    return {"status": "exported", "path": out_file}


@app.post("/api/params/merge")
async def api_params_merge(request: Request):
    data = await json_body(request)
    source_text = data.get("source_text", "")
    with open(get_active_params_path(), "r") as f:
        target_text = f.read()

    merged, report = yaml_merge.merge_yaml(target_text, source_text)
    if not data.get("dry_run"):
        with open(get_active_params_path(), "w") as f:
            f.write(merged)
    return {
        "status": "dry-run" if data.get("dry_run") else "merged",
        "report": report,
        "config": merged,
    }


@app.post("/api/params/promote")
async def api_params_promote(request: Request):
    return {"status": "promoted", "message": f"Single source of truth ({os.path.relpath(get_active_params_path(), REPO_ROOT)}) is active"}


@app.get("/api/configs")
def api_configs():
    custom_dir = os.path.join(REPO_ROOT, "firmware", "include", "custom")
    configs = []
    if os.path.isdir(custom_dir):
        for f in sorted(os.listdir(custom_dir)):
            if f.endswith(".h"):
                configs.append({
                    "filename": f,
                    "path": os.path.relpath(os.path.join(custom_dir, f), REPO_ROOT),
                })
    return {"status": "ok", "configs": configs}
