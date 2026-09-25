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
    SIM_MCU,
    text_field,
    yaml_merge,
)

import gen_bare_config  # scripts/ is on sys.path via core


def write_bare_robot(name: str):
    """(Re)generate `bare_<mcu>` from the bare rule and return its path, or None.

    A bare module is a rule, not a file: selecting one writes it fresh, so it
    never carries an older release's pins or template (the pipeline does the same
    on every run). `bare_sim` is the Sim MCU robot: no board, every simulated
    device on, every pin -1."""
    m = re.fullmatch(r"bare_([a-z0-9]+)", name or "")
    if not m or m.group(1) not in gen_bare_config.KNOWN:
        return None
    path = os.path.join(CONFIG_DIR, f"{name}_config.yaml")
    with open(path, "w") as fh:
        yaml.safe_dump(gen_bare_config.bare_config(m.group(1)), fh, sort_keys=False)
    return path


# The Sim MCU robot is always offered in the Robot selector.
try:
    if os.path.isdir(CONFIG_DIR) and not os.path.isfile(os.path.join(CONFIG_DIR, "bare_sim_config.yaml")):
        write_bare_robot("bare_sim")
except Exception as exc:  # a read-only config dir must not stop the cockpit
    print(f"[routes_config] bare_sim not written: {exc}")


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
    # The simulated robot's load and drivetrain losses. Merged rather than
    # replaced: the form carries the mass and the four loss terms, and the room
    # (map_width, the obstacle wall) is set elsewhere -- a whole-block assignment
    # would drop it.
    if "simulation" in data and isinstance(data["simulation"], dict):
        ctrl.setdefault("simulation", {}).update(data["simulation"])
    if "base_controller" in data and isinstance(data["base_controller"], dict):
        ctrl.update(data["base_controller"])

    # Re-derive the Nav2 limits from the motors this robot now has.
    #
    # ON by default, because the alternative is what shipped: limits asking 103%
    # of a differential base's motors, 171% at the smoother's ceiling and 129%
    # on mecanum, with nothing checking. Changing the wheel diameter or the mass
    # silently invalidates hand-tuned limits, and this is the one place that
    # knows the change happened. `kinematics.auto_nav2_limits: false` hands
    # control back to whoever wants to tune by hand -- and then nothing here
    # touches their values.
    import drivetrain_report as dr

    try:
        nav2_changes = dr.apply_nav2_limits(params)
    except (ValueError, TypeError, SystemExit) as exc:
        # A half-entered chassis must not block saving the chassis.
        nav2_changes, nav2_error = {}, str(exc)
    else:
        nav2_error = None

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
    if nav2_changes:
        message += (f" Nav2 limits re-derived from the motors "
                    f"({len(nav2_changes)} value(s) changed).")
    elif nav2_error:
        message += f" Nav2 limits NOT re-derived: {nav2_error}"

    return {
        "success": True,
        "controller": controller_name,
        "header_ok": header_ok,
        "message": message,
        "nav2_auto": dr.auto_limits_enabled(params),
        "nav2_changes": {k: v for k, v in nav2_changes.items()},
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


@app.post("/api/hardware/sim_mode")
async def api_toggle_sim_mode(request: Request):
    data = await json_body(request)
    enabled = bool(data.get("enabled", False))
    params = load_params()
    ctrl = params.setdefault("base_controller", {})
    controller_name = data.get("controller") or ctrl.get("name") or "pico2"
    # The Sim MCU has no real device to switch to: turning its simulation off
    # left bare_sim with no IMU, wheels, LiDAR, sonar or battery at all.
    if not enabled and str(ctrl.get("name") or "").lower() == SIM_MCU:
        raise HTTPException(status_code=400, detail=(
            "The Sim MCU is simulation only. To design real hardware, pick your board "
            "on the Base & MCU tab."))

    sensors = ctrl.setdefault("sensors", {})
    sensors["use_sim_wheel"] = enabled
    sensors["use_sim_imu"] = enabled
    sensors["use_sim_ld19"] = enabled
    sensors["use_sim_mag"] = enabled
    if "use_sim_env" in sensors or not enabled:
        sensors["use_sim_env"] = enabled
    if "use_sim_sonar" in sensors or not enabled:
        sensors["use_sim_sonar"] = enabled
    sensors["use_sim_battery"] = enabled

    save_params(params)
    res = regenerate_firmware_headers(controller_name)
    return {
        "success": True,
        "enabled": enabled,
        "controller": controller_name,
        "message": f"Sim simulation mode {'enabled' if enabled else 'disabled'} for [{controller_name}].",
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
    target_yaml = write_bare_robot(name)

    # 1. Match by exact config filename e.g. pico2_mecanum_config.yaml or rover_pico2.yaml
    for candidate in ([] if target_yaml else [f"{name}_config.yaml", f"{name}.yaml", f"{name}_config.yml", f"{name}.yml"]):
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


# ---------------------------------------------------------------------------
# The drivetrain HUD.
#
# The Kinematics HUD on the Hardware tab has always shown circumference, ticks
# per metre and a top speed from `pi*d*rpm/60`. Those are arithmetic. What a
# person actually needs before they order parts -- how fast does it accelerate,
# how long to reach speed, how far to stop, and is the Nav2 tuning in this same
# config asking for more than the motors can give -- needs the motor model, and
# until now the only way to see it was to flash test_acc and drive the board.
#
# It is computed HERE rather than in the browser on purpose. scripts/
# drivetrain_report.py parses the model's constants out of sim_wheel.h, so the
# HUD moves when the firmware moves; a JavaScript reimplementation would be a
# second opinion about the robot that drifts silently from the first. Same rule
# as the report itself, one layer up.
@app.post("/api/drivetrain/performance")
async def api_drivetrain_performance(request: Request):
    """Speed, acceleration and the Nav2 budget check for a config.

    The body is a partial config -- `kinematics`, optionally `base_controller.
    simulation` and `nav2`. Absent blocks fall back to the saved params, so the
    HUD can post just the kinematics fields the user is editing and still be
    judged against the robot's real Nav2 tuning.
    """
    import drivetrain_report as dr

    data = await json_body(request)
    if not isinstance(data, dict):
        raise HTTPException(status_code=400, detail="expected a JSON object")

    def _overlay(base, over):
        # A shallow overlay per block is right here: the HUD sends whole scalar
        # fields, and a deep merge into the Nav2 tree would let a half-typed
        # value from one node reach another.
        out = dict(base or {})
        out.update({k: v for k, v in (over or {}).items() if v is not None})
        return out

    params = copy.deepcopy(load_params()) or {}
    for key in ("kinematics", "nav2"):
        if isinstance(data.get(key), dict):
            params[key] = _overlay(params.get(key), data[key])
    sim = (data.get("base_controller") or {}).get("simulation")
    if isinstance(sim, dict):
        bc = params.setdefault("base_controller", {})
        bc["simulation"] = _overlay(bc.get("simulation"), sim)

    try:
        d = dr.drivetrain(params)
    except (TypeError, ValueError) as exc:
        raise HTTPException(status_code=400, detail=f"kinematics: {exc}")
    if d["max_rpm"] <= 0 or d["circ"] <= 0:
        # Not an error: the HUD asks on every keystroke, and a half-typed
        # wheel_diameter is a normal state of an input box.
        return {"ready": False,
                "reason": "kinematics.max_rpm and wheel_diameter must be positive"}
    p = dr.performance(d)

    # The same demands report() judges, in the same order, so the HUD and the
    # command line cannot disagree about whether a config is over budget.
    nav = params.get("nav2") or {}
    smoother_v = dr._dig(nav, "max_velocity")
    smoother_a = dr._dig(nav, "max_accel")
    checks, over = [], False
    for label, v, w in (
            ("Controller target", dr._dig(nav, "desired_linear_vel"),
             dr._dig(nav, "rotate_to_heading_angular_vel")),
            ("Smoother envelope",
             smoother_v[0] if isinstance(smoother_v, list) else None,
             smoother_v[2] if isinstance(smoother_v, list) and len(smoother_v) > 2 else None)):
        if v is None or w is None:
            checks.append({"label": label, "set": False})
            continue
        need = dr.demand_rpm(d, float(v), float(w))
        pct = need / d["command_rpm"] * 100.0 if d["command_rpm"] > 0 else 0.0
        over = over or need > d["command_rpm"]
        checks.append({"label": label, "set": True, "linear": float(v), "angular": float(w),
                       "need_rpm": need, "budget_rpm": d["command_rpm"], "percent": pct,
                       "over": need > d["command_rpm"]})
    if isinstance(smoother_a, list) and smoother_a:
        # Against the SETTLED acceleration: a rate limit the base meets only
        # while the pack is still stiff is one it misses for the rest of the move.
        asked, have = float(smoother_a[0]), p["lin_acc_held"]
        over = over or asked > have
        checks.append({"label": "Smoother accel", "set": True, "asked": asked, "have": have,
                       "percent": (asked / have * 100.0) if have > 0 else 0.0,
                       "over": asked > have})

    # What test_acc itself would print, simulated on its own 20 ms / 1 s profile.
    # This is the block a velocity smoother should be tuned from: it is what the
    # robot does in the second a manoeuvre lasts, where the closed form above is
    # the asymptote it converges on given longer.
    lin_run = dr.run_test_acc(d, rotate=False)
    rot_run = dr.run_test_acc(d, rotate=True)

    return {
        "ready": True,
        "base": d["base"], "wheels": d["wheels"],
        "measured": {
            "max_vel": lin_run["max_vel"], "max_acc": lin_run["max_acc"],
            "t_to_90": lin_run["t_to_90"], "stop": lin_run["stop"],
            "max_vel_ang": rot_run["max_vel"], "max_acc_ang": rot_run["max_acc"],
            "stop_ang": rot_run["stop"],
        },
        "motor_rpm": d["motor_rpm"], "command_rpm": d["command_rpm"],
        "volt_ratio": d["volt_ratio"], "radius": d["radius"],
        "circumference": d["circ"], "mass": d["mass"],
        "ticks_per_m": (float(params.get("kinematics", {}).get("counts_per_rev", 0) or 0)
                        / d["circ"]) if d["circ"] > 0 else 0.0,
        "model": {"gear_efficiency": d["gear_eff"], "gear_drag_rpm": d["coulomb"],
                  "battery_sag": d["sag"], "battery_sag_tau_ms": d["sag_tau_ms"],
                  "driver_drop": d["drv_drop"], "driver_resistance": d["drv_r"],
                  "motor_stall_amps": d["stall_a"],
                  "driver_current_limit": d["ilimit_a"]},
        "max_linear": p["lin_vel"], "max_angular": p["ang_vel"],
        "accel_first": p["lin_acc"], "accel_held": p["lin_acc_held"],
        "ang_accel_first": p["ang_acc"], "ang_accel_held": p["ang_acc_held"],
        "t_to_90": p["t_to_90"], "tau_ms": p["tau"] * 1000.0,
        "stop_distance": p["stop_dist"],
        "checks": checks,
        "over_budget": over,
        # What the limits WOULD become, so the HUD can show them before the user
        # saves -- and so a manual tuner can see what the motors suggest without
        # having their own values overwritten to find out.
        "auto_limits": dr.auto_limits_enabled(params),
        "suggested": {k.split("ros__parameters.")[-1]: v
                      for k, v in dr.suggest_nav2_limits(d, p).items()},
        # The radius rule is the thing people get wrong, so it is stated rather
        # than left to be inferred from a number.
        "radius_note": ("mecanum turns on (lr + fr)/2 -- the rollers put the "
                        "wheelbase into the yaw term"
                        if d["base"] == "mecanum" else
                        "skid steer: lr/2 scaled by angular_scale (scrub)"
                        if d["base"] in ("4wd", "skid_steer") else
                        "differential: lr/2"),
    }
