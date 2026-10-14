"""Linorobot2 Cockpit routes -- a real robot's Bring-up checklist (Hardware Tests).

The web UI's twin of scripts/robot_bringup.py: one record (bringup_checklist.py, in the
config directory's state/), one set of arithmetic, so a step done on the command line shows
as done here and the other way round, and the 1-Click asks the same record.

/api/bringup/checklist   every step, its state, and whether the 1-Click may move the robot
/api/bringup/mark        record a step: pass / fail / skip, with what was seen
/api/bringup/sensors     the sensors step's verdict from the readings the page collected
/api/bringup/calc        the arithmetic: invert flags, counts per rev, battery divider
/api/bringup/apply       write a calibration into the robot's config; when the tool is open
                         it is closed, and the env write that makes the board run the new
                         values is handed back as a command handle
/api/bringup/run         a ROS-side step (lidar, speed, acc) as a command handle: the same
                         robot_bringup.py the command line runs, streamed by /api/exec

The interactive steps (sensors, motors, rpm, battery) run through the tool session
(routes_tool.py): the page drives the board and posts what it saw here.
"""
import os
import shlex
import sys

from fastapi import HTTPException, Request

from core import REPO_ROOT, actions, app, get_active_params_path, json_body
import bringup_checklist as bc  # noqa: E402  (scripts/, via core's sys.path)
import tool_session  # noqa: E402

# What the page may write, and nothing else: each calibration's own keys.
_WRITABLE = {f"base_controller.pins.motor{n}.invert" for n in range(1, 5)} \
    | {f"base_controller.pins.encoder{n}.invert" for n in range(1, 5)} \
    | {"kinematics.counts_per_rev", "kinematics.max_rpm", "base_controller.pins.battery.r1"}


def _robot():
    import mcu_env
    path = get_active_params_path()
    params = mcu_env.load_yaml(path)
    return path, params, mcu_env.robot_name_of(params, path)


@app.get("/api/bringup/checklist")
def api_bringup_checklist():
    path, params, robot = _robot()
    out = bc.checklist(robot, params, path)
    out["config"] = os.path.basename(path)
    import gen_firmware_header
    kin = params.get("kinematics") or {}
    pins = (params.get("base_controller") or {}).get("pins") or {}
    bat = pins.get("battery") if isinstance(pins.get("battery"), dict) else {}
    out["current"] = {
        "counts_per_rev": gen_firmware_header.counts_per_rev(kin) if kin else None,
        "max_rpm": kin.get("max_rpm"),
        "battery": {"r1": bat.get("r1"), "r2": bat.get("r2"), "pin": bat.get("pin")},
        "invert": {str(n): {"motor": bool((pins.get(f"motor{n}") or {}).get("invert", False)),
                            "encoder": bool((pins.get(f"encoder{n}") or {}).get("invert", False))}
                   for n in range(1, out["parts"]["wheels"] + 1)},
    }
    return out


@app.post("/api/bringup/mark")
async def api_bringup_mark(request: Request):
    data = await json_body(request)
    path, params, robot = _robot()
    values = data.get("values") if isinstance(data.get("values"), dict) else {}
    try:
        e = bc.mark(robot, params, str(data.get("stage") or ""), str(data.get("status") or ""),
                    str(data.get("summary") or ""), values=values, by="gui")
    except ValueError as exc:
        raise HTTPException(status_code=400, detail=str(exc))
    return {"ok": True, "entry": e}


@app.post("/api/bringup/sensors")
async def api_bringup_sensors(request: Request):
    """{rest: [...], moving: [...]}: [TOOL_JSON] sensors readings while the robot was held
    still, then turned and shaken. Judged and recorded."""
    data = await json_body(request)
    rest = [r for r in (data.get("rest") or []) if isinstance(r, dict)][:2000]
    moving = [r for r in (data.get("moving") or []) if isinstance(r, dict)][:5000]
    path, params, robot = _robot()
    p = bc.parts(params, path)
    ok, lines, m = bc.sensors_judge(rest, moving, p)
    if not (p["imu"] or p["mag"]):
        ok, lines = True, ["no real IMU or magnetometer in this robot's config"]
    if data.get("record", True):
        bc.mark(robot, params, "sensors", "pass" if ok else "fail", "; ".join(lines), values=m, by="gui")
    return {"ok": ok, "lines": lines, "motion": m}


@app.post("/api/bringup/calc")
async def api_bringup_calc(request: Request):
    """The arithmetic only; nothing is written."""
    data = await json_body(request)
    kind = str(data.get("kind") or "")
    path, params, _ = _robot()
    try:
        if kind == "invert":
            w = int(data["wheel"])
            fix = bc.invert_fix(bool(data["turned_forward"]), float(data["rpm"]))
            return {"fix": fix, "changes": bc.changes_for(params, "invert", wheel=w, **fix)}
        if kind == "cpr_tacho":
            import gen_firmware_header
            cpr = gen_firmware_header.counts_per_rev(params.get("kinematics") or {})
            new = bc.cpr_from_tacho(cpr, float(data["encoder_rpm"]), float(data["tacho_rpm"]))
            return {"from": cpr, "cpr": new, "off_pct": round(abs(new - cpr) / cpr * 100, 1),
                    "changes": bc.changes_for(params, "cpr", cpr=new)}
        if kind == "cpr_turns":
            import gen_firmware_header
            cpr = gen_firmware_header.counts_per_rev(params.get("kinematics") or {})
            new = bc.cpr_from_turns(int(data["counts"]), float(data["turns"]))
            return {"from": cpr, "cpr": new, "off_pct": round(abs(new - cpr) / cpr * 100, 1),
                    "counted_down": int(data["counts"]) < 0,
                    "changes": bc.changes_for(params, "cpr", cpr=new)}
        if kind == "max_rpm":
            return {"changes": bc.changes_for(params, "max_rpm", rpm=float(data["rpm"]))}
        if kind == "battery":
            bat = ((params.get("base_controller") or {}).get("pins") or {}).get("battery") or {}
            if not isinstance(bat, dict) or bat.get("r2") in (None, 0):
                raise ValueError("this robot's battery has no ADC divider (pins.battery r1/r2) to trim")
            t = bc.battery_trim(float(bat.get("r1") or 0), float(bat["r2"]),
                                float(data["reported"]), float(data["measured"]))
            return {**t, "changes": bc.changes_for(params, "battery", r1=t["r1"])}
    except (KeyError, TypeError) as exc:
        raise HTTPException(status_code=400, detail=f"{kind}: missing or bad field ({exc})")
    except ValueError as exc:
        raise HTTPException(status_code=400, detail=str(exc))
    raise HTTPException(status_code=400, detail=f"unknown calculation {kind!r}")


@app.post("/api/bringup/apply")
async def api_bringup_apply(request: Request):
    """{changes: {dotted.path: value}} into the robot's config, then the env write. The tool
    holds the board's console, so an open tool session is left first (`base`)."""
    data = await json_body(request)
    changes = data.get("changes")
    if not isinstance(changes, dict) or not changes:
        raise HTTPException(status_code=400, detail="changes: {path: value}")
    bad = sorted(k for k in changes if k not in _WRITABLE)
    if bad:
        raise HTTPException(status_code=400, detail=f"not a bring-up calibration: {', '.join(bad)}")
    for k, v in changes.items():
        if k.endswith(".invert") and not isinstance(v, bool):
            raise HTTPException(status_code=400, detail=f"{k}: true or false")
        if not k.endswith(".invert") and not isinstance(v, (int, float)):
            raise HTTPException(status_code=400, detail=f"{k}: a number")
    path, params, robot = _robot()
    done = bc.apply(path, changes)
    closed = False
    s = tool_session.REGISTRY.get(robot)
    if s:
        import time
        s.leave()
        time.sleep(1.0)
        tool_session.REGISTRY.remove(robot)
        closed = True
    import routes_tool
    handle = actions.prepare(routes_tool._env_base_command(routes_tool._robot(), str(data.get("port") or "")))
    return {"ok": True, "written": done, "tool_closed": closed, "handle": handle}


@app.post("/api/bringup/run")
async def api_bringup_run(request: Request):
    """A ROS-side step as a command handle for /api/exec: robot_bringup.py, as on the
    command line, with the robot stack running in this container."""
    data = await json_body(request)
    stage = str(data.get("stage") or "")
    if stage not in ("lidar", "speed", "acc"):
        raise HTTPException(status_code=400, detail="stage: lidar, speed or acc")
    path, params, robot = _robot()
    if tool_session.REGISTRY.get(robot):
        raise HTTPException(status_code=409, detail="The tool is open: press Back to robot first "
                                                    "(this step needs the robot running).")
    argv = [sys.executable, "-u", os.path.join(REPO_ROOT, "scripts", "robot_bringup.py"),
            "--params", path, "--by", "gui", "--yes", stage]
    if stage == "acc" and data.get("write"):
        argv.append("--write")
    return {"handle": actions.prepare(" ".join(shlex.quote(x) for x in argv))}
