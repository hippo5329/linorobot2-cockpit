#!/usr/bin/env python3
# Copyright (c) 2026 Linorobot contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""A real robot's bring-up: what has been checked, and the arithmetic that fixes what was not.

One module for both front ends -- scripts/robot_bringup.py on the command line and the
web UI's Bring-up checklist (web/backend/routes_bringup.py) -- so the two can never
disagree about what "the motors are done" means or what a tachometer reading changes.

The order (each step needs the ones before it):

  sensors   every real IMU / magnetometer answers, and its readings follow the robot
            when a person turns and shakes it                      (in hand, bench)
  battery   the reported voltage agrees with a multimeter; an ADC divider is trimmed
  motors    each wheel turns forward on a forward command, and its encoder counts up
            -- the invert flags fixed where not                     (wheels lifted)
  rpm       the encoder's RPM agrees with a laser tachometer: counts_per_rev and the
            measured max_rpm                                         (wheels lifted)
  lidar     the scan arrives at its rate, all round, with sane ranges
  ---- the 1-Click may move the robot only past this line (gate()) ----
  speed     commanded vs wheels vs LiDAR speeds (speed_check.py)    (floor, 0.6 m clear)
  acc       test_acc: the drivetrain identified (Wi-Fi robots)      (floor, 1.0 m clear)

A step is required only for parts the robot really has (mcu_env.simulation_parts): a
robot with no magnetometer is not asked to calibrate one, and a robot whose wheels are
simulated is not a real robot at all -- nothing here gates it.

The record lives in <config dir>/state/bringup/<robot>.json, beside the flash stamps: the
backend and the pipeline both see it. Each pass keeps a fingerprint of the config values
it vouched for; when those values change afterwards, the pass is shown as stale and the
gate asks for the step again.
"""
import copy
import hashlib
import json
import math
import os
import statistics
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)

import cockpit_paths  # noqa: E402

STAGES = ("sensors", "battery", "motors", "rpm", "lidar", "speed", "acc")
# The steps the 1-Click needs before it may move a real robot (user, 2026-10-14: "one
# click can be used on real robot only after sensors and motors are working correctly").
GATE = ("sensors", "battery", "motors", "rpm", "lidar")
STATUSES = ("pass", "fail", "skip")

TITLES = {
    "sensors": "Sensors answer and follow the robot",
    "battery": "Battery voltage agrees with a multimeter",
    "motors": "Every wheel turns forward, every encoder counts up",
    "rpm": "Wheel RPM agrees with a tachometer",
    "lidar": "LiDAR scans all round at its rate",
    "speed": "Speeds agree with the LiDAR (on the floor)",
    "acc": "test_acc identified the drivetrain (on the floor)",
}
WHERE = {
    "sensors": "robot in your hands, wheels off the floor",
    "battery": "a multimeter on the battery terminals",
    "motors": "wheels lifted off the floor",
    "rpm": "wheels lifted, a laser tachometer",
    "lidar": "robot stack running (Start the robot)",
    "speed": "on the floor, 0.6 m clear all round, robot stack running",
    "acc": "on the floor, 1.0 m clear all round, robot stack running, Wi-Fi",
}


# ---------------------------------------------------------------- the robot's parts
def _base_wheels(params: dict) -> int:
    base = str((params.get("kinematics") or {}).get("base_type", "2wd")).lower()
    return 4 if base in ("4wd", "skid_steer", "mecanum") else 2


def parts(params: dict, params_path: str = None) -> dict:
    """What this robot really has, as the bring-up sees it (config mode: the YAML stands)."""
    import mcu_env
    _, real = mcu_env.simulation_parts(params, "config", params_path)
    tgt = params.get("base_controller") or {}
    joined = " ".join(real)
    pins = tgt.get("pins") or {}
    bat = pins.get("battery")
    try:
        adc_pin = int(bat.get("pin", -1)) if isinstance(bat, dict) else int(bat if bat is not None else -1)
    except (TypeError, ValueError):
        adc_pin = -1
    current = str((tgt.get("sensors") or {}).get("current", "NONE")).strip().upper()
    return {
        "wheels_real": "wheels" in joined,
        "wheels": _base_wheels(params),
        "imu": any(p.startswith("IMU") for p in real),
        "mag": any(p.startswith("magnetometer") for p in real),
        "battery": "battery sense" in joined,
        # An ADC divider can be trimmed; a current monitor (INA219) reads its own volts.
        "battery_adc": adc_pin >= 0,
        "battery_chip": "" if current in ("", "NONE", "OFF") else current,
        "lidar": any(p.startswith("LiDAR") for p in real),
        "wifi": mcu_env.transport_is_wifi(tgt.get("transport")),
        "mecanum": str((params.get("kinematics") or {}).get("base_type", "")).lower() == "mecanum",
    }


def required(stage: str, p: dict) -> bool:
    """Whether this robot needs the step at all."""
    if not p["wheels_real"]:
        return False                  # simulated wheels: not a real robot, nothing to gate
    return {
        "sensors": p["imu"] or p["mag"],
        "battery": p["battery"],
        "motors": True,
        "rpm": True,
        "lidar": p["lidar"],
        "speed": p["lidar"],
        "acc": p["wifi"],
    }[stage]


# ---------------------------------------------------------------- fingerprints
def _section(params: dict, stage: str):
    """The config values a step vouches for: when these change, its pass is stale."""
    tgt = params.get("base_controller") or {}
    pins = tgt.get("pins") or {}
    kin = params.get("kinematics") or {}
    if stage == "sensors":
        s = tgt.get("sensors") or {}
        return {k: s.get(k) for k in ("imu", "mag", "imu_orientation", "mag_orientation")} | {
            "i2c": {k: v for k, v in pins.items() if "i2c" in k or k in ("sda", "scl")}}
    if stage == "battery":
        return {"battery": pins.get("battery"), "current": (tgt.get("sensors") or {}).get("current")}
    if stage == "motors":
        return {k: pins.get(k) for k in sorted(pins) if k.startswith(("motor", "encoder"))} | {
            "driver": tgt.get("driver_type"), "base": kin.get("base_type")}
    if stage == "rpm":
        return {k: kin.get(k) for k in ("counts_per_rev", "encoder_ppr", "gear_ratio", "quadrature",
                                       "max_rpm", "wheel_diameter")}
    if stage == "lidar":
        return {"lidar": tgt.get("lidar")}
    if stage in ("speed", "acc"):
        return {k: kin.get(k) for k in ("wheel_diameter", "lr_wheels_distance", "fr_wheels_distance",
                                       "angular_scale", "counts_per_rev", "max_rpm", "pid")}
    raise KeyError(stage)


def fingerprint(params: dict, stage: str) -> str:
    blob = json.dumps(_section(params, stage), sort_keys=True, default=str)
    return hashlib.sha256(blob.encode()).hexdigest()[:16]


# ---------------------------------------------------------------- the record
def record_path(robot: str, directory: str = None) -> str:
    d = os.path.join(cockpit_paths.state_dir(directory), "bringup")
    os.makedirs(d, exist_ok=True)
    safe = "".join(c if c.isalnum() or c in "-_." else "_" for c in robot) or "robot"
    return os.path.join(d, f"{safe}.json")


def load(robot: str, directory: str = None) -> dict:
    try:
        with open(record_path(robot, directory), encoding="utf-8") as fh:
            data = json.load(fh)
        return data if isinstance(data, dict) else {}
    except (OSError, ValueError):
        return {}


def mark(robot: str, params: dict, stage: str, status: str, summary: str = "",
         values: dict = None, by: str = "cli", directory: str = None) -> dict:
    """Record a step's result. Written whole, through a temp file (the backend and a CLI
    run may both write)."""
    if stage not in STAGES:
        raise ValueError(f"stage must be one of {', '.join(STAGES)}, not {stage!r}")
    if status not in STATUSES:
        raise ValueError(f"status must be one of {', '.join(STATUSES)}, not {status!r}")
    data = load(robot, directory)
    entry = {"status": status, "at": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
             "by": by, "summary": str(summary or "")[:500], "values": values or {},
             "fingerprint": fingerprint(params, stage)}
    data.setdefault("stages", {})[stage] = entry
    data["robot"] = robot
    path = record_path(robot, directory)
    tmp = f"{path}.{os.getpid()}.tmp"
    with open(tmp, "w", encoding="utf-8") as fh:
        json.dump(data, fh, indent=2, sort_keys=True)
    os.replace(tmp, path)
    return entry


def checklist(robot: str, params: dict, params_path: str = None, directory: str = None) -> dict:
    """Every step with its state: pass / fail / skip / stale / todo / not needed."""
    p = parts(params, params_path)
    rec = (load(robot, directory).get("stages") or {})
    rows = []
    for stage in STAGES:
        need = required(stage, p)
        e = rec.get(stage) or {}
        state = e.get("status") or "todo"
        if state == "pass" and e.get("fingerprint") != fingerprint(params, stage):
            state = "stale"
        if not need and state == "todo":
            state = "not needed"
        rows.append({"stage": stage, "title": TITLES[stage], "where": WHERE[stage],
                     "required": need, "gate": stage in GATE, "state": state,
                     "at": e.get("at"), "by": e.get("by"), "summary": e.get("summary", ""),
                     "values": e.get("values", {})})
    ok, missing = gate(robot, params, params_path, directory, _rows=rows)
    return {"robot": robot, "parts": p, "real_robot": p["wheels_real"], "stages": rows,
            "gate_ok": ok, "missing": missing}


def gate(robot: str, params: dict, params_path: str = None, directory: str = None, _rows=None):
    """(ok, [why, ...]): may the 1-Click move this robot? A step that is required and
    not passed (or passed for config values that have since changed) holds it; `skip`
    counts as passed -- a person decided it, and the record says so."""
    rows = _rows if _rows is not None else checklist(robot, params, params_path, directory)["stages"]
    missing = []
    for r in rows:
        if r["stage"] not in GATE or not r["required"]:
            continue
        if r["state"] in ("pass", "skip"):
            continue
        how = {"todo": "not done", "fail": "failed", "stale": "the config changed since it passed"}
        missing.append(f"{r['stage']}: {how.get(r['state'], r['state'])} -- {r['title']}")
    return not missing, missing


# ---------------------------------------------------------------- sensors
def sensor_motion(samples: list) -> dict:
    """What a person's turning and shaking did to each reading, from the tool's
    [TOOL_JSON] sensors lines. A live chip changes; a stuck bus repeats one value.

    Thresholds sit well past a resting chip's noise and well under a hand's motion:
    gyro 0.5 rad/s (a slow turn is ~1), accelerometer span 2 m/s^2 on some axis (a tilt
    of 15 deg moves gravity's share by 2.5), magnetometer span 10 uT (a quarter turn in
    the Earth's ~50 uT field moves an axis by tens), RPY 20 deg."""
    out = {}

    def span(key):
        rows = [s[key] for s in samples if isinstance(s.get(key), list) and len(s[key]) == 3]
        if not rows:
            return None
        return [round(max(r[i] for r in rows) - min(r[i] for r in rows), 3) for i in range(3)], rows

    acc = span("acc")
    if acc:
        spans, rows = acc
        g = statistics.median(math.sqrt(sum(v * v for v in r)) for r in rows)
        out["imu"] = {"acc_span": spans, "gravity": round(g, 2),
                      "gravity_ok": 8.8 <= g <= 10.8, "moved_acc": max(spans) >= 2.0}
    gyr = [s["gyr"] for s in samples if isinstance(s.get("gyr"), list) and len(s["gyr"]) == 3]
    if gyr:
        peak = max(max(abs(v) for v in r) for r in gyr)
        out.setdefault("imu", {}).update({"gyro_peak": round(peak, 3), "moved_gyro": peak >= 0.5})
    rpy = span("rpy")
    if rpy:
        spans, _ = rpy
        out["rpy"] = {"span": spans, "moved": max(spans) >= 20.0}
    mag = span("mag_ut")
    if mag:
        spans, rows = mag
        norm = statistics.median(math.sqrt(sum(v * v for v in r)) for r in rows)
        out["mag"] = {"span_ut": spans, "field_ut": round(norm, 1), "moved": max(spans) >= 10.0}
    bats = [s["bat"] for s in samples if isinstance(s.get("bat"), (int, float))]
    if bats:
        out["battery_v"] = round(statistics.median(bats), 3)
    return out


def sensors_judge(rest: list, moving: list, p: dict) -> tuple:
    """(passed, [lines], motion) for the sensors step: gravity judged on the readings taken
    while the robot was held still, the motion on everything."""
    still = sensor_motion(rest)
    m = sensor_motion(rest + moving)
    if "imu" in still and "imu" in m:
        m["imu"]["gravity"] = still["imu"]["gravity"]
        m["imu"]["gravity_ok"] = still["imu"]["gravity_ok"]
    ok, lines = sensors_verdict(m, p)
    return ok, lines, m


def sensors_verdict(motion: dict, p: dict) -> tuple:
    """(passed, [lines]) for the sensors step."""
    lines, ok = [], True
    if p["imu"]:
        imu = motion.get("imu")
        if not imu:
            ok = False
            lines.append("IMU: configured, but no readings came (not detected on the bus?)")
        else:
            moved = imu.get("moved_acc") or imu.get("moved_gyro")
            if not imu.get("gravity_ok"):
                ok = False
                lines.append(f"IMU: |acc| at rest {imu['gravity']} m/s^2, not ~9.8 (scale or units wrong)")
            if not moved:
                ok = False
                lines.append("IMU: the readings did not follow the motion (turn and shake the robot)")
            if imu.get("gravity_ok") and moved:
                lines.append(f"IMU: gravity {imu['gravity']} m/s^2, gyro peak {imu.get('gyro_peak')} rad/s -- follows")
    if p["mag"]:
        mag = motion.get("mag")
        if not mag:
            ok = False
            lines.append("magnetometer: configured, but no readings came")
        elif not mag["moved"]:
            ok = False
            lines.append(f"magnetometer: field {mag['field_ut']} uT but it did not change as the robot turned")
        else:
            lines.append(f"magnetometer: field {mag['field_ut']} uT, span {mag['span_ut']} uT -- follows")
            if not 20 <= mag["field_ut"] <= 70:
                lines.append("  (the Earth's field is 25-65 uT: a larger one is a motor or a steel part "
                             "nearby -- calibrate it, or mount the chip away from them)")
    return ok, lines


# ---------------------------------------------------------------- battery
def battery_trim(r1: float, r2: float, reported_v: float, measured_v: float) -> dict:
    """A new R1 that makes the ADC divider report what the multimeter reads.

    The firmware computes V = V_adc * (R1 + R2) / R2 (battery.cpp). Resistors are 1-5 %
    parts and the ADC's reference is not exact either; scaling the ratio by
    measured / reported folds both into one number, kept in R1 (R2 stays as typed).
    Refused past +-25 %: that is a wrong pin or a wrong divider, not a tolerance."""
    if r2 <= 0 or r1 < 0:
        raise ValueError("R1 and R2 must be positive (R2 > 0)")
    if reported_v <= 0.5:
        raise ValueError(f"the board reports {reported_v} V: nothing on the pin to calibrate")
    k = measured_v / reported_v
    if not 0.75 <= k <= 1.25:
        raise ValueError(f"the board reads {reported_v:.2f} V against {measured_v:.2f} V on the meter "
                         f"({(k - 1) * 100:+.0f} %): check the pin and the resistor values first")
    ratio = (r1 + r2) / r2 * k
    return {"r1": round((ratio - 1.0) * r2, 1), "r2": r2, "ratio": round(ratio, 4),
            "scale": round(k, 4)}


# ---------------------------------------------------------------- motors
def invert_fix(turned_forward: bool, rpm: float) -> dict:
    """The invert flags a wheel needs, from what a person saw and what the encoder said,
    on a FORWARD command.

    turned_forward  the wheel turned the way that drives the robot forward
    rpm             the encoder's RPM in that run (its sign is the encoder's direction)

    A motor wired backwards turns the wheel backwards: flip the motor. The encoder must
    then count UP while the wheel drives forward -- after the motor flip, so a wheel seen
    turning backwards with a negative RPM needs the motor flipped only (both reverse
    together), and one seen backwards with a positive RPM needs both."""
    if abs(rpm) < 1.0:
        raise ValueError("the encoder counted nothing: check its wiring and pins before "
                         "deciding the direction")
    return {"flip_motor": not turned_forward, "flip_encoder": (rpm > 0) != turned_forward}


def cpr_from_tacho(cpr: float, encoder_rpm: float, tacho_rpm: float) -> int:
    """counts_per_rev that makes the encoder's RPM read what the tachometer read.
    RPM_enc = counts/s * 60 / cpr, so cpr_true = cpr * RPM_enc / RPM_tacho."""
    encoder_rpm, tacho_rpm = abs(encoder_rpm), abs(tacho_rpm)
    if tacho_rpm < 5 or encoder_rpm < 5:
        raise ValueError("measure with the wheel turning (at least 5 RPM on both)")
    new = cpr * encoder_rpm / tacho_rpm
    if not 0.2 <= new / cpr <= 5:
        raise ValueError(f"{cpr:.0f} -> {new:.0f} counts per rev is a factor of {new / cpr:.2f}: "
                         "a gear ratio or the quadrature x4 is missing -- check the motor's data")
    return int(round(new))


def cpr_from_turns(counts: int, turns: float) -> int:
    """counts_per_rev from the `enc` counter while the wheel is turned by hand N marked turns."""
    if turns <= 0:
        raise ValueError("turns must be positive")
    if abs(counts) < 20:
        raise ValueError(f"{counts} counts: the encoder is not counting")
    return int(round(abs(counts) / turns))


# ---------------------------------------------------------------- writing the config
def _get(d: dict, path: str):
    for k in path.split("."):
        if not isinstance(d, dict):
            return None
        d = d.get(k)
    return d


def _set(d: dict, path: str, value) -> None:
    keys = path.split(".")
    for k in keys[:-1]:
        if not isinstance(d.get(k), dict):
            d[k] = {}
        d = d[k]
    d[keys[-1]] = value


def changes_for(params: dict, kind: str, **kw) -> dict:
    """{dotted.path: new value} for one calibration. Nothing is written here."""
    if kind == "invert":
        w = int(kw["wheel"])
        out = {}
        for flag, sec in (("flip_motor", f"base_controller.pins.motor{w}.invert"),
                          ("flip_encoder", f"base_controller.pins.encoder{w}.invert")):
            if kw.get(flag):
                out[sec] = not bool(_get(params, sec))
        return out
    if kind == "invert_set":
        w = int(kw["wheel"])
        out = {}
        if kw.get("motor") is not None:
            out[f"base_controller.pins.motor{w}.invert"] = bool(kw["motor"])
        if kw.get("encoder") is not None:
            out[f"base_controller.pins.encoder{w}.invert"] = bool(kw["encoder"])
        return out
    if kind == "cpr":
        return {"kinematics.counts_per_rev": int(kw["cpr"])}
    if kind == "max_rpm":
        rpm = float(kw["rpm"])
        if not 10 <= rpm <= 2000:
            raise ValueError(f"max_rpm {rpm}: outside 10..2000")
        return {"kinematics.max_rpm": int(round(rpm))}
    if kind == "battery":
        return {"base_controller.pins.battery.r1": float(kw["r1"])}
    raise ValueError(f"unknown calibration {kind!r}")


def apply(params_path: str, changes: dict, robot: str = None, directory: str = None) -> dict:
    """Write the changes into the robot's config; {path: {"from", "to"}}. The file before
    is kept in state/bringup/ (the config directory is a git repo too)."""
    import yaml
    with open(params_path, encoding="utf-8") as fh:
        text = fh.read()
    params = yaml.safe_load(text) or {}
    done = {}
    new = copy.deepcopy(params)
    for path, value in changes.items():
        old = _get(new, path)
        if old == value:
            continue
        _set(new, path, value)
        done[path] = {"from": old, "to": value}
    if not done:
        return done
    stamp = time.strftime("%Y%m%d-%H%M%S", time.gmtime())
    bdir = os.path.join(cockpit_paths.state_dir(directory), "bringup")
    os.makedirs(bdir, exist_ok=True)
    base = os.path.basename(params_path)
    with open(os.path.join(bdir, f"{base}.{stamp}.bak"), "w", encoding="utf-8") as fh:
        fh.write(text)
    tmp = f"{params_path}.{os.getpid()}.tmp"
    with open(tmp, "w", encoding="utf-8") as fh:
        yaml.safe_dump(new, fh, sort_keys=False)
    os.replace(tmp, params_path)
    return done
