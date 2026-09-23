#!/usr/bin/env python3
# ==============================================================================
# drivetrain_report.py — what a robot's motors can actually deliver
#
# Enter a robot config; get the speed and acceleration it can reach, and whether
# the Nav2 limits in that same config are asking for more than that.
#
# This exists because nothing checked. The shipped limits asked 103% of the
# motors on a differential base and 171% at the velocity smoother's ceiling, and
# on mecanum -- which turns on (lr + fr)/2 rather than lr/2, so the same angular
# velocity costs it 66% more wheel speed -- 129%. Nav2 then commands what it
# cannot get, Kinematics scales the whole request down to fit, and tracking
# degrades: three mecanum legs left the room on 2026-09-23 before anyone looked
# at the motors. The arithmetic is not hard; it was simply never written down.
#
# The model is the firmware's own (firmware/common/lib/encoder/fake_wheel.h): a
# brushed DC gear motor whose torque falls linearly from stall to no-load, a
# gearbox that returns part of it, constant gear drag, viscous friction, and a
# pack that sags under the current all the wheels draw together.
#
# Its constants are PARSED from that header rather than copied here. A tool that
# restates the model's numbers drifts from it silently, and then reports a robot
# that does not exist -- which is the same class of fault as the table of sensor
# support that had to be checked against the drivers.
#
#   python3 scripts/drivetrain_report.py --params <robot>_config.yaml
# ==============================================================================
import argparse
import math
import os
import re
import sys

try:
    import yaml
except ImportError:
    print("PyYAML is required: sudo apt install -y python3-yaml", file=sys.stderr)
    sys.exit(1)

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FAKE_WHEEL_H = os.path.join(REPO_ROOT, "firmware", "common", "lib", "encoder",
                            "fake_wheel.h")


def model_defaults(path=FAKE_WHEEL_H):
    """The wheel model's constants, read from the firmware that implements it."""
    want = {
        "FAKE_WHEEL_TAU_MS": "tau_ms",
        "FAKE_WHEEL_REF_MASS": "ref_mass",
        "FAKE_WHEEL_MAX_ACCEL_RPM": "accel_clamp",
        "FAKE_WHEEL_FRICTION": "viscous",
        "FAKE_ROBOT_MASS": "mass",
        "FAKE_GEAR_EFFICIENCY": "gear_eff",
        "FAKE_WHEEL_COULOMB_RPM": "coulomb",
        "FAKE_BATT_SAG": "sag",
    }
    out = {}
    with open(path, encoding="utf-8") as fh:
        for line in fh:
            m = re.match(r"\s*#define\s+(\w+)\s+([0-9.]+)", line)
            if m and m.group(1) in want:
                out.setdefault(want[m.group(1)], float(m.group(2)))
    missing = set(want.values()) - set(out)
    if missing:
        raise SystemExit(f"{path}: cannot find {sorted(missing)} -- the model moved, "
                         f"and this report would be describing a robot that does not exist")
    return out


def _dig(node, key, depth=0):
    """Nav2 params are nested <node>: <node>: ros__parameters: ..., and which
    level a key sits at differs between distros (the RotationShim nests
    FollowPath on some). Search rather than assume a path."""
    if depth > 7 or not isinstance(node, dict):
        return None
    if key in node:
        return node[key]
    for v in node.values():
        got = _dig(v, key, depth + 1)
        if got is not None:
            return got
    return None


def drivetrain(params):
    """Everything the report needs, resolved from the config."""
    kine = params.get("kinematics", {}) or {}
    bc = params.get("base_controller", {}) or {}
    sim = bc.get("simulation", {}) or {}
    d = model_defaults()

    # The env overrides the compiled-in defaults, so the config's simulation
    # block is what a real run would use.
    for cfg_key, name in (("robot_mass", "mass"), ("gear_efficiency", "gear_eff"),
                          ("gear_drag_rpm", "coulomb"), ("battery_sag", "sag")):
        if sim.get(cfg_key) is not None:
            d[name] = float(sim[cfg_key])

    base = str(kine.get("base_type", "2wd")).lower()
    lr = float(kine.get("lr_wheels_distance", 0.0))
    fr = float(kine.get("fr_wheels_distance", 0.0))
    wheel_d = float(kine.get("wheel_diameter", 0.0))
    # The same radius Kinematics::rotationRadius() uses, and for the same reason:
    # a mecanum's rollers put the wheelbase into the yaw term.
    if base == "mecanum":
        radius = (lr + fr) / 2.0
    elif base in ("4wd", "skid_steer"):
        radius = (lr / 2.0) * float(kine.get("angular_scale", 1.0))
    else:
        radius = lr / 2.0
    d.update(base=base, radius=radius, circ=math.pi * wheel_d,
             max_rpm=float(kine.get("max_rpm", 0)) * float(kine.get("max_rpm_ratio", 1.0)),
             wheels=4 if base in ("4wd", "skid_steer", "mecanum") else 2)
    return d


def performance(d):
    """Achievable speed and acceleration, from the model.

    tau scales with mass: a heavier robot takes longer to reach the same speed
    through the same gearbox. Sag is applied at the demand the manoeuvre makes --
    full stall current from rest, almost none at terminal speed -- so the two
    numbers are not computed under the same voltage, because the robot is not.
    """
    tau = (d["tau_ms"] / 1000.0) * (d["mass"] / d["ref_mass"])
    tau = max(tau, 0.001)

    # From rest: every wheel demands full stall current, so the pack sags most.
    no_load_from_rest = d["max_rpm"] / (1.0 + d["sag"])
    a_rpm = d["gear_eff"] * no_load_from_rest / tau - d["coulomb"]
    a_rpm = min(a_rpm, d["accel_clamp"])
    lin_acc = max(a_rpm, 0.0) * d["circ"] / 60.0

    # Terminal speed: accel = 0, and at that point the current -- and so the sag
    # -- has almost gone, so the no-load speed is back to full.
    #   eff*(no_load - w)/tau = w*viscous + coulomb
    w = (d["gear_eff"] * d["max_rpm"] / tau - d["coulomb"]) / \
        (d["gear_eff"] / tau + d["viscous"])
    w = max(w, 0.0)
    lin_vel = w * d["circ"] / 60.0

    return {
        "tau": tau,
        "lin_vel": lin_vel,
        "lin_acc": lin_acc,
        "ang_vel": lin_vel / d["radius"] if d["radius"] > 0 else 0.0,
        "ang_acc": lin_acc / d["radius"] if d["radius"] > 0 else 0.0,
        "t_to_90": 2.3 * tau,
        "stop_dist": lin_vel * tau / d["gear_eff"] if d["gear_eff"] > 0 else 0.0,
        "wheel_rpm_terminal": w,
    }


def demand_rpm(d, vx, wz):
    """Wheel speed a combined translate-and-rotate asks for, in rpm.

    They ADD: the outer wheel carries the linear component plus the rotational
    one. This is the number the old limits got wrong -- each looked survivable
    alone."""
    if d["circ"] <= 0:
        return 0.0
    return (abs(vx) * 60.0 / d["circ"]) + (abs(wz) * d["radius"] * 60.0 / d["circ"])


def report(params, name=""):
    d = drivetrain(params)
    if d["max_rpm"] <= 0 or d["circ"] <= 0:
        raise SystemExit("kinematics.max_rpm and wheel_diameter must be positive")
    p = performance(d)
    nav = params.get("nav2", {}) or {}
    vx = _dig(nav, "desired_linear_vel")
    wz = _dig(nav, "rotate_to_heading_angular_vel")
    smoother_v = _dig(nav, "max_velocity")
    smoother_a = _dig(nav, "max_accel")

    out = []
    out.append(f"=== {name or params.get('robot', {}).get('name', 'robot')}"
               f"   base={d['base']}  {d['wheels']} driven wheels")
    out.append(f"    motors {d['max_rpm']:.0f} rpm effective at the wheel, "
               f"{d['circ'] / math.pi * 1000:.0f} mm wheels, {d['mass']:.2f} kg")
    out.append(f"    gearbox {d['gear_eff'] * 100:.0f}% efficient, drag {d['coulomb']:.0f} rpm/s, "
               f"pack sag {d['sag'] * 100:.0f}% at full stall")
    out.append(f"    turns on {d['radius']:.4f} m"
               + ("  (mecanum: (lr+fr)/2)" if d["base"] == "mecanum" else "  (lr/2)"))
    out.append("")
    out.append("--- what it can do")
    out.append(f"    max speed          {p['lin_vel']:5.2f} m/s      {p['ang_vel']:5.2f} rad/s")
    # FROM REST, which is the peak: torque falls linearly as the wheel speeds
    # up, so this is the most the base will ever manage and it manages less at
    # every speed above zero. Comparing the smoother's rate limit against the
    # peak is therefore generous -- a limit that only just fits here will not be
    # met near top speed.
    out.append(f"    max acceleration   {p['lin_acc']:5.2f} m/s2     {p['ang_acc']:5.2f} rad/s2"
               f"   (from rest; falls as speed rises)")
    out.append(f"    time to 0.9x max   {p['t_to_90']:5.2f} s        (tau {p['tau'] * 1000:.0f} ms)")
    out.append("")
    out.append("--- what the config asks for")

    verdict_lines, over = [], False
    for label, v, w in (("controller target", vx, wz),
                        ("smoother envelope",
                         smoother_v[0] if isinstance(smoother_v, list) else None,
                         smoother_v[2] if isinstance(smoother_v, list) and len(smoother_v) > 2 else None)):
        if v is None or w is None:
            verdict_lines.append(f"    {label:18} not set in this config")
            continue
        need = demand_rpm(d, v, w)
        pct = need / d["max_rpm"] * 100.0
        flag = "OVER BUDGET" if need > d["max_rpm"] else "ok"
        if need > d["max_rpm"]:
            over = True
        verdict_lines.append(f"    {label:18} {v:.2f} m/s + {w:.2f} rad/s "
                             f"-> {need:6.1f} rpm = {pct:5.1f}% of {d['max_rpm']:.0f}   {flag}")
    out += verdict_lines

    if isinstance(smoother_a, list) and smoother_a:
        asked = float(smoother_a[0])
        pct = asked / p["lin_acc"] * 100.0 if p["lin_acc"] > 0 else float("inf")
        flag = "OVER BUDGET" if asked > p["lin_acc"] else "ok"
        if asked > p["lin_acc"]:
            over = True
        out.append(f"    smoother accel     {asked:.2f} m/s2 "
                   f"-> {pct:5.1f}% of the {p['lin_acc']:.2f} m/s2 it can reach   {flag}")

    out.append("")
    out.append("    VERDICT: the config asks for more than the motors can give"
               if over else "    VERDICT: within the motors' budget")
    return "\n".join(out), over


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--params", action="append", required=True,
                    help="robot config YAML; repeat to compare several")
    args = ap.parse_args()
    any_over = False
    for path in args.params:
        with open(path, encoding="utf-8") as fh:
            params = yaml.safe_load(fh) or {}
        text, over = report(params, os.path.basename(path))
        print(text)
        print()
        any_over = any_over or over
    return 1 if any_over else 0


if __name__ == "__main__":
    sys.exit(main())
