#!/usr/bin/env python3
"""The pack's sag over a run: how far the voltage fell under load from where it stood at rest,
and in which step -- the number that says whether a pack (and its cutoff) holds a robot up.

    battery_sag.py log --out FILE [--ns NS]     one JSON line per /battery message: {"t", "v"}

The 1-Click runs `log` beside a robot from bringup to the end, marks its steps on a timeline
(clearance, exploration, test_acc, goals, the drive suite), and prints one BATTERY_SAG line
from the samples and the steps (summarize / sag_line). test_acc is the heaviest load a robot
sees, and the one moment the host hears no /battery (micro-ROS is off while a tool runs), so
the board measures that itself and prints

    BATTERY worst 10.42 V  sag 0.56 V from 10.98 V at rest

which test_acc_figures() reads out of the transcript.

The rest voltage is the median of the at-rest samples before the first load; the worst is
the lowest single sample under load. /battery comes at about 1 Hz from most boards, so a
shorter dip than that can be missed: the line says how many samples it is based on.
"""
import argparse
import json
import math
import re
import sys
import time

TEST_ACC_WORST = re.compile(r"BATTERY worst\s+([\d.]+) V\s+sag\s+([-\d.]+) V from\s+([\d.]+) V at rest")


def read_samples(path: str) -> list:
    """[(t, v)] from a log written by `log`; a missing or torn file reads as what it holds."""
    out = []
    try:
        with open(path, encoding="utf-8") as fh:
            for ln in fh:
                try:
                    d = json.loads(ln)
                    t, v = float(d["t"]), float(d["v"])
                except (ValueError, KeyError, TypeError):
                    continue
                if math.isfinite(v) and v > 0.5:
                    out.append((t, v))
    except OSError:
        pass
    return out


def test_acc_figures(text: str):
    """{"worst", "sag", "rest"} from a test_acc transcript's last BATTERY worst line, or None."""
    m = None
    for m in TEST_ACC_WORST.finditer(text or ""):
        pass
    if not m:
        return None
    return {"worst": float(m.group(1)), "sag": float(m.group(2)), "rest": float(m.group(3))}


def _median(vals):
    s = sorted(vals)
    n = len(s)
    return s[n // 2] if n % 2 else 0.5 * (s[n // 2 - 1] + s[n // 2])


def summarize(samples: list, phases: list, test_acc=None):
    """The sag of a run: samples [(t, v)], phases [(t_start, label)] (a label ending
    "(at rest)" is a stretch with the motors off), test_acc the board's own figures or None.
    The rest voltage is the median of the at-rest samples before the first load; the worst is
    the lowest sample under load (an at-rest stretch after a load is the pack recovering:
    neither). Returns None with nothing to report, else {"rest", "worst", "sag", "pct",
    "where", "n", "phases": {label: lowest}}."""
    phases = sorted(phases)

    def phase_at(t):
        label = ""
        for t0, lab in phases:
            if t >= t0:
                label = lab
        return label

    def at_rest(label):
        return label.endswith("(at rest)") or not label

    first_load = next((t0 for t0, lab in phases if not at_rest(lab)), None)
    rest_vals = [v for t, v in samples if at_rest(phase_at(t)) and (first_load is None or t < first_load)]
    rest = _median(rest_vals) if rest_vals else None
    per, worst, where = {}, None, ""
    for t, v in samples:
        lab = phase_at(t)
        if at_rest(lab):
            continue
        per[lab] = min(per.get(lab, v), v)
        if worst is None or v < worst:
            worst, where = v, lab
    if test_acc:
        per["test_acc"] = min(per.get("test_acc", test_acc["worst"]), test_acc["worst"])
        if rest is None:
            rest = test_acc["rest"]
        if worst is None or test_acc["worst"] < worst:
            worst, where = test_acc["worst"], "test_acc"
    if rest is None or worst is None:
        return None
    sag = rest - worst
    return {"rest": rest, "worst": worst, "sag": sag, "pct": 100.0 * sag / rest if rest else 0.0,
            "where": where, "n": len(samples), "phases": per}


def sag_line(s) -> str:
    """The one line the web UI shows (app-workflow.js oneClickBatterySag)."""
    if not s:
        return "BATTERY_SAG: not measured (no /battery reading under load)"
    each = ", ".join(f"{k} {v:.2f} V" for k, v in s["phases"].items())
    return (f"BATTERY_SAG: worst {s['worst']:.2f} V during {s['where']}, {s['sag']:.2f} V "
            f"({s['pct']:.1f} %) below {s['rest']:.2f} V at rest"
            + (f"; lowest per step: {each}" if each else "")
            + f" ({s['n']} /battery samples)")


def log(out: str, ns: str = "") -> int:
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import BatteryState
    rclpy.init()
    node = rclpy.create_node("battery_sag_log")
    fh = open(out, "a", encoding="utf-8", buffering=1)

    def on_battery(m):
        v = float(m.voltage)
        if math.isfinite(v) and v > 0.5:
            fh.write(json.dumps({"t": round(time.time(), 3), "v": round(v, 3)}) + "\n")

    # best effort: the firmware's /battery is best effort, and a reliable reader never hears it
    node.create_subscription(BatteryState, f"{ns.rstrip('/')}/battery" if ns else "battery",
                             on_battery, qos_profile_sensor_data)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, Exception):
        pass
    finally:
        fh.close()
        try:
            node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass
    return 0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    lg = sub.add_parser("log")
    lg.add_argument("--out", required=True)
    lg.add_argument("--ns", default="")
    a = ap.parse_args(argv)
    return log(a.out, a.ns)


if __name__ == "__main__":
    sys.exit(main())
