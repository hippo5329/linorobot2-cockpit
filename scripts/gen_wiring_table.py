#!/usr/bin/env python3
# ==============================================================================
# gen_wiring_table.py — the wiring chart for a robot, from its config
#
# What you want at the bench is a sheet that says "GP12 -> encoder 1 A", not a
# YAML file. The config engine upstream emits one as wiring_table.md; the
# cockpit had the data (every pin the Pin Matrix holds) and no sheet.
#
# Two tables, because two questions get asked at the bench. "Where does this
# wire go?" is answered by FUNCTION -> GPIO. "What is already on this pin?" is
# answered by GPIO -> functions, and it is the second one that catches a shared
# pin: the two-PWM bridge scheme puts one enable on four motors, which is
# correct and looks like a collision until it is shown as one row.
#
# The pin catalogue's findings are appended so a sheet that describes an
# impossible pinout says so on the same page.
# ==============================================================================
import argparse
import os
import sys

import yaml

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO_ROOT, "scripts"))
import pin_catalog  # noqa: E402

DIRECTION_WORD = {"out": "output", "in": "input", "adc": "analog in", "io": "I2C"}


def _pin(v):
    try:
        return int(v)
    except (TypeError, ValueError):
        return -1


def collect(tgt: dict):
    """(function, gpio, direction, note) for every wired pin, in the order a
    person wires them: power first, then drive, then sensors."""
    pins = tgt.get("pins") or {}
    rows = []
    bat = pins.get("battery") or {}
    if _pin(bat.get("pin", -1)) >= 0:
        note = ""
        r1, r2 = bat.get("r1"), bat.get("r2")
        if r1 is not None and r2 is not None:
            note = f"divider {r1} Ω / {r2} Ω to GND"
        rows.append(("Battery sense", _pin(bat["pin"]), "adc", note))
    if _pin(pins.get("dac", -1)) >= 0:
        rows.append(("DAC sweep (calibration only)", _pin(pins["dac"]), "out",
                     "jumper to the battery sense pin while calibrating"))
    for n in range(1, 5):
        m = pins.get(f"motor{n}") or {}
        # Inversion swaps the DIRECTION the firmware drives, so it is noted on
        # the direction pins and not on the PWM, which inverts nothing.
        inv = "direction inverted" if m.get("invert") else ""
        for key, label in (("pwm", "PWM"), ("in_a", "IN A"), ("in_b", "IN B")):
            if _pin(m.get(key, -1)) >= 0:
                rows.append((f"Motor {n} {label}", _pin(m[key]), "out",
                             inv if key != "pwm" else ""))
    for n in range(1, 5):
        e = pins.get(f"encoder{n}") or {}
        inv = "inverted" if e.get("invert") else ""
        for key, label in (("pin_a", "A"), ("pin_b", "B")):
            if _pin(e.get(key, -1)) >= 0:
                rows.append((f"Encoder {n} {label}", _pin(e[key]), "in", inv))
    i2c = pins.get("i2c") or {}
    for key, label in (("sda", "SDA"), ("scl", "SCL")):
        if _pin(i2c.get(key, -1)) >= 0:
            rows.append((f"I2C {label}", _pin(i2c[key]), "io", ""))
    sonar = pins.get("sonar") or {}
    if _pin(sonar.get("trigger", -1)) >= 0:
        rows.append(("Sonar trigger", _pin(sonar["trigger"]), "out", "HC-SR04"))
    if _pin(sonar.get("echo", -1)) >= 0:
        rows.append(("Sonar echo", _pin(sonar["echo"]), "in", "HC-SR04 (5 V echo needs a divider)"))
    if _pin(pins.get("led", -1)) >= 0:
        rows.append(("Status LED", _pin(pins["led"]), "out", ""))
    lidar = tgt.get("lidar") or {}
    if str(lidar.get("comm_mode", "")).lower() == "serial" and _pin(lidar.get("rx_pin", -1)) >= 0:
        rows.append(("LiDAR TX -> MCU RX", _pin(lidar["rx_pin"]), "in",
                     f"{lidar.get('model', 'LD19')} @ {lidar.get('baudrate', 230400)} baud"))
    return rows


def render(params: dict) -> str:
    tgt = params.get("base_controller") or {}
    robot = (params.get("robot") or {}).get("name", "robot")
    mcu = str(tgt.get("mcu") or tgt.get("name") or "").lower()
    key = pin_catalog.mcu_key(mcu)
    label = pin_catalog.CATALOG[key]["label"] if key else (mcu or "unknown MCU")
    rows = collect(tgt)
    sensors = tgt.get("sensors") or {}

    out = [f"# Wiring — {robot}", "",
           f"MCU: **{label}**  ·  transport: `{tgt.get('transport', 'serial')}`"
           f"  ·  driver: `{tgt.get('driver_type', 'GENERIC_2_IN')}`", ""]

    led_only = bool(rows) and all(fn == "Status LED" for fn, _, _, _ in rows)
    if not rows or led_only:
        out += [("_Only the onboard LED is wired: this is a bare module. Every other "
                 "function is simulated or absent._" if led_only else
                 "_No pins are wired: this is a bare module. Every function below is "
                 "simulated or absent._"), ""]
    if rows:
        out += ["## By function", "", "| Function | GPIO | Direction | Note |", "|---|---:|---|---|"]
        for fn, gpio, d, note in rows:
            out.append(f"| {fn} | {gpio} | {DIRECTION_WORD.get(d, d)} | {note} |")
        out.append("")

        by_gpio = {}
        for fn, gpio, d, note in rows:
            by_gpio.setdefault(gpio, []).append(fn)
        out += ["## By GPIO", "", "| GPIO | Functions |", "|---:|---|"]
        for gpio in sorted(by_gpio):
            fns = by_gpio[gpio]
            shared = "  ⚠ shared" if len(fns) > 1 else ""
            out.append(f"| {gpio} | {', '.join(fns)}{shared} |")
        out.append("")

    named = [(k, v) for k, v in (("IMU", sensors.get("imu")), ("Magnetometer", sensors.get("mag")),
                                 ("Environment", sensors.get("env")), ("Current", sensors.get("current")))
             if v and str(v).upper() not in ("NONE", "FAKE")]
    if named or sensors.get("use_fake_wheel") or sensors.get("use_fake_imu"):
        out += ["## Sensors", ""]
        for k, v in named:
            out.append(f"- {k}: `{v}` on the I2C bus")
        fakes = [n for n, f in (("wheels", "use_fake_wheel"), ("IMU", "use_fake_imu"),
                                ("magnetometer", "use_fake_mag"), ("LiDAR", "use_fake_ld19"))
                 if sensors.get(f)]
        if fakes:
            out.append(f"- Simulated: {', '.join(fakes)}")
        out.append("")

    findings = pin_catalog.check_config(params)
    if findings:
        out += ["## Pin catalogue findings", ""]
        for level, msg in findings:
            out.append(f"- **{level}**: {msg}")
        out.append("")
    else:
        out += ["_Pin catalogue: every wired pin checked out for this MCU._", ""]
    return "\n".join(out)


def main() -> int:
    ap = argparse.ArgumentParser(description="Wiring chart for a robot config, as Markdown.")
    ap.add_argument("--params", required=True, help="the robot's <robot>_config.yaml")
    ap.add_argument("--out", help="write here instead of stdout")
    a = ap.parse_args()
    with open(a.params) as fh:
        params = yaml.safe_load(fh) or {}
    text = render(params)
    if a.out:
        with open(a.out, "w") as fh:
            fh.write(text)
        print(f"wrote {a.out}")
    else:
        sys.stdout.write(text)
    return 0


if __name__ == "__main__":
    sys.exit(main())
