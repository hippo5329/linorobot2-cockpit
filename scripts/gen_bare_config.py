#!/usr/bin/env python3
"""gen_bare_config.py -- the bare-module config, generated rather than stored.

A bare module is not a robot design. It is the ABSENCE of one: every pin
unconnected, every sensor simulated, so a freshly plugged board can be flashed
and talked to before a single wire exists. That is a rule, not a file, and it
is identical on every MCU apart from the name of the silicon -- which is why
shipping one YAML per board (rover_pico2, pico, picow, pico2w) meant four files
that had to be kept in sync by hand and drifted anyway: `rover_pico2` carried
`led: 25` while the UI's own bare design used `led: -1`, and nothing reconciled
them.

So it is generated. One function, one rule, every board:

    every pin -1, every sensor faked, 2WD, nothing on the bus.

The scaffolding a robot config also needs -- geometry, ekf, slam, nav2 -- is
taken from the shipped mecanum reference rather than duplicated here, so tuning
that ships stays the tuning a bare board gets, and this file never has to be
updated when a Nav2 key is added.

    python3 scripts/gen_bare_config.py pico2            # YAML to stdout
    python3 scripts/gen_bare_config.py esp32 -o out.yaml
"""
import argparse
import copy
import os
import sys

import yaml

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO_ROOT, "scripts"))

import pin_catalog  # noqa: E402

# The reference that donates the non-hardware scaffolding. Any shipped config
# would do; this is the one the RP2 release images are built from.
DONOR = os.path.join(REPO_ROOT, "config", "reference", "pico2_mecanum_config.yaml")

# PlatformIO env / mcu -> the board string and a human label.
BOARDS = {
    "pico":    ("rpipico",      "Raspberry Pi Pico (RP2040)"),
    "picow":   ("rpipicow",     "Raspberry Pi Pico W (RP2040 + CYW43)"),
    "pico2":   ("rpipico2",     "Raspberry Pi Pico 2 (RP2350)"),
    "pico2w":  ("rpipico2w",    "Raspberry Pi Pico 2 W (RP2350 + CYW43)"),
    "esp32":   ("esp32dev",     "ESP32 (WROOM)"),
    "esp32s3": ("esp32-s3-devkitc-1", "ESP32-S3"),
}

# A bare board has no wheels to measure, so these are placeholders that make the
# kinematics maths well-formed rather than claims about hardware. They are the
# values the bare design has always used.
BARE_KINEMATICS = {
    "base_type": "2wd",
    "stamped_cmd_vel": "auto",
    "wheel_diameter": 0.152,
    "lr_wheels_distance": 0.271,
    "fr_wheels_distance": 0,
    "max_rpm": 140,
    "max_rpm_ratio": 0.85,
    "counts_per_rev": 4000,
    "pwm_bits": 10,
    "pwm_frequency": 20000,
    "motor_operating_voltage": 12,
    "motor_power_max_voltage": 12,
    "pid": {"kp": 0.6, "ki": 0.8, "kd": 0.5},
}


def bare_pins() -> dict:
    """Every pin unconnected.

    Including the LED. A wired design names its LED because the blink pattern
    is the only thing a board says before micro-ROS is up, but "all pins -1" is
    what bare means, and an LED pin is still a pin being driven -- on a W board
    GP25 is not even a GPIO. A user who wants the blink sets it in the Pins tab.
    """
    pins = {}
    for n in range(1, 5):
        pins[f"motor{n}"] = {"pwm": -1, "in_a": -1, "in_b": -1, "invert": n % 2 == 0}
        pins[f"encoder{n}"] = {"pin_a": -1, "pin_b": -1, "invert": n % 2 == 0}
    pins["i2c"] = {"sda": -1, "scl": -1}
    pins["led"] = -1
    pins["battery"] = {"pin": -1, "r1": 30000, "r2": 7500}
    pins["sonar"] = {"trigger": -1, "echo": -1}
    return pins


def bare_sensors() -> dict:
    """Everything simulated: a bare board must never wait on a bus that is empty.

    An I2C read to a chip that is not there is a NACK loop, and the 50 Hz
    control loop is what stalls. FAKE is not a convenience here, it is what
    keeps a wireless-less, sensorless board answering micro-ROS at all.
    """
    return {
        "imu": "FAKE",
        "mag": "NONE",
        "use_fake_imu": True,
        "use_fake_mag": True,
        "use_fake_wheel": True,
        "use_fake_ld19": True,
        "use_fake_env": True,
        "current": "NONE",
        "env": "NONE",
    }


def bare_config(mcu: str, name: str = None, donor_path: str = None) -> dict:
    """The bare-module config for `mcu`, complete and ready to save."""
    key = (mcu or "").strip().lower()
    if key not in BOARDS:
        raise ValueError(f"unknown mcu {mcu!r}; known: {', '.join(sorted(BOARDS))}")
    board, label = BOARDS[key]
    robot_name = name or f"bare_{key}"

    with open(donor_path or DONOR) as fh:
        params = yaml.safe_load(fh)

    # Keep only the parts that are not a statement about this board's hardware.
    params = {k: copy.deepcopy(v) for k, v in params.items()
              if k in ("geometry", "ekf", "slam", "nav2", "ros_distro")}

    params["robot"] = {
        "name": robot_name,
        "description": f"{label} bare module -- all pins N/C, fake sensors",
    }
    params["base_controller"] = {
        "name": key,
        "description": f"{label}, bare module, zero-wiring default",
        "mcu": key,
        "board": board,
        "driver_type": "BTS7960",
        "transport": "serial",
        "serial_port": "/dev/ttyACM0" if key.startswith("pico") else "/dev/ttyUSB0",
        "baudrate": 921600,
        "lidar": {"model": "ld19", "comm_mode": "topic", "raw_scan_topic": "raw_scan"},
        "sensors": bare_sensors(),
        "pins": bare_pins(),
    }
    params["kinematics"] = copy.deepcopy(BARE_KINEMATICS)
    # Put the blocks back in the order every shipped config uses, so a generated
    # file and a hand-written one diff cleanly against each other.
    order = ["robot", "base_controller", "kinematics", "geometry", "ekf", "slam",
             "nav2", "ros_distro"]
    return {k: params[k] for k in order if k in params}


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("mcu", choices=sorted(BOARDS), help="which silicon")
    ap.add_argument("--name", help="robot name (default bare_<mcu>)")
    ap.add_argument("-o", "--out", help="write here instead of stdout")
    a = ap.parse_args()

    params = bare_config(a.mcu, a.name)
    findings = pin_catalog.check_config(params)
    for level, msg in findings:
        print(f"[pins] {level}: {msg}", file=sys.stderr)
    if any(level == "error" for level, _ in findings):
        print("Error: the generated bare config has pin errors.", file=sys.stderr)
        return 1

    text = yaml.safe_dump(params, sort_keys=False, default_flow_style=False)
    if a.out:
        with open(a.out, "w") as fh:
            fh.write(text)
        print(f"wrote {a.out}")
    else:
        sys.stdout.write(text)
    return 0


if __name__ == "__main__":
    sys.exit(main())
