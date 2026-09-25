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

    every pin -1, every sensor simd, 2WD, nothing on the bus.

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
import re
import sys

import yaml

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO_ROOT, "scripts"))

import pin_catalog  # noqa: E402

# The reference that donates the non-hardware scaffolding. Every reference
# carries the same ekf/slam/nav2 template (tests/test_default_chassis.py), so any
# would do; the GenDrv is the differential one, which a bare 2WD module is.
DONOR = os.path.join(REPO_ROOT, "config", "reference", "gendrv_config.yaml")

# PlatformIO env / mcu -> the board string and a human label.
BOARDS = {
    "pico":    ("rpipico",      "Raspberry Pi Pico (RP2040)"),
    "picow":   ("rpipicow",     "Raspberry Pi Pico W (RP2040 + CYW43)"),
    "pico2":   ("rpipico2",     "Raspberry Pi Pico 2 (RP2350)"),
    "pico2w":  ("rpipico2w",    "Raspberry Pi Pico 2 W (RP2350 + CYW43)"),
    "esp32":   ("esp32dev",     "ESP32 (WROOM)"),
    "esp32s3": ("esp32-s3-devkitc-1", "ESP32-S3"),
}

# The Sim MCU: no silicon at all. sim_base_node stands in for the board on the
# robot computer, so this bare robot has no board string, no port and no baud --
# it stands on its own rather than borrowing pico2's (user, 2026-09-25): every
# simulated device on, every pin -1, the LED included.
SIM_MCU = "sim"
KNOWN = sorted(list(BOARDS) + [SIM_MCU])

# A bare board has no wheels to measure, so the kinematics are the ONE default
# chassis every bare preset, reference config and release image share -- taken
# from gen_firmware_header.bare_mcu_params so this file cannot drift from it
# again (it did: 0.152 m wheels, inverted even-numbered motors, LED -1).
import gen_firmware_header  # noqa: E402


def bare_kinematics() -> dict:
    kin = copy.deepcopy(gen_firmware_header.bare_mcu_params("esp32")["kinematics"])
    kin.setdefault("stamped_cmd_vel", "auto")
    kin.setdefault("fr_wheels_distance", 0)
    kin.setdefault("motor_operating_voltage", 12)
    kin.setdefault("motor_power_max_voltage", 12)
    kin.setdefault("pid", {"kp": 0.6, "ki": 0.8, "kd": 0.5})
    return kin


def _bare_comm_mode(mcu: str) -> str:
    """Which sink a bare board's synthetic scan takes by default.

    Not one answer for every MCU, because the links are not comparable.

    An RP2 carries the scan over micro-ROS and still holds the control loop:
    measured on the bench 2026-09-20, /odom and /imu at 50.0 Hz with /raw_scan
    at 85-100 Hz alongside. So `topic` is right there -- a bare Pico needs no
    wiring at all to produce a scan.

    An ESP32 cannot. The same configuration drops every topic to 40-45 Hz,
    because 921600 baud is carrying the scan and the 50 Hz loop together (33 Hz
    on a GenDrv that also reads four I2C sensors). Its scan has to leave by a
    UART (`serial`, LIDAR_RXD to a bridge) or over the radio (`udp`) -- and a
    BARE ESP32 has neither wired, so it has no scan source, which is the honest
    default rather than one that quietly halves the control rate.
    """
    return "topic" if mcu.startswith("pico") else "serial"


# The simulated world, written out IN FULL rather than left to the firmware's
# #ifndef fallbacks.
#
# The firmware is happy either way -- an absent key keeps its compiled default,
# which is what makes a blank env boot -- so this is for the person reading the
# config. Simulation mode is this project's default, so the simulated room, mass and
# drivetrain losses ARE the robot on every bench run, and a config that lists
# only what someone chose to override describes none of it. Sweeping one of them
# then starts from a value you can see.
#
# The values are PARSED from the headers that implement the model, for the same
# reason drivetrain_report.py parses them: a table restating another file's
# numbers drifts from it silently, and then the config describes a robot that
# does not exist. robot_radius is deliberately NOT here -- it follows the largest
# robot_radius the costmaps plan with, and a literal would break that agreement
# (see mcu_env.py, and the soak that sat in a lethal cell for 154 goals).
SIM_DEFAULTS = (
    ("map_width",          "lidar/sim_ld19.h",    "SIM_MAP_WIDTH",           float),
    ("map_height",         "lidar/sim_ld19.h",    "SIM_MAP_HEIGHT",          float),
    ("wall_obstacle",      "lidar/sim_ld19.h",    "SIM_WALL_OBSTACLE",       bool),
    ("wall_x1",            "lidar/sim_ld19.h",    "SIM_WALL_X1",             float),
    ("wall_y1",            "lidar/sim_ld19.h",    "SIM_WALL_Y1",             float),
    ("wall_x2",            "lidar/sim_ld19.h",    "SIM_WALL_X2",             float),
    ("wall_y2",            "lidar/sim_ld19.h",    "SIM_WALL_Y2",             float),
    ("robot_mass",         "encoder/sim_wheel.h", "SIM_ROBOT_MASS",          float),
    ("wheel_noise_rpm",    "encoder/sim_wheel.h", "SIM_WHEEL_NOISE_RPM",     float),
    ("gear_efficiency",    "encoder/sim_wheel.h", "SIM_GEAR_EFFICIENCY",     float),
    ("gear_drag_rpm",      "encoder/sim_wheel.h", "SIM_WHEEL_COULOMB_RPM",   float),
    ("battery_sag",        "encoder/sim_wheel.h", "SIM_BATT_SAG",            float),
    ("battery_sag_tau_ms", "encoder/sim_wheel.h", "SIM_BATT_SAG_TAU_MS",     float),
    ("driver_drop",        "encoder/sim_wheel.h", "SIM_DRV_DROP",            float),
    ("driver_resistance",  "encoder/sim_wheel.h", "SIM_DRV_R",               float),
    ("motor_stall_amps",   "encoder/sim_wheel.h", "SIM_MOTOR_STALL_A",       float),
    ("driver_current_limit", "encoder/sim_wheel.h", "SIM_DRV_ILIMIT_A",      float),
    # How the IMU is mounted, in degrees. Zero is a level part -- which is what
    # the model assumed unconditionally until 2026-09-25, so no simulated robot
    # could reach the gravity-in-ax/ay path the EKF has to cope with. Present in
    # every config, defaulting to the firmware's own zero, so the sweep that
    # tilts it does not need a firmware build.
    ("imu_mount_roll_deg",  "encoder/sim_wheel.h"     , "SIM_IMU_MOUNT_ROLL_DEG",  float),
    ("imu_mount_pitch_deg", "encoder/sim_wheel.h"     , "SIM_IMU_MOUNT_PITCH_DEG", float),
)


def bare_simulation(strict: bool = False) -> dict:
    """The `simulation:` block, every key present, values from the firmware.

    A macro this cannot find is SKIPPED, with a warning, rather than raising --
    and that is the whole lesson of 2026-09-24. The first version exited, on the
    reasoning that writing a stale default is worse than stopping. True of the
    value; false of the blast radius. Inside a released container the firmware
    tree under /ws is the IMAGE's copy, while the bench stages the repo's
    scripts over the top, so a header that moved in the repo and not in the
    image made this generator exit -- and it is called during bringup, so the
    exit took down the whole Nav2 run. Every RP2 leg of a matrix died in three
    seconds with "cannot find #define SIM_GEAR_EFFICIENCY" and nothing to do
    with the robot.

    An absent key is not a stale value: mcu_env writes nothing for it and the
    firmware keeps its own #ifndef default, which is the correct behaviour and
    is what happened before any of these keys existed. So the degraded case is
    exactly the old case, and the run continues.

    Drift is still caught -- in the tests, which is where the repo's own copies
    are compared and where a failure costs nobody an hour of bench time. Pass
    `strict=True` to get the old behaviour when that is what you want.
    """
    cache = {}
    out = {}
    missing = []
    for cfg_key, rel, macro, cast in SIM_DEFAULTS:
        path = os.path.join(REPO_ROOT, "firmware", "common", "lib", rel)
        if rel not in cache:
            try:
                with open(path, encoding="utf-8") as fh:
                    cache[rel] = fh.read()
            except OSError as exc:
                cache[rel] = ""
                print(f"[gen_bare_config] {path}: {exc}", file=sys.stderr)
        # The FIRST definition wins: these headers guard each macro with #ifndef
        # and a #define, so a later line is the same value, and SIM_ROBOT_MASS
        # has an earlier ROBOT_WEIGHT branch that is not a number at all.
        m = re.search(r"^\s*#define\s+" + macro + r"\s+(-?[0-9.]+)f?\s*(?://.*)?$",
                      cache[rel], re.MULTILINE)
        if not m:
            missing.append((cfg_key, macro))
            continue
        value = float(m.group(1))
        out[cfg_key] = bool(value) if cast is bool else value
    if missing:
        names = ", ".join(f"{k} (#define {m})" for k, m in missing)
        if strict:
            raise SystemExit(f"gen_bare_config: cannot read {names} from the firmware "
                             f"headers -- the simulated world moved")
        print(f"[gen_bare_config] not in this tree's firmware headers, so left out of "
              f"the config (the firmware's own defaults stand): {names}", file=sys.stderr)
    return out


def bare_pins(mcu: str = "esp32") -> dict:
    """Every pin unconnected -- except the onboard LED.

    A bare module still has its LED, simulation mode drives the real one, and a board
    on a bench should blink out of the box (user rule, 2026-09-22): the pin is
    the MCU's own, from the same table the release image uses. Invert flags are
    OFF: the default is forward, for motors and encoders alike; a real chassis
    gets its inversions measured, never inherited from a default.
    """
    pins = {}
    for n in range(1, 5):
        pins[f"motor{n}"] = {"pwm": -1, "in_a": -1, "in_b": -1, "invert": False}
        pins[f"encoder{n}"] = {"pin_a": -1, "pin_b": -1, "invert": False}
    pins["i2c"] = {"sda": -1, "scl": -1}
    pins["led"] = -1 if mcu == SIM_MCU else \
        gen_firmware_header.bare_mcu_params(mcu)["base_controller"]["pins"]["led"]
    pins["battery"] = {"pin": -1, "r1": 30000, "r2": 7500}
    pins["sonar"] = {"trigger": -1, "echo": -1}
    return pins


def bare_sensors() -> dict:
    """Everything simulated: a bare board must never wait on a bus that is empty.

    An I2C read to a chip that is not there is a NACK loop, and the 50 Hz
    control loop is what stalls. SIM is not a convenience here, it is what
    keeps a wireless-less, sensorless board answering micro-ROS at all.
    """
    return {
        "imu": "SIM",
        "mag": "NONE",
        "use_sim_imu": True,
        "use_sim_mag": True,
        "use_sim_wheel": True,
        "use_sim_ld19": True,
        "use_sim_env": True,
        "use_sim_sonar": True,
        "use_sim_battery": True,
        "current": "NONE",
        "env": "NONE",
    }


def bare_config(mcu: str, name: str = None, donor_path: str = None) -> dict:
    """The bare-module config for `mcu`, complete and ready to save."""
    key = (mcu or "").strip().lower()
    if key not in KNOWN:
        raise ValueError(f"unknown mcu {mcu!r}; known: {', '.join(KNOWN)}")
    board, label = BOARDS.get(key, ("", "Sim MCU (sim_base_node on this computer)"))
    robot_name = name or f"bare_{key}"

    with open(donor_path or DONOR) as fh:
        params = yaml.safe_load(fh)

    # Keep only the parts that are not a statement about this board's hardware.
    params = {k: copy.deepcopy(v) for k, v in params.items()
              if k in ("geometry", "ekf", "slam", "nav2", "ros_distro")}

    params["robot"] = {
        "name": robot_name,
        "description": f"{label} bare module -- all pins N/C, simulated sensors",
    }
    if key == SIM_MCU:
        # Nothing to build, probe or flash: only what the simulated base reads.
        # The firmware itself, compiled for the robot computer (firmware/host):
        # a micro-ROS client over udp4 to the agent on the same machine, its
        # LD19 emulator streaming to the udp_server driver. bringup.launch.py
        # enforces these whatever a config says (scripts/host_firmware.py);
        # they are written here so the file says what runs.
        params["base_controller"] = {
            "name": SIM_MCU,
            "description": "Sim MCU: the firmware on the robot computer, micro-ROS over udp4, no board",
            "mcu": "host",
            "driver_type": "BTS7960",
            "transport": "udp4",
            "agent_ip": "127.0.0.1",
            "udp_port": 8888,
            "lidar": {"model": "ld19", "comm_mode": "udp", "server_ip": "127.0.0.1", "udp_port": 8889,
                      "raw_scan_topic": "raw_scan"},
            "sensors": bare_sensors(),
            "simulation": bare_simulation(),
            "pins": bare_pins(SIM_MCU),
        }
    else:
        params["base_controller"] = {
            "name": key,
            "description": f"{label}, bare module, zero-wiring default",
            "mcu": key,
            "board": board,
            "driver_type": "BTS7960",
            "transport": "serial",
            "serial_port": "/dev/ttyACM0" if key.startswith("pico") else "/dev/ttyUSB0",
            "baudrate": 921600,
            "lidar": {"model": "ld19", "comm_mode": _bare_comm_mode(key),
                      "raw_scan_topic": "raw_scan"},
            "sensors": bare_sensors(),
            "simulation": bare_simulation(),
            "pins": bare_pins(key),
        }
    params["kinematics"] = bare_kinematics()
    # Put the blocks back in the order every shipped config uses, so a generated
    # file and a hand-written one diff cleanly against each other.
    order = ["robot", "base_controller", "kinematics", "geometry", "ekf", "slam",
             "nav2", "ros_distro"]
    params = {k: params[k] for k in order if k in params}

    # NOT auto-derived here, deliberately.
    #
    # Deriving the limits for the regenerated bare config is the consistent
    # thing to do -- the drivetrain VARIANTS get it, and a bare config otherwise
    # pairs the donor's derived envelope with an underived max_rpm_ratio. But
    # doing it RAISES a 2wd base's limits (it sags less than the four-wheel case
    # the fractions were calibrated on), and 2wd is the slice failing 3/10 on
    # hardware across two independent matrix runs while skid_steer and mecanum
    # pass 10/10. Raising the limits of the one drivetrain that is already
    # marginal is not something the evidence supports.
    #
    # So the bare config keeps the donor's numbers until the 2wd flakiness is
    # understood. See cockpit/PROGRESS.md.
    return params


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("mcu", choices=KNOWN, help="which silicon, or `sim` for the Sim MCU")
    ap.add_argument("--name", help="robot name (default bare_<mcu>)")
    ap.add_argument("-o", "--out", help="write here instead of stdout")
    a = ap.parse_args()

    params = bare_config(a.mcu, a.name)
    # The Sim MCU has no pins to check.
    findings = [] if a.mcu == SIM_MCU else pin_catalog.check_config(params)
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
