#!/usr/bin/env python3
"""Applying a reference design, or a board's bare module, to the user's own robot.

User decisions, 2026-10-06 (the lab's cockpit/docs/robots-sim-vs-reference.md):
  * the user names their robot first; a reference design is a template applied to it, and
    the robot records it (robot.reference) -- real from then on, nothing simulated;
  * a design changes the CONTROLLER side (base_controller); the robot keeps its chassis --
    kinematics, geometry, and its Nav2/EKF/SLAM tuning -- because the user "may have a
    chassis and just want to try a different controller";
  * a robot still on the untouched default chassis has none to keep and takes the design's;
  * a KIT (robot.kit: the XRP, the Maker's Pet) brings its own motors, wheels and chassis,
    which the user may then override with their own;
  * a named robot with no design is a bare module of its board, every device simulated.
"""
import copy

import gen_bare_config

# The chassis: what a user keeps when trying another controller.
CHASSIS_BLOCKS = ("kinematics", "geometry", "ekf", "slam", "nav2")
# The keys that say a robot has its own chassis: wheels, track, drivetrain, motors.
CHASSIS_KEYS = ("base_type", "wheel_diameter", "lr_wheels_distance", "fr_wheels_distance",
                "motor_max_rpm", "max_rpm_ratio", "counts_per_rev", "motor_operating_voltage",
                "motor_power_max_voltage")


def custom_chassis(params: dict) -> bool:
    """True when the robot's chassis is its own, not the one default chassis every bare
    module starts with (the same on every MCU)."""
    default = gen_bare_config.bare_config("pico2")
    k, dk = params.get("kinematics") or {}, default.get("kinematics") or {}
    if any(k.get(key) != dk.get(key) for key in CHASSIS_KEYS if key in k or key in dk):
        return True
    return (params.get("geometry") or {}) != (default.get("geometry") or {})


def apply_design(robot: dict, design: dict, design_name: str):
    """(new config, chassis_kept, is_kit): the design applied to the user's robot."""
    name = (robot.get("robot") or {}).get("name")
    out = copy.deepcopy(design)
    is_kit = bool((design.get("robot") or {}).get("kit"))
    kept = not is_kit and custom_chassis(robot)
    if kept:
        for block in CHASSIS_BLOCKS:
            if block in robot:
                out[block] = copy.deepcopy(robot[block])
    out.setdefault("robot", {})
    out["robot"]["name"] = name
    out["robot"]["reference"] = design_name
    return out, kept, is_kit


def apply_bare(robot_name: str, mcu: str) -> dict:
    """The user's robot as a bare module of `mcu`: every pin unconnected, every device
    simulated, no design."""
    out = gen_bare_config.bare_config(mcu)
    out["robot"]["name"] = robot_name
    out["robot"].pop("reference", None)
    return out


# Settings on the controller that are the user's, not the design's or the wiring's: kept when a
# real robot goes back to a bare module.
KEPT_CONTROLLER_KEYS = ("serial_port", "baudrate", "console", "domain_id", "robot_ip",
                        "telemetry", "wifi")


def revert_to_bare(robot: dict) -> dict:
    """A real robot back to a bare module of its own board: every pin unassigned (but the
    LED), every device simulated, no design -- and everything else the robot's own: its name,
    its chassis (kinematics, geometry, Nav2/EKF/SLAM tuning) and its port, domain and network
    names. The link is the bare module's (USB serial).

    User, 2026-10-13: "offer an option to revert a real robot to bare module, ie, every pin
    unassigned." apply_bare() is the board switch: it takes the bare module's chassis too."""
    ctrl = robot.get("base_controller") or {}
    mcu = str(ctrl.get("mcu") or ctrl.get("name") or "").lower()
    if mcu not in gen_bare_config.KNOWN:
        raise ValueError(f"no bare module for the board '{mcu}'")
    out = gen_bare_config.bare_config(mcu)
    for block in CHASSIS_BLOCKS + ("supervisor",):
        if block in robot:
            out[block] = copy.deepcopy(robot[block])
    for key in KEPT_CONTROLLER_KEYS:
        if key in ctrl:
            out["base_controller"][key] = copy.deepcopy(ctrl[key])
    name = (robot.get("robot") or {}).get("name")
    out.setdefault("robot", {})["name"] = name
    for key in ("reference", "kit"):
        out["robot"].pop(key, None)
    return out
