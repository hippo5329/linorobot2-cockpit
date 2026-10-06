#!/usr/bin/env python3
"""Is the robot being told to move and not moving? The virtual gamepad's stall check.

The Teleop tab warns "Robot is not moving -- something is in the way" when this says so
twice in a row (app-workflow.js checkStall). Until 2026-10-06 the server behind it was a stub
that always answered `stalled: false`, so the warning could never fire.

`verdict()` is pure, so it is tested without ROS: scripts/gamepad_publisher.py feeds it the
command it is publishing and the odometry it hears, and writes the result where the cockpit
reads it (GamepadRunner.status()).
"""
import math

# A command worth judging: a creep below these is lost in friction and odometry noise.
MIN_LINEAR = 0.05        # m/s
MIN_ANGULAR = 0.2        # rad/s
# How long a command must be held before the base is expected to follow: the drivetrain's
# spin-up plus the odometry pipeline. A single early sample is the robot accelerating.
SPIN_UP_S = 1.0
# Moving at under this fraction of what was commanded is not moving.
FOLLOW_FRACTION = 0.25
# Odometry older than this says nothing about now.
ODOM_FRESH_S = 1.0


def verdict(commanded, measured, held_s, odom_age_s, subscribers=None):
    """{"stalled": bool | None, "reason": str}. None means it cannot tell.

    commanded, measured: (linear_x, linear_y, angular_z). held_s: how long the current
    non-zero command has been held. odom_age_s: age of the last odometry, None if none yet.
    subscribers: how many readers the command topic has matched, None if not known. With
    none, every command is lost before it reaches a base: right after the pad starts, DDS
    discovery of its new publisher took seconds, and the check called that a stall.
    """
    lin_c = math.hypot(commanded[0], commanded[1])
    ang_c = abs(commanded[2])
    if lin_c < MIN_LINEAR and ang_c < MIN_ANGULAR:
        return {"stalled": False, "reason": "no command to follow"}
    if subscribers == 0:
        return {"stalled": None, "reason": "nothing subscribes to the command topic yet: cannot tell"}
    if held_s < SPIN_UP_S:
        return {"stalled": False, "reason": f"spinning up ({held_s:.1f} s)"}
    if odom_age_s is None:
        return {"stalled": None, "reason": "no odometry heard: cannot tell"}
    if odom_age_s > ODOM_FRESH_S:
        return {"stalled": None, "reason": f"odometry {odom_age_s:.1f} s old: cannot tell"}
    lin_m = math.hypot(measured[0], measured[1])
    ang_m = abs(measured[2])
    # Every component worth commanding must be followed; a turn on the spot that does not
    # turn is a stall even though no linear speed was asked for.
    lagging = []
    if lin_c >= MIN_LINEAR and lin_m < FOLLOW_FRACTION * lin_c:
        lagging.append(f"linear {lin_m:.2f} of {lin_c:.2f} m/s")
    if ang_c >= MIN_ANGULAR and ang_m < FOLLOW_FRACTION * ang_c:
        lagging.append(f"angular {ang_m:.2f} of {ang_c:.2f} rad/s")
    if lagging:
        return {"stalled": True, "reason": "commanded but not moving: " + ", ".join(lagging)}
    return {"stalled": False, "reason": "following the command"}
