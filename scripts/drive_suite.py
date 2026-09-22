#!/usr/bin/env python3
"""Six manoeuvres, checked for sign and magnitude against odometry.

Rates prove the board talks. Only this proves it MOVES, and moves the way it
was told: the fake-wheel invert bug and the PID integral windup both produced
perfect 50 Hz topics on a robot that was spinning on the spot or pinned at
both rails. Every hardware run ends with this, after the topic gate.

Each manoeuvre is 5 s of a constant command, preceded by 2.5 s of zero so the
base settles. The check is the extreme of the reported twist in the commanded
direction, against the command, with a tolerance loose enough that a base
which tracks badly still passes: this is a sign-and-magnitude test, not a
performance one (see scripts/test_nav2_goal.py for the same reasoning).

    python3 scripts/drive_suite.py                       # plain Twist
    python3 scripts/drive_suite.py geometry_msgs/msg/TwistStamped
    python3 scripts/drive_suite.py --config ~/linorobot2-config/lino1_config.yaml
    python3 scripts/drive_suite.py --prefix lino1

A namespaced robot has to be addressed by its namespace or the suite drives
nothing and then reports the silence as the board's fault. `--config` reads
`base_controller.topic_prefix` through the same normaliser the launchers and
mcu_env use, so the suite reaches the robot the config describes; `--prefix`
says it outright. Neither given, the topics are the unprefixed ones and the
behaviour is exactly what it always was.

Each line also says WHERE the robot was, because the suite now runs after the
Nav2 legs and its position is therefore uncontrolled. The simulated room clamps
the pose at its walls and at the obstacle wall (x = 2.0, |y| <= 1.5), and a
clamp acting during a manoeuvre is differentiated into a velocity the base was
never commanded: measured 2026-09-22 on the GenDrv, both spins reported
vx ~ +0.15 m/s while commanded (0.00, +/-1.50), and both were positive. Without
the pose that reads as a base fault; with it, a "near obstacle wall" note says
what it is. The room's geometry is mirrored here from firmware/common/lib/lidar/
fake_ld19.h -- if it changes there, change it here.

Exit status 0 when all six pass; the verdict line says how many did.
"""
import os
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import cockpit_paths  # noqa: E402  -- the prefix rule lives in one place


# The simulated room, mirrored from fake_ld19.h so a manoeuvre can say whether
# the pose it reports was being clamped.
ROOM_W, ROOM_H = 10.0, 6.0
ROBOT_R = 0.30
WALL_X, WALL_HALF_SPAN = 2.0, 1.5
NEAR = 0.05          # "against" a surface: within this of where the clamp holds


def _where(x: float, y: float) -> str:
    """Which surface, if any, the robot is being held against."""
    if x != x or y != y:                       # NaN: no pose seen
        return "pose unknown"
    notes = []
    if abs(abs(x) - (ROOM_W / 2 - ROBOT_R)) < NEAR:
        notes.append("room wall x")
    if abs(abs(y) - (ROOM_H / 2 - ROBOT_R)) < NEAR:
        notes.append("room wall y")
    if abs(y) <= WALL_HALF_SPAN + ROBOT_R and abs(abs(x - WALL_X) - ROBOT_R) < NEAR:
        notes.append("OBSTACLE WALL")
    return ", ".join(notes) if notes else "clear"


def _topics(prefix: str) -> tuple:
    """(cmd_vel, odom) for a robot whose namespace is `prefix` ("" = plain)."""
    ns = f"/{prefix}" if prefix else ""
    return f"{ns}/cmd_vel", f"{ns}/odom/unfiltered"


def main() -> int:
    argv = sys.argv[1:]
    prefix = ""
    rest = []
    i = 0
    while i < len(argv):
        if argv[i] == "--prefix" and i + 1 < len(argv):
            prefix = argv[i + 1].strip().strip("/")
            i += 2
        elif argv[i] == "--config" and i + 1 < len(argv):
            import yaml
            with open(os.path.expanduser(argv[i + 1])) as fh:
                prefix = cockpit_paths.robot_namespace(yaml.safe_load(fh) or {})
            i += 2
        else:
            rest.append(argv[i])
            i += 1
    cmd_topic, odom_topic = _topics(prefix)

    rclpy.init()
    node = Node("drive_suite")
    tname = rest[0].strip() if rest else "geometry_msgs/msg/Twist"
    stamped = "TwistStamped" in tname
    if stamped:
        from geometry_msgs.msg import TwistStamped as T
    else:
        from geometry_msgs.msg import Twist as T
    if prefix:
        print(f"[drive_suite] robot namespace /{prefix}: {cmd_topic} -> {odom_topic}",
              flush=True)
    pub = node.create_publisher(T, cmd_topic, 10)

    seen = {"vx": [], "wz": [], "x": float("nan"), "y": float("nan")}

    def _odom(m):
        seen["vx"].append(m.twist.twist.linear.x)
        seen["wz"].append(m.twist.twist.angular.z)
        seen["x"] = m.pose.pose.position.x
        seen["y"] = m.pose.pose.position.y

    node.create_subscription(Odometry, odom_topic, _odom, qos_profile_sensor_data)

    def command(lin: float, ang: float, secs: float) -> None:
        m = T()
        if stamped:
            m.twist.linear.x = lin
            m.twist.angular.z = ang
        else:
            m.linear.x = lin
            m.angular.z = ang
        t0 = time.time()
        while time.time() - t0 < secs:
            if stamped:
                m.header.stamp = node.get_clock().now().to_msg()
            pub.publish(m)
            rclpy.spin_once(node, timeout_sec=0.02)
            time.sleep(0.03)

    def run(label: str, lin: float, ang: float, secs: float = 5.0) -> bool:
        command(0.0, 0.0, 2.5)
        x0, y0 = seen["x"], seen["y"]
        seen["vx"].clear()
        seen["wz"].clear()
        command(lin, ang, secs)
        vx = seen["vx"] or [0.0]
        wz = seen["wz"] or [0.0]
        # The extreme in the commanded direction.
        got_vx = max(vx) if lin >= 0 else min(vx)
        got_wz = max(wz) if ang >= 0 else min(wz)
        ok_vx = abs(got_vx - lin) < max(0.12, abs(lin) * 0.45)
        ok_wz = abs(got_wz - ang) < max(0.45, abs(ang) * 0.45)
        x1, y1 = seen["x"], seen["y"]
        where = _where(x1, y1)
        print("%-12s cmd(%+.2f,%+.2f)  odom vx %+.3f (want %+.2f) %s   wz %+.3f (want %+.2f) %s"
              "   pose (%+.2f,%+.2f)->(%+.2f,%+.2f) %s"
              % (label, lin, ang, got_vx, lin, "ok" if ok_vx else "BAD",
                 got_wz, ang, "ok" if ok_wz else "BAD", x0, y0, x1, y1, where), flush=True)
        if not (ok_vx and ok_wz) and where not in ("clear", "pose unknown"):
            print("             ^ held against %s: the clamp moves the pose every cycle and "
                  "that shows up as a velocity nobody commanded. Not a base fault." % where,
                  flush=True)
        return ok_vx and ok_wz

    # Wait for the board before judging it: no odom in 10 s is its own failure.
    t0 = time.time()
    while not seen["vx"] and time.time() - t0 < 10:
        rclpy.spin_once(node, timeout_sec=0.1)
    if not seen["vx"]:
        print(f"VERDICT: nothing on {odom_topic} in 10 s -- is the agent up?")
        return 2

    results = [
        run("forward",     0.25,  0.00),
        run("backward",   -0.25,  0.00),
        run("left turn",   0.20,  0.80),
        run("right turn",  0.20, -0.80),
        run("left spin",   0.00,  1.50),
        run("right spin",  0.00, -1.50),
    ]
    command(0.0, 0.0, 2.5)
    print("VERDICT: %d/6 manoeuvres correct" % sum(results), flush=True)
    rclpy.shutdown()
    return 0 if all(results) else 1


if __name__ == "__main__":
    sys.exit(main())
