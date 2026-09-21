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

Exit status 0 when all six pass; the verdict line says how many did.
"""
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

ODOM_TOPIC = "/odom/unfiltered"   # the board's own report, before the EKF


def main() -> int:
    rclpy.init()
    node = Node("drive_suite")
    tname = sys.argv[1].strip() if len(sys.argv) > 1 else "geometry_msgs/msg/Twist"
    stamped = "TwistStamped" in tname
    if stamped:
        from geometry_msgs.msg import TwistStamped as T
    else:
        from geometry_msgs.msg import Twist as T
    pub = node.create_publisher(T, "/cmd_vel", 10)

    seen = {"vx": [], "wz": []}
    node.create_subscription(
        Odometry, ODOM_TOPIC,
        lambda m: (seen["vx"].append(m.twist.twist.linear.x),
                   seen["wz"].append(m.twist.twist.angular.z)),
        qos_profile_sensor_data)

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
        print("%-12s cmd(%+.2f,%+.2f)  odom vx %+.3f (want %+.2f) %s   wz %+.3f (want %+.2f) %s"
              % (label, lin, ang, got_vx, lin, "ok" if ok_vx else "BAD",
                 got_wz, ang, "ok" if ok_wz else "BAD"), flush=True)
        return ok_vx and ok_wz

    # Wait for the board before judging it: no odom in 10 s is its own failure.
    t0 = time.time()
    while not seen["vx"] and time.time() - t0 < 10:
        rclpy.spin_once(node, timeout_sec=0.1)
    if not seen["vx"]:
        print(f"VERDICT: nothing on {ODOM_TOPIC} in 10 s -- is the agent up?")
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
