#!/usr/bin/env python3
"""A short exploring drive, measured by odometry: out, a full turn, back, a full turn.

For a sensor that does not see all round (a depth camera, a masked LiDAR): SLAM's
first map may not contain the robot, and nothing -- Nav2 or frontier exploration
-- can plan from outside the map. This drives just enough for SLAM to map the
robot's surroundings.

Measured, not timed: the pipeline used to publish each leg with `ros2 topic pub`
for a fixed time, and the CLI's own ~2 s start-up ate into every leg -- 0.8 m
became ~0.4 m, under slam_toolbox's 0.5 m minimum_travel_distance, so SLAM
added nothing and the map never reached the robot (measured 2026-09-26, lyrical,
camera only, rooms world).

    explore_nudge.py [--distance 0.8] [--stamped] [--speed 0.15] [--yaw-rate 0.4]
                     [--guard M --radius R]
exit 0 when the whole pattern was driven, 1 when a leg timed out.

--guard (a real robot, whose room is not the simulation's): the LiDAR watches the
corridor the robot's width along each straight leg, and the leg ends early when the
first return in it is M metres from the robot's edge; a spin does not move it.
"""
import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--distance", type=float, default=0.8, help="metres out, and back")
    ap.add_argument("--speed", type=float, default=0.15)
    ap.add_argument("--yaw-rate", type=float, default=0.4)
    ap.add_argument("--stamped", action="store_true", help="publish TwistStamped (lyrical)")
    ap.add_argument("--leg-timeout", type=float, default=40.0)
    ap.add_argument("--guard", type=float, default=None,
                    help="a real robot: end a straight leg this far (m) short of the first return in its path")
    ap.add_argument("--radius", type=float, default=0.20, help="the robot's radius (m), for --guard")
    args = ap.parse_args()

    rclpy.init()
    node = Node("explore_nudge")
    pub = node.create_publisher(TwistStamped if args.stamped else Twist, "cmd_vel", 10)
    st = {"x": None, "y": None, "yaw": None}

    def on_odom(m):
        q = m.pose.pose.orientation
        st["x"], st["y"] = m.pose.pose.position.x, m.pose.pose.position.y
        st["yaw"] = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    node.create_subscription(Odometry, "odom", on_odom, 10)

    scan = {"m": None, "laser": None}
    if args.guard is not None:
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import LaserScan
        import scan_match
        node.create_subscription(LaserScan, "scan", lambda m: scan.__setitem__("m", m),
                                 qos_profile_sensor_data)

    def clear_ahead(direction):
        """Free distance (m) from the robot's edge to the first return in its corridor."""
        if scan["m"] is None:
            return 0.0                      # no scan, no room: a guarded leg does not move
        if scan["laser"] is None:
            scan["laser"] = scan_match.laser_pose(node, scan["m"].header.frame_id, tries=10) \
                or (0.0, 0.0, 0.0)
        pts = scan_match.scan_points(scan_match.scan_dict(scan["m"]), laser=scan["laser"])
        ahead = [px * direction for px, py in pts if px * direction > 0 and abs(py) <= args.radius]
        return (min(ahead) - args.radius) if ahead else float("inf")

    def send(vx, wz):
        t = Twist()
        t.linear.x, t.angular.z = vx, wz
        if args.stamped:
            s = TwistStamped()
            s.header.stamp = node.get_clock().now().to_msg()
            s.header.frame_id = "base_link"
            s.twist = t
            pub.publish(s)
        else:
            pub.publish(t)

    end = time.time() + 10
    while st["x"] is None and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    if st["x"] is None:
        print("❌ NUDGE: no /odom")
        return 1

    def leg(vx, wz, done):
        x0, y0, last = st["x"], st["y"], st["yaw"]
        turned, t0 = 0.0, time.time()
        while time.time() - t0 < args.leg_timeout:
            rclpy.spin_once(node, timeout_sec=0.05)
            turned += abs(math.remainder(st["yaw"] - last, 2 * math.pi))
            last = st["yaw"]
            if done(math.hypot(st["x"] - x0, st["y"] - y0), turned):
                send(0.0, 0.0)
                return True
            if args.guard is not None and vx and clear_ahead(1.0 if vx > 0 else -1.0) <= args.guard:
                send(0.0, 0.0)
                print(f"  NUDGE: {math.hypot(st['x'] - x0, st['y'] - y0):.2f} m and the path is "
                      f"closing ({args.guard:.2f} m guard): this leg ends here")
                return True
            send(vx, wz)
        send(0.0, 0.0)
        return False

    ok = True
    for name, vx, wz, done in (
            ("out", args.speed, 0.0, lambda d, a: d >= args.distance),
            ("turn", 0.0, args.yaw_rate, lambda d, a: a >= 2 * math.pi),
            ("back", -args.speed, 0.0, lambda d, a: d >= args.distance),
            ("turn", 0.0, args.yaw_rate, lambda d, a: a >= 2 * math.pi)):
        if not leg(vx, wz, done):
            print(f"  ⚠️ NUDGE: the '{name}' leg timed out after {args.leg_timeout:.0f} s")
            ok = False
            break
        for _ in range(10):           # let the base settle between legs
            send(0.0, 0.0)
            rclpy.spin_once(node, timeout_sec=0.05)
    node.destroy_node()
    rclpy.shutdown()
    print("  ✅ NUDGE: out, turn, back, turn -- driven by odometry" if ok else "❌ NUDGE: incomplete")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
