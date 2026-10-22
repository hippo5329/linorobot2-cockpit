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
With --guard the pattern starts with a look: a full turn on the spot that measures
the free corridor at every heading, then "out" goes the clearest way and the robot
faces its start heading again at the end. Straight ahead only, a box 1.22 m in front
cut "out" to 0.00 m: two turns on one spot gave camera-only SLAM nothing to place,
map->odom drifted 0.6 m and Nav2's first goal aborted (Yahboom, 2026-10-22).
"""
import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node

BACK_SLACK = 0.01   # m: "back" within this of the distance "out" drove is home
SLAM_TRAVEL = 0.5   # m: slam_toolbox's minimum_travel_distance -- less adds no scan


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
            scan["laser"] = scan_match.laser_pose(node, scan["m"].header.frame_id) \
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

    driven = {}

    def leg(name, vx, wz, done):
        x0, y0, last = st["x"], st["y"], st["yaw"]
        turned, t0 = 0.0, time.time()

        def gone():
            driven[name] = math.hypot(st["x"] - x0, st["y"] - y0)
            return driven[name]

        while time.time() - t0 < args.leg_timeout:
            rclpy.spin_once(node, timeout_sec=0.05)
            turned += abs(math.remainder(st["yaw"] - last, 2 * math.pi))
            last = st["yaw"]
            if done(gone(), turned):
                send(0.0, 0.0)
                return True
            if args.guard is not None and vx and clear_ahead(1.0 if vx > 0 else -1.0) <= args.guard:
                send(0.0, 0.0)
                print(f"  NUDGE: {gone():.2f} m and the path is "
                      f"closing ({args.guard:.2f} m guard): this leg ends here")
                return True
            send(vx, wz)
        send(0.0, 0.0)
        return False

    # "back" undoes what "out" DROVE, not what it was asked for: with "out" cut short by
    # the guard (a wall 1.14 m ahead, 0.00 m driven), reversing the full distance left an
    # MC200 0.82 m behind home, and Nav2's first goal began vacuous (2026-10-20).
    def settle():
        for _ in range(10):           # let the base settle between legs
            send(0.0, 0.0)
            rclpy.spin_once(node, timeout_sec=0.05)

    def look():
        """A full turn on the spot; the heading (odom yaw) with the most free corridor.
        The start heading wins whenever it has room for the whole "out" leg."""
        want = args.distance + args.guard
        here = clear_ahead(1.0)
        if here >= want:
            return st["yaw"], here, False
        best, last, turned, t0 = (here, st["yaw"]), st["yaw"], 0.0, time.time()
        while turned < 2 * math.pi and time.time() - t0 < args.leg_timeout:
            send(0.0, args.yaw_rate)
            rclpy.spin_once(node, timeout_sec=0.05)
            turned += abs(math.remainder(st["yaw"] - last, 2 * math.pi))
            last = st["yaw"]
            c = clear_ahead(1.0)
            if c > best[0] + 0.05:
                best = (c, st["yaw"])
        send(0.0, 0.0)
        return best[1], best[0], True

    def face(name, target):
        delta = math.remainder(target - st["yaw"], 2 * math.pi)
        if abs(delta) < math.radians(2):
            return True
        return leg(name, 0.0, math.copysign(args.yaw_rate, delta), lambda d, a: a >= abs(delta))

    start_yaw = st["yaw"]
    plan = [("out", args.speed, 0.0, lambda d, a: d >= args.distance),
            ("turn", 0.0, args.yaw_rate, lambda d, a: a >= 2 * math.pi),
            ("back", -args.speed, 0.0, lambda d, a: d >= driven.get("out", 0.0) - BACK_SLACK),
            ("turn", 0.0, args.yaw_rate, lambda d, a: a >= 2 * math.pi)]
    ok = True
    if args.guard is not None:
        heading, room, spun = look()
        settle()
        turn_by = math.degrees(math.remainder(heading - start_yaw, 2 * math.pi))
        print(f"  NUDGE: look -- the clearest way is {turn_by:+.0f} deg from the start heading "
              f"({'open' if math.isinf(room) else f'{room:.2f} m free'})")
        ok = face("face out", heading)
        if not ok:
            print(f"  ⚠️ NUDGE: the 'face out' turn timed out after {args.leg_timeout:.0f} s")
        settle()
        # the look was the first full turn, when it spun; the end faces the start heading again
        plan = plan[:3] + ([] if spun else plan[3:]) + [("face home", None, None, None)]
    for name, vx, wz, done in (plan if ok else []):
        if name == "face home":
            good = face(name, start_yaw)
        else:
            good = leg(name, vx, wz, done)
        if not good:
            print(f"  ⚠️ NUDGE: the '{name}' leg timed out after {args.leg_timeout:.0f} s")
            ok = False
            break
        settle()
    node.destroy_node()
    rclpy.shutdown()
    if ok and driven.get("out", 0.0) < SLAM_TRAVEL:
        print(f"  ⚠️ NUDGE: only {driven.get('out', 0.0):.2f} m out (SLAM adds a scan every "
              f"{SLAM_TRAVEL:.1f} m): the map may not grow round the robot")
    elif ok:
        print("  ✅ NUDGE: out, turn, back, turn -- driven by odometry")
    else:
        print("❌ NUDGE: incomplete")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
