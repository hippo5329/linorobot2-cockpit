#!/usr/bin/env python3
# Copyright (c) 2026 Linorobot contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Stop a real robot's Nav2 motion when its eyes or its room give out.

    motion_guard.py --params CFG --trip-file F --ready-file R [--ns NS] [--room-bound]

The 1-Click runs this beside a real robot's exploration and goal legs. It watches:

  scan      /scan must keep coming at the LiDAR model's own rate (lidar_check.MODEL_HZ),
            at least SCAN_RATE_SHARE of it over the last SCAN_WINDOW_S, and never be older
            than SCAN_STALE_S. A tracked robot at 8 % charge gave 6.9 Hz scans of 456 beams
            from the start, then none at all while exploring; the robot was at a wall by then.
  battery   /battery's percentage (a 0..1 fraction; NaN = not measured) at least
            --battery-min-start before anything moves and --battery-min after.
  room      with --room-bound: the outline of the room as the LiDAR saw it at the start
            (the convex hull of the returns, in the map frame). A Nav2 plan ending outside
            it, less --room-margin, is a goal through a gap the robot cannot pass -- that
            same robot's explorer chose a frontier beyond the garage wall and pushed it into
            the wall. The robot itself leaving the outline means the map no longer holds it.

Before motion: the scan and battery are judged for READY_WINDOW_S; a fault then writes the
trip file and nothing is told it may move. During: a fault writes the trip file and cancels
every navigate_to_pose goal, again every CANCEL_EVERY_S while it lasts (the explorer sends a
new goal at once; it blacklists a frontier only when the robot has made no progress for its
progress_timeout). An out-of-room plan cancels that goal and is reported, not a trip: the
explorer then gives that frontier up.

The trip file holds one line, "MOTION_GUARD_TRIPPED: <why>"; the ready file "MOTION_GUARD_READY".
"""
import argparse
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

SCAN_RATE_SHARE = 0.85     # of the model's rate
SCAN_WINDOW_S = 3.0
SCAN_STALE_S = 1.0
READY_WINDOW_S = 4.0
CANCEL_EVERY_S = 0.5
OUTLINE_SCANS = 10


# ---- the judgements, kept free of ROS so they can be tested -----------------------------
def scan_rate(stamps: list, now: float, window: float = SCAN_WINDOW_S) -> float:
    """Scans per second over the last `window` seconds of arrival times."""
    recent = [t for t in stamps if now - t <= window]
    if len(recent) < 2:
        return 0.0
    return (len(recent) - 1) / max(recent[-1] - recent[0], 1e-6)


def scan_fault(stamps: list, now: float, want_hz: float, started: float) -> str:
    """Why the scan is not good enough to move on, or ''."""
    if not stamps:
        return "no /scan at all" if now - started > SCAN_STALE_S else ""
    age = now - stamps[-1]
    if age > SCAN_STALE_S:
        return f"/scan stopped: the last one is {age:.1f} s old"
    if want_hz and now - started >= SCAN_WINDOW_S:
        hz = scan_rate(stamps, now)
        floor = SCAN_RATE_SHARE * want_hz
        if hz < floor:
            return (f"/scan at {hz:.1f} Hz, under {floor:.1f} Hz ({SCAN_RATE_SHARE:.0%} of the "
                    f"model's {want_hz:.0f} Hz): the LiDAR is short of power or its link of room")
    return ""


def battery_fault(fraction, minimum: float) -> str:
    if fraction is None or fraction != fraction or minimum <= 0:
        return ""
    if fraction < minimum:
        return f"battery at {fraction * 100:.0f} %, under {minimum * 100:.0f} %"
    return ""


def hull(points: list) -> list:
    """Counter-clockwise convex hull (Andrew's monotone chain)."""
    pts = sorted(set((round(x, 3), round(y, 3)) for x, y in points))
    if len(pts) < 3:
        return pts

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    lo, hi = [], []
    for p in pts:
        while len(lo) >= 2 and cross(lo[-2], lo[-1], p) <= 0:
            lo.pop()
        lo.append(p)
    for p in reversed(pts):
        while len(hi) >= 2 and cross(hi[-2], hi[-1], p) <= 0:
            hi.pop()
        hi.append(p)
    return lo[:-1] + hi[:-1]


def inside_by(poly: list, x: float, y: float) -> float:
    """How far (x, y) lies inside the counter-clockwise polygon: the least distance to an
    edge's line, negative when outside any edge."""
    if len(poly) < 3:
        return float("inf")
    least = float("inf")
    for (x1, y1), (x2, y2) in zip(poly, poly[1:] + poly[:1]):
        ex, ey = x2 - x1, y2 - y1
        length = math.hypot(ex, ey)
        if length < 1e-9:
            continue
        least = min(least, (ex * (y - y1) - ey * (x - x1)) / length)
    return least


def plan_outside(poly: list, end: tuple, margin: float) -> str:
    d = inside_by(poly, *end)
    if d < margin:
        where = "beyond the room's outline" if d < 0 else f"{d:.2f} m inside the room's outline"
        return (f"plan to ({end[0]:+.2f}, {end[1]:+.2f}) ends {where}, needs {margin:.2f} m: "
                f"a frontier through a gap, not a room")
    return ""


def want_hz(params: dict) -> float:
    import lidar_check
    return lidar_check.expected_hz(params)


# ---- ROS ----------------------------------------------------------------------------------
def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--params", required=True)
    ap.add_argument("--trip-file", required=True)
    ap.add_argument("--ready-file", required=True)
    ap.add_argument("--ns", default="")
    ap.add_argument("--room-bound", action="store_true")
    ap.add_argument("--room-margin", type=float, default=0.30)
    ap.add_argument("--battery-min-start", type=float, default=0.10)
    ap.add_argument("--battery-min", type=float, default=0.05)
    a = ap.parse_args(argv)

    import yaml
    import rclpy
    import rclpy.time
    from action_msgs.srv import CancelGoal
    from nav_msgs.msg import Path
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import BatteryState, LaserScan
    import tf2_ros

    with open(a.params) as fh:
        params = yaml.safe_load(fh) or {}
    hz = want_hz(params)
    prefix = f"{a.ns}/" if a.ns else ""
    map_frame, base_frame = f"{prefix}map", f"{prefix}base_link"

    rclpy.init()
    node = Node("motion_guard", namespace=a.ns or None)
    tf_buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(tf_buf, node)
    st = {"stamps": [], "battery": None, "outline_pts": [], "outline": [], "scans_in": 0}

    def on_scan(m):
        st["stamps"].append(time.time())
        st["stamps"] = st["stamps"][-200:]
        if a.room_bound and st["scans_in"] < OUTLINE_SCANS:
            try:
                tf = tf_buf.lookup_transform(map_frame, m.header.frame_id, rclpy.time.Time())
            except Exception:
                return
            t, q = tf.transform.translation, tf.transform.rotation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            c, s = math.cos(yaw), math.sin(yaw)
            for i, r in enumerate(m.ranges):
                if m.range_min < r < m.range_max and math.isfinite(r):
                    ang = m.angle_min + i * m.angle_increment
                    lx, ly = r * math.cos(ang), r * math.sin(ang)
                    st["outline_pts"].append((t.x + c * lx - s * ly, t.y + s * lx + c * ly))
            st["scans_in"] += 1
            if st["scans_in"] == OUTLINE_SCANS:
                st["outline"] = hull(st["outline_pts"])

    def on_battery(m):
        st["battery"] = m.percentage

    canceller = node.create_client(CancelGoal, "navigate_to_pose/_action/cancel_goal")

    def cancel_all():
        if canceller.service_is_ready():
            canceller.call_async(CancelGoal.Request())   # zero goal id, zero stamp: every goal

    def on_plan(m):
        if not (a.room_bound and st["outline"] and m.poses):
            return
        end = m.poses[-1].pose.position
        why = plan_outside(st["outline"], (end.x, end.y), a.room_margin)
        if why:
            print(f"MOTION_GUARD goal cancelled: {why}", flush=True)
            cancel_all()

    node.create_subscription(LaserScan, "scan", on_scan, qos_profile_sensor_data)
    node.create_subscription(BatteryState, "battery", on_battery, 10)
    node.create_subscription(Path, "plan", on_plan, 10)

    def trip(why: str):
        with open(a.trip_file, "w") as fh:
            fh.write(f"MOTION_GUARD_TRIPPED: {why}\n")
        print(f"MOTION_GUARD_TRIPPED: {why}", flush=True)

    started = time.time()
    print(f"MOTION_GUARD watching: /scan >= {SCAN_RATE_SHARE * hz:.1f} Hz (model {hz:.0f}), "
          f"battery >= {a.battery_min_start:.0%} to start / {a.battery_min:.0%} after"
          + (f", room outline (margin {a.room_margin:.2f} m)" if a.room_bound else ""), flush=True)
    while time.time() - started < READY_WINDOW_S:
        rclpy.spin_once(node, timeout_sec=0.1)
    why = (scan_fault(st["stamps"], time.time(), hz, started)
           or battery_fault(st["battery"], a.battery_min_start))
    if why:
        trip(why)
        node.destroy_node()
        rclpy.shutdown()
        return 1
    with open(a.ready_file, "w") as fh:
        fh.write("MOTION_GUARD_READY\n")
    print("MOTION_GUARD_READY", flush=True)

    tripped, last_cancel = "", 0.0
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            now = time.time()
            if not tripped:
                tripped = (scan_fault(st["stamps"], now, hz, started)
                           or battery_fault(st["battery"], a.battery_min))
                if not tripped and a.room_bound and st["outline"]:
                    try:
                        tf = tf_buf.lookup_transform(map_frame, base_frame, rclpy.time.Time())
                        p = tf.transform.translation
                        if inside_by(st["outline"], p.x, p.y) < 0:
                            tripped = (f"the robot at ({p.x:+.2f}, {p.y:+.2f}) is outside the room's "
                                       f"outline: the map no longer holds it")
                    except Exception:
                        pass
                if tripped:
                    trip(tripped)
            if tripped and now - last_cancel >= CANCEL_EVERY_S:
                cancel_all()
                last_cancel = now
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 1 if tripped else 0


if __name__ == "__main__":
    sys.exit(main())
