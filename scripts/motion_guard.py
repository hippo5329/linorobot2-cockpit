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
            at least SCAN_RATE_SHARE of it over the last SCAN_WINDOW_S (the same floor
            bring-up's lidar_check judges by), and never be older than scan_stale_s(). A
            model with no known rate is held to the rate it gave while the guard got ready.
            A tracked robot at 8 % charge gave 6.9 Hz scans of 456 beams from the start, then
            none at all while exploring; the robot was at a wall by then.
  battery   /battery, as a 0..1 fraction: its percentage, or -- when that is NaN (some boards
            always, and the firmware's own curve below min_v) -- its voltage between the
            config's battery min_v and max_v. The median over BATTERY_WINDOW_S (a motor's
            sag is not the pack) at least --battery-min-start before anything moves and
            --battery-min after. Neither a percentage nor min_v/max_v: not judged, and the
            guard says so.
  room      with --room-bound: the outline of the room as the LiDAR saw it at the start
            (the convex hull of the returns, in the map frame). A Nav2 plan ending outside
            it, less --room-margin, is a goal through a gap the robot cannot pass -- that
            same robot's explorer chose a frontier beyond the garage wall and pushed it into
            the wall. The robot itself leaving the outline means the map no longer holds it.

Before motion: the scan and battery are judged for READY_WINDOW_S; a fault then writes the
trip file and nothing is told it may move. During: a fault writes the trip file and cancels
every navigate_to_pose goal, at once again whenever the action's status shows a goal accepted
or running (the explorer sends a new goal at once; it blacklists a frontier only when the
robot has made no progress for its progress_timeout), and every CANCEL_EVERY_S as a backstop. An out-of-room plan cancels that goal and is reported, not a trip: the
explorer then gives that frontier up.

The trip file holds one line, "MOTION_GUARD_TRIPPED: <why>"; the ready file "MOTION_GUARD_READY".
"""
import argparse
import json
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import lidar_check  # noqa: E402  (no ROS: the floor bring-up judges by)

# One floor, shared with bring-up's lidar_check (its comment says why 80 % over 5 s).
SCAN_RATE_SHARE = lidar_check.RATE_FLOOR_SHARE     # of the model's rate
SCAN_WINDOW_S = lidar_check.RATE_WINDOW_S
SCAN_STALE_S = 1.0
# A LiDAR forwarded by the board over Wi-Fi (lidar.comm_mode udp) arrives in bursts: the
# GenDrv's link has measured gaps of 0.5-2 s with the rate over 5 s still whole. One such gap is
# the link, not a dead LiDAR; a link that gaps often fails the rate floor anyway.
SCAN_STALE_FORWARDED_S = 2.0
READY_WINDOW_S = 6.0       # at least SCAN_WINDOW_S: the rate is judged only over a full window
CANCEL_EVERY_S = 0.5
OUTLINE_SCANS = 10
# The floor the robot stood on while the outline was taken is room, whatever the LiDAR
# saw: a disc this much wider than --room-margin goes into the hull at each of those
# poses. A masked sector (the robot's own body) leaves no returns behind it, and the
# hull's back edge then ran 0.26 m behind the start: the MC200 (118-263 deg masked)
# had its goal home cancelled as "a frontier through a gap" (2026-10-17).
ROBOT_SPOT_M = 0.25
BATTERY_WINDOW_S = 3.0     # the battery is judged on the median of this long, not the last sample
# action_msgs/GoalStatus: a goal the action server has taken and may be driving
GOAL_ACCEPTED, GOAL_EXECUTING = 1, 2


# ---- the judgements, kept free of ROS so they can be tested -----------------------------
def scan_rate(stamps: list, now: float, window: float = SCAN_WINDOW_S) -> float:
    """Scans per second over the last `window` seconds of arrival times."""
    return lidar_check.window_rate(stamps, now, window)


def scan_stale_s(hz: float, forwarded: bool = False) -> float:
    """How old the last scan may be: three of the LiDAR's own periods (a 5 Hz LiDAR's normal
    0.2 s gap plus jitter tripped nothing at 1 s, but a 2 Hz one would), never under 1 s --
    or SCAN_STALE_FORWARDED_S for a LiDAR the board forwards over Wi-Fi."""
    floor = SCAN_STALE_FORWARDED_S if forwarded else SCAN_STALE_S
    return max(floor, 3.0 / hz) if hz > 0 else floor


def scan_fault(stamps: list, now: float, want_hz: float, started: float, stale_s: float = None,
               model_hz: float = None) -> str:
    """Why the scan is not good enough to move on, or ''. `want_hz`: the rate the floor is a
    share of -- the model's, or the one measured while getting ready for a model with none
    (`model_hz` 0 says which, for the message)."""
    stale_s = SCAN_STALE_S if stale_s is None else stale_s
    if not stamps:
        return "no /scan at all" if now - started > stale_s else ""
    age = now - stamps[-1]
    if age > stale_s:
        return f"/scan stopped: the last one is {age:.1f} s old"
    if want_hz and now - started >= SCAN_WINDOW_S:
        hz = scan_rate(stamps, now)
        floor = SCAN_RATE_SHARE * want_hz
        if hz < floor:
            whose = ("model's" if model_hz is None or model_hz else "measured at the start,")
            return (f"/scan at {hz:.1f} Hz, under {floor:.1f} Hz ({SCAN_RATE_SHARE:.0%} of the "
                    f"{whose} {want_hz:.0f} Hz): the LiDAR is short of power or its link of room")
    return ""


def judged_rate(model_hz: float, stamps: list, now: float) -> float:
    """The rate the floor is a share of: the model's, or -- for a model lidar_check.MODEL_HZ
    does not know, which left no floor at all -- the rate the LiDAR gave while the guard got
    ready (READY_WINDOW_S, a whole SCAN_WINDOW_S)."""
    return model_hz if model_hz else scan_rate(stamps, now)


def battery_fraction(percentage, voltage, vmin: float = 0.0, vmax: float = 0.0):
    """The pack as a 0..1 fraction, or None when it cannot be told: the percentage when it is
    a number, else the voltage on the firmware's own curve (battery.cpp sigmoidal) between the
    config's min_v and max_v -- which returns NaN itself below min_v, so the percentage is
    missing exactly when the pack is lowest. Clamped to 0..1."""
    if percentage is not None and math.isfinite(percentage):
        return min(1.0, max(0.0, float(percentage)))
    if voltage is None or not math.isfinite(voltage) or voltage <= 0 or not vmax > vmin > 0:
        return None
    x = (float(voltage) - vmin) / (vmax - vmin)
    if x <= 0:
        return 0.0
    pct = 105.0 - 105.0 / (1.0 + (1.724 * x) ** 5.5)
    return min(1.0, max(0.0, pct / 100.0))


def battery_limits(params: dict):
    """(min_v, max_v) from base_controller.pins.battery, or (0, 0) when the config has none."""
    bat = (((params or {}).get("base_controller") or {}).get("pins") or {}).get("battery")
    if not isinstance(bat, dict):
        return 0.0, 0.0
    try:
        vmin, vmax = float(bat.get("min_v") or 0.0), float(bat.get("max_v") or 0.0)
    except (TypeError, ValueError):
        return 0.0, 0.0
    return (vmin, vmax) if vmax > vmin > 0 else (0.0, 0.0)


class Battery:
    """The pack's samples as they arrive; judged on their median over BATTERY_WINDOW_S: a
    robot accelerating sags its pack for a moment, and the latest sample alone stopped it."""

    def __init__(self, vmin: float = 0.0, vmax: float = 0.0, window: float = BATTERY_WINDOW_S):
        self.vmin, self.vmax, self.window = vmin, vmax, window
        self.samples = []          # (arrival, fraction or None)

    def add(self, now: float, percentage, voltage):
        self.samples.append((now, battery_fraction(percentage, voltage, self.vmin, self.vmax)))
        self.samples = [s for s in self.samples if now - s[0] <= 2 * self.window]

    def fraction(self, now: float):
        vals = sorted(f for t, f in self.samples if now - t <= self.window and f is not None)
        if not vals:
            return None
        n = len(vals)
        return vals[n // 2] if n % 2 else 0.5 * (vals[n // 2 - 1] + vals[n // 2])

    def judgeable(self) -> bool:
        """Whether the config lets a NaN percentage be judged at all (by the voltage)."""
        return self.vmax > self.vmin > 0


def battery_fault(fraction, minimum: float) -> str:
    if fraction is None or fraction != fraction or minimum <= 0:
        return ""
    if fraction < minimum:
        return f"battery at {fraction * 100:.0f} %, under {minimum * 100:.0f} %"
    return ""


def goal_running(status_codes) -> bool:
    """True when the navigate_to_pose status lists a goal accepted or executing: after a trip,
    one more goal to cancel at once."""
    return any(c in (GOAL_ACCEPTED, GOAL_EXECUTING) for c in status_codes)


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


def robot_spot(x: float, y: float, radius: float, n: int = 16) -> list:
    """Points on a circle of `radius` round (x, y): the floor the robot occupies, for the hull."""
    return [(x + radius * math.cos(2 * math.pi * k / n), y + radius * math.sin(2 * math.pi * k / n))
            for k in range(n)]


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
    # Every goal refused for the room's outline, one JSON line each: explore_lite resends a
    # cancelled frontier (a cancel does not blacklist it), so the exploration watcher reads
    # this to tell "the room is explored, what is left is beyond it" from a stall.
    ap.add_argument("--fence-file", default="")
    ap.add_argument("--battery-min-start", type=float, default=0.10)
    ap.add_argument("--battery-min", type=float, default=0.05)
    a = ap.parse_args(argv)

    import yaml
    import rclpy
    import rclpy.time
    from action_msgs.msg import GoalStatusArray
    from action_msgs.srv import CancelGoal
    from nav_msgs.msg import Path
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from sensor_msgs.msg import BatteryState, LaserScan
    import tf2_ros

    with open(a.params) as fh:
        params = yaml.safe_load(fh) or {}
    model_hz = want_hz(params)
    hz = model_hz
    stale_s = scan_stale_s(model_hz, lidar_check.forwarded(params))
    battery = Battery(*battery_limits(params))
    prefix = f"{a.ns}/" if a.ns else ""
    map_frame, base_frame = f"{prefix}map", f"{prefix}base_link"

    rclpy.init()
    node = Node("motion_guard", namespace=a.ns or None)
    tf_buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(tf_buf, node)
    st = {"stamps": [], "outline_pts": [], "outline": [], "scans_in": 0, "tripped": ""}

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
            st["outline_pts"].extend(robot_spot(t.x, t.y, a.room_margin + ROBOT_SPOT_M))
            st["scans_in"] += 1
            if st["scans_in"] == OUTLINE_SCANS:
                st["outline"] = hull(st["outline_pts"])

    def on_battery(m):
        battery.add(time.time(), m.percentage, m.voltage)

    canceller = node.create_client(CancelGoal, "navigate_to_pose/_action/cancel_goal")

    def cancel_all():
        if canceller.service_is_ready():
            canceller.call_async(CancelGoal.Request())   # zero goal id, zero stamp: every goal

    def on_status(m):
        # After a trip, a goal the explorer sends is cancelled the moment Nav2 accepts it, not
        # up to CANCEL_EVERY_S later: the status topic changes with every goal's state, so this
        # is one callback behind the acceptance, where faster polling would only call the
        # cancel service more often and still lag. The robot moved in bursts between cancels.
        if st["tripped"] and goal_running(g.status for g in m.status_list):
            cancel_all()

    def on_plan(m):
        if not (a.room_bound and st["outline"] and m.poses):
            return
        end = m.poses[-1].pose.position
        why = plan_outside(st["outline"], (end.x, end.y), a.room_margin)
        if why:
            print(f"MOTION_GUARD goal cancelled: {why}", flush=True)
            cancel_all()
            if a.fence_file:
                try:
                    with open(a.fence_file, "a", encoding="utf-8") as fh:
                        fh.write(json.dumps({"t": round(time.time(), 2), "x": round(end.x, 2),
                                             "y": round(end.y, 2)}) + "\n")
                except OSError:
                    pass

    node.create_subscription(LaserScan, "scan", on_scan, qos_profile_sensor_data)
    # best effort: matches the firmware's best-effort /battery and a reliable one (sim_base_node);
    # a reliable reader never hears a best-effort writer -- the guard judged no battery on MC200
    node.create_subscription(BatteryState, "battery", on_battery, qos_profile_sensor_data)
    node.create_subscription(Path, "plan", on_plan, 10)
    # an action's status is published reliable and transient-local (rcl_action's default)
    node.create_subscription(GoalStatusArray, "navigate_to_pose/_action/status", on_status,
                             QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                                        durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def trip(why: str):
        with open(a.trip_file, "w") as fh:
            fh.write(f"MOTION_GUARD_TRIPPED: {why}\n")
        print(f"MOTION_GUARD_TRIPPED: {why}", flush=True)

    started = time.time()
    scan_says = (f"/scan >= {SCAN_RATE_SHARE * hz:.1f} Hz (model {hz:.0f})" if hz else
                 f"/scan >= {SCAN_RATE_SHARE:.0%} of the rate it gives while getting ready "
                 f"(no rate known for this model)")
    bat_says = (f"battery >= {a.battery_min_start:.0%} to start / {a.battery_min:.0%} after "
                f"(median over {BATTERY_WINDOW_S:.0f} s"
                + (f"; by voltage {battery.vmin:g}-{battery.vmax:g} V when the percentage is NaN)"
                   if battery.judgeable() else "; percentage only: no battery min_v/max_v in the config)"))
    print(f"MOTION_GUARD watching: {scan_says}, scan never older than {stale_s:.1f} s, {bat_says}"
          + (f", room outline (margin {a.room_margin:.2f} m)" if a.room_bound else ""), flush=True)
    while time.time() - started < READY_WINDOW_S:
        rclpy.spin_once(node, timeout_sec=0.1)
    now = time.time()
    if not hz:
        hz = judged_rate(model_hz, st["stamps"], now)
        print(f"MOTION_GUARD the LiDAR gives {hz:.1f} Hz while getting ready: the floor is "
              f"{SCAN_RATE_SHARE * hz:.1f} Hz", flush=True)
        stale_s = scan_stale_s(hz, lidar_check.forwarded(params))
    frac = battery.fraction(now)
    if frac is None:
        print("MOTION_GUARD battery: NOT JUDGED -- " + (
            "no /battery reading came" if not battery.samples else
            "its percentage is NaN and the config has no battery min_v/max_v to read the voltage by")
            + "; nothing stops the robot on a low pack", flush=True)
    why = (scan_fault(st["stamps"], now, hz, started, stale_s, model_hz)
           or ("" if hz else "no /scan rate to judge by")
           or battery_fault(frac, a.battery_min_start))
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
                tripped = (scan_fault(st["stamps"], now, hz, started, stale_s, model_hz)
                           or battery_fault(battery.fraction(now), a.battery_min))
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
                    st["tripped"] = tripped
                    trip(tripped)
                    last_cancel = 0.0           # cancel now, not at the next tick
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
