#!/usr/bin/env python3
"""Follow a frontier exploration to its end and say how much it mapped.

Listens to explore_lite's explore/status (transient local) and /map. Success
is explore_lite saying the robot is back where it started (returned_to_origin)
after exploration_complete -- or complete, when return_to_init is off. The map's
known area (free + occupied cells) is printed as it grows, so a stalled
exploration is visible as a number that stops moving.

exit 0: explored and back;  1: timed out;  2: exploration never reported starting;
     3: back, but the map is smaller than --min-area;  4: the room is explored and what is
     left lies beyond it -- the motion guard keeps refusing frontier goals past the room's
     outline (--fence-file) and the map has stopped growing; the caller drives the robot home.
     Without this a real robot with the room bound explored its room in 9 minutes and spent
     the rest of its time limit on one frontier through a doorway, cancelled 129 times
     (MC200, 2026-10-16): explore_lite resends a cancelled goal, it only blacklists a failed one;
     6: the battery reached --stop-volts (the median of its last 10 /battery readings, under load):
     the caller drives the robot home while it still can; 7: the robot stalled -- under 5 cm and
     15 deg of travel on the map in --stall-s -- the caller drives it home. A hexapod pinned
     against a chair leg spent the rest of its run there, and on another run the battery read
     6.45 V on the way home after a stop at 7.0 V (miniHexa, 2026-11-08/09). "Complete" is explore_lite's
     word that it found no more frontiers, not proof it mapped the space: a camera
     run declared complete after 8 s with 5.7 m2 of a 60 m2 world and went home.
"""
import argparse
import json
import math
import sys
import time

FENCE_RECENT_S = 30.0     # a refusal this recent: explore_lite is still asking for the far side
GROWTH_M2 = 0.5           # the map "still growing" by at least this much over --fence-quiet
STALL_M, STALL_DEG = 0.05, 15.0  # less travel than this over --stall-s is a stall


def battery_low(volts, stop) -> bool:
    """The median of the last 10 readings under `stop` (V); a lone dip under a step's load is not."""
    last = sorted(volts[-10:])
    return bool(stop > 0 and len(last) >= 5 and last[(len(last) - 1) // 2] < stop)


def stalled(poses, now, window) -> bool:
    """poses: [(t, x, y, yaw rad)] on the map, oldest first. True when they span `window` s and every
    pose since now - window lies within STALL_M and STALL_DEG of the first of them."""
    if window <= 0 or not poses or now - poses[0][0] < window:
        return False
    recent = [p for p in poses if p[0] >= now - window]
    if not recent:
        return False
    t0, x0, y0, a0 = recent[0]
    for _, x, y, a in recent:
        turn = abs(math.degrees(math.atan2(math.sin(a - a0), math.cos(a - a0))))
        if math.hypot(x - x0, y - y0) >= STALL_M or turn >= STALL_DEG:
            return False
    return True



def known_area_m2(grid) -> float:
    res = grid.info.resolution
    return sum(1 for v in grid.data if v >= 0) * res * res


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--timeout", type=float, default=900.0, help="seconds for the whole exploration")
    ap.add_argument("--no-return", action="store_true", help="success at exploration_complete")
    ap.add_argument("--fence-file", default="", help="the motion guard's refused goals (JSON lines)")
    ap.add_argument("--fence-quiet", type=float, default=60.0,
                    help="s without map growth, refusals still coming: the room is explored")
    ap.add_argument("--stop-volts", type=float, default=0.0,
                    help="stop when the battery's median of 10 readings is under this (V; 0 = no check)")
    ap.add_argument("--stall-s", type=float, default=0.0,
                    help="stop when the robot moved under 5 cm and 15 deg on the map in this many s (0 = off)")
    ap.add_argument("--base-frame", default="base_link", help="the robot's frame, for --stall-s")
    ap.add_argument("--min-area", type=float, default=0.0,
                    help="m² the map must know by the end (0 = no check); the world's size, less its walls")
    args = ap.parse_args()
    # ROS here, not at import: fence_refusals is read (and tested) without a ROS install
    import rclpy
    from nav_msgs.msg import OccupancyGrid
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    try:
        from explore_lite_msgs.msg import ExploreStatus
    except ImportError:            # the image carries it; a dev machine may not
        ExploreStatus = None
    if ExploreStatus is None:
        print("❌ EXPLORE: explore_lite_msgs is not installed")
        return 2

    rclpy.init()
    node = Node("explore_watch")
    state = {"status": [], "area": 0.0}
    latched = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
    node.create_subscription(ExploreStatus, "explore/status",
                             lambda m: state["status"].append(m.status), latched)
    node.create_subscription(OccupancyGrid, "map",
                             lambda m: state.__setitem__("area", known_area_m2(m)), latched)
    volts, poses = [], []
    if args.stop_volts > 0:
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import BatteryState
        node.create_subscription(BatteryState, "battery", lambda m: volts.append(float(m.voltage)),
                                 qos_profile_sensor_data)
    tf = None
    if args.stall_s > 0:
        import tf2_ros
        from rclpy.time import Time
        tf = tf2_ros.Buffer()
        tf2_ros.TransformListener(tf, node)

    t0 = time.time()
    last_print = 0.0
    printed = 0
    goal = "exploration_complete" if args.no_return else "returned_to_origin"
    ok = False
    fenced = 0
    stop = ""                    # "battery" | "stalled"
    growth = []                  # (t, area): the map's known area over the last --fence-quiet s
    while time.time() - t0 < args.timeout:
        rclpy.spin_once(node, timeout_sec=0.5)
        for s in state["status"][printed:]:
            print(f"  [explore] {time.time() - t0:5.0f} s  {s}  (known area {state['area']:.1f} m²)", flush=True)
        printed = len(state["status"])
        if goal in state["status"]:
            ok = True
            break
        now = time.time()
        growth.append((now, state["area"]))
        growth = [g for g in growth if now - g[0] <= args.fence_quiet]
        refused = fence_refusals(args.fence_file)
        if (refused and now - refused[-1] <= FENCE_RECENT_S and now - t0 >= args.fence_quiet
                and growth and now - growth[0][0] >= args.fence_quiet - 1.0
                and state["area"] - growth[0][1] < GROWTH_M2):
            fenced = len(refused)
            break
        if battery_low(volts, args.stop_volts):
            stop = "battery"
            break
        if tf is not None:
            try:
                q = tf.lookup_transform("map", args.base_frame, Time()).transform
                yaw = 2.0 * math.atan2(q.rotation.z, q.rotation.w)
                poses.append((now, q.translation.x, q.translation.y, yaw))
                poses = [p for p in poses if now - p[0] <= args.stall_s + 5.0]
            except Exception:
                pass                 # no map frame yet: no pose, no stall
            if stalled(poses, now, args.stall_s):
                stop = "stalled"
                break
        if time.time() - last_print > 30:
            print(f"  [explore] {time.time() - t0:5.0f} s  known area {state['area']:.1f} m²", flush=True)
            last_print = time.time()
    node.destroy_node()
    rclpy.shutdown()
    elapsed = time.time() - t0
    if stop == "battery":
        last = sorted(volts[-10:])
        print(f"⚠️ EXPLORE: stopped at the battery floor after {elapsed:.0f} s -- median "
              f"{last[(len(last) - 1) // 2]:.2f} V under {args.stop_volts:.2f} V; known area {state['area']:.1f} m²")
        return 6
    if stop == "stalled":
        print(f"❌ EXPLORE: stalled after {elapsed:.0f} s -- under {STALL_M * 100:.0f} cm and {STALL_DEG:.0f} deg "
              f"of travel in {args.stall_s:.0f} s; known area {state['area']:.1f} m²")
        return 7
    if fenced:
        if state["area"] < args.min_area:
            print(f"❌ EXPLORE: the room is explored after {elapsed:.0f} s, but known area "
                  f"{state['area']:.1f} m² is under the {args.min_area:.0f} m² this world holds")
            return 3
        print(f"✅ EXPLORE: room_explored after {elapsed:.0f} s -- {fenced} frontier goal(s) beyond "
              f"the room's outline refused, no new map in {args.fence_quiet:.0f} s; known area "
              f"{state['area']:.1f} m²")
        return 4
    if ok and state["area"] < args.min_area:
        print(f"❌ EXPLORE: {goal} after {elapsed:.0f} s, but known area {state['area']:.1f} m² "
              f"is under the {args.min_area:.0f} m² this world holds")
        return 3
    if ok:
        print(f"✅ EXPLORE: {goal} after {elapsed:.0f} s; known area {state['area']:.1f} m²")
        return 0
    if not state["status"]:
        print(f"❌ EXPLORE: explore_lite never reported starting within {elapsed:.0f} s")
        return 2
    print(f"❌ EXPLORE: no {goal} within {args.timeout:.0f} s (last: {state['status'][-1]}); "
          f"known area {state['area']:.1f} m²")
    return 1


def fence_refusals(path: str) -> list:
    """The times of the motion guard's refused goals, oldest first."""
    if not path:
        return []
    out = []
    try:
        with open(path, encoding="utf-8") as fh:
            for ln in fh:
                try:
                    out.append(float(json.loads(ln)["t"]))
                except (ValueError, KeyError, TypeError):
                    pass
    except OSError:
        pass
    return out


if __name__ == "__main__":
    sys.exit(main())
