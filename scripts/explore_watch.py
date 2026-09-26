#!/usr/bin/env python3
"""Follow a frontier exploration to its end and say how much it mapped.

Listens to explore_lite's explore/status (transient local) and /map. Success
is explore_lite saying the robot is back where it started (returned_to_origin)
after exploration_complete -- or complete, when return_to_init is off. The map's
known area (free + occupied cells) is printed as it grows, so a stalled
exploration is visible as a number that stops moving.

exit 0: explored and back;  1: timed out;  2: exploration never reported starting;
     3: back, but the map is smaller than --min-area. "Complete" is explore_lite's
     word that it found no more frontiers, not proof it mapped the space: a camera
     run declared complete after 8 s with 5.7 m2 of a 60 m2 world and went home.
"""
import argparse
import sys
import time

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

try:
    from explore_lite_msgs.msg import ExploreStatus
except ImportError:            # the image carries it; a dev machine may not
    ExploreStatus = None


def known_area_m2(grid) -> float:
    res = grid.info.resolution
    return sum(1 for v in grid.data if v >= 0) * res * res


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--timeout", type=float, default=900.0, help="seconds for the whole exploration")
    ap.add_argument("--no-return", action="store_true", help="success at exploration_complete")
    ap.add_argument("--min-area", type=float, default=0.0,
                    help="m² the map must know by the end (0 = no check); the world's size, less its walls")
    args = ap.parse_args()
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

    t0 = time.time()
    last_print = 0.0
    printed = 0
    goal = "exploration_complete" if args.no_return else "returned_to_origin"
    ok = False
    while time.time() - t0 < args.timeout:
        rclpy.spin_once(node, timeout_sec=0.5)
        for s in state["status"][printed:]:
            print(f"  [explore] {time.time() - t0:5.0f} s  {s}  (known area {state['area']:.1f} m²)", flush=True)
        printed = len(state["status"])
        if goal in state["status"]:
            ok = True
            break
        if time.time() - last_print > 30:
            print(f"  [explore] {time.time() - t0:5.0f} s  known area {state['area']:.1f} m²", flush=True)
            last_print = time.time()
    node.destroy_node()
    rclpy.shutdown()
    elapsed = time.time() - t0
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


if __name__ == "__main__":
    sys.exit(main())
