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
"""Does the LiDAR see the robot itself where Nav2's collision monitor looks?

    self_returns.py --params ROBOT_config.yaml [--scans 10] [--topic /scan]

Nothing moves. Run it with Nav2 up, under the stack's ROS environment (the 1-Click does).

Nav2's collision monitor (nav2.collision_monitor) counts the returns of its `scan` source
inside the robot's footprint -- the local costmap's published footprint for the
FootprintApproach polygon -- and with min_points (6) or more of them it sees a collision in
progress and holds every command at zero, for as long as they are there. A LiDAR that sees
its own mount or the robot's body just past the config's lidar.mask.sectors is exactly
that: on a mecanum robot 21 returns at 0.03-0.05 m, bearings 226-243 deg, sat outside a
[130, 225] mask, Nav2 came up active, and the robot never moved (cmd_vel_smoothed 0.03 m/s,
/cmd_vel 0.00, 180 s a leg). Nothing said why: clearance.py drops returns that near as the
robot's own body, and the collision monitor does not.

So this reads the monitor's own topic (the masked /scan), keeps the returns inside the
footprint radius that persist in most of --scans scans (a single stray return is not the
robot), and refuses when they reach min_points: the count, the bearings in lidar.mask's
convention (robot frame, degrees, 0 ahead, counter-clockwise), the ranges, and the sector
to add. Prints SELF_RETURNS OK or SELF_RETURNS REFUSED: ...; exit 0, 1, or 2 with no scan.

The pure part (self_return_arcs, collision_settings, the messages) has no ROS: lidar_check.py
uses it at bring-up on the whole-circle scan, with the config's mask applied the way the
scan filter applies it.
"""
import argparse
import math
import os
import statistics
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import lidar_mask  # noqa: E402

DEFAULT_MIN_POINTS = 6      # nav2_collision_monitor's polygon default, and the launcher's
DEFAULT_RADIUS_M = 0.10     # nav2_costmap_2d's robot_radius default
PAD_DEG = 5.0               # a suggested sector reaches this far past the returns
HOLDING_ACTIONS = ("stop", "approach")   # the polygon actions that zero the command


def _rp(block):
    """A Nav2 node's parameters from `<node>: {ros__parameters: ...}` or `<node>: <node>: ...`."""
    block = block or {}
    return block.get("ros__parameters", block) or {}


def collision_settings(params: dict) -> dict:
    """{'topic', 'min_points', 'radius'} of the collision monitor's scan source and the polygon
    that would hold the robot, from the robot config's nav2 block -- with the launcher's
    defaults (nav2.launch.py) where the config sets none. 'topic' is None when no scan source
    or no stopping polygon is configured: then nothing here can hold the robot."""
    nav2 = (params or {}).get("nav2") or {}
    cm = _rp(nav2.get("collision_monitor"))
    configured = "observation_sources" in cm
    raw = cm.get("observation_sources") if configured else ["scan"]
    sources = list(raw) if isinstance(raw, (list, tuple)) else str(raw or "").split()
    topic = None
    for s in sources:
        blk = cm.get(s) if configured else {"type": "scan", "topic": "scan"}
        if isinstance(blk, dict) and str(blk.get("type", "")).lower() == "scan" \
                and blk.get("enabled", True) is not False:
            topic = str(blk.get("topic") or s)
            break

    # the local costmap's footprint, which /local_costmap/published_footprint carries: the
    # polygon when one is given (its farthest corner), else the robot_radius circle -- no
    # margin, the monitor uses the footprint as it is
    import gen_firmware_header
    lc = _rp((nav2.get("local_costmap") or {}).get("local_costmap") or nav2.get("local_costmap"))
    cost_r = gen_firmware_header.footprint_radius(lc.get("footprint"))
    if cost_r is None:
        try:
            cost_r = float(lc.get("robot_radius"))
        except (TypeError, ValueError):
            cost_r = DEFAULT_RADIUS_M

    polys = cm.get("polygons") if configured else ["FootprintApproach"]
    polys = list(polys) if isinstance(polys, (list, tuple)) else str(polys or "").split()
    holding = []
    for name in polys:
        p = cm.get(name) if configured else {"action_type": "approach", "min_points": DEFAULT_MIN_POINTS,
                                             "footprint_topic": "/local_costmap/published_footprint"}
        if not isinstance(p, dict) or p.get("enabled", True) is False:
            continue
        if str(p.get("action_type", "")).lower() not in HOLDING_ACTIONS:
            continue
        try:
            mp = int(p.get("min_points", DEFAULT_MIN_POINTS))
        except (TypeError, ValueError):
            mp = DEFAULT_MIN_POINTS
        r = None
        if str(p.get("type", "polygon")).lower() == "circle":
            try:
                r = float(p.get("radius"))
            except (TypeError, ValueError):
                r = None
        elif p.get("points") not in (None, "", "[]"):
            r = gen_firmware_header.footprint_radius(p.get("points"))
        holding.append((max(1, mp), r if r else cost_r))
    if not holding:
        topic = None
    mp, r = min(holding) if holding else (DEFAULT_MIN_POINTS, cost_r)
    return {"topic": topic, "min_points": mp, "radius": r}


def widened_mask(controller: dict) -> list:
    """The config's lidar.mask.sectors as the scan filter applies them: (start, width) in the
    robot frame, each widened by margin_deg on both sides (lidar_mask.filter_chain_params)."""
    m = lidar_mask.mask_config(controller)
    out = []
    for start, width in m["sectors"]:
        wide = min(width + 2.0 * m["margin_deg"], 359.0)
        out.append(((start - (wide - width) / 2.0) % 360.0, wide))
    return out


def _in_box(x, y, boxes) -> bool:
    # A mask box removes a beam ending inside it; a 2D scan lies in the LiDAR's plane, which
    # the boxes we know span in z (min_z -1, max_z 1), so x and y decide.
    return any(b["min_x"] <= x <= b["max_x"] and b["min_y"] <= y <= b["max_y"] for b in boxes)


def inside_returns(scan: dict, radius: float, laser=(0.0, 0.0, 0.0), masked=(), boxes=()) -> list:
    """[(bearing_deg, range_m)] of one scan's returns inside the radius round base_link.

    The bearing is the beam's in lidar.mask's convention (robot frame, degrees, 0 ahead,
    counter-clockwise: the scan angle plus the mount's yaw), so a sector built from it is
    the one the scan filter would cut. Not a return: NaN, inf, 0 (no echo), anything under
    the scan's range_min or over its range_max -- the collision monitor skips those too.
    `masked` sectors and `boxes` (the config's mask, for a whole-circle scan) are left out."""
    out = []
    a0, inc = float(scan["angle_min"]), float(scan["angle_increment"])
    rmin, rmax = float(scan.get("range_min", 0.0)), float(scan.get("range_max", math.inf))
    lx, ly, lyaw = laser
    for i, r in enumerate(scan["ranges"]):
        if r is None:
            continue
        r = float(r)
        if not math.isfinite(r) or r <= 0.0 or r < rmin or r > rmax:
            continue
        th = a0 + i * inc + lyaw
        x, y = lx + r * math.cos(th), ly + r * math.sin(th)
        if math.hypot(x, y) >= radius:
            continue
        b = math.degrees(th) % 360.0
        if masked and lidar_mask.occluded(b, masked):
            continue
        if boxes and _in_box(x, y, boxes):
            continue
        out.append((b, r))
    return out


def self_return_arcs(scans, radius: float, min_points: int = DEFAULT_MIN_POINTS,
                     laser=(0.0, 0.0, 0.0), masked=(), boxes=()) -> list:
    """The arcs of returns inside the robot that persist, or [] when they number fewer than
    min_points (what the collision monitor would let through).

    A bearing (1-degree bin) counts when it holds a return inside the radius in more than
    half of the scans, so one stray return -- a foot, a flicker -- does not. Each arc is a dict:
    start, end (whole degrees, counter-clockwise from start to end, robot frame), count (the
    median, over the scans, of its returns), rmin, rmax (m)."""
    if isinstance(scans, dict):
        scans = [scans]
    per_scan = [inside_returns(s, radius, laser, masked, boxes) for s in scans]
    n = len(per_scan)
    if not n:
        return []
    hits = {}
    for rets in per_scan:
        for k in {int(b) % 360 for b, _ in rets}:
            hits[k] = hits.get(k, 0) + 1
    keep = sorted(k for k, h in hits.items() if h * 2 > n or n == 1)
    if not keep:
        return []
    # contiguous bins (a one-degree gap allowed: a beam every 0.8 deg can skip a bin), the
    # arc across 0 deg joined
    groups = [[keep[0]]]
    for k in keep[1:]:
        if k - groups[-1][-1] <= 2:
            groups[-1].append(k)
        else:
            groups.append([k])
    if len(groups) > 1 and groups[0][0] + 360 - groups[-1][-1] <= 2:
        groups[0] = groups.pop() + groups[0]
    arcs = []
    for g in groups:
        bins = set(g)
        first = g[0]
        counts, offs, rngs = [], [], []
        for rets in per_scan:
            mine = [(b, r) for b, r in rets if int(b) % 360 in bins]
            counts.append(len(mine))
            offs += [(b - first) % 360.0 for b, _ in mine]
            rngs += [r for _, r in mine]
        lo, hi = first + min(offs), first + max(offs)
        arcs.append({"start": int(math.floor(lo)) % 360, "end": int(math.ceil(hi)) % 360,
                     "count": int(statistics.median(counts)),
                     "rmin": min(rngs), "rmax": max(rngs)})
    if sum(a["count"] for a in arcs) < max(1, int(min_points)):
        return []
    return arcs


def suggest(arc: dict, pad: float = PAD_DEG) -> list:
    """The lidar.mask.sectors entry [from, to] that covers an arc, `pad` degrees past each end."""
    return [int(round(arc["start"] - pad)) % 360, int(round(arc["end"] + pad)) % 360]


def merged_sectors(existing, extra) -> list:
    """The config's sectors ([from, to] pairs) with `extra` ones added, overlaps joined: the
    whole lidar.mask.sectors value to paste. Tenth-of-a-degree resolution."""
    cover = [False] * 3600
    for s in list(existing or []) + list(extra or []):
        a, b = float(s[0]) % 360.0, float(s[1]) % 360.0
        w = (b - a) % 360.0
        for k in range(int(round(w * 10)) + 1):
            cover[int(round(a * 10 + k)) % 3600] = True
    if all(cover):
        return [[0, 359]]
    # start from an uncovered cell so no run is cut at 0
    z = cover.index(False)
    runs, cur = [], None
    for k in range(1, 3601):
        i = (z + k) % 3600
        if cover[i] and cur is None:
            cur = i
        elif not cover[i] and cur is not None:
            runs.append([cur, (i - 1) % 3600])
            cur = None
    fmt = lambda t: int(t / 10) if t % 10 == 0 else t / 10
    return sorted([[fmt(a), fmt(b)] for a, b in runs], key=lambda r: r[0])


def arcs_text(arcs) -> str:
    return ", ".join(f"{a['start']}-{a['end']} deg ({a['count']} at {a['rmin']:.2f}-{a['rmax']:.2f} m)"
                     for a in arcs)


def sectors_text(arcs) -> str:
    return ", ".join(str(suggest(a)) for a in arcs)


def config_sectors(controller: dict) -> list:
    mask = (((controller or {}).get("lidar") or {}).get("mask") or {})
    return [list(s) for s in (mask.get("sectors") or []) if isinstance(s, (list, tuple))]


def refusal(arcs, radius: float, min_points: int, topic: str, controller: dict = None) -> list:
    """The lines that say why Nav2 cannot move the robot, and what to do."""
    total = sum(a["count"] for a in arcs)
    lines = [f"❌ NAV2 HELD: {total} LiDAR returns inside the robot's {radius:.2f} m footprint on "
             f"{topic}, at {arcs_text(arcs)} -- the LiDAR sees the robot itself (its body or "
             f"mount) or something touching it. Nav2's collision monitor counts {min_points} or "
             f"more as a collision in progress and holds every command at zero.",
             f"   Next: add lidar.mask.sectors {sectors_text(arcs)} ({PAD_DEG:.0f} deg past the "
             f"returns) or move what the LiDAR sees; until then Nav2's collision monitor holds "
             f"the robot still."]
    have = config_sectors(controller)
    if have:
        lines.append(f"   With the config's {have}: lidar.mask.sectors: "
                     f"{merged_sectors(have, [suggest(a) for a in arcs])}")
    return lines


def short_reason(arcs) -> str:
    return (f"{sum(a['count'] for a in arcs)} LiDAR returns inside the footprint at "
            + ", ".join(f"{a['start']}-{a['end']} deg" for a in arcs)
            + f"; add lidar.mask.sectors {sectors_text(arcs)}")


# ---- ROS ----

def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--params", required=True, help="the robot's config YAML")
    ap.add_argument("--scans", type=int, default=10, help="scans to judge by (~1 s at 10 Hz)")
    ap.add_argument("--topic", default="", help="default: the collision monitor's scan source")
    ap.add_argument("--timeout", type=float, default=10.0)
    a = ap.parse_args(argv)
    import yaml
    with open(os.path.expanduser(a.params)) as fh:
        params = yaml.safe_load(fh) or {}
    import cockpit_paths
    cs = collision_settings(params)
    if not (a.topic or cs["topic"]):
        print("SELF_RETURNS OK: the collision monitor has no scan source that stops the robot")
        return 0
    ns = cockpit_paths.robot_namespace(params)
    topic = a.topic or (f"/{ns}/" if ns else "/") + cs["topic"].lstrip("/")

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan
    import scan_match
    rclpy.init()
    node = Node("self_returns")
    scans, frame = [], [None]

    def on_scan(m):
        frame[0] = m.header.frame_id
        scans.append(scan_match.scan_dict(m))
    node.create_subscription(LaserScan, topic, on_scan, qos_profile_sensor_data)
    end = time.time() + a.timeout
    while len(scans) < a.scans and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    laser = None
    if scans:
        base = f"{ns}/base_link" if ns else "base_link"
        laser = scan_match.laser_pose(node, frame[0] or "laser", base_frame=base, tries=10)
    node.destroy_node()
    rclpy.shutdown()
    if not scans:
        print(f"SELF_RETURNS NO_SCAN: nothing on {topic} in {a.timeout:.0f} s")
        return 2
    if laser is None:
        print(f"  ⚠️ no TF base_link -> {frame[0]}; judging in the scan frame")
        laser = (0.0, 0.0, 0.0)
    arcs = self_return_arcs(scans, cs["radius"], cs["min_points"], laser)
    print(f"  {len(scans)} scans on {topic}: returns inside the {cs['radius']:.2f} m footprint "
          f"(the collision monitor's min_points {cs['min_points']}): "
          + (arcs_text(arcs) if arcs else "fewer than that"))
    if not arcs:
        print("SELF_RETURNS OK")
        return 0
    for ln in refusal(arcs, cs["radius"], cs["min_points"], topic, params.get("base_controller")):
        print(ln)
    print("SELF_RETURNS REFUSED: " + short_reason(arcs))
    return 1


if __name__ == "__main__":
    sys.exit(main())
