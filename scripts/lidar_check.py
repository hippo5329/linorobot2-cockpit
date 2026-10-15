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
"""Is the LiDAR healthy? Its rate, its coverage and its ranges, from the running stack.

    lidar_check.py [--params ROBOT_config.yaml] [--seconds 10] [--json OUT.json]

Nothing moves. Run it with the robot brought up (no SLAM or Nav2 needed), in the cockpit's
container; it runs itself under the stack's ROS environment, as speed_check.py does.

It reads the whole circle (scan_raw when a lidar.mask makes /scan a narrower view) and
prints:

  rate       scans per second, against what the model should give (LD19/LD06/STL-27L: 10 Hz):
             RATE_FLOOR_SHARE of it over the worst RATE_WINDOW_S, the motion guard's own floor
  beams      per scan, and the share of them that are valid returns (finite, in range)
  coverage   the share of 10-degree sectors with at least one valid return -- a LiDAR
             half-covered by the chassis or a cable, or a driver at the wrong baud
             (scans of garbage), shows up here
  ranges     nearest, median and farthest valid return
  room       the nearest return in each quadrant (front, left, back, right), the same
             numbers clearance.py and speed_check.py judge by
  self       returns inside the robot's footprint that lidar.mask does not cover (the robot
             itself, past its mask): named with the sector to add, and a FAIL when they reach
             Nav2's collision monitor's min_points -- it would hold the robot still

and LIDAR_CHECK PASS or LIDAR_CHECK FAIL (with the reasons), then one LIDAR_CHECK_JSON line.
Exit 0 on PASS, 1 on FAIL, 2 when no scan arrived.
"""
import argparse
import json
import math
import os
import statistics
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

# Scans per second a model gives at its default speed; others are judged at >= 3 Hz.
MODEL_HZ = {"LD19": 10.0, "LD06": 10.0, "STL27L": 10.0, "STL-27L": 10.0, "LD20": 10.0,
            "RPLIDAR_A1": 5.5, "RPLIDAR_A2": 10.0, "RPLIDAR_C1": 10.0, "YDLIDAR_X4": 7.0}
# ONE rate floor for bring-up and for the run (motion_guard.py imports these): a share of the
# model's rate over a window of RATE_WINDOW_S. Bring-up passed a LiDAR at 60 % of its rate and
# the guard tripped at 80 %, so a 7 Hz LD19 passed here and then stopped every run. 80 % is
# for a LiDAR starved of power (a tracked robot's 6.9 Hz), not a robot computer busy for a
# moment: a UNO Q fell to 8.4 Hz over 3 s while Nav2 came up, 9.0 over 5 s.
RATE_FLOOR_SHARE = 0.80
RATE_WINDOW_S = 5.0
UNKNOWN_MODEL_HZ = 3.0  # an unknown model's floor here (the guard uses what it measured)
MIN_VALID = 0.30        # share of beams with a return: a room gives far more
MIN_COVERAGE = 0.75     # share of 10-deg sectors seen: the chassis may hide a quarter


def expected_hz(params: dict) -> float:
    lidar = ((params or {}).get("base_controller") or {}).get("lidar") or {}
    model = str((lidar.get("model") if isinstance(lidar, dict) else lidar) or "").strip().upper()
    return MODEL_HZ.get(model, 0.0)


def forwarded(params: dict) -> bool:
    """True when the board forwards the LiDAR over the network (lidar.comm_mode udp)."""
    lidar = ((params or {}).get("base_controller") or {}).get("lidar") or {}
    return isinstance(lidar, dict) and str(lidar.get("comm_mode", "")).strip().lower() == "udp"


def window_rate(stamps: list, now: float, window: float = RATE_WINDOW_S) -> float:
    """Scans per second over the last `window` seconds of arrival times before `now`."""
    recent = [t for t in stamps if 0.0 <= now - t <= window]
    if len(recent) < 2:
        return 0.0
    return (len(recent) - 1) / max(recent[-1] - recent[0], 1e-6)


def worst_window_rate(stamps: list, window: float = RATE_WINDOW_S) -> float:
    """The lowest window_rate over every full window of a reading (the whole reading's rate
    when it is shorter than one window): what the motion guard would have seen at worst."""
    stamps = sorted(stamps)
    if len(stamps) < 2:
        return 0.0
    if stamps[-1] - stamps[0] < window:
        return window_rate(stamps, stamps[-1], stamps[-1] - stamps[0])
    return min(window_rate(stamps, t, window) for t in stamps if t - stamps[0] >= window)


def analyse(scans: list, seconds: float, want_hz: float = 0.0) -> dict:
    """The verdict from a list of scan_match.scan_dict scans (no ROS)."""
    out = {"scans": len(scans), "seconds": round(seconds, 2)}
    reasons = []
    if not scans:
        out.update({"pass": False, "reasons": ["no scan arrived"]})
        return out
    stamps = sorted(s["stamp"] for s in scans)
    span = stamps[-1] - stamps[0]
    hz = (len(stamps) - 1) / span if span > 0 else 0.0
    beams = [len(s["ranges"]) for s in scans]
    valid, ranges, sectors = [], [], set()
    for s in scans:
        n = 0
        for i, r in enumerate(s["ranges"]):
            if r is None or not math.isfinite(r) or r < max(s["range_min"], 0.02) or r > s["range_max"]:
                continue
            n += 1
            ranges.append(r)
            a = math.degrees(s["angle_min"] + i * s["angle_increment"]) % 360.0
            sectors.add(int(a // 10) % 36)
        valid.append(n / max(1, len(s["ranges"])))
    worst = worst_window_rate(stamps)
    out.update({
        "hz": round(hz, 2), "worst_hz": round(worst, 2), "want_hz": want_hz, "beams": int(statistics.median(beams)),
        "valid": round(statistics.median(valid), 3), "coverage": round(len(sectors) / 36.0, 3),
        "nearest": round(min(ranges), 3) if ranges else None,
        "median": round(statistics.median(ranges), 3) if ranges else None,
        "farthest": round(max(ranges), 3) if ranges else None,
    })
    floor_hz = RATE_FLOOR_SHARE * want_hz if want_hz else UNKNOWN_MODEL_HZ
    if worst < floor_hz:
        reasons.append(f"rate {worst:.1f} Hz over the worst {RATE_WINDOW_S:.0f} s, under {floor_hz:.1f} Hz"
                       + (f" ({RATE_FLOOR_SHARE:.0%} of the model's {want_hz:.0f}; the motion guard "
                          f"would stop every run)" if want_hz else ""))
    if out["valid"] < MIN_VALID:
        reasons.append(f"only {out['valid'] * 100:.0f} % of beams are valid returns (wrong baud, "
                       f"a dirty window, or nothing in range)")
    if out["coverage"] < MIN_COVERAGE:
        reasons.append(f"returns in only {out['coverage'] * 100:.0f} % of the circle -- something "
                       f"blocks the LiDAR, or its mount hides part of it")
    out["pass"] = not reasons
    out["reasons"] = reasons
    return out


def self_return_check(scans: list, params: dict, laser=(0.0, 0.0, 0.0)) -> dict:
    """Returns inside the robot's footprint that the config's lidar.mask does not cover (no ROS).

    The bring-up reads the whole circle (scan_raw), so the mask is applied here the way the
    scan filter applies it (each sector widened by margin_deg, the boxes); what is left is what
    Nav2's collision monitor will see on /scan. {'arcs': [...] (every persistent arc, see
    self_returns.self_return_arcs), 'lines': [what to print], 'fail': reason or ''}: it fails
    when they reach the monitor's min_points -- a robot that cannot drive under Nav2 does not
    pass its LiDAR bring-up. A LiDAR with a nearest return of 0.03 m and a median of 1.86 m
    passed here while its own mount, past the mask, held Nav2 still."""
    import self_returns
    bc = (params or {}).get("base_controller") or {}
    cs = self_returns.collision_settings(params or {})
    try:
        masked = self_returns.widened_mask(bc)
        boxes = lidar_mask_boxes(bc)
    except ValueError:
        masked, boxes = [], []
    arcs = self_returns.self_return_arcs(scans, cs["radius"], 1, laser, masked, boxes)
    out = {"arcs": arcs, "lines": [], "fail": ""}
    if not arcs:
        return out
    total = sum(a["count"] for a in arcs)
    holds = bool(cs["topic"]) and total >= cs["min_points"]
    out["lines"].append(
        f"  {'❌' if holds else '⚠️'} self      {total} returns inside the {cs['radius']:.2f} m footprint, "
        f"not masked: {self_returns.arcs_text(arcs)} -- add lidar.mask.sectors "
        f"{self_returns.sectors_text(arcs)} ({self_returns.PAD_DEG:.0f} deg past the returns) or move "
        f"what the LiDAR sees" + (f"; Nav2's collision monitor holds the robot still at "
                                  f"{cs['min_points']} or more" if holds else ""))
    have = self_returns.config_sectors(bc)
    if have:
        out["lines"].append(f"            with the config's {have}: lidar.mask.sectors: "
                            f"{self_returns.merged_sectors(have, [self_returns.suggest(a) for a in arcs])}")
    if holds:
        out["fail"] = (f"{total} returns inside the robot's footprint at "
                       + ", ".join(f"{a['start']}-{a['end']} deg" for a in arcs)
                       + f", outside lidar.mask.sectors -- Nav2's collision monitor would hold the "
                         f"robot still; add lidar.mask.sectors {self_returns.sectors_text(arcs)}")
    return out


def lidar_mask_boxes(controller: dict) -> list:
    import lidar_mask
    return lidar_mask.mask_config(controller)["boxes"]


def ros_env_command(argv: list) -> str:
    import shlex
    import one_click_pipeline
    args = " ".join(shlex.quote(x) for x in argv)
    return (f"{one_click_pipeline.get_ros_env(os.environ.get('ROS_DISTRO', 'auto'))} && "
            f"LIDAR_CHECK_IN_ROS_ENV=1 exec python3 {shlex.quote(os.path.abspath(__file__))} {args}")


def run(a, params: dict) -> int:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan
    import clearance
    import cockpit_paths
    import scan_match
    ns = cockpit_paths.robot_namespace(params)
    pre = f"/{ns}" if ns else ""
    rclpy.init()
    node = Node("lidar_check")
    names = set()
    end = time.time() + 8.0
    topic = f"{pre}/scan"
    while time.time() < end:
        names = {n for n, _ in node.get_topic_names_and_types()}
        if f"{pre}/scan_raw" in names:
            topic = f"{pre}/scan_raw"
            break
        if f"{pre}/scan" in names and time.time() > end - 4.0:
            break
        rclpy.spin_once(node, timeout_sec=0.2)
    scans, frame = [], [None]

    def on_scan(m):
        frame[0] = m.header.frame_id
        scans.append(scan_match.scan_dict(m, stamp=time.time()))
    node.create_subscription(LaserScan, topic, on_scan, qos_profile_sensor_data)
    print(f"  reading {topic} for {a.seconds:.0f} s (nothing moves)", flush=True)
    t0 = time.time()
    first = None
    while time.time() - t0 < a.seconds + (5.0 if not scans else 0.0):
        rclpy.spin_once(node, timeout_sec=0.05)
        if scans and first is None:
            first = time.time()
        if first and time.time() - first >= a.seconds:
            break
    res = analyse(scans, time.time() - (first or t0), expected_hz(params))
    res["topic"] = topic
    if scans:
        laser = scan_match.laser_pose(node, frame[0] or "laser", tries=10) or (0.0, 0.0, 0.0)
        P = scan_match.median_points(scans[-10:], laser)
        q = clearance.quadrants(P)
        res["room"] = {k: (None if v == math.inf else round(v, 3)) for k, v in q.items()}
        selfr = self_return_check(scans[-10:], params, laser)
        res["self_returns"] = selfr["arcs"]
        if selfr["fail"]:
            res["pass"] = False
            res["reasons"] = list(res.get("reasons") or []) + [selfr["fail"]]
    node.destroy_node()
    rclpy.shutdown()
    if not scans:
        print(f"LIDAR_CHECK FAIL: no scan on {topic} -- is the robot brought up, and is the LiDAR "
              f"powered (it needs the battery, not USB)?")
        print("LIDAR_CHECK_JSON " + json.dumps(res))
        return 2
    print(f"  rate      {res['hz']:.1f} Hz, {res['worst_hz']:.1f} over the worst {RATE_WINDOW_S:.0f} s"
          + (f" (the model: {res['want_hz']:.0f})" if res["want_hz"] else ""))
    print(f"  beams     {res['beams']} per scan, {res['valid'] * 100:.0f} % valid")
    print(f"  coverage  {res['coverage'] * 100:.0f} % of the circle")
    print(f"  ranges    nearest {res['nearest']} m, median {res['median']} m, farthest {res['farthest']} m")
    if "room" in res:
        print("  room      " + ", ".join(f"{k} {'none' if v is None else f'{v:.2f} m'}"
                                       for k, v in res["room"].items()))
    for ln in (selfr["lines"] if scans else []):
        print(ln)
    if res["pass"]:
        print("LIDAR_CHECK PASS")
    else:
        print("LIDAR_CHECK FAIL: " + "; ".join(res["reasons"]))
    print("LIDAR_CHECK_JSON " + json.dumps(res))
    if a.json:
        with open(a.json, "w") as fh:
            json.dump(res, fh)
    return 0 if res["pass"] else 1


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--params", default="", help="the robot's config YAML (LiDAR model, namespace)")
    ap.add_argument("--seconds", type=float, default=10.0, help="how long to read (10 s)")
    ap.add_argument("--json", default="", help="write the result here too")
    a = ap.parse_args(argv)
    if os.environ.get("LIDAR_CHECK_IN_ROS_ENV") != "1":
        os.execvp("bash", ["bash", "-c", ros_env_command(sys.argv[1:] if argv is None else list(argv))])
    params = {}
    if a.params:
        import yaml
        with open(os.path.expanduser(a.params)) as fh:
            params = yaml.safe_load(fh) or {}
    return run(a, params)


if __name__ == "__main__":
    sys.exit(main())
