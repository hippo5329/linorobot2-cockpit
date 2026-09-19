#!/usr/bin/env python3
# ==============================================================================
# scan_stream.py — /scan sampler for the Cockpit LiDAR viewer
#
# Prints one compact JSON object per line on stdout so the supervisor can relay
# it straight into an SSE `scan` event. `ros2 topic echo --csv` was used here
# before, but CSV has no field names and the browser canvas needs
# angle_min / angle_increment / ranges, so the viewer could never plot anything.
#
# Ranges are decimated before printing: an LD19 turns out ~450 points per scan
# at 10 Hz and the canvas is only 480 px wide, so sending every point costs
# bandwidth the viewer cannot show.
#
# Emits {"event": "status", ...} lines when no publisher shows up, so the UI can
# say so instead of drawing an empty circle forever.
# ==============================================================================

import argparse
import json
import sys
import time

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
    from sensor_msgs.msg import LaserScan
except ImportError as exc:
    # Name the module that actually failed, per AGENTS.md 12 -- "rclpy not found"
    # was wrong far more often than right: rclpy imports even from a venv that
    # hides the system dist-packages, and the real casualty is numpy, pulled in
    # by rclpy.node -> rosgraph_msgs. Carry the exception and the interpreter
    # through the JSON envelope; both are the diagnosis.
    print(json.dumps({
        "event": "status",
        "level": "error",
        "message": f"cannot import the ROS 2 Python stack: {exc}",
        "interpreter": sys.executable,
    }), flush=True)
    sys.exit(1)


class ScanStreamer(Node):
    def __init__(self, topic: str, max_points: int, max_hz: float):
        super().__init__("linorobot2_scan_streamer")
        self.topic = topic
        self.max_points = max_points
        self.min_period = (1.0 / max_hz) if max_hz > 0 else 0.0
        self.last_emit = 0.0
        self.count = 0

        # SensorDataQoS: /scan is best-effort volatile (AGENTS.md section 7).
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(LaserScan, topic, self.on_scan, sensor_qos)

    def on_scan(self, msg: LaserScan):
        now = time.time()
        if self.min_period and (now - self.last_emit) < self.min_period:
            return
        self.last_emit = now

        ranges = list(msg.ranges)
        step = max(1, len(ranges) // self.max_points) if self.max_points > 0 else 1
        if step > 1:
            ranges = ranges[::step]

        # JSON has no NaN/Infinity in the strict sense the browser parses, and
        # an out-of-range return is exactly what those values mean -- send 0.0,
        # which drawScan() already skips.
        clean = [0.0 if (r != r or r in (float("inf"), float("-inf"))) else round(float(r), 4)
                 for r in ranges]

        self.count += 1
        print(json.dumps({
            "event": "scan",
            "frame_id": msg.header.frame_id,
            "angle_min": round(float(msg.angle_min), 6),
            "angle_max": round(float(msg.angle_max), 6),
            "angle_increment": round(float(msg.angle_increment) * step, 6),
            "range_min": round(float(msg.range_min), 3),
            "range_max": round(float(msg.range_max), 3),
            "count": len(clean),
            "seq": self.count,
            "ranges": clean,
        }), flush=True)


def main() -> int:
    ap = argparse.ArgumentParser(description="Stream /scan as JSON lines for the Cockpit viewer.")
    ap.add_argument("--topic", default="/scan")
    ap.add_argument("--max-points", type=int, default=360, help="decimate each scan to at most N points")
    ap.add_argument("--max-hz", type=float, default=10.0, help="cap emitted frames per second (0 = uncapped)")
    ap.add_argument("--wait-sec", type=float, default=8.0, help="seconds to wait for a publisher before warning")
    args = ap.parse_args()

    rclpy.init()
    node = ScanStreamer(args.topic, args.max_points, args.max_hz)
    print(json.dumps({
        "event": "status",
        "level": "info",
        "message": f"Subscribed to {args.topic}, waiting for scans...",
    }), flush=True)

    started = time.time()
    warned = False
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.5)
            if not warned and node.count == 0 and (time.time() - started) > args.wait_sec:
                warned = True
                pubs = node.count_publishers(args.topic)
                print(json.dumps({
                    "event": "status",
                    "level": "warn",
                    "message": (
                        f"No scans after {args.wait_sec:.0f}s ({pubs} publisher(s) on {args.topic}). "
                        "Start the laser driver (or Bringup) first."
                    ),
                }), flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
