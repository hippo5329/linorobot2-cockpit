#!/usr/bin/env python3
# ==============================================================================
# topic_stream.py — the Topic Monitor's rate and echo, from a direct rclpy subscription
#
# One JSON object per line on stdout, relayed by the supervisor as SSE events:
#   --mode hz     {"event": "hz", ...} once a second: the rate over the window,
#                 the min/max gap, and how long since the last message (a stall
#                 shows at once, where an average would hide it)
#   --mode echo   {"event": "msg", "text": <YAML>} per message, at most --max-hz a
#                 second (the rest counted in "skipped": a 50 Hz /odom is not
#                 something a page can show, and the count says what was dropped)
#   --mode once   one {"event": "result", "hz": ...} after --secs: the Monitor's
#                 "Measure all" and /api/ros2/hz_single
#   --mode mcu    one {"event": "topics", "topics": [...]}: the topics the MCU's own node
#                 (`<robot>_base_node`, main.cpp) publishes or subscribes to -- the
#                 Monitor shows the board's view only (user, 2026-10-06), not the graph
# plus {"event": "status", ...} while the topic has no publisher.
#
# Why not `ros2 topic hz` / `ros2 topic echo` (what the Robot Config Engine's viewer
# ran): the CLI copies the publisher's QoS through a graph lookup, and under the
# cockpit's Fast DDS profile that lookup does not find the micro-ROS agent's
# publishers -- "does not appear to be published yet" on a topic the 1-Click gate
# had just measured at 50 Hz (runners.check_bringup_health). A best-effort
# subscription matches best-effort and reliable publishers alike and needs no lookup.
# ==============================================================================

import argparse
import collections
import json
import sys
import time

try:
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
    from rosidl_runtime_py import message_to_yaml
    from rosidl_runtime_py.utilities import get_message
except ImportError as exc:
    # Name the module that actually failed (see scan_stream.py).
    print(json.dumps({"event": "status", "level": "error",
                      "message": f"cannot import the ROS 2 Python stack: {exc}",
                      "interpreter": sys.executable}), flush=True)
    sys.exit(1)


def emit(obj):
    print(json.dumps(obj), flush=True)


def rate_of(stamps):
    """(rate Hz, min gap s, max gap s) over the arrival times, like `ros2 topic hz`."""
    if len(stamps) < 2:
        return None, None, None
    gaps = [b - a for a, b in zip(stamps, list(stamps)[1:])]
    span = stamps[-1] - stamps[0]
    return (len(stamps) - 1) / span if span > 0 else None, min(gaps), max(gaps)


class TopicStream(Node):
    def __init__(self, topic, msg_type, mode, window, max_hz, truncate):
        super().__init__("linorobot2_topic_monitor")
        self.topic, self.mode = topic, mode
        self.stamps = collections.deque(maxlen=window)
        self.count = 0
        self.skipped = 0
        self.min_period = 1.0 / max_hz if max_hz > 0 else 0.0
        self.last_emit = 0.0
        self.truncate = truncate
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST,
                         depth=10, durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(msg_type, topic, self.on_msg, qos)

    def on_msg(self, msg):
        now = time.monotonic()
        self.stamps.append(now)
        self.count += 1
        if self.mode != "echo":
            return
        if now - self.last_emit < self.min_period:
            self.skipped += 1
            return
        self.last_emit = now
        text = message_to_yaml(msg, truncate_length=self.truncate)
        emit({"event": "msg", "seq": self.count, "skipped": self.skipped, "text": text})

    def hz_report(self):
        rate, gmin, gmax = rate_of(self.stamps)
        since = time.monotonic() - self.stamps[-1] if self.stamps else None
        return {"topic": self.topic, "hz": round(rate, 2) if rate else 0.0, "count": self.count,
                "min_gap": round(gmin, 4) if gmin is not None else None,
                "max_gap": round(gmax, 4) if gmax is not None else None,
                "since_last": round(since, 2) if since is not None else None,
                "window": len(self.stamps)}


MCU_NODE_SUFFIX = "_base_node"   # mcu_env: env["node"] = f"{robot}_base_node"; default linorobot_base_node
# ...but a node that joined after the board connected never learns that name: the micro-ROS
# agent creates the board's DDS entities itself and announces their node only when its graph
# changes, so a fresh subscriber sees this placeholder on every MCU endpoint (measured
# 2026-10-06 on the Sim MCU: 25 s, still the placeholder; only the old ros2 daemon knew the
# name). Nothing else in the stack makes bare DDS entities, so it marks the MCU as surely.
AGENT_PLACEHOLDER = "_CREATED_BY_BARE_DDS_APP_"


def is_mcu_endpoint(node_name: str) -> bool:
    return node_name.endswith(MCU_NODE_SUFFIX) or node_name == AGENT_PLACEHOLDER


def mcu_topics(node, wait, settle=2.0):
    """[{topic, type, direction: pub|sub, node}] for the MCU's node.

    Discovery arrives an endpoint at a time: the first non-empty answer held two of the Sim
    MCU's nine topics. So the answer is the set once it has stopped changing for `settle` s
    (or whatever there is at `wait` s)."""
    deadline = time.monotonic() + wait
    last, since = None, time.monotonic()
    while True:
        found = []
        for name, types in node.get_topic_names_and_types():
            for direction, infos in (("pub", node.get_publishers_info_by_topic(name)),
                                     ("sub", node.get_subscriptions_info_by_topic(name))):
                owners = sorted({i.node_name for i in infos if is_mcu_endpoint(i.node_name)})
                if owners:
                    named = [o for o in owners if o != AGENT_PLACEHOLDER]
                    found.append({"topic": name, "type": types[0] if types else "",
                                  "direction": direction, "node": named[0] if named else "the MCU"})
        key = sorted((f["topic"], f["direction"]) for f in found)
        now = time.monotonic()
        if key != last:
            last, since = key, now
        elif found and now - since >= settle:
            return found
        if now >= deadline:
            return found
        rclpy.spin_once(node, timeout_sec=0.25)


def resolve_type(node, topic, wait):
    """The topic's message type from the graph, waiting up to `wait` s for a publisher."""
    deadline = time.monotonic() + wait
    told = False
    while True:
        for name, types in node.get_topic_names_and_types():
            if name == topic and types:
                return types[0]
        if time.monotonic() >= deadline:
            return None
        if not told:
            emit({"event": "status", "level": "info", "message": f"waiting for {topic} to appear..."})
            told = True
        rclpy.spin_once(node, timeout_sec=0.25)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--topic", default="")
    ap.add_argument("--mode", choices=("hz", "echo", "once", "mcu"), default="hz")
    ap.add_argument("--type", default="", help="message type; looked up in the graph when left out")
    ap.add_argument("--secs", type=float, default=6.0, help="--mode once: the sampling window")
    ap.add_argument("--wait", type=float, default=10.0, help="how long to wait for the topic to appear")
    ap.add_argument("--window", type=int, default=10000, help="arrivals the rate is averaged over")
    ap.add_argument("--max-hz", type=float, default=5.0, help="--mode echo: messages shown per second")
    ap.add_argument("--truncate", type=int, default=32, help="--mode echo: array elements shown")
    a = ap.parse_args()

    rclpy.init()
    probe = rclpy.create_node("linorobot2_topic_monitor_probe")
    if a.mode == "mcu":
        emit({"event": "topics", "topics": mcu_topics(probe, a.wait)})
        probe.destroy_node()
        rclpy.shutdown()
        return 0
    if not a.topic:
        ap.error("--topic is required")
    type_name = a.type or resolve_type(probe, a.topic, a.wait)
    probe.destroy_node()
    if not type_name:
        emit({"event": "status", "level": "error", "message": f"{a.topic}: no publisher, not in the graph"})
        if a.mode == "once":
            emit({"event": "result", "topic": a.topic, "hz": 0.0, "count": 0, "advertised": False})
        rclpy.shutdown()
        return 0
    try:
        msg_type = get_message(type_name)
    except (AttributeError, ModuleNotFoundError, ValueError) as exc:
        emit({"event": "status", "level": "error", "message": f"{a.topic}: cannot load {type_name}: {exc}"})
        rclpy.shutdown()
        return 1

    node = TopicStream(a.topic, msg_type, a.mode, a.window, a.max_hz, a.truncate)
    emit({"event": "start", "topic": a.topic, "type": type_name, "mode": a.mode})
    try:
        if a.mode == "once":
            end = time.monotonic() + a.secs
            while time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.1)
            emit({"event": "result", "type": type_name, "advertised": True, **node.hz_report()})
            return 0
        next_report = time.monotonic() + 1.0
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if a.mode == "hz" and time.monotonic() >= next_report:
                next_report += 1.0
                emit({"event": "hz", **node.hz_report()})
    except (KeyboardInterrupt, ExternalShutdownException):
        pass          # the supervisor closed the stream: rclpy's SIGTERM handler shut the context
    except Exception:
        if rclpy.ok():
            raise     # a real fault; one raised after shutdown is only the stop arriving mid-spin
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
