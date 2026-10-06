#!/usr/bin/env python3
"""Persistent cmd_vel publisher for the Console's virtual gamepad.

The browser gamepad produces a stream of small velocity updates as the stick is
dragged. Spawning `ros2 topic pub` per update is not usable -- node startup and
discovery cost far more than the interval between updates -- so Console starts
this once and feeds it target velocities on stdin, one "lx ly az" line each.

It republishes the latest target at a fixed rate, which is what a real teleop
node does: the base stops when cmd_vel goes quiet, so the command has to be
held. It also carries a deadman timeout -- if the browser stops sending (tab
closed, laptop asleep, Wi-Fi dropped) the robot is commanded to zero rather
than driving on the last command it heard.

It publishes the message type the FIRMWARE was built to hear, which is not the
same on every distro. nav2 1.4 (kilted) flipped nav2_util::TwistPublisher to
TwistStamped, so the boundary is KILTED: jazzy and older drive /cmd_vel plain,
kilted and everything after it drive it stamped. `stamped_cmd_vel: auto`
follows that, and so does the firmware -- with USE_STAMPED_CMD_VEL it
subscribes TwistStamped on /cmd_vel and puts the plain Twist subscriber on
/cmd_vel_unstamped instead.

So a plain Twist on /cmd_vel, which is what this published unconditionally
until 2026-09-23, reached a post-kilted robot's subscriber as the wrong type
on the right topic and was dropped -- the gamepad moved and the base did not,
with nothing said on either side.
"""

import argparse
import json
import os
import sys
import threading
import time

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node

# One source of truth for which distros stamp /cmd_vel: the same function the
# firmware header generator uses to decide USE_STAMPED_CMD_VEL. Importing it
# rather than repeating the list is the point -- two copies of that list would
# disagree on exactly the distro nobody tested.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gen_firmware_header import distro_stamps_cmd_vel  # noqa: E402
import stall_check  # noqa: E402


def resolve_cmd_vel_type(choice: str, distro: str = None) -> bool:
    """True when /cmd_vel should carry TwistStamped.

    "auto" asks the distro -- the same question gen_firmware_header.py asked
    when it built the firmware this is driving, so the answer is kilted and
    later stamped, jazzy and older plain.
    """
    c = (choice or "auto").strip().lower()
    if c in ("twist_stamped", "twiststamped", "stamped"):
        return True
    if c in ("twist", "unstamped"):
        return False
    if distro is None:
        distro = os.environ.get("ROS_DISTRO", "")
    return distro_stamps_cmd_vel(distro)


class GamepadPublisher(Node):
    def __init__(self, topic, rate_hz, timeout_s, stamped=False, odom_topic="/odom", status_file=None):
        super().__init__("console_gamepad")
        self.stamped = stamped
        self.msg_type = TwistStamped if stamped else Twist
        self.pub = self.create_publisher(self.msg_type, topic, 10)
        self.timeout_s = timeout_s
        self.lock = threading.Lock()
        self.target = (0.0, 0.0, 0.0)
        self.last_cmd_time = 0.0
        # The stall check: what the base measures against what it is told (stall_check.py).
        self.status_file = status_file
        self.measured = (0.0, 0.0, 0.0)
        self.odom_time = None
        self.held_since = None
        self.create_subscription(Odometry, odom_topic, self.on_odom, 10)
        self.create_timer(1.0 / rate_hz, self.tick)
        if status_file:
            self.create_timer(0.25, self.write_status)
        self.get_logger().info(
            "publishing %s on %s at %g Hz, deadman timeout %gs"
            % (self.msg_type.__name__, topic, rate_hz, timeout_s)
        )

    def _msg(self, lx, ly, az):
        """A command in whichever type this robot's firmware subscribes to.

        frame_id is left empty on purpose: the firmware's TwistStamped message
        carries a 64-byte frame_id buffer and reads only .twist, so a name here
        would be copied into that buffer for nothing.
        """
        twist = Twist()
        twist.linear.x = lx
        twist.linear.y = ly
        twist.angular.z = az
        if not self.stamped:
            return twist
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist = twist
        return msg

    def on_odom(self, msg):
        t = msg.twist.twist
        with self.lock:
            self.measured = (t.linear.x, t.linear.y, t.angular.z)
            self.odom_time = time.monotonic()

    def write_status(self):
        """The stall verdict where the cockpit reads it, replaced atomically."""
        now = time.monotonic()
        with self.lock:
            lx, ly, az = self.target
            if (now - self.last_cmd_time) > self.timeout_s:
                lx = ly = az = 0.0
            measured = self.measured
            odom_age = None if self.odom_time is None else now - self.odom_time
            # A command nobody receives is not being held: the base cannot follow what
            # never reaches it, so the spin-up clock starts once a reader has matched.
            subscribers = self.pub.get_subscription_count()
            moving = abs(lx) + abs(ly) + abs(az) > 0 and subscribers > 0
            if not moving:
                self.held_since = None
            elif self.held_since is None:
                self.held_since = now
            held = 0.0 if self.held_since is None else now - self.held_since
        v = stall_check.verdict((lx, ly, az), measured, held, odom_age, subscribers)
        status = {"t": time.time(), **v,
                  "commanded": {"linear_x": lx, "linear_y": ly, "angular_z": az},
                  "measured": {"linear_x": measured[0], "linear_y": measured[1], "angular_z": measured[2]},
                  "odom_age_s": odom_age, "held_s": held, "subscribers": subscribers}
        tmp = self.status_file + ".tmp"
        try:
            with open(tmp, "w") as fh:
                json.dump(status, fh)
            os.replace(tmp, self.status_file)
        except OSError:
            pass

    def tick(self):
        with self.lock:
            lx, ly, az = self.target
            stale = (time.monotonic() - self.last_cmd_time) > self.timeout_s
        if stale:
            lx = ly = az = 0.0
        self.pub.publish(self._msg(lx, ly, az))

    def set_target(self, lx, ly, az):
        with self.lock:
            self.target = (lx, ly, az)
            self.last_cmd_time = time.monotonic()

    def stop(self):
        with self.lock:
            self.target = (0.0, 0.0, 0.0)


def read_stdin(node):
    for line in sys.stdin:
        parts = line.split()
        if not parts:
            continue
        if parts[0] == "stop":
            node.stop()
            continue
        try:
            lx, ly, az = (float(p) for p in parts[:3])
        except ValueError:
            continue
        node.set_target(lx, ly, az)
    # stdin closed: the browser is gone, so stop the robot
    node.stop()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--topic", default="/cmd_vel")
    ap.add_argument("--rate", type=float, default=20.0)
    ap.add_argument("--timeout", type=float, default=0.5)
    ap.add_argument("--odom-topic", default="/odom", help="what the stall check measures against")
    ap.add_argument("--status-file", default=None, help="where the stall verdict is written (the cockpit reads it)")
    ap.add_argument("--cmd-vel-type", default="auto",
                    choices=("auto", "twist", "twist_stamped"),
                    help="message type for --topic; auto asks ROS_DISTRO, the same "
                         "question that decided USE_STAMPED_CMD_VEL in the firmware")
    args = ap.parse_args()

    rclpy.init()
    node = GamepadPublisher(args.topic, args.rate, args.timeout,
                            stamped=resolve_cmd_vel_type(args.cmd_vel_type),
                            odom_topic=args.odom_topic, status_file=args.status_file)
    threading.Thread(target=read_stdin, args=(node,), daemon=True).start()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # leave the base stopped, not coasting on the last command
        node.pub.publish(node._msg(0.0, 0.0, 0.0))
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
