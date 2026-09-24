#!/usr/bin/env python3
# ==============================================================================
# sim_laser_node.py — Virtual 10x6m Room LaserScan Publisher
#
# Raycasts a virtual 10x6m room with obstacle wall, matching sim_ld19.h geometry.
# Enables bare MCU boards (Pico 2, ESP32) with 0 wiring to run full SLAM and Nav2.
# ==============================================================================

import math
import random
import sys
import time

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan
except ImportError as exc:
    # Name the module that actually failed, per AGENTS.md 12: this block used to
    # report "rclpy not found" for every import error in the group, and rclpy is
    # the one import that rarely fails -- it sits on PYTHONPATH and imports even
    # from a venv that hides the system dist-packages, where the real casualty is
    # numpy (pulled in several frames down by rclpy.node -> rosgraph_msgs). The
    # generic message sends the reader to source a setup.bash that was already
    # sourced. Print the exception and the interpreter; both are the diagnosis.
    print(f"Error: cannot import the ROS 2 Python stack: {exc}", file=sys.stderr)
    print(f"  interpreter: {sys.executable}", file=sys.stderr)
    print("  Check that ROS 2 setup.bash is sourced AND that this interpreter can",
          file=sys.stderr)
    print("  see the system dist-packages (a venv with include-system-site-packages",
          file=sys.stderr)
    print("  = false on PATH ahead of /usr/bin will hide numpy).", file=sys.stderr)
    sys.exit(1)


# Room geometry (identical to sim_ld19.h)
ROOM_MIN_X = -5.0
ROOM_MAX_X =  5.0
ROOM_MIN_Y = -3.0
ROOM_MAX_Y =  3.0

# Obstacle wall
WALL_X1 =  2.0
WALL_Y1 = -1.5
WALL_X2 =  2.0
WALL_Y2 =  1.5

SEGMENTS = [
    # Perimeter
    (ROOM_MIN_X, ROOM_MIN_Y, ROOM_MAX_X, ROOM_MIN_Y),
    (ROOM_MAX_X, ROOM_MIN_Y, ROOM_MAX_X, ROOM_MAX_Y),
    (ROOM_MAX_X, ROOM_MAX_Y, ROOM_MIN_X, ROOM_MAX_Y),
    (ROOM_MIN_X, ROOM_MAX_Y, ROOM_MIN_X, ROOM_MIN_Y),
    # Interior Obstacle
    (WALL_X1, WALL_Y1, WALL_X2, WALL_Y2),
]


class SimLaserNode(Node):
    def __init__(self):
        super().__init__("sim_laser_node")
        # Where the LiDAR sits and what its frame is called come from the robot
        # config (geometry.laser), handed over by bringup.launch.py, so the
        # synthetic scan is taken from the same place the TF tree says it is.
        self.declare_parameter("frame_id", "laser")
        self.declare_parameter("offset_x", 0.0)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.offset_x = float(self.get_parameter("offset_x").value)
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.pose_yaw = 0.0

        # QoS configuration
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
        )

        # Relative topic names, so a namespaced launch (multi-robot topic_prefix)
        # puts these under /<prefix>/. With no namespace they resolve to /scan
        # and /odom exactly as before -- an absolute "/scan" would have ignored
        # the namespace and published at the root, colliding between robots.
        self.scan_pub = self.create_publisher(LaserScan, "scan", sensor_qos)
        self.create_subscription(Odometry, "odom", self._odom_cb, 10)
        self.timer = self.create_timer(0.1, self._publish_scan)  # 10 Hz

        self.num_points = 456
        self.angle_min = -math.pi
        self.angle_max = math.pi
        self.angle_step = (self.angle_max - self.angle_min) / self.num_points

        self.get_logger().info("Simulated LD19 LaserScan Node active (10 Hz, 10x6m virtual room).")

    def _odom_cb(self, msg: Odometry):
        self.pose_x = msg.pose.pose.position.x
        self.pose_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.pose_yaw = math.atan2(siny_cosp, cosy_cosp)

    def _publish_scan(self):
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.angle_min = self.angle_min
        msg.angle_max = self.angle_max
        msg.angle_increment = self.angle_step
        msg.time_increment = 0.0
        msg.scan_time = 0.1
        msg.range_min = 0.05
        msg.range_max = 12.0

        ranges = []
        intensities = []

        # Sensor position forward of base footprint (geometry.laser.x)
        sensor_ox = self.pose_x + self.offset_x * math.cos(self.pose_yaw)
        sensor_oy = self.pose_y + self.offset_x * math.sin(self.pose_yaw)

        for i in range(self.num_points):
            ray_angle = self.pose_yaw + self.angle_min + i * self.angle_step
            cos_a = math.cos(ray_angle)
            sin_a = math.sin(ray_angle)

            min_dist = 12.0
            for (x1, y1, x2, y2) in SEGMENTS:
                sx = x2 - x1
                sy = y2 - y1
                denom = cos_a * sy - sin_a * sx
                if abs(denom) < 1e-6:
                    continue

                px = x1 - sensor_ox
                py = y1 - sensor_oy

                t = (px * sy - py * sx) / denom
                u = (px * sin_a - py * cos_a) / denom

                if t > 0.03 and 0.0 <= u <= 1.0:
                    if t < min_dist:
                        min_dist = t

            if min_dist >= 12.0:
                ranges.append(float("inf"))
                intensities.append(0.0)
            else:
                # Add +-5mm noise
                noise = (random.random() - 0.5) * 0.01
                dist = max(0.05, min_dist + noise)
                ranges.append(dist)
                intensities.append(200.0)

        msg.ranges = ranges
        msg.intensities = intensities
        self.scan_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SimLaserNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
