#!/usr/bin/env python3
"""The simulated depth camera: a typical stereo depth camera looking at the simulated room.

There is no depth camera on the bench, so this is what the depth path runs on:
it publishes what the camera's driver would -- a depth image and its
camera_info -- and the REAL depthimage_to_laserscan turns it into /scan, so
the converter, its frame and SLAM/Nav2 on a narrow scan are all in the loop.

It sees the same room as the LiDAR. The walls come from base_controller.simulation
through depth_camera.sim_room(), the keys mcu_env.py hands the firmware's
sim_ld19.h, and the pose is odom/unfiltered: the firmware's own simulated pose,
which is what sim_ld19.h raycasts from. So the camera, the board's LD19 and the
map SLAM draws describe one room.

The camera is a typical one, per the project's rule for simulated sensors: an
Intel RealSense D435 in its 424x240 depth mode, 87 x 58 degrees, 0.2 m minimum
Z, 10 m maximum, 15 fps. Its depth noise is the stereo formula
    sigma_z = z^2 * subpixel / (f * baseline)
with the D435's 50 mm baseline, 0.08 subpixel and f = 212 / tan(43.5 deg) =
223 px at this width: 0.0072 * z^2 m, 7 mm at 1 m and 29 mm at 2 m.

The walls are vertical and the room has no floor or ceiling in the model, so a
column's depth is the same in every row; the image is one raycast per column,
broadcast down. The converter reads the centre rows, where that is exact.
"""
import math
import sys

try:
    import numpy as np
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import CameraInfo, Image
    from geometry_msgs.msg import TransformStamped
    from tf2_ros import StaticTransformBroadcaster
except ImportError as exc:
    print(f"Error: cannot import the ROS 2 Python stack: {exc}", file=sys.stderr)
    print(f"  interpreter: {sys.executable}", file=sys.stderr)
    sys.exit(1)

import depth_camera as dc

from depth_camera import (WIDTH, HEIGHT, HFOV, VFOV, FX, FY, CX, CY,  # noqa: E402,F401
                          RATE_HZ, column_angles, depth_row)


class SimDepthNode(Node):
    def __init__(self):
        super().__init__("sim_depth_node")
        self.declare_parameter("frame_id", dc.SIM_FRAME)
        for k in ("offset_x", "offset_y", "offset_yaw"):
            self.declare_parameter(k, 0.0)
        for k, v in dc.ROOM_DEFAULTS.items():
            self.declare_parameter(k, v)
        self.declare_parameter("pose_topic", "odom/unfiltered")
        g = lambda k: self.get_parameter(k).value  # noqa: E731
        self.frame = str(g("frame_id"))
        self.optical = f"{self.frame}_sim_depth_optical_frame"
        self.off_x, self.off_y, self.off_yaw = float(g("offset_x")), float(g("offset_y")), float(g("offset_yaw"))
        room = {k: g(k) for k in dc.ROOM_DEFAULTS}
        self.declare_parameter("walls", [0.0])     # interior walls, x1,y1,x2,y2 end to end
        self.segments = dc.room_segments(room, dc.walls_from_flat(self.get_parameter("walls").value))
        # World "map": a saved occupancy map is the world instead (depth_camera.GridWorld).
        self.declare_parameter("world_map", "")
        self.declare_parameter("world_start", [0.0, 0.0, 0.0])
        wm = str(self.get_parameter("world_map").value or "")
        if wm:
            self.segments = dc.GridWorld(wm, tuple(self.get_parameter("world_start").value))
            self.get_logger().info(f"world: the saved map {wm}, start {tuple(self.get_parameter('world_start').value)}")
        self.angles = column_angles()
        self.rng = np.random.default_rng()
        self.pose = (0.0, 0.0, 0.0)

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST,
                         depth=5, durability=DurabilityPolicy.VOLATILE)
        # Relative names: a namespaced (topic_prefix) launch puts them under /<prefix>/.
        self.img_pub = self.create_publisher(Image, dc.SIM_DEPTH_TOPIC.lstrip("/"), qos)
        self.info_pub = self.create_publisher(CameraInfo, dc.SIM_INFO_TOPIC.lstrip("/"), qos)
        self.create_subscription(Odometry, str(g("pose_topic")), self._odom, qos)

        # The optical frame hangs off the camera frame the URDF places: z forward,
        # x right, y down. A real driver publishes its own; this is ours.
        self.static = StaticTransformBroadcaster(self)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.frame
        t.child_frame_id = self.optical
        # rpy (-pi/2, 0, -pi/2) as a quaternion
        t.transform.rotation.x, t.transform.rotation.y = -0.5, 0.5
        t.transform.rotation.z, t.transform.rotation.w = -0.5, 0.5
        self.static.sendTransform(t)

        self.create_timer(1.0 / RATE_HZ, self._tick)
        self.get_logger().info(
            f"Simulated depth camera: D435 {WIDTH}x{HEIGHT} @ {RATE_HZ:.0f} Hz, "
            f"{math.degrees(HFOV):.0f}x{math.degrees(VFOV):.0f} deg, room "
            f"{room['map_width']}x{room['map_height']} m, obstacle {'on' if room['wall_obstacle'] else 'off'}.")

    def _odom(self, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)
        self.pose_stamp = msg.header.stamp

    def _tick(self):
        x, y, yaw = self.pose
        ox = x + self.off_x * math.cos(yaw) - self.off_y * math.sin(yaw)
        oy = y + self.off_x * math.sin(yaw) + self.off_y * math.cos(yaw)
        row = depth_row(ox, oy, yaw + self.off_yaw, self.segments, self.angles, self.rng)
        mm = np.clip(np.rint(row * 1000.0), 0, 65535).astype(np.uint16)
        # The image shows the world from the pose it was rendered at, so it
        # carries that pose's time -- as a real camera stamps the exposure, not
        # the moment the driver publishes. Stamped "now" it was newer than any
        # transform yet published: slam_toolbox (scan_queue_size 1) dropped
        # every scan waiting for one, and a camera-only robot never grew its
        # map past the first view (measured 2026-09-26, jazzy: 0 SLAM updates
        # over a 1 m drive).
        stamp = getattr(self, "pose_stamp", None) or self.get_clock().now().to_msg()

        img = Image()
        img.header.stamp, img.header.frame_id = stamp, self.optical
        img.height, img.width = HEIGHT, WIDTH
        img.encoding = "16UC1"                 # millimetres, as a RealSense publishes
        img.is_bigendian = 0
        img.step = WIDTH * 2
        img.data = np.tile(mm, (HEIGHT, 1)).tobytes()
        self.img_pub.publish(img)

        info = CameraInfo()
        info.header = img.header
        info.height, info.width = HEIGHT, WIDTH
        info.distortion_model = "plumb_bob"
        info.d = [0.0] * 5
        info.k = [FX, 0.0, CX, 0.0, FY, CY, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [FX, 0.0, CX, 0.0, 0.0, FY, CY, 0.0, 0.0, 0.0, 1.0, 0.0]
        self.info_pub.publish(info)


def main(args=None):
    rclpy.init(args=args)
    node = SimDepthNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
