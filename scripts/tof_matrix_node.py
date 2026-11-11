#!/usr/bin/env python3
# ==============================================================================
# tof_matrix_node.py — the 8x8 ToF (DFRobot SEN0628) as a PointCloud2
#
# One node for both connections (base_controller.tof.connection):
#   usb  the board's USB port, read here (it streams text frames at power-on)
#   i2c  the MCU reads it and publishes tof/raw (128 bytes, 64 x uint16 mm); this
#        node only converts
# and publishes tof/points (PointCloud2, x/y/z float32) in tof_link for the local
# costmap and the collision monitor (nav2.launch.py adds them). Every frame is
# published, an empty cloud when no zone sees anything: the collision monitor
# treats a silent source as unsafe and stops the robot, so "nothing ahead" must
# never look like "no sensor". The geometry is in tof_matrix.py.
# ==============================================================================

import os
import sys
import threading
import time

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import PointCloud2, PointField
    from std_msgs.msg import UInt8MultiArray
except ImportError as exc:
    print(f"Error: cannot import the ROS 2 Python stack: {exc}", file=sys.stderr)
    print(f"  interpreter: {sys.executable}", file=sys.stderr)
    sys.exit(1)

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import tof_matrix as tm  # noqa: E402


class TofMatrixNode(Node):
    def __init__(self):
        super().__init__("tof_matrix_node")
        p = self.declare_parameter
        p("connection", "usb")               # usb | i2c
        p("port", "")                         # usb: /dev/serial/by-id/... of the board
        p("raw_topic", "tof/raw")             # i2c: the MCU's frames
        p("points_topic", "tof/points")
        p("frame_id", "tof_link")
        p("rotation", 0)                      # 0/90/180/270: how the board is turned on its mount
        p("mirror", False)
        p("min_range", tm.MIN_RANGE_M)
        p("max_range", tm.MAX_RANGE_M)
        p("stale_warn_s", 1.0)
        # The robot's /scan when it has no LiDAR and no depth camera (depth_camera.scan_source
        # "tof"): bringup sets scan_topic; empty publishes no scan.
        p("scan_topic", "")
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.connection = str(g("connection")).lower()
        self.frame_id = str(g("frame_id"))
        self.rotation = int(g("rotation"))
        self.mirror = bool(g("mirror"))
        self.min_range = float(g("min_range"))
        self.max_range = float(g("max_range"))
        self.stale_warn_s = float(g("stale_warn_s"))
        tm.orient([0] * 64, self.rotation)   # refuse a bad rotation at start, not per frame

        self.pub = self.create_publisher(PointCloud2, str(g("points_topic")), qos_profile_sensor_data)
        self.scan_pub = None
        if str(g("scan_topic")):
            from sensor_msgs.msg import LaserScan
            self._LaserScan = LaserScan
            self.scan_pub = self.create_publisher(LaserScan, str(g("scan_topic")), qos_profile_sensor_data)
        self.frames = 0
        self.last_frame = time.monotonic()
        self._warned_stale = False
        self.create_timer(1.0, self._watch)

        if self.connection == "usb":
            self.port = str(g("port"))
            if not self.port:
                raise SystemExit("tof_matrix_node: connection usb needs a port "
                                 "(base_controller.tof.usb_port)")
            self._stop = threading.Event()
            threading.Thread(target=self._usb_loop, daemon=True).start()
        elif self.connection == "i2c":
            self.create_subscription(UInt8MultiArray, str(g("raw_topic")), self._on_raw,
                                     qos_profile_sensor_data)
        elif self.connection == "sim":
            self._init_sim()
        else:
            raise SystemExit(f"tof_matrix_node: connection must be usb, i2c or sim, got {self.connection!r}")
        self.get_logger().info(
            f"8x8 ToF over {self.connection} -> {g('points_topic')} in {self.frame_id} "
            f"(rotation {self.rotation}, mirror {self.mirror}, {self.min_range}-{self.max_range} m)")

    # -- sources ---------------------------------------------------------------
    def _init_sim(self):
        """Simulation mode: render the room from the robot's pose, like sim_depth_node.py.

        The same parameters the simulated camera gets from bringup.launch.py (the room,
        interior walls, or a saved map as the world, and where the sensor sits on the
        robot), so the ToF sees the walls the simulated LiDAR and camera see.
        """
        import numpy as np
        from nav_msgs.msg import Odometry
        import depth_camera as dc
        p = self.declare_parameter
        for k in ("offset_x", "offset_y", "offset_yaw"):
            p(k, 0.0)
        for k, v in dc.ROOM_DEFAULTS.items():
            p(k, v)
        p("walls", [0.0])
        p("world_map", "")
        p("world_start", [0.0, 0.0, 0.0])
        p("pose_topic", "odom/unfiltered")
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.off = (float(g("offset_x")), float(g("offset_y")), float(g("offset_yaw")))
        room = {k: g(k) for k in dc.ROOM_DEFAULTS}
        self.segments = dc.room_segments(room, dc.walls_from_flat(g("walls")))
        if str(g("world_map") or ""):
            self.segments = dc.GridWorld(str(g("world_map")), tuple(g("world_start")))
        self.cam = tm.sim_optics()
        self.angles = dc.column_angles(self.cam)
        self.rng = np.random.default_rng()
        self.pose = (0.0, 0.0, 0.0)
        self.pose_stamp = None
        self._dc = dc
        self.create_subscription(Odometry, str(g("pose_topic")), self._on_odom, qos_profile_sensor_data)
        self.create_timer(1.0 / self.cam.rate_hz, self._sim_tick)

    def _on_odom(self, msg):
        import math
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)
        self.pose_stamp = msg.header.stamp

    def _sim_tick(self):
        import math
        x, y, yaw = self.pose
        ox_, oy_, oyaw = self.off
        ox = x + ox_ * math.cos(yaw) - oy_ * math.sin(yaw)
        oy = y + ox_ * math.sin(yaw) + oy_ * math.cos(yaw)
        row = self._dc.depth_row(ox, oy, yaw + oyaw, self.segments, self.angles, self.rng, self.cam)
        # The frame shows the world from the pose it was rendered at: stamp it with that
        # pose's time, as the simulated camera does (sim_depth_node.py).
        self._publish(tm.sim_frame(list(row)), stamp=self.pose_stamp, oriented=True)

    def _on_raw(self, msg):
        try:
            frame = tm.decode_raw(msg.data)
        except ValueError as e:
            self.get_logger().warning(f"tof/raw: {e}", throttle_duration_sec=5.0)
            return
        self._publish(frame)

    def _usb_loop(self):
        import serial  # python3-serial: in the robot image
        parser = tm.FrameParser()
        while not self._stop.is_set():
            try:
                with serial.Serial(self.port, 115200, timeout=0.2) as s:
                    self.get_logger().info(f"reading {self.port}")
                    while not self._stop.is_set():
                        chunk = s.read(512)
                        for frame in parser.feed(chunk):
                            self._publish(frame)
            except (OSError, serial.SerialException) as e:
                self.get_logger().warning(f"{self.port}: {e}; retrying in 2 s", throttle_duration_sec=10.0)
                time.sleep(2.0)

    # -- output ----------------------------------------------------------------
    def _publish(self, frame, stamp=None, oriented=False):
        # A simulated frame is rendered already in view order (row 0 top, column 0 left):
        # the mount's rotation/mirror describe the real board only.
        rot, mir = (0, False) if oriented else (self.rotation, self.mirror)
        pts = tm.to_points(frame, rot, mir, self.min_range, self.max_range)
        fields, step, data = tm.pack_pointcloud2_xyz(pts)
        msg = PointCloud2()
        msg.header.stamp = stamp or self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.height = 1
        msg.width = len(pts)
        msg.fields = [PointField(name=n, offset=o, datatype=t, count=c) for n, o, t, c in fields]
        msg.is_bigendian = False
        msg.point_step = step
        msg.row_step = step * len(pts)
        msg.data = data
        msg.is_dense = True
        self.pub.publish(msg)
        if self.scan_pub is not None:
            scan = self._LaserScan()
            scan.header = msg.header
            scan.angle_min, scan.angle_increment = tm.scan_angles()
            scan.angle_max = scan.angle_min + scan.angle_increment * (tm.ZONES - 1)
            scan.scan_time = 1.0 / 15.0
            scan.time_increment = 0.0
            scan.range_min = float(self.min_range)
            scan.range_max = float(self.max_range)
            scan.ranges = [float(r) for r in tm.to_scan_ranges(frame, rot, mir, self.min_range, self.max_range)]
            self.scan_pub.publish(scan)
        self.frames += 1
        self.last_frame = time.monotonic()
        if self._warned_stale:
            self.get_logger().info("8x8 ToF frames are back")
            self._warned_stale = False

    def _watch(self):
        if time.monotonic() - self.last_frame > self.stale_warn_s and not self._warned_stale:
            self.get_logger().warning(
                f"no 8x8 ToF frame for {self.stale_warn_s:.1f} s ({self.connection}); "
                "the collision monitor stops the robot while its source is silent")
            self._warned_stale = True


def main():
    rclpy.init()
    node = TofMatrixNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
