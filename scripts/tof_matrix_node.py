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
        else:
            raise SystemExit(f"tof_matrix_node: connection must be usb or i2c, got {self.connection!r}")
        self.get_logger().info(
            f"8x8 ToF over {self.connection} -> {g('points_topic')} in {self.frame_id} "
            f"(rotation {self.rotation}, mirror {self.mirror}, {self.min_range}-{self.max_range} m)")

    # -- sources ---------------------------------------------------------------
    def _on_raw(self, msg):
        try:
            frame = tm.decode_raw(msg.data)
        except ValueError as e:
            self.get_logger().warn(f"tof/raw: {e}", throttle_duration_sec=5.0)
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
                self.get_logger().warn(f"{self.port}: {e}; retrying in 2 s", throttle_duration_sec=10.0)
                time.sleep(2.0)

    # -- output ----------------------------------------------------------------
    def _publish(self, frame):
        pts = tm.to_points(frame, self.rotation, self.mirror, self.min_range, self.max_range)
        fields, step, data = tm.pack_pointcloud2_xyz(pts)
        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
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
        self.frames += 1
        self.last_frame = time.monotonic()
        if self._warned_stale:
            self.get_logger().info("8x8 ToF frames are back")
            self._warned_stale = False

    def _watch(self):
        if time.monotonic() - self.last_frame > self.stale_warn_s and not self._warned_stale:
            self.get_logger().warn(
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
