#!/usr/bin/env python3
# ==============================================================================
# fake_base_node.py — the robot's base, simulated on the robot computer
#
# The other half of scripts/fake_laser_node.py. That one already raycasts the
# 10x6 m room and publishes /scan from /odom; this publishes the /odom, so the
# whole stack -- SLAM, Nav2, the map -- runs with no microcontroller, no USB
# cable and no micro-ROS agent.
#
# Why: every result this project has produced needed a board. The Nav2 matrix is
# three parallel streams because there are four boards on two subnets, a leg
# takes eight minutes, and a SLAM or Nav2 regression cannot be reproduced in CI
# at all. This runs anywhere, in seconds, and is deterministic.
#
# It is NOT a replacement for the board legs. What it removes -- micro-ROS, the
# serial or Wi-Fi transport, the board's timing, its 4 KB env partition -- is a
# large part of what those thirty legs exist to test. It is a diagnostic
# instrument and a CI leg, and the gate stays on hardware.
#
# THE MODEL IS NOT REIMPLEMENTED HERE. scripts/drivetrain_report.py already
# carries a transcription of FakeEncoder::integrate(), the shared battery, and
# PID::compute() with its anti-windup, all checked against the firmware by
# tests/test_test_acc_simulation.py and tests/test_pid_auto_tune.py. This node
# imports them. A second copy of the wheel model is the one thing that would
# make this instrument lie.
#
#   ros2 run ... fake_base_node.py --ros-args -p params:=<robot>_config.yaml
# ==============================================================================
import math
import os
import sys

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO_ROOT, "scripts"))

import yaml

import drivetrain_report as dr

# The ROS 2 stack is imported but NOT required to import this module.
#
# Everything that decides how the simulated robot moves -- target_rpm() and
# Wheels -- is plain Python below, and it is the part worth testing. Exiting at
# import time would make it reachable only from a sourced ROS environment, so
# the hermetic suite could not check the transcription against the firmware,
# which is the one thing that keeps this instrument honest. The failure is
# reported where it actually matters: main().
ROS_IMPORT_ERROR = None
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
    from geometry_msgs.msg import Twist, TwistStamped, TransformStamped
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu
    from tf2_ros import TransformBroadcaster
except ImportError as exc:                                  # pragma: no cover
    # Name the module that actually failed (AGENTS.md 12): rclpy is the import
    # that rarely fails, and a generic "rclpy not found" sends the reader to
    # source a setup.bash they already sourced.
    ROS_IMPORT_ERROR = exc
    Node = object


def _quat_from_yaw(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class Wheels:
    """The four simulated wheels, their PIDs, and the pack they share.

    Every piece is drivetrain_report's, which is the firmware's.
    """

    def __init__(self, d, pid):
        self.d = d
        self.pack = dr._Pack(d)
        self.wheels = [dr._Wheel(d, self.pack, i) for i in range(4)]
        self.kp = float(pid.get("kp", 0.6))
        self.ki = float(pid.get("ki", 0.8))
        self.kd = float(pid.get("kd", 0.5))
        self.integral = [0.0] * 4
        self.prev_error = [0.0] * 4

    def step(self, target_rpm, dt):
        """One control cycle: PID per wheel, then advance the plant."""
        pwm_max = self.d["pwm_max"]
        for i, wheel in enumerate(self.wheels):
            setpoint = target_rpm[i]
            error = setpoint - wheel.rpm
            self.integral[i] += error
            derivative = error - self.prev_error[i]
            if self.ki != 0.0:
                i_max = pwm_max / abs(self.ki)
                self.integral[i] = min(max(self.integral[i], -i_max), i_max)
            if setpoint == 0.0 and abs(error) < 0.5:
                self.integral[i] = 0.0
                derivative = 0.0
            u = self.kp * error + self.ki * self.integral[i] + self.kd * derivative
            u = min(max(u, -pwm_max), pwm_max)
            self.prev_error[i] = error
            wheel.duty = u / pwm_max
            # Only the first wheel advances the shared pack's filter, exactly as
            # on the board, where the other three call busScale() in the same
            # microsecond and its dt is zero.
            wheel.step(dt, i == 0)
        return [w.rpm for w in self.wheels]


def target_rpm(d, vx, vy, wz):
    """Kinematics::calculateRPM(), including the peak-based scaler.

    Transcribed rather than approximated because the scaler is the interesting
    part: an unreachable command comes out as the SAME motion more slowly, and
    clipping individual wheels instead would change the RATIO between them --
    which for a mecanum is the direction.
    """
    if d["base"] != "mecanum":
        vy = 0.0
    circ = d["circ"]
    if circ <= 0:
        return [0.0] * 4
    x_rpm = vx * 60.0 / circ
    y_rpm = vy * 60.0 / circ
    tan_rpm = (wz * d["radius"]) * 60.0 / circ

    combos = [x_rpm - y_rpm - tan_rpm, x_rpm + y_rpm + tan_rpm,
              x_rpm + y_rpm - tan_rpm, x_rpm - y_rpm + tan_rpm]
    peak = max(abs(w) for w in combos)
    max_rpm = d["command_rpm"]
    if peak > max_rpm > 0.0:
        scale = max_rpm / peak
        combos = [w * scale for w in combos]
    return [min(max(w, -max_rpm), max_rpm) for w in combos]


class FakeBaseNode(Node):
    def __init__(self):
        super().__init__("fake_base_node")
        self.declare_parameter("params", "")
        self.declare_parameter("stamped_cmd_vel", False)
        self.declare_parameter("rate", 50.0)          # CONTROL_TIMER, 50 Hz
        self.declare_parameter("publish_tf", True)

        path = str(self.get_parameter("params").value)
        if not path or not os.path.isfile(path):
            raise SystemExit("fake_base_node: -p params:=<robot>_config.yaml is required "
                             f"(got {path!r})")
        with open(path, encoding="utf-8") as fh:
            params = yaml.safe_load(fh) or {}
        self.d = dr.drivetrain(params)
        if self.d["circ"] <= 0 or self.d["max_rpm"] <= 0:
            raise SystemExit("fake_base_node: kinematics.wheel_diameter and max_rpm "
                             "must be positive")
        self.wheels = Wheels(self.d, (params.get("kinematics") or {}).get("pid") or {})

        # No magnetometer here, so this base publishes imu/data itself and no
        # madgwick runs -- the rule bringup.launch.py follows for a real board
        # with no mag fitted, applied to the simulated one so the two stacks are
        # the same shape.
        sensor_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                history=HistoryPolicy.KEEP_LAST, depth=10,
                                durability=DurabilityPolicy.VOLATILE)
        self.odom_pub = self.create_publisher(Odometry, "odom/unfiltered", sensor_qos)
        self.imu_pub = self.create_publisher(Imu, "imu/data", sensor_qos)
        self.tf = TransformBroadcaster(self) if \
            bool(self.get_parameter("publish_tf").value) else None

        if bool(self.get_parameter("stamped_cmd_vel").value):
            self.create_subscription(TwistStamped, "cmd_vel", self._cmd_stamped, 10)
        else:
            self.create_subscription(Twist, "cmd_vel", self._cmd, 10)

        self.cmd = (0.0, 0.0, 0.0)
        self.cmd_time = self.get_clock().now()
        self.x = self.y = self.yaw = 0.0
        self.prev_vx = 0.0
        rate = float(self.get_parameter("rate").value)
        self.dt = 1.0 / rate
        self.create_timer(self.dt, self._tick)
        self.get_logger().info(
            f"fake base: {self.d['base']}, {self.d['wheels']} wheels, "
            f"{self.d['mass']:.1f} kg, turns on {self.d['radius']:.4f} m, "
            f"budget {self.d['command_rpm']:.0f} rpm, at {rate:.0f} Hz")

    def _cmd(self, msg):
        self.cmd = (msg.linear.x, msg.linear.y, msg.angular.z)
        self.cmd_time = self.get_clock().now()

    def _cmd_stamped(self, msg):
        self.cmd = (msg.twist.linear.x, msg.twist.linear.y, msg.twist.angular.z)
        self.cmd_time = self.get_clock().now()

    def _tick(self):
        now = self.get_clock().now()
        # The firmware's 200 ms command timeout. Without it a node that stops
        # publishing leaves the simulated robot driving for ever, which is a
        # failure mode the board does not have and would send someone hunting a
        # controller bug that is not there.
        if (now - self.cmd_time).nanoseconds > 200_000_000:
            self.cmd = (0.0, 0.0, 0.0)

        vx, vy, wz = self.cmd
        rpm = self.wheels.step(target_rpm(self.d, vx, vy, wz), self.dt)
        # Read the wheels BACK through the kinematics, never integrate the
        # command: the same radius on both sides is what made a wrong mecanum
        # turn radius invisible on the bench for months.
        meas_x, meas_wz = dr._velocities(self.d, rpm)
        meas_y = 0.0
        if self.d["base"] == "mecanum":
            r1, r2, r3, r4 = rpm
            meas_y = ((-r1 + r2 + r3 - r4) / float(self.d["wheels"]) / 60.0) * self.d["circ"]

        self.yaw += meas_wz * self.dt
        cos_h, sin_h = math.cos(self.yaw), math.sin(self.yaw)
        self.x += (meas_x * cos_h - meas_y * sin_h) * self.dt
        self.y += (meas_x * sin_h + meas_y * cos_h) * self.dt

        stamp = now.to_msg()
        qx, qy, qz, qw = _quat_from_yaw(self.yaw)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = meas_x
        odom.twist.twist.linear.y = meas_y
        odom.twist.twist.angular.z = meas_wz
        self.odom_pub.publish(odom)

        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = "imu_link"
        imu.orientation.x, imu.orientation.y = qx, qy
        imu.orientation.z, imu.orientation.w = qz, qw
        imu.angular_velocity.z = meas_wz
        imu.linear_acceleration.x = (meas_x - self.prev_vx) / self.dt
        imu.linear_acceleration.z = 9.81
        self.imu_pub.publish(imu)
        self.prev_vx = meas_x

        if self.tf is not None:
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = "odom"
            t.child_frame_id = "base_link"
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.rotation.x = qx
            t.transform.rotation.y = qy
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            self.tf.sendTransform(t)


def main():
    if ROS_IMPORT_ERROR is not None:
        print(f"Error: cannot import the ROS 2 Python stack: {ROS_IMPORT_ERROR}",
              file=sys.stderr)
        print(f"  interpreter: {sys.executable}", file=sys.stderr)
        return 1
    rclpy.init()
    node = FakeBaseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main() or 0)
