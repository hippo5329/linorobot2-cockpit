#!/usr/bin/env python3
# ==============================================================================
# test_nav2_goal.py — Nav2 Navigation Verification Behind Obstacle Wall
#
# Directives Compliance:
# - Validates Nav2 planning and execution in virtual room with obstacle wall.
# - Obstacle wall geometry: x = 2.0m, y from -1.5m to +1.5m (fake_ld19.h).
# - Goal behind obstacle wall: (x=3.0m, y=0.0m).
# - Verifies:
#   1. /navigate_to_pose action server availability.
#   2. Planned path (/plan) circumvents the obstacle wall (|y| > 1.3m near x=2.0m).
#   3. Active /cmd_vel velocities issued by controller to execute path.
#   4. The base ACTUALLY MOVED in response, measured on /odom.
#
# (4) used to say "tracked", and that is all it did: the odom delta was computed,
# formatted into the success message, and never compared to anything. The pass
# condition was (2) and (3) alone -- a plan from the planner and a count of the
# controller's own publications, both entirely host-side. An unplugged board
# satisfies both, and so does a board whose firmware subscribes to a /cmd_vel
# type the controller does not publish. That is not hypothetical: every -lyrical
# prebuilt image in rc-20260919 was built without USE_STAMPED_CMD_VEL while nav2
# on lyrical publishes TwistStamped, and this test passed all of them.
# ==============================================================================

import argparse
import math
import sys
import time
from typing import List, Optional

try:
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from action_msgs.msg import GoalStatus
    from geometry_msgs.msg import PoseStamped, Twist, TwistStamped
    from nav_msgs.msg import Path, Odometry
    from nav2_msgs.action import NavigateToPose
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


def _yaw(q) -> float:
    """Yaw from a quaternion. Enough for "did this thing turn at all"."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Nav2GoalTester(Node):
    def __init__(self, goal_x: float = 3.0, goal_y: float = 0.0, timeout_sec: float = 30.0,
                 cmd_vel_type: str = "auto"):
        super().__init__("nav2_goal_tester")
        self.goal_x = goal_x
        self.goal_y = goal_y
        self.timeout_sec = timeout_sec

        self.action_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        
        self.path_received: Optional[Path] = None
        self.path_avoids_wall: bool = False
        self.cmd_vel_count: int = 0
        self.cmd_vel_stamped_count: int = 0
        self.initial_odom: Optional[Odometry] = None
        self.latest_odom: Optional[Odometry] = None
        self.odom_max_dist: float = 0.0
        self.odom_max_yaw: float = 0.0
        self.cmd_peak_lin: float = 0.0
        self.cmd_peak_ang: float = 0.0
        self.odom_peak_lin: float = 0.0
        self.odom_peak_ang: float = 0.0
        self.goal_accepted: bool = False
        self.goal_completed: bool = False

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.create_subscription(Path, "/plan", self._plan_cb, reliable_qos)

        # /cmd_vel carries exactly ONE type per run -- Twist on Jazzy, TwistStamped on
        # Lyrical (and on Jazzy when the config asks for it). Subscribing to both in
        # one node is not a way to accept either: rcl rejects the second with
        # "create_subscription() called for existing topic name rt/cmd_vel with
        # incompatible type", and the RCLError is raised in this constructor -- so the
        # test died before sending a goal, on every run, exiting 1 with an empty
        # stdout that read like a navigation failure.
        self.cmd_vel_type = self._resolve_cmd_vel_type(cmd_vel_type)
        if self.cmd_vel_type == "twist_stamped":
            self.create_subscription(TwistStamped, "/cmd_vel", self._cmd_vel_stamped_cb, 10)
        else:
            self.create_subscription(Twist, "/cmd_vel", self._cmd_vel_cb, 10)
        self.create_subscription(Odometry, "/odom", self._odom_cb, 10)

        self.get_logger().info(
            f"Nav2 Goal Tester initialized for target ({goal_x:.2f}, {goal_y:.2f}) behind "
            f"obstacle wall, /cmd_vel as {self.cmd_vel_type}."
        )

    def _resolve_cmd_vel_type(self, requested: str) -> str:
        """Pick the single /cmd_vel type to subscribe with.

        An explicit request wins (the pipeline already knows, from the distro and the
        robot config, which one it launched Nav2 with). Otherwise ask the graph: by
        the time this test runs the Nav2 servers are active, so the publisher exists
        and names its own type. Unstamped is the fallback -- it is the Jazzy default.
        """
        if requested in ("twist", "twist_stamped"):
            return requested
        deadline = time.time() + 5.0
        while time.time() < deadline:
            for info in self.get_publishers_info_by_topic("/cmd_vel"):
                if info.topic_type.endswith("TwistStamped"):
                    return "twist_stamped"
                if info.topic_type.endswith("Twist"):
                    return "twist"
            time.sleep(0.2)
        self.get_logger().warn("No /cmd_vel publisher found; assuming unstamped Twist.")
        return "twist"

    def _plan_cb(self, msg: Path):
        self.path_received = msg
        # Inspect path to verify it routes around obstacle wall (x=2.0, y in [-1.5, 1.5])
        # Direct line from (0,0) to (3.0,0) has y=0 near x=2.0.
        # Circumventing path must have |y| > 1.3 when x is near 2.0 (e.g. 1.5 < x < 2.5)
        routes_around = False
        reaches_behind = False
        for p in msg.poses:
            px = p.pose.position.x
            py = p.pose.position.y
            if 1.5 <= px <= 2.5 and abs(py) >= 1.2:
                routes_around = True
            if px >= 2.5:
                reaches_behind = True

        if routes_around and reaches_behind:
            self.path_avoids_wall = True
            self.get_logger().info(f"✅ Verified global path ({len(msg.poses)} waypoints) routes around obstacle wall!")

    def _note_command(self, tw):
        self.cmd_peak_lin = max(self.cmd_peak_lin, abs(tw.linear.x))
        self.cmd_peak_ang = max(self.cmd_peak_ang, abs(tw.angular.z))

    def _cmd_vel_cb(self, msg: Twist):
        self.cmd_vel_count += 1
        self._note_command(msg)

    def _cmd_vel_stamped_cb(self, msg: TwistStamped):
        self.cmd_vel_count += 1
        self.cmd_vel_stamped_count += 1
        self._note_command(msg.twist)

    def _odom_cb(self, msg: Odometry):
        if self.initial_odom is None:
            self.initial_odom = msg
        self.latest_odom = msg
        # The PEAK excursion from the start pose, not the current one. A robot that
        # drives out and comes back, or that spins past its start yaw, has moved --
        # and the only thing this check exists to catch is a base that never moved
        # at all, where every sample is identical to the first.
        p0 = self.initial_odom.pose.pose.position
        p1 = msg.pose.pose.position
        self.odom_max_dist = max(self.odom_max_dist, math.hypot(p1.x - p0.x, p1.y - p0.y))
        dyaw = abs(_yaw(msg.pose.pose.orientation) - _yaw(self.initial_odom.pose.pose.orientation))
        self.odom_max_yaw = max(self.odom_max_yaw, min(dyaw, 2 * math.pi - dyaw))
        # The reported twist, not the integrated pose, is what decides whether the
        # base responded -- see the note on `responded()` in run_test().
        self.odom_peak_lin = max(self.odom_peak_lin, abs(msg.twist.twist.linear.x))
        self.odom_peak_ang = max(self.odom_peak_ang, abs(msg.twist.twist.angular.z))

    def send_goal(self) -> bool:
        self.get_logger().info("Waiting for /navigate_to_pose action server...")
        if not self.action_client.wait_for_server(timeout_sec=self.timeout_sec):
            self.get_logger().error(f"Action server /navigate_to_pose not available after {self.timeout_sec}s!")
            return False

        self.get_logger().info("Action server connected. Dispatching goal behind obstacle wall...")
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = self.goal_x
        goal_msg.pose.pose.position.y = self.goal_y
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.w = 1.0

        send_goal_future = self.action_client.send_goal_async(
            goal_msg, feedback_callback=self._feedback_cb
        )
        send_goal_future.add_done_callback(self._goal_response_cb)
        return True

    def _goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Nav2 goal rejected!")
            return
        self.goal_accepted = True
        self.get_logger().info("✅ Nav2 goal accepted by bt_navigator.")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_cb)

    def _feedback_cb(self, feedback_msg):
        fb = feedback_msg.feedback
        self.get_logger().debug(f"Nav2 feedback: distance_remaining={fb.distance_remaining:.2f}m")

    def _result_cb(self, future):
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.goal_completed = True
            self.get_logger().info("🎉 Nav2 goal SUCCEEDED! Robot reached target behind obstacle wall.")
        else:
            self.get_logger().info(f"Nav2 goal finished with status code: {status}")


def run_test(goal_x: float = 3.0, goal_y: float = 0.0, timeout: float = 30.0, min_cmds: int = 5,
             cmd_vel_type: str = "auto", noise_lin: float = 0.03,
             noise_ang: float = 0.10, require_motion: bool = True) -> bool:
    rclpy.init()
    node = Nav2GoalTester(goal_x=goal_x, goal_y=goal_y, timeout_sec=timeout,
                          cmd_vel_type=cmd_vel_type)

    def moved() -> bool:
        """Did the base respond to what it was told?

        The reported TWIST, not the integrated pose. A fake-mode board at rest
        still reports a little of both -- one bench run sampled vel_lin=0.006 m/s
        and vel_ang=0.036 rad/s while standing still -- and integrating that over
        a 25-second window accumulates 0.15 m of "travel", which sails past any
        displacement threshold small enough to be worth setting. Velocity does not
        accumulate, so the noise floor stays a floor.

        A floor, and deliberately nothing more. An earlier version also required
        the response to reach a fraction of the commanded peak, which sounds
        stricter and is simply a different question: it fails a base that hears
        the command and tracks it badly -- geared down, loaded, or PID-detuned --
        and that is a performance judgement with no place in a release gate. What
        this exists to catch is a base that never heard the command at all, and
        the floor catches exactly that.
        """
        return node.odom_peak_lin >= noise_lin or node.odom_peak_ang >= noise_ang

    def verdict(headline: str) -> bool:
        """Every exit goes through here, so the motion rule cannot be skipped by one
        branch -- which is how the old version passed. It computed this same odom
        delta, formatted it into the success message, and never compared it to
        anything."""
        detail = (f"{node.cmd_vel_count} cmd_vel msgs ({node.cmd_vel_stamped_count} stamped) "
                  f"peaking at {node.cmd_peak_lin:.3f} m/s / {node.cmd_peak_ang:.3f} rad/s; "
                  f"odom reported up to {node.odom_peak_lin:.3f} m/s / "
                  f"{node.odom_peak_ang:.3f} rad/s and moved "
                  f"{node.odom_max_dist:.3f} m / {node.odom_max_yaw:.3f} rad")
        if not require_motion or moved():
            print(f"✅ {headline}: {detail}")
            return True
        print(f"❌ {headline}, BUT THE BASE NEVER MOVED: {detail}")
        if node.cmd_vel_count >= min_cmds:
            # The diagnostic that matters. Nav2 is commanding and the base is not
            # responding, which on this stack is nearly always one thing: the
            # firmware subscribes to a different /cmd_vel type than the controller
            # publishes. nav2 >= kilted publishes TwistStamped; an image built
            # without USE_STAMPED_CMD_VEL listens for plain Twist and hears nothing.
            print(f"   The controller published {node.cmd_vel_count} commands as "
                  f"{node.cmd_vel_type} and /odom did not change. Check that the "
                  f"firmware was built for this distro's /cmd_vel contract "
                  f"(USE_STAMPED_CMD_VEL in lino_base_config.h); a mismatch is silent "
                  f"on both sides.")
        else:
            print(f"   Only {node.cmd_vel_count} commands were published, so the "
                  f"controller -- not the base -- is the place to look.")
        return False

    try:
        if not node.send_goal():
            return False

        start_time = time.time()
        while time.time() - start_time < timeout:
            rclpy.spin_once(node, timeout_sec=0.2)

            # Planned around the wall AND commanding AND the base answered. The
            # first two are measured entirely host-side: the plan comes from the
            # planner and the count from the controller's own publications, so a
            # board that is unplugged satisfies both.
            if node.path_avoids_wall and node.cmd_vel_count >= min_cmds and \
                    (moved() or not require_motion):
                return verdict("NAV2 VERIFICATION SUCCESS: path planned around the obstacle wall")

            if node.goal_completed and (moved() or not require_motion):
                return verdict("NAV2 GOAL COMPLETED SUCCESSFULLY")

        # The commanded peak belongs here too. Without it a timeout line says the
        # base reported 0.059 rad/s and leaves the reader unable to tell a base
        # ignoring a brisk command from one faithfully following a tiny one --
        # which is the whole question when a run fails.
        print(f"⚠️ Nav2 test timeout after {timeout}s: goal_accepted={node.goal_accepted}, "
              f"path_avoids_wall={node.path_avoids_wall}, cmd_vel_count={node.cmd_vel_count}, "
              f"cmd_peak={node.cmd_peak_lin:.3f}m/s,{node.cmd_peak_ang:.3f}rad/s, "
              f"odom_peak={node.odom_peak_lin:.3f}m/s,{node.odom_peak_ang:.3f}rad/s, "
              f"odom_moved={node.odom_max_dist:.3f}m,{node.odom_max_yaw:.3f}rad")
        if node.goal_completed:
            return verdict("NAV2 GOAL COMPLETED (after the window)")
        if node.goal_accepted and node.path_avoids_wall:
            return verdict("NAV2 PATH PLANNING AROUND THE OBSTACLE WALL VERIFIED")
        return False

    finally:
        node.destroy_node()
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description="Test Nav2 Goal Behind Obstacle Wall")
    parser.add_argument("--goal-x", type=float, default=3.0, help="Goal X coordinate (m)")
    parser.add_argument("--goal-y", type=float, default=0.0, help="Goal Y coordinate (m)")
    parser.add_argument("--timeout", type=float, default=25.0, help="Timeout in seconds")
    parser.add_argument("--min-cmds", type=int, default=5, help="Minimum cmd_vel commands to confirm driving")
    parser.add_argument("--cmd-vel-type", choices=["auto", "twist", "twist_stamped"], default="auto",
                        help="Type published on /cmd_vel; 'auto' reads it off the graph")
    parser.add_argument("--noise-lin", type=float, default=0.03,
                        help="Linear speed (m/s) at or below which /odom is considered at rest")
    parser.add_argument("--noise-ang", type=float, default=0.10,
                        help="Angular speed (rad/s) at or below which /odom is considered at rest; "
                             "a fake-mode board at rest has been seen reporting 0.036")
    parser.add_argument("--no-require-motion", action="store_true",
                        help="Pass on planning alone, without the base responding. For bringing "
                             "a host-side stack up with no board attached; never for a release "
                             "test, where a firmware that ignores /cmd_vel is exactly the fault "
                             "this catches.")
    args = parser.parse_args()

    success = run_test(goal_x=args.goal_x, goal_y=args.goal_y, timeout=args.timeout,
                       min_cmds=args.min_cmds, cmd_vel_type=args.cmd_vel_type,
                       noise_lin=args.noise_lin, noise_ang=args.noise_ang,
                       require_motion=not args.no_require_motion)
    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
