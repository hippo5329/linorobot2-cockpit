#!/usr/bin/env python3
# ==============================================================================
# test_nav2_goal.py — Nav2 Navigation Verification Behind Obstacle Wall
#
# Directives Compliance:
# - Validates Nav2 planning and execution in virtual room with obstacle wall.
# - Obstacle wall geometry: x = 2.0m, y from -1.5m to +1.5m (fake_ld19.h).
# - Goal behind obstacle wall: (x=3.0m, y=0.0m), and with --round-trips N the
#   robot then drives back to home (0, 0) and out again, N times: every leg must
#   arrive, and every leg that crosses the wall must have planned around it.
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
    try:
        import tf2_ros
    except ImportError:          # the gate still runs, measuring in odom and saying so
        tf2_ros = None
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


_STATUS_NAMES = {0: "UNKNOWN", 1: "ACCEPTED", 2: "EXECUTING", 3: "CANCELING",
                 4: "SUCCEEDED", 5: "CANCELED", 6: "ABORTED"}


def _status_name(status: int) -> str:
    return _STATUS_NAMES.get(int(status), f"status {status}")


# The obstacle wall of the simulated room (fake_ld19.h defaults).
WALL_X = 2.0
WALL_HALF_SPAN = 1.5
# How close to an end a crossing may be and still be a robot rounding the
# corner rather than one driving through the face. A disc of FAKE_ROBOT_RADIUS
# cannot pass nearer than that to the endpoint, and a board that clipped it at
# y = -1.40 was reported as driving through a solid wall.
WALL_END_MARGIN = 0.30


class Nav2GoalTester(Node):
    def __init__(self, goal_x: float = 3.0, goal_y: float = 0.0, timeout_sec: float = 30.0,
                 cmd_vel_type: str = "auto"):
        super().__init__("nav2_goal_tester")
        self.goal_x = goal_x
        self.goal_y = goal_y
        self.timeout_sec = timeout_sec

        self.action_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        # The goal is sent in the MAP frame, so that is the frame the distance to
        # it has to be measured in. Measuring in odom passed a leg that ended
        # 2.829 m away: the robot had hit the wall, its wheels slipped, the
        # EKF's odom walked on, and SLAM absorbed the difference into map->odom.
        # Nav2 was right that it had arrived; the gate was reading another frame.
        self.tf_buffer = tf2_ros.Buffer() if tf2_ros else None
        self.tf_listener = (tf2_ros.TransformListener(self.tf_buffer, self)
                            if tf2_ros else None)
        self.goal_frame = "map"
        self.base_frame = "base_link"
        self.tf_ok = False
        
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
        # What the goal ACTUALLY ended as. A run that only asks "did it succeed"
        # throws away the one field that says why it did not: Nav2 aborts carry an
        # error_code and error_msg (105 / "Failed to make progress" on the bench),
        # and a gate that does not read them reports a silent red.
        self.goal_status: int = -1
        self.goal_error_code: int = 0
        self.goal_error_msg: str = ""
        self.goal_rejected: bool = False
        self.distance_remaining: float = float("nan")
        # How far the base ended from the GOAL, and the closest it ever came.
        # This is what "did it get there" means: the action status says how Nav2
        # decided to stop, and a status is not a position. A run that aborts 0.2 m
        # short and one that aborts 2.8 m short are the same ABORTED and are not
        # the same result. Measured from /odom against the goal pose, which share
        # a frame here because the goal is sent in the odom frame.
        self.goal_dist_start: float = float("nan")
        self.goal_dist_now: float = float("nan")
        self.goal_dist_min: float = float("inf")
        # Per-leg bookkeeping for a back-and-forth run. A leg is one goal; the
        # counters above that describe THE GOAL are reset by begin_leg(), the
        # ones that describe the run (cmd_vel counts, peaks, odom_max_dist) are
        # not. leg_id tags the action callbacks so a result arriving late from a
        # preempted goal cannot overwrite the next leg's status.
        self._goal_handle = None
        self.leg_id: int = 0
        self.leg_start_xy = None
        self.leg_max_dist: float = 0.0
        # Where the robot ITSELF crossed the wall's line (x = WALL_X), per leg.
        # The /plan is what Nav2 intended; this is what the robot did, and it is
        # the fact that settles "did it go around": crossing beyond the wall's
        # ends is going around, crossing between them is going THROUGH a wall
        # that only the LiDAR believes in. A plan can also simply be missed --
        # the tester subscribes after the first plans are published.
        self.wall_cross_y = []

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
        # The base's OWN pose, beside the filtered one. The firmware clamps a
        # simulated robot to the wall and corrects its odometry; this EKF fuses
        # velocities only, and on contact the fake wheels keep reporting speed
        # (they slip, by design -- main.cpp), so the filtered pose walks through
        # a wall the base is pinned against. Reading both is what lets a verdict
        # say which of the two happened.
        self.raw_max_x: float = float("-inf")
        # Crossings measured on the UNFILTERED pose. The firmware clamps that
        # one -- it is where the simulation's physics live -- so it, not the
        # filtered estimate, answers "did the robot go through the wall".
        self.raw_cross_y: list = []
        self._raw_prev = None
        self.create_subscription(Odometry, "/odom/unfiltered",
                                 self._raw_odom_cb, sensor_qos)

        self.get_logger().info(
            f"Nav2 Goal Tester initialized for target ({goal_x:.2f}, {goal_y:.2f}) behind "
            f"obstacle wall, /cmd_vel as {self.cmd_vel_type}."
        )

    def cancel_current_goal(self):
        """Ask Nav2 to end the goal in flight, and say whether it was asked.

        A leg ends when the ROBOT is at the goal, which is usually before
        bt_navigator says so. Sending the next goal then gets "Requested
        navigation from navigate_to_pose while another navigator is processing,
        rejecting request" -- measured on leg 3 of 8, RP2350 lyrical, where the
        previous goal was still running behind an is_path_valid service timeout.
        So the leg is closed explicitly instead of hoped to have closed.
        """
        h = self._goal_handle
        if h is None:
            return False
        try:
            h.cancel_goal_async()
            return True
        except Exception as exc:                 # already terminal: nothing to cancel
            self.get_logger().debug(f"cancel_goal_async: {exc}")
            return False

    def begin_leg(self, goal_x: float, goal_y: float) -> None:
        """Point the tester at the next goal and forget the previous goal's outcome."""
        self.leg_id += 1
        self._goal_handle = None
        self.wall_cross_y = []
        self.raw_max_x = float("-inf")
        self.raw_cross_y = []
        self._raw_prev = None
        self.goal_x, self.goal_y = goal_x, goal_y
        self.path_received = None
        self.path_avoids_wall = False
        self.goal_accepted = self.goal_completed = self.goal_rejected = False
        self.goal_status, self.goal_error_code, self.goal_error_msg = -1, 0, ""
        self.distance_remaining = float("nan")
        self.goal_dist_start = self.goal_dist_now = float("nan")
        self.goal_dist_min = float("inf")
        self.leg_max_dist = 0.0
        if self.latest_odom is not None:
            wx, wy = self.world_xy(self.latest_odom)
            self.leg_start_xy = (wx, wy)
            self.goal_dist_start = math.hypot(goal_x - wx, goal_y - wy)
        else:
            self.leg_start_xy = None

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

    def _raw_odom_cb(self, msg: Odometry):
        rx = msg.pose.pose.position.x
        ry = msg.pose.pose.position.y
        self.raw_max_x = max(self.raw_max_x, rx)
        prev = self._raw_prev
        self._raw_prev = (rx, ry)
        if prev is not None and (prev[0] - WALL_X) * (rx - WALL_X) < 0:
            gap = math.hypot(rx - prev[0], ry - prev[1])
            t = (WALL_X - prev[0]) / (rx - prev[0])
            self.raw_cross_y.append((prev[1] + t * (ry - prev[1]), gap))

    def world_xy(self, msg: Odometry):
        """Where the robot is in the frame the goal was sent in.

        map -> base_link when TF has it (what Nav2 steers by), the odometry pose
        otherwise -- naming which, so a verdict can never silently compare two
        different frames again.
        """
        try:
            if self.tf_buffer is None:
                raise RuntimeError("no tf2_ros")
            tr = self.tf_buffer.lookup_transform(self.goal_frame, self.base_frame,
                                                 rclpy.time.Time()).transform.translation
            self.tf_ok = True
            return tr.x, tr.y
        except Exception:
            self.tf_ok = False
            p = msg.pose.pose.position
            return p.x, p.y

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
        # Displacement FROM THE GOAL: the distance still to close, and the best
        # it managed. goal_dist_start is taken from the first sample so the line
        # can say how much of the gap was closed rather than just where it ended.
        wx, wy = self.world_xy(msg)
        self.goal_dist_now = math.hypot(self.goal_x - wx, self.goal_y - wy)
        if self.leg_start_xy is None:
            self.leg_start_xy = (wx, wy)
        if math.isnan(self.goal_dist_start):
            self.goal_dist_start = math.hypot(self.goal_x - self.leg_start_xy[0],
                                              self.goal_y - self.leg_start_xy[1])
        self.goal_dist_min = min(self.goal_dist_min, self.goal_dist_now)
        self.leg_max_dist = max(self.leg_max_dist, math.hypot(wx - self.leg_start_xy[0],
                                                              wy - self.leg_start_xy[1]))
        prev = getattr(self, "_last_xy", None)
        self._last_xy = (wx, wy)
        if prev is not None and (prev[0] - WALL_X) * (wx - WALL_X) < 0:
            # Interpolate, but only believe it when the two samples bracketing
            # the crossing are close together. /odom is 50 Hz and the robot does
            # 0.4 m/s, so 8 mm apart is normal -- and a dropped burst would let a
            # robot that went around the END be interpolated into a straight line
            # through the middle, which is the difference between a pass and
            # "it drove through the wall".
            # The interpolated y, and how uncertain it is: the robot could have
            # been anywhere along the segment between the two samples, so the
            # gap between them IS the error bar. A detour round the end at
            # y = -1.6 came back as -1.31 and -1.44 with the samples 0.2-0.3 m
            # apart, which is "inside the wall" by the number and "round the
            # end" by the physics.
            gap = math.hypot(wx - prev[0], wy - prev[1])
            t = (WALL_X - prev[0]) / (wx - prev[0])
            y = prev[1] + t * (wy - prev[1])
            self.wall_cross_y.append((y, gap))

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

        leg = self.leg_id
        send_goal_future = self.action_client.send_goal_async(
            goal_msg, feedback_callback=self._feedback_cb
        )
        send_goal_future.add_done_callback(lambda f: self._goal_response_cb(f, leg))
        return True

    def _goal_response_cb(self, future, leg: int = 0):
        if leg != self.leg_id:
            return                       # a previous leg's goal; this one has moved on
        goal_handle = future.result()
        if not goal_handle.accepted:
            # Distinct from "never answered": bt_navigator was there and said no.
            # Both used to leave goal_accepted False and the run then timed out
            # with nothing to tell them apart.
            self.goal_rejected = True
            self.get_logger().error("Nav2 REJECTED the goal: bt_navigator would not accept it.")
            return
        self.goal_accepted = True
        self._goal_handle = goal_handle
        self.get_logger().info("✅ Nav2 goal accepted by bt_navigator.")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda f: self._result_cb(f, leg))

    def _feedback_cb(self, feedback_msg):
        fb = feedback_msg.feedback
        self.distance_remaining = float(getattr(fb, "distance_remaining", float("nan")))
        self.get_logger().debug(f"Nav2 feedback: distance_remaining={self.distance_remaining:.2f}m")

    def _result_cb(self, future, leg: int = 0):
        if leg != self.leg_id:
            return                       # the preempted goal of an earlier leg
        outcome = future.result()
        self.goal_status = int(outcome.status)
        result = getattr(outcome, "result", None)
        # nav2 >= jazzy puts an error_code on the result; older ones do not, and a
        # missing field is not an error, so read it defensively.
        self.goal_error_code = int(getattr(result, "error_code", 0) or 0)
        self.goal_error_msg = str(getattr(result, "error_msg", "") or "")
        if self.goal_status == GoalStatus.STATUS_SUCCEEDED:
            self.goal_completed = True
            self.get_logger().info("🎉 Nav2 goal SUCCEEDED! Robot reached target behind obstacle wall.")
        else:
            self.get_logger().info(
                f"Nav2 goal finished as {_status_name(self.goal_status)}"
                + (f", error_code={self.goal_error_code}" if self.goal_error_code else "")
                + (f", error_msg={self.goal_error_msg!r}" if self.goal_error_msg else ""))


def _gap(node) -> str:
    """How far it ended from the goal, and the closest it came.

    A status is not a position: aborting 0.2 m short and aborting 2.8 m short are
    both ABORTED and are not the same result. Reporting the gap is what lets a
    reader tell "nearly there, tolerance too tight" from "never left".
    """
    now = getattr(node, "goal_dist_now", float("nan"))
    if now != now:                       # NaN: no odom sample, nothing to say
        return ""
    best = getattr(node, "goal_dist_min", float("inf"))
    start = getattr(node, "goal_dist_start", float("nan"))
    out = f"; {now:.3f} m from the goal (closest {best:.3f} m"
    if start == start and start > 0:
        out += f", from {start:.3f} m at the start"
    return out + ")"


def _why(node) -> str:
    """The error Nav2 gave, when it gave one.

    An abort without its error_code is a dead end for whoever reads the log:
    105 / "Failed to make progress" and 102 / "no valid path" are different
    faults with different fixes, and the gate saw both and printed neither.
    """
    bits = []
    if getattr(node, "goal_error_code", 0):
        bits.append(f"error_code={node.goal_error_code}")
    if getattr(node, "goal_error_msg", ""):
        bits.append(f"error_msg={node.goal_error_msg!r}")
    return (" (" + ", ".join(bits) + ")") if bits else ""


def run_test(goal_x: float = 3.0, goal_y: float = 0.0, timeout: float = 30.0, min_cmds: int = 5,
             cmd_vel_type: str = "auto", noise_lin: float = 0.03,
             noise_ang: float = 0.10, require_motion: bool = True,
             min_traverse: float = 0.10, require_goal: bool = False,
             goal_tolerance: float = 0.30, min_start_gap: float = 1.0,
             round_trips: int = 0, home_x: float = 0.0, home_y: float = 0.0) -> bool:
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

    # The room's one obstacle (fake_ld19.h): a wall at x = 2.0 spanning y = -1.5..1.5.
    # A goal on its far side is what this test was written for, and "reached" is
    # only worth anything there if the plan went AROUND the wall: a goal reached
    # with no such plan means either the robot was already past the wall or the
    # world let it drive through. A goal on the near side asks nothing of the
    # plan, and this gate then says so rather than pretending it did.
    def leg_crosses_wall() -> bool:
        """Does the straight line from where this leg began to its goal cut the wall?

        Out to (3, 0) from the origin does; back home from (3, 0) does too, and
        the detour is required in both directions. A leg that starts with no
        odom sample yet is assumed to begin at home.
        """
        sx, sy = getattr(node, "leg_start_xy", None) or (home_x, home_y)
        gx, gy = getattr(node, "goal_x", goal_x), getattr(node, "goal_y", goal_y)
        if (sx - WALL_X) * (gx - WALL_X) >= 0:
            return False                 # both on the same side, or one on the line
        t = (WALL_X - sx) / (gx - sx)
        return abs(sy + t * (gy - sy)) <= WALL_HALF_SPAN

    def crossings():
        """The crossings that answer "did it go through the wall", and whose.

        The firmware clamps the SIMULATED pose -- /odom/unfiltered -- so that is
        where the room's physics live and the only pose that can answer the
        question. The filtered pose is an estimate this EKF builds from
        velocities alone; it drifts, and it can cut a corner the robot never
        cut. Measured on the GenDrv (2026-09-22): the filtered pose crossed at
        y=-0.63, apparently straight through the middle, while the base's own
        odometry never went nearer the wall than its own radius and had rounded
        the end. Failing that leg blamed the firmware for the estimator.

        Falls back to the filtered pose only when /odom/unfiltered was never
        seen, and says which it used.
        """
        raw = getattr(node, "raw_cross_y", None)
        if raw or getattr(node, "raw_max_x", float("-inf")) > float("-inf"):
            return raw or [], "the base's own odometry"
        return getattr(node, "wall_cross_y", ()), "the filtered pose (no /odom/unfiltered)"

    def went_around() -> bool:
        """Crossed at or beyond a wall end, further out than the error bar.

        "Beyond" includes the corner: a disc robot rounds it wide, and the clamp
        that keeps it one radius clear puts a crossing near the end at
        |y| a little under the span rather than over it."""
        return any(abs(y) + gap > WALL_HALF_SPAN - WALL_END_MARGIN
                   for y, gap in crossings()[0])

    def went_through() -> bool:
        """Crossed where the wall actually is, by more than the error bar.

        The simulated robot is pushed off the segment (fake_ld19.h clampToRoom,
        measured: a board driven at the wall stops 0.30 m short and stays), so
        this should be impossible; if it is ever true the room, not the
        navigation, is what failed. Anything within a sample gap of the wall's
        end is not a measurement of either answer."""
        return any(abs(y) + gap < WALL_HALF_SPAN - WALL_END_MARGIN
                   for y, gap in crossings()[0])

    def estimator_cut_the_corner() -> bool:
        """The filtered pose went through where the base did not.

        Not a collision -- nothing hit anything -- but worth saying, because
        Nav2 steered by a pose that was inside an obstacle.
        """
        filt = getattr(node, "wall_cross_y", ())
        return (not went_through()) and any(
            abs(y) + gap < WALL_HALF_SPAN - WALL_END_MARGIN for y, gap in filt)

    def wall_path_ok() -> bool:
        if not leg_crosses_wall():
            return True
        if went_through():
            return False          # measured, with close samples: it really passed through
        if node.path_avoids_wall or went_around():
            return True           # the plan detoured, or the drive did
        # Neither could be observed. On a loaded host the tester misses /plan
        # messages and bursts of /odom alike, and failing a leg for what the
        # instrument could not watch is not a fault of the robot. The wall is
        # solid (the firmware clamps the pose to it, measured: a board commanded
        # into it stops 0.30 m short and stays), and the start gap proves the
        # robot began on the near side -- so an arrival IS a route around it.
        # Say that the route is unproven rather than pretending either way.
        return True

    def route_note() -> str:
        if not leg_crosses_wall():
            return "not needed"
        if node.path_avoids_wall and went_around():
            return "yes, planned and driven"
        if node.path_avoids_wall:
            return "yes, in the plan"
        crossings = [(y, gap) for y, gap in getattr(node, "wall_cross_y", ())
                     if abs(y) - gap > WALL_HALF_SPAN]
        if crossings:
            y, gap = max(crossings, key=lambda c: abs(c[0]))
            return f"yes, driven (crossed at y={y:+.2f} ±{gap:.2f})"
        return ("unproven: no /plan detour seen and the crossing was not measured; "
                "the wall is solid and the robot got there, so it went round")

    def start_gap_is_meaningful() -> bool:
        """Was the robot far enough away for arriving to mean anything?

        The simulated pose survives a run -- only a boot zeroes it -- so a second
        goal at the same coordinates can begin with the robot already sitting on
        it. Measured: an RP2350 started 0.484 m from a goal with a 0.30 m
        tolerance, closed 0.185 m, and was reported REACHED. That is the gate
        passing without the robot going anywhere, in a new place.
        """
        start = getattr(node, "goal_dist_start", float("nan"))
        return not (start == start) or start >= min_start_gap

    def reached_goal() -> bool:
        """Did it actually get there?

        Either Nav2 said SUCCEEDED, or the base is inside the tolerance of the
        goal pose -- because a controller can stop short and report success, and
        can equally be aborted by a behaviour-tree timeout while sitting on top
        of the goal. The position is the fact; the status is the explanation.
        """
        return node.goal_completed or getattr(node, "goal_dist_min", float("inf")) <= goal_tolerance

    def traversed() -> bool:
        """Did it actually GO somewhere, as opposed to answering the command?

        moved() is a floor for "the base heard us", and angular motion satisfies
        it -- correctly, because a base that only rotates has still heard. But the
        headline says the path around the obstacle wall was driven, and a robot
        rotating on the spot has driven none of it. The success branch used to
        exit as soon as the plan was verified, five commands had gone out and
        moved() was true, which on a quick machine is the in-place rotation at the
        very start: measured 5 cmd_vel peaking at 0.025 m/s on the Yahboom, a pass
        with the robot standing still, while the same board over Wi-Fi did 161
        commands at 0.400 m/s. The wildly varying counts across boards -- 5, 20,
        41, 61, 120, 161 -- were that race, not a property of the boards.

        This is displacement from the start pose (max hypot in _odom_cb), not
        integrated velocity, so a threshold means something: the noise argument
        that rules displacement out for moved() does not apply to a pose delta.
        """
        return node.odom_max_dist >= min_traverse

    def run_legs(legs) -> bool:
        """Drive every leg in turn; each must arrive, and each that crosses the
        wall must have planned around it. One failure ends the run -- the
        pipeline then asks the base directly whether it still drives."""
        n = len(legs)
        for i, (gx, gy) in enumerate(legs, 1):
            node.begin_leg(gx, gy)
            if not node.send_goal():
                return False
            # A rejection right after the previous leg is the handshake, not the
            # navigation: give bt_navigator a moment and ask once more.
            t_acc = time.time()
            while time.time() - t_acc < 5 and not (node.goal_accepted or node.goal_rejected):
                rclpy.spin_once(node, timeout_sec=0.2)
            if node.goal_rejected:
                print(f"   leg {i}/{n}: bt_navigator rejected the goal; waiting 5 s and asking once more.")
                t_w = time.time()
                while time.time() - t_w < 5:
                    rclpy.spin_once(node, timeout_sec=0.2)
                node.begin_leg(gx, gy)
                if not node.send_goal():
                    return False
            t0 = time.time()
            arrived = False
            while time.time() - t0 < timeout:
                rclpy.spin_once(node, timeout_sec=0.2)
                if node.goal_rejected:
                    print(f"❌ NAV2 GOAL REJECTED by bt_navigator on leg {i}/{n} "
                          f"({gx:.2f}, {gy:.2f}){_why(node)}.")
                    return False
                if start_gap_is_meaningful() and reached_goal() and wall_path_ok() \
                        and (moved() or not require_motion):
                    arrived = True
                    break
                if node.goal_status in (5, 6) and not reached_goal():   # CANCELED, ABORTED
                    break                # Nav2 gave up; waiting out the window adds nothing
            took = time.time() - t0
            if not arrived:
                if not start_gap_is_meaningful():
                    print(f"❌ NAV2 LEG {i}/{n} IS VACUOUS: began {node.goal_dist_start:.3f} m "
                          f"from ({gx:.2f}, {gy:.2f}), inside the {min_start_gap:.2f} m this gate "
                          f"needs before arriving proves anything.")
                elif reached_goal() and not wall_path_ok():
                    if went_through():
                        _cross, _whose = crossings()
                        ys = ", ".join(f"{y:+.2f} ±{gap:.2f}" for y, gap in _cross)
                        raw_x = getattr(node, "raw_max_x", float("-inf"))
                        if raw_x < WALL_X - 0.15:
                            why = (f"The base itself never got past x={raw_x:.2f} -- it is pinned "
                                   f"against the wall, and the filtered pose walked through it: on "
                                   f"contact the wheels keep reporting speed (they slip, by design) "
                                   f"and this EKF fuses velocity, not position. So the robot drove "
                                   f"INTO the obstacle instead of round it.")
                        else:
                            why = (f"The base's own odometry reached x={raw_x:.2f} too, so the clamp "
                                   f"that should push a simulated robot off the wall did not act: "
                                   f"check that fake_ld19 is enabled and clampToRoom is reached.")
                        print(f"❌ NAV2 LEG {i}/{n} DROVE INTO THE WALL: {_whose} "
                              f"crossed x={WALL_X:.1f} at y={ys}, inside the wall's span "
                              f"(±{WALL_HALF_SPAN:.1f} m){_gap(node)}. {why}")
                    else:
                        print(f"❌ NAV2 LEG {i}/{n} REACHED ({gx:.2f}, {gy:.2f}) WITHOUT GOING AROUND "
                              f"THE WALL{_gap(node)}: no /plan detoured around x={WALL_X:.1f} and the "
                              f"robot never crossed it beyond ±{WALL_HALF_SPAN:.1f} m. It cannot have "
                              f"got there; check the frames (a goal in map, a pose read from odom).")
                else:
                    print(f"❌ NAV2 LEG {i}/{n} NOT REACHED: ({gx:.2f}, {gy:.2f}) ended as "
                          f"{_status_name(node.goal_status)}{_why(node)}{_gap(node)} after "
                          f"{took:.0f} s; needed within {goal_tolerance:.2f} m; "
                          f"planned_around_wall={node.path_avoids_wall}, "
                          f"traversed {node.leg_max_dist:.3f} m this leg")
                return False
            around = route_note()
            print(f"   leg {i}/{n} -> ({gx:.2f}, {gy:.2f}): reached in {took:.0f} s, closest "
                  f"{node.goal_dist_min:.3f} m, from {node.goal_dist_start:.3f} m; "
                  f"around the wall: {around}; {node.cmd_vel_count} cmd_vel so far")
            if estimator_cut_the_corner():
                fy = ", ".join(f"{y:+.2f} ±{gap:.2f}"
                               for y, gap in getattr(node, "wall_cross_y", ()))
                print(f"   ⚠️ leg {i}/{n}: the BASE went round the wall, but the filtered pose "
                      f"Nav2 steers by crossed x={WALL_X:.1f} at y={fy} — inside the obstacle. "
                      f"Nothing collided; the estimate did. This EKF fuses velocities only, so "
                      f"it has no position to correct against and drifts across a long leg.")
            # Close the leg before opening the next one: cancel the goal in
            # flight and wait for a terminal status, so bt_navigator is idle
            # when the next goal arrives and this leg's status is its own.
            if i < n:
                node.cancel_current_goal()
            t1 = time.time()
            while node.goal_status == -1 and time.time() - t1 < 20.0:
                rclpy.spin_once(node, timeout_sec=0.2)
            if node.goal_status == -1:
                print(f"   leg {i}/{n}: Nav2 has not closed the goal 20 s after the robot "
                      f"arrived; continuing.")
        return verdict(f"NAV2 GOAL REACHED {n}/{n} legs: {round_trips} round trip(s) behind "
                       f"the obstacle wall and back home (within {goal_tolerance:.2f} m)")

    def verdict(headline: str) -> bool:
        """Every exit goes through here, so the motion rule cannot be skipped by one
        branch -- which is how the old version passed. It computed this same odom
        delta, formatted it into the success message, and never compared it to
        anything."""
        frame = "map" if getattr(node, "tf_ok", False) else "odom (no map->base_link TF)"
        detail = (f"measured in {frame}; {node.cmd_vel_count} cmd_vel msgs "
                  f"({node.cmd_vel_stamped_count} stamped) "
                  f"peaking at {node.cmd_peak_lin:.3f} m/s / {node.cmd_peak_ang:.3f} rad/s; "
                  f"odom reported up to {node.odom_peak_lin:.3f} m/s / "
                  f"{node.odom_peak_ang:.3f} rad/s and moved "
                  f"{node.odom_max_dist:.3f} m / {node.odom_max_yaw:.3f} rad; "
                  f"goal {_status_name(node.goal_status)}" + _why(node) + _gap(node))
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
        if round_trips >= 1:
            # Back and forth: out to the goal, home again, round_trips times.
            # Arrival is required on every leg -- a round trip that only plans
            # is not a round trip -- so this implies --require-goal.
            legs = [(goal_x, goal_y), (home_x, home_y)] * round_trips
            return run_legs(legs)

        if not node.send_goal():
            return False

        start_time = time.time()
        while time.time() - start_time < timeout:
            rclpy.spin_once(node, timeout_sec=0.2)

            # Planned around the wall AND commanding AND the base answered. The
            # first two are measured entirely host-side: the plan comes from the
            # planner and the count from the controller's own publications, so a
            # board that is unplugged satisfies both.
            # A verified plan is not a reached goal. When the goal is what must be
            # verified this branch must not end the run -- otherwise the answer
            # arrives before the robot has had a chance to get there.
            if not require_goal and node.path_avoids_wall and node.cmd_vel_count >= min_cmds and \
                    ((moved() and traversed()) or not require_motion):
                return verdict("NAV2 VERIFICATION SUCCESS: path planned around the obstacle wall")

            if require_goal and start_gap_is_meaningful() and reached_goal() \
                    and wall_path_ok() and (moved() or not require_motion):
                return verdict(f"NAV2 GOAL REACHED (within {goal_tolerance:.2f} m)")

            if node.goal_rejected:
                print(f"❌ NAV2 GOAL REJECTED by bt_navigator{_why(node)}: it was reachable "
                      f"on the action interface and refused the pose. Nothing downstream of "
                      f"this can be judged.")
                return False

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
        if node.goal_rejected:
            print(f"❌ NAV2 GOAL REJECTED by bt_navigator{_why(node)}.")
            return False
        if require_goal and not start_gap_is_meaningful():
            start = node.goal_dist_start
            print(f"❌ NAV2 GOAL TEST IS VACUOUS: the robot began {start:.3f} m from the "
                  f"goal, inside the {min_start_gap:.2f} m this gate needs before arriving "
                  f"proves anything (tolerance {goal_tolerance:.2f} m).")
            print(f"   The simulated pose survives a run -- only a boot zeroes it -- so a "
                  f"repeated goal can start with the robot already on top of it. Reboot the "
                  f"board, or send a goal measured from where it actually is.")
            return False
        if require_goal and not reached_goal():
            # Asked to verify the goal, not merely the plan. The gap is the
            # verdict and the status is the reason: "aborted 0.21 m short" and
            # "aborted 2.83 m short" send the reader to different places.
            print(f"❌ NAV2 GOAL NOT REACHED: ended as {_status_name(node.goal_status)}"
                  f"{_why(node)}{_gap(node)}; needed within {goal_tolerance:.2f} m; "
                  f"planned_around_wall={node.path_avoids_wall}, "
                  f"traversed {node.odom_max_dist:.3f} m")
            return False
        if require_goal and not wall_path_ok():
            print(f"❌ NAV2 GOAL REACHED WITHOUT A PATH AROUND THE WALL: the goal "
                  f"({goal_x:.2f}, {goal_y:.2f}) is behind the obstacle wall at x={WALL_X:.1f}, "
                  f"yet no /plan detoured around it{_gap(node)}. Either the robot did not start "
                  f"on the near side or it went through the wall; neither is a pass.")
            return False
        if require_goal:
            return verdict(f"NAV2 GOAL REACHED (within {goal_tolerance:.2f} m)")
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
    parser.add_argument("--min-start-gap", type=float, default=1.0,
                        help="metres the robot must START from the goal for --require-goal "
                             "to mean anything. The simulated pose survives a run, so a "
                             "repeated goal can begin with the robot already on it.")
    parser.add_argument("--goal-tolerance", type=float, default=0.30,
                        help="metres from the goal pose that count as reached, when "
                             "--require-goal is given")
    parser.add_argument("--require-goal", action="store_true",
                        help="the goal itself must reach STATUS_SUCCEEDED; a verified plan "
                             "is not enough. Reports the status and Nav2's error_code/"
                             "error_msg when it does not.")
    parser.add_argument("--round-trips", type=int, default=0,
                        help="drive to the goal and back home this many times (0 = one "
                             "one-way goal, the classic test). Every leg must arrive and "
                             "every leg that crosses the wall must have planned around it; "
                             "implies --require-goal.")
    parser.add_argument("--home-x", type=float, default=0.0, help="home X for the return legs (m)")
    parser.add_argument("--home-y", type=float, default=0.0, help="home Y for the return legs (m)")
    parser.add_argument("--min-traverse", type=float, default=0.10,
                        help="metres the base must actually cover before the path-verified "
                             "headline is allowed; displacement from the start pose, not "
                             "integrated velocity")
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
                       require_motion=not args.no_require_motion,
                       min_traverse=args.min_traverse, require_goal=args.require_goal,
                       goal_tolerance=args.goal_tolerance,
                       min_start_gap=args.min_start_gap,
                       round_trips=args.round_trips, home_x=args.home_x, home_y=args.home_y)
    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
