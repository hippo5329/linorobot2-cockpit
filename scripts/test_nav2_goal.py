#!/usr/bin/env python3
# ==============================================================================
# test_nav2_goal.py — Nav2 Navigation Verification Behind Obstacle Wall
#
# Directives Compliance:
# - Validates Nav2 planning and execution in virtual room with obstacle wall.
# - Obstacle wall geometry: x = 2.0m, y from -1.5m to +1.5m (sim_ld19.h).
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
    from sensor_msgs.msg import LaserScan
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


# The obstacle wall of the simulated room (sim_ld19.h defaults).
WALL_X = 2.0
WALL_HALF_SPAN = 1.5
# How close to an end a crossing may be and still be a robot rounding the
# corner rather than one driving through the face. A disc of SIM_ROBOT_RADIUS
# cannot pass nearer than that to the endpoint, and a board that clipped it at
# y = -1.40 was reported as driving through a solid wall.
WALL_END_MARGIN = 0.30

# An abort inside this window, having moved less than this far and planned
# nothing, is read as the stack still coming up and is retried once per run.
STARTUP_ABORT_SEC = 5.0
STARTUP_ABORT_DIST = 0.05
STARTUP_SETTLE_SEC = 15.0

# A PHANTOM SUCCESS: Nav2 reports the goal SUCCEEDED with the robot nowhere
# near it. reached_goal() already refuses to believe the status over the
# position -- see its docstring, and the Yahboom leg 2/8 of 2026-09-22 that was
# "reached in 2 s, closest 2.830 m". What was missing is that the leg then sat
# out the whole 180 s window waiting for an arrival that the stack had already
# stopped driving towards, and failed the run on one such leg.
#
# Seen twice in the 2026-09-23 mecanum slice, both on lyrical: a GenDrv leg
# SUCCEEDED 2.9 s after dispatch, 2.84 m away, 0.114 m travelled, with no
# complaint logged at all; a YB-EET01 leg the same but after error_code 105.
# Every goal in this tester is dispatched one MILLISECOND after the previous
# leg's cancel returns a terminal status, and that 1 ms gap is on the passing
# legs too -- which is the shape of a race in bt_navigator's unwinding, not of
# a robot that cannot drive. The board's drive suite scored 8/8 both times.
#
# So: confirm it is not the tester's own pose lagging, stop waiting, and ask
# once more per run. If the second attempt drives, the first was a phantom and
# the line says so; if it fails again, the leg fails on its own evidence.
PHANTOM_CONFIRM_SEC = 2.0
PHANTOM_SETTLE_SEC = 5.0

# Breathing room between one leg's terminal status and the next leg's dispatch.
# Cheap insurance against the same race: a goal accepted while the tree is
# still halting is the one that comes back instantly.
LEG_SETTLE_SEC = 0.5

# How far from home the robot may be before the leg is abandoned as a runaway.
#
# The simulated room is about 11.6 x 9.4 m (a saved map measured 232x188 cells
# at 0.05 m) and every goal in this test is within 3 m of the origin, so a base
# more than this from home is not navigating badly -- it has left.
#
# Measured on 2026-09-23: a leg asked to return to (0.00, 0.00) came within
# 1.326 m of home, kept going, and ended 13.667 m away having traversed
# 11.235 m, with nav2.log logging no transform or path complaint at all. The
# gate sat and watched for the full window.
#
# On the bench that is a wasted leg. On a real robot it is the hazard, and the
# thing that would stop it there -- nav2_collision_monitor with a real scan --
# is exactly what sim mode does not have. Catching it here is what makes the
# behaviour visible before there is a robot to be hurt by it.
RUNAWAY_RADIUS_M = 6.0

# How much longer to wait for bt_navigator to acknowledge a goal after the first
# 5 s. Generous on purpose: the cost of waiting is seconds on one leg of a soak,
# and the cost of not waiting is a red charged to the robot for a handshake that
# was merely slow.
ACCEPT_GRACE_S = 25.0

# How much TF history has to exist before the first goal is dispatched, and how
# long to wait for it. Lifecycle "active" says every node configured and
# activated; it does not say the transform tree is assembled. Measured on
# 2026-09-23, on legs that had just been told the stack was active:
#
#   Could not find a connection between 'odom' and 'base_link' because they are
#   not part of the same tree. Tf has two or more unconnected trees.
#   Invalid frame ID "base_link" passed to canTransform argument source_frame
#   Requested time ...734.581103 but the earliest data is at ...734.756795
#
# The last one is why this is a HISTORY requirement and not just "can you do it
# now": the buffer answered for the present and had nothing 176 ms back, which
# is what the costmaps ask for. One second is chosen against the slowest thing
# in the chain -- the global costmap updates at 1 Hz on these configs.
TF_READY_HISTORY_SEC = 1.0
TF_READY_TIMEOUT_SEC = 40.0
# How long to wait for Nav2 to give a cancelled goal a terminal status: between
# legs, and once more on the way out. Named so a test can shrink them.
GOAL_CLOSE_SEC = 20.0
GOAL_CLOSE_EXIT_SEC = 10.0


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
        self.odom_frame = "odom"
        # Longest observed interval between map->odom stamps; see
        # sample_map_odom_freshness().
        self.map_odom_max_gap = 0.0
        # The sampler's own worst interval, reported alongside the lag: this
        # node is single-threaded, and a lag measured by a starved sampler
        # deserves less weight than one measured by a prompt one.
        self.map_odom_max_sample_dt = 0.0
        self._last_map_odom_stamp = None
        self._last_map_odom_wall = None
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
        # velocities only, and on contact the sim wheels keep reporting speed
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

        # /scan itself, beside the map->odom stamp.
        #
        # The map->odom meter says the stamp stood still; with restamp_tf false
        # that stamp IS the scan stamp, so it cannot say which of two very
        # different things happened:
        #
        #   * the board stopped producing scans  -- the STAMPS gap, or
        #   * the scans were produced on time and arrived in a burst -- the
        #     stamps are evenly spaced and the ARRIVALS gap.
        #
        # The first is the emulator or the firmware loop; the second is the
        # transport or DDS. Measuring only one of them is how "scan gap" stayed
        # a single undifferentiated suspect. Both are recorded here, in a plain
        # callback: no lookup, no I/O, nothing that can perturb the run the way
        # an unthrottled TF lookup in the leg loop did.
        self.scan_count: int = 0
        self.scan_max_stamp_gap: float = 0.0
        self.scan_max_arrival_gap: float = 0.0
        self._scan_prev_stamp = None
        self._scan_prev_arrival = None
        self.create_subscription(LaserScan, "/scan", self._scan_cb, sensor_qos)

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

    def _scan_cb(self, msg):
        """Record the worst stamp interval and the worst arrival interval."""
        self.scan_count += 1
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        arrival = time.time()
        if self._scan_prev_stamp is not None:
            gap = stamp - self._scan_prev_stamp
            if gap > self.scan_max_stamp_gap:
                self.scan_max_stamp_gap = gap
        if self._scan_prev_arrival is not None:
            gap = arrival - self._scan_prev_arrival
            if gap > self.scan_max_arrival_gap:
                self.scan_max_arrival_gap = gap
        # A zero or backwards stamp is a board whose clock has not synced yet;
        # it would otherwise register as one enormous gap on the first scans.
        if stamp > 0.0:
            self._scan_prev_stamp = stamp
        self._scan_prev_arrival = arrival

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

    def map_odom_offset(self):
        """The correction SLAM is applying, or None when TF cannot say.

        This is the number that separates "the base drove there" from "the
        estimate went there": the base cannot know about map->odom, and SLAM
        cannot move the wheels.
        """
        try:
            if self.tf_buffer is None:
                return None
            tr = self.tf_buffer.lookup_transform(self.goal_frame, self.odom_frame,
                                                 rclpy.time.Time()).transform.translation
            return (tr.x, tr.y)
        except Exception:
            return None

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
        # Time 0, not now(). A goal in the map frame is a PLACE, not an
        # observation: (3.0, 0.0) is where it is at every instant, and tf2
        # reads a zero stamp as "the latest transform you have".
        #
        # now() asks for the future and says so. map->odom comes from SLAM and
        # odom->base_link from the EKF at ~50 Hz, so the newest TF is always
        # some milliseconds OLDER than this line executes, and the planner's
        # transform of the end pose throws:
        #
        #   ABORTED error_code=102 'Failed to transform end pose to global frame'
        #   Lookup would require extrapolation into the future. Requested time
        #   1790127458.311765 but the latest data is at 1790127458.307006
        #
        # Five milliseconds. It has cost a leg in most matrices this project
        # has run, on every board and both distros, and it is why
        # planner_server's transform_tolerance was raised 0.3 -> 0.5 -> 1.0 in
        # the reference configs, each time with a comment about this exact
        # message -- a tolerance cannot fix a request for a time that has not
        # happened yet.
        #
        # The gate does not lose anything by this. If the tree were genuinely
        # stale the zero stamp would quietly use old data, but the verdict is
        # measured from map->base_link independently, so a stale transform
        # shows up as a goal that was not reached, never as a false pass.
        goal_msg.pose.header.stamp = rclpy.time.Time().to_msg()
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


def tf_history_ready(buffer, goal_frame: str, base_frame: str,
                     now_ns: int, history_sec: float):
    """Can the buffer answer goal <- base as it was `history_sec` ago?

    Split out from the node so the rule can be tested without a ROS graph. The
    answer that matters is not "is there a transform now" -- a tree one message
    old satisfies that and still has no history for a costmap to look back
    through. Returns (ready, detail); detail is tf2's own complaint, which
    names which frame is missing or how short the buffer is, and is worth
    printing verbatim because the three failures it distinguishes need three
    different fixes.
    """
    if buffer is None:
        return False, "no tf2_ros: this build cannot check the transform tree"
    try:
        stamp = rclpy.time.Time(nanoseconds=max(0, now_ns - int(history_sec * 1e9)))
        buffer.lookup_transform(goal_frame, base_frame, stamp)
        return True, f"{goal_frame} <- {base_frame} answers for {history_sec:.1f} s ago"
    except Exception as exc:                     # tf2 raises several distinct types
        return False, str(exc).strip() or exc.__class__.__name__


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
    out += ")"
    # What NAV2 thought the gap was, from its own last feedback. The two numbers
    # answer different questions and only together say where the fault is: this
    # gate measures map -> base_link out of TF, the same transform Nav2 steers
    # by, so agreement means the controller stopped short of a goal it could see,
    # and disagreement means Nav2's pose estimate is not the one in TF.
    #
    # Needed for the 2wd leg that ended "as SUCCEEDED; 2.908 m from the goal"
    # after moving 0.100 m: with only one of the numbers there is no telling
    # whether Nav2 lied about arriving or was told it had already arrived.
    rem = getattr(node, "distance_remaining", float("nan"))
    if rem == rem:
        out += f" [nav2's own feedback: {rem:.3f} m remaining]"
    return out


# How far the three printed numbers may disagree before the reading is incoherent.
# map->base_link, /odom and map->odom are sampled by separate callbacks, so they
# never agree exactly; 0.5 m is far looser than the millimetres a settled tree
# shows and far tighter than the metres a frame mix-up produces.
POSE_COHERENCE_TOL_M = 0.5


def _pose_is_coherent(node):
    """Do the map pose, the odom pose and map->odom describe ONE robot?

    They are related by definition: map_pose = odom_pose + (map->odom). When that
    identity fails, the three were latched at incompatible instants and the "map"
    pose is not a map pose -- it is an odom pose wearing the label.

    Returns None when there is not enough to judge (no TF, no odom), which is not
    a failure: it just means no verdict may rest on the frame.

    THIS IS THE SECOND TIME THIS PROJECT HAS PAID FOR THE SAME MISTAKE, in the
    opposite direction. The first was the soak stopping a healthy run because it
    measured drift in map instead of /odom. This is a *false red* built the same
    way: on 2026-09-25, round ~180 of a 500-round soak, the verdict read

        ended at (-5.73, +1.91) in map, odom pose (-5.73, +1.91),
        map->odom (+5.91, -1.69) m after 0 s

    -- a "RAN AWAY" 6 m outside the room, declared after ZERO seconds. The map
    pose equalled the odom pose exactly while map->odom was nearly 6 m, so
    odom + offset put the robot at (+0.18, +0.22): home, where it actually was.
    The leg was judged in the first instant of the stack's life, before SLAM had
    published a converged map->odom, so map->base_link was composed through an
    identity offset.

    It gets worse with time, which is what makes it worth a guard rather than a
    note: /odom wanders steadily from its origin over hundreds of rounds (SLAM
    absorbs it into map->odom and the base stays home in map), so the longer a
    soak runs, the more certain it is that any incoherent instant reads as a
    runaway. A 500-round soak would have produced these indefinitely.
    """
    xy = getattr(node, "_last_xy", None)
    odom = getattr(node, "latest_odom", None)
    off = node.map_odom_offset() if hasattr(node, "map_odom_offset") else None
    if xy is None or odom is None or off is None or not getattr(node, "tf_ok", False):
        return None
    p = odom.pose.pose.position
    return math.hypot((p.x + off[0]) - xy[0], (p.y + off[1]) - xy[1]) <= POSE_COHERENCE_TOL_M


def _runaway(node) -> bool:
    """Has the base left the room?

    Distance from HOME, not from the goal: every goal in this test is within
    3 m of the origin, so one number covers both directions of every leg. A
    pose that has never been seen is not a runaway.

    Two things disqualify a runaway verdict before the distance is even looked at,
    and both are about whether the measurement could mean what it says:

      * an incoherent pose (see _pose_is_coherent) -- the number is in the wrong
        frame, and in a long soak the wrong frame ALWAYS looks like a runaway;
      * a base that never moved. Leaving a 6 m room requires crossing 6 m of
        floor, and `odom_max_dist` is the peak excursion the wheels themselves
        reported. A "runaway" whose base travelled ~nothing is an estimate that
        jumped, not a robot that drove -- and the base cannot be fooled about its
        own wheels, which is exactly why this test records that number separately.

    Module level, like _why/_gap/_where, so the rule is one implementation and
    can be exercised without a ROS graph.
    """
    xy = getattr(node, "_last_xy", None)
    if xy is None:
        return False
    if math.hypot(xy[0], xy[1]) <= RUNAWAY_RADIUS_M:
        return False
    if _pose_is_coherent(node) is False:
        return False
    # Half the radius: generous enough that a genuine runaway which slipped or
    # was pushed still counts, strict enough that a pose jump never does.
    if getattr(node, "odom_max_dist", 0.0) < RUNAWAY_RADIUS_M / 2:
        return False
    return True


def _sample_map_odom(node):
    """Record how long map->odom has gone without a new stamp.

    This is the measurement the recurring `error_code=102` needed. The
    message reads like a race --

        Lookup would require extrapolation into the future. Requested time
        X but the latest data is at X-0.018

    -- eighteen milliseconds, which looks like the request simply arrived
    between two 50 Hz publishes. It is not. nav2_util::transformPoseInTargetFrame
    calls tf_buffer.transform(..., timeout) and tf2 raises
    ExtrapolationException from that call ONLY after canTransform has
    already waited the whole timeout: the tolerance in force is the local
    costmap's 0.5 s, so the transform did not arrive for at least half a
    second. The 18 ms is the age of the last stamp before the stall, not
    the size of the gap.

    So the thing to measure is the longest interval between map->odom
    stamps -- but read it correctly, which took two tries.

    slam_toolbox's publishTransformLoop runs at 50 Hz
    (transform_publish_period: 0.02) and it is tempting to conclude that a
    gap means the loop stalled. It does not. The loop stamps what it sends
    from the SCAN, not from the clock:

        msg.header.stamp = scan_timestamp + transform_timeout_;   // restamp_tf false

    and scan_header is assigned at the top of laserCallback, on every
    incoming scan. So the stamp advances at the /scan rate and the loop
    merely republishes the same stamp in between. Our /scan is 10 Hz, so a
    ~100 ms interval is the floor and means nothing is wrong.

    A 600 ms interval therefore means SIX SCAN PERIODS WITH NO SCAN. The
    fault is upstream of SLAM -- the board's emulator, the LiDAR serial
    path, or the driver -- and slam_toolbox is only the messenger. Measured
    scan rates already show it: 5.30 Hz seen on a serial leg against a
    nominal 10.
    """
    if getattr(node, "tf_buffer", None) is None:
        return
    # RATE-LIMITED, and that is not an optimisation.
    #
    # First written without this, called once per spin_once() in the leg loop,
    # it put a TF lookup between the tester and every single callback it
    # processes -- feedback, /plan, /odom, the lot. The 2wd slice went from
    # 10/10 to 6/10 on identical firmware and images the first time it ran,
    # with two legs driving out of the room. Whether the sampler caused that or
    # merely coincided with it was no longer answerable, which is the whole
    # problem: a diagnostic that can perturb the thing it measures makes every
    # result afterwards arguable.
    #
    # 10 Hz is ample. The gap being hunted is half a SECOND in a 50 Hz
    # transform; sampling faster than the fault cannot make it more visible.
    now = time.time()
    if now - getattr(node, "_last_map_odom_sample", 0.0) < 0.1:
        return
    node._last_map_odom_sample = now
    try:
        tr = node.tf_buffer.lookup_transform(getattr(node, "goal_frame", "map"),
                                             getattr(node, "odom_frame", "odom"),
                                             rclpy.time.Time())
    except Exception:
        return
    stamp = tr.header.stamp.sec + tr.header.stamp.nanosec * 1e-9
    last = getattr(node, "_last_map_odom_stamp", None)
    last_wall = getattr(node, "_last_map_odom_wall", None)
    if last is not None and last_wall is not None and stamp > last:
        # How far the transform fell BEHIND wall time, not how long between our
        # own samples.
        #
        # This used to report `stamp - last`, the stamp delta between two
        # samples -- and since map->odom always carries a near-current stamp,
        # that delta IS the sampling interval. This node is single-threaded, so
        # whenever its executor was busy the metric reported the tester's own
        # scheduling as the robot's transform standing still. Every GenDrv leg
        # said "map->odom's stamp stood still for up to 560 ms" while a
        # dedicated subscriber watching the same transform through the same
        # drive measured 49.1 publishes/s, 49.1 stamp advances/s and not one
        # stall over 250 ms (2026-09-24).
        #
        # The lag is the honest quantity: wall time elapsed minus stamp advance.
        # A healthy transform gives ~0 however late this sampler ran, and a real
        # stall shows up at its true size.
        lag = (now - last_wall) - (stamp - last)
        if lag > getattr(node, "map_odom_max_gap", 0.0):
            node.map_odom_max_gap = lag
        sample_dt = now - last_wall
        if sample_dt > getattr(node, "map_odom_max_sample_dt", 0.0):
            node.map_odom_max_sample_dt = sample_dt
    if last is None or stamp > last:
        node._last_map_odom_stamp = stamp
        node._last_map_odom_wall = now
    # HOW BIG the correction is, not just how fresh.
    #
    # The same lookup already has it and was throwing it away. map->odom is
    # SLAM's correction to the wheels; on a simulated robot in a 10 x 6 m room
    # it should stay small, because the sim LiDAR sees a room that matches the
    # wheels exactly. On the 2026-09-23 mecanum slice a leg aborted with
    # error_code=203, "failed to plan from (3.80, -3.11)", while the drive suite
    # moments later showed the base at (+0.01, -0.00) driving 8/8 -- so the
    # correction had walked about 4.9 m and taken the robot's map pose outside
    # the room, and the planner refused a start it could not see.
    #
    # Reported on PASSING legs too, for the reason the gap is: a leg that passes
    # with a 3 m correction is one leg away from that abort, and a gate that only
    # shows the number when it has already failed cannot see it coming.
    t = tr.transform.translation
    offset = math.hypot(t.x, t.y)
    if offset > getattr(node, "map_odom_max_offset", 0.0):
        node.map_odom_max_offset = offset


def _map_odom_offset_note(node, room_half_y: float = 2.69) -> str:
    """How far SLAM's correction wandered, and whether that is survivable.

    room_half_y is the y half-extent the firmware clamps the simulated base to
    (SIM_MAP_HEIGHT 6.0 m, less the robot radius). A correction bigger than
    that can put the map pose outside the room on its own, with the base still
    where it should be -- which is exactly the 203 this measures.
    """
    worst = getattr(node, "map_odom_max_offset", 0.0)
    if worst <= 0.0:
        return ""
    note = f"; map->odom reached {worst:.2f} m"
    if worst >= room_half_y:
        note += (f" -- past the {room_half_y:.2f} m the room allows, so the map pose "
                 f"can be outside a room the BASE is still inside")
    return note


def _map_odom_gap_note(node, tolerance: float = 0.5) -> str:
    """Empty when map->odom kept up, or a note naming the worst stall."""
    worst = getattr(node, "map_odom_max_gap", 0.0)
    if worst <= 0.0:
        return ""
    # Sampled at 10 Hz, so a gap is known to within ~100 ms and a reported
    # value at or below that is "no stall seen", not a measurement.
    if worst <= 0.12:
        # Say what the sampler itself did, so a small lag measured by a badly
        # starved sampler is not read as a strong result.
        late = getattr(node, "map_odom_max_sample_dt", 0.0)
        extra = (f", this sampler's own worst interval {late * 1000:.0f} ms"
                 if late > 0.25 else "")
        return (f"; map->odom kept up (fell behind wall time by at most "
                f"{worst * 1000:.0f} ms{extra})")
    note = f"; map->odom fell behind wall time by up to {worst * 1000:.0f} ms"
    if worst >= tolerance:
        note += (f" -- at or past the {tolerance:.1f} s transform tolerance, so a "
                 f"controller TF error here is a symptom of the scan gap, not of TF")
    return note


def _scan_gap_note(node, nominal_hz: float = 10.0) -> str:
    """Name which end of the scan path gapped: the board's, or the wire's.

    Reported together, because the interesting case is the DIFFERENCE. Scans
    stamped 100 ms apart that arrive 1.2 s apart were generated on time and
    held up in transport; stamps 1.2 s apart mean nothing was generated to
    hold up, and no amount of transport tuning touches it.
    """
    if not getattr(node, "scan_count", 0):
        return "; no /scan seen by the tester"
    period = 1.0 / nominal_hz
    stamp_gap = getattr(node, "scan_max_stamp_gap", 0.0)
    arrival_gap = getattr(node, "scan_max_arrival_gap", 0.0)
    if max(stamp_gap, arrival_gap) <= period * 1.5:
        return f"; /scan steady ({node.scan_count} scans, no interval over {period * 1500:.0f} ms)"
    note = (f"; /scan gapped: worst stamp interval {stamp_gap * 1000:.0f} ms, "
            f"worst arrival interval {arrival_gap * 1000:.0f} ms "
            f"({node.scan_count} scans, {period * 1000:.0f} ms nominal)")
    if stamp_gap > period * 1.5 and arrival_gap <= stamp_gap * 1.2:
        note += " -- scans were not produced, so this is upstream of the wire"
    elif arrival_gap > stamp_gap * 1.5:
        # NOT "so this is the transport". The arrival interval is measured in
        # THIS process, whose executor is single-threaded and spins with a
        # 0.2 s timeout while also running the leg logic and a 10 Hz TF lookup.
        # A late arrival here is the transport OR this tester being busy, and
        # nothing on this side can separate them. Said as a fact plus its
        # ambiguity, because the first version of this line named the transport
        # outright and would have sent the next reader to tune DDS on the
        # strength of the tester's own scheduling.
        note += (" -- stamped on time and seen late, which is the transport or this "
                 "tester's own single-threaded executor; measured here they cannot "
                 "be told apart")
    return note


def _where(node) -> str:
    """WHERE it ended, not just how far off -- and in which frame.

    A leg that reports "13.667 m from the goal" after traversing 11.235 m has
    three explanations and the distance alone cannot tell them apart:

      * the base really drove out of the room -- odom says 11 m and map->odom
        is small;
      * SLAM's correction ran away -- odom says it went nowhere and map->odom
        carries the 11 m;
      * the measurement is the odom fallback, because TF could not answer, and
        the two frames were never comparable.

    So print all three: the map pose the verdict was measured from, the base's
    own odometry, and the correction between them. Measured on a mecanum leg
    (2026-09-23) whose nav2.log logged no transform or path complaint at all,
    which is what made the distance uninterpretable.
    """
    bits = []
    xy = getattr(node, "_last_xy", None)
    if xy:
        frame = (getattr(node, "goal_frame", "map") if getattr(node, "tf_ok", False)
                 else "odom (TF could not answer)")
        bits.append(f"ended at ({xy[0]:+.2f}, {xy[1]:+.2f}) in {frame}")
    odom = getattr(node, "latest_odom", None)
    if odom is not None:
        p = odom.pose.pose.position
        bits.append(f"odom pose ({p.x:+.2f}, {p.y:+.2f})")
    off = node.map_odom_offset() if hasattr(node, "map_odom_offset") else None
    if off is not None:
        bits.append(f"map->odom ({off[0]:+.2f}, {off[1]:+.2f}) m")
    # The three numbers above are related by definition. When they are not, say so
    # HERE, next to them, rather than leaving a reader to do the subtraction: an
    # incoherent triple is why a leg can fail in the stack's first instant, and it
    # is the difference between "the robot went there" and "nobody knows where it
    # was". _runaway() refuses to fire on one; this is how it gets reported.
    if _pose_is_coherent(node) is False:
        p = node.latest_odom.pose.pose.position
        bits.append(f"INCOHERENT: odom+offset puts it at ({p.x + off[0]:+.2f}, "
                    f"{p.y + off[1]:+.2f}), so these were latched at different "
                    f"instants and the map pose is not one (SLAM had not published "
                    f"a converged map->odom yet)")
    return ("; " + ", ".join(bits)) if bits else ""


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

        The reported TWIST, not the integrated pose. A sim-mode board at rest
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

    # The room's one obstacle (sim_ld19.h): a wall at x = 2.0 spanning y = -1.5..1.5.
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

        The simulated robot is pushed off the segment (sim_ld19.h clampToRoom,
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

        The POSITION decides. A controller can stop short and report success,
        and can equally be aborted by a behaviour-tree timeout while sitting on
        top of the goal -- so the status explains, and the distance is the fact.

        This used to be `goal_completed OR inside the tolerance`, which let a
        SUCCEEDED override the position entirely. Measured on the Yahboom
        (2026-09-22): leg 2/8 was reported "reached in 2 s, closest 2.830 m"
        against a 0.40 m tolerance, because Nav2 called the goal SUCCEEDED while
        the robot sat where leg 1 had left it. Leg 3 then began 0.292 m from its
        own goal and was correctly failed as vacuous -- one leg's false arrival
        became the next leg's missing journey.

        The status is only consulted when the position was never measured: with
        no pose there is nothing better, and the verdict says so.
        """
        dist = getattr(node, "goal_dist_min", float("inf"))
        if dist == dist and dist != float("inf"):     # a pose was seen
            return dist <= goal_tolerance
        return node.goal_completed

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

    def close_goal(node, wait: float) -> bool:
        """Cancel the goal in flight and spin until Nav2 gives it a terminal status.

        A leg ends when the ROBOT is at the goal, which is routinely before
        bt_navigator says so. Whoever sends the next goal -- this process or the
        next invocation -- gets "another navigator is processing, rejecting
        request" until the old one lets go. Returns True if the goal is closed.
        """
        if node.goal_status != -1:
            return True
        node.cancel_current_goal()
        t = time.time()
        asked_twice = False
        while node.goal_status == -1 and time.time() - t < wait:
            rclpy.spin_once(node, timeout_sec=0.2)
            # One repeat halfway: a cancel sent while the server is blocked on a
            # service call is dropped, and the second one lands.
            if not asked_twice and time.time() - t > wait / 2:
                node.cancel_current_goal()
                asked_twice = True
        return node.goal_status != -1

    def run_legs(legs) -> bool:
        """Drive every leg in turn; each must arrive, and each that crosses the
        wall must have planned around it. One failure ends the run -- the
        pipeline then asks the base directly whether it still drives."""
        n = len(legs)
        retried_startup = False
        retried_phantom = False

        def await_arrival(t0: float) -> bool:
            """Spin until this leg resolves. Returns whether the robot arrived.

            ONE loop, shared by the first dispatch and both retries. It used to
            be copied: the first copy sampled map->odom and watched for a
            runaway, the retry's copy did neither -- so a retried leg was
            measured by a weaker instrument than a first attempt, silently, and
            a runaway on a retry would have been reported as a plain timeout.
            """
            phantom_since = None
            while time.time() - t0 < timeout:
                rclpy.spin_once(node, timeout_sec=0.2)
                # Sampled here rather than in a callback: the point is how long
                # map->odom went WITHOUT a new stamp, and a transform that has
                # stopped arriving fires no callback to notice it by.
                _sample_map_odom(node)
                if node.goal_rejected:
                    return False
                if start_gap_is_meaningful() and reached_goal() and wall_path_ok() \
                        and (moved() or not require_motion):
                    return True
                if _runaway(node):
                    return False         # gone; see the report below
                if node.goal_status in (5, 6) and not reached_goal():   # CANCELED, ABORTED
                    return False         # Nav2 gave up; waiting out the window adds nothing
                if node.goal_status == 4 and not reached_goal():        # SUCCEEDED
                    # A phantom -- but give the tester's OWN pose a moment to
                    # catch up first. Breaking on the first sample would turn a
                    # real arrival whose last /odom had not landed yet into a
                    # failure, which is the mirror of the bug being fixed.
                    if phantom_since is None:
                        phantom_since = time.time()
                    elif time.time() - phantom_since > PHANTOM_CONFIRM_SEC:
                        return False
            return False

        for i, (gx, gy) in enumerate(legs, 1):
            node.begin_leg(gx, gy)
            if not node.send_goal():
                return False
            # A rejection right after the previous leg is the handshake, not the
            # navigation: give bt_navigator a moment and ask once more.
            t_acc = time.time()
            while time.time() - t_acc < 5 and not (node.goal_accepted or node.goal_rejected):
                rclpy.spin_once(node, timeout_sec=0.2)
            # The THIRD outcome, which used to fall through as though the goal were
            # under way: neither accepted nor rejected within 5 s. That is a slow
            # handshake, not a verdict -- bt_navigator logs it as "Timed out while
            # waiting for action server to acknowledge goal request for
            # compute_path_to_pose", and it cost round 157 of the 2026-09-24 soak.
            # Falling through started the leg's clock on a goal the server had not
            # yet taken, so the timeout was charged to navigation that had not begun.
            # Wait the rest of the way instead: an unacknowledged goal is the
            # handshake being slow, and the retry below is for a REFUSED one.
            if not (node.goal_accepted or node.goal_rejected):
                print(f"   leg {i}/{n}: no acknowledgement after 5 s; the handshake is "
                      f"slow, not the navigation. Waiting up to {ACCEPT_GRACE_S:.0f} s more.")
                t_g = time.time()
                while time.time() - t_g < ACCEPT_GRACE_S and not (node.goal_accepted
                                                                 or node.goal_rejected):
                    rclpy.spin_once(node, timeout_sec=0.2)
                if not (node.goal_accepted or node.goal_rejected):
                    print(f"❌ NAV2 GOAL NOT ACKNOWLEDGED on leg {i}/{n} "
                          f"({gx:.2f}, {gy:.2f}) after {5 + ACCEPT_GRACE_S:.0f} s: "
                          f"bt_navigator never answered send_goal, so this leg never "
                          f"started. Nothing here is about the robot.")
                    return False
            if node.goal_rejected:
                print(f"   leg {i}/{n}: bt_navigator rejected the goal; waiting 5 s and asking once more.")
                t_w = time.time()
                while time.time() - t_w < 5:
                    rclpy.spin_once(node, timeout_sec=0.2)
                node.begin_leg(gx, gy)
                if not node.send_goal():
                    return False
            t0 = time.time()
            arrived = await_arrival(t0)
            took = time.time() - t0
            if node.goal_rejected:
                print(f"❌ NAV2 GOAL REJECTED by bt_navigator on leg {i}/{n} "
                      f"({gx:.2f}, {gy:.2f}){_why(node)}.")
                return False
            # An abort in the first seconds, with no plan and no motion, is the
            # stack still coming up -- not a navigation failure. The lifecycle
            # says "active" once every node has configured, which is before the
            # costmaps have a scan to build from, so the first goal can be
            # accepted and dropped with nothing attempted. Measured on the
            # GenDrv (2026-09-22, jazzy): ABORTED after 0 s, 0.000 m traversed,
            # planned_around_wall=False, on a stack whose /scan was 8.8 Hz and
            # whose /map was publishing. A real abort takes time and shows
            # movement, so this costs nothing when the failure is genuine.
            if (not arrived and node.goal_status == 6 and took < STARTUP_ABORT_SEC
                    and node.leg_max_dist < STARTUP_ABORT_DIST
                    and not node.path_avoids_wall and not retried_startup):
                retried_startup = True
                print(f"   leg {i}/{n}: aborted after {took:.1f} s having moved "
                      f"{node.leg_max_dist:.3f} m and planned nothing — the stack was still "
                      f"settling. Waiting {STARTUP_SETTLE_SEC} s and asking once more.")
                t_w = time.time()
                while time.time() - t_w < STARTUP_SETTLE_SEC:
                    rclpy.spin_once(node, timeout_sec=0.2)
                node.begin_leg(gx, gy)
                if not node.send_goal():
                    return False
                t0 = time.time()
                arrived = await_arrival(t0)
                took = time.time() - t0

            # Nav2 said it arrived and the robot is not there. Ask once more.
            if (not arrived and node.goal_status == 4 and not reached_goal()
                    and start_gap_is_meaningful() and not retried_phantom):
                retried_phantom = True
                print(f"   leg {i}/{n}: Nav2 reported the goal SUCCEEDED after {took:.1f} s "
                      f"with the robot {node.goal_dist_min:.3f} m away, having moved "
                      f"{node.leg_max_dist:.3f} m. The position decides, so this leg has not "
                      f"arrived. Waiting {PHANTOM_SETTLE_SEC:.0f} s and asking once more: if it "
                      f"drives this time the first answer was a phantom, and if it does not, "
                      f"the failure is the robot's.")
                t_w = time.time()
                while time.time() - t_w < PHANTOM_SETTLE_SEC:
                    rclpy.spin_once(node, timeout_sec=0.2)
                node.begin_leg(gx, gy)
                if not node.send_goal():
                    return False
                t0 = time.time()
                arrived = await_arrival(t0)
                took = time.time() - t0
                if arrived:
                    print(f"   leg {i}/{n}: it drove on the second ask, so the first SUCCEEDED "
                          f"was a phantom -- bt_navigator answering for a goal it never ran.")
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
                                   f"check that sim_ld19 is enabled and clampToRoom is reached.")
                        print(f"❌ NAV2 LEG {i}/{n} DROVE INTO THE WALL: {_whose} "
                              f"crossed x={WALL_X:.1f} at y={ys}, inside the wall's span "
                              f"(±{WALL_HALF_SPAN:.1f} m){_gap(node)}. {why}")
                    else:
                        print(f"❌ NAV2 LEG {i}/{n} REACHED ({gx:.2f}, {gy:.2f}) WITHOUT GOING AROUND "
                              f"THE WALL{_gap(node)}: no /plan detoured around x={WALL_X:.1f} and the "
                              f"robot never crossed it beyond ±{WALL_HALF_SPAN:.1f} m. It cannot have "
                              f"got there; check the frames (a goal in map, a pose read from odom).")
                else:
                    if _runaway(node):
                        print(f"❌ NAV2 LEG {i}/{n} RAN AWAY: asked for ({gx:.2f}, {gy:.2f}), "
                              f"left the {RUNAWAY_RADIUS_M:.0f} m room instead"
                              f"{_why(node)}{_gap(node)}{_where(node)} after {took:.0f} s. "
                              f"The base was still taking commands, so this is the controller "
                              f"driving it away, not a stall -- and nothing in sim mode would "
                              f"have stopped it.")
                        return False
                    # WHERE first, then why. The order is not cosmetic: on the
                    # 2026-09-23 mecanum 203 this line stopped INSIDE _why's
                    # error_msg repr --
                    #
                    #   ended as ABORTED (error_code=203, error_msg='GridBased
                    #   plugin failed to plan from (3.80, -3.11) [q: 0.00, 0.
                    #
                    # -- and everything after was gone, including _where, which
                    # is the one thing that separates "the base drove out of the
                    # room" from "the estimate drifted there". I could not
                    # establish what truncated it (no timeout marker, no stray
                    # newline, the line simply ends), so the fix is to put the
                    # irreplaceable part where a tail cut cannot reach it rather
                    # than to claim a cause. Nav2's error_msg is the replaceable
                    # half: it is in nav2.log too, and _nav2_complaints lifts it
                    # out separately.
                    print(f"❌ NAV2 LEG {i}/{n} NOT REACHED: ({gx:.2f}, {gy:.2f}) ended as "
                          f"{_status_name(node.goal_status)}{_where(node)}{_gap(node)}{_why(node)} after "
                          f"{took:.0f} s; needed within {goal_tolerance:.2f} m; "
                          f"planned_around_wall={node.path_avoids_wall}, "
                          f"traversed {node.leg_max_dist:.3f} m this leg"
                          f"{_map_odom_gap_note(node)}{_map_odom_offset_note(node)}{_scan_gap_note(node)}")
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
            # EVERY leg, including the last: the next goal need not come from
            # this process. A soak that left its final goal open had the next
            # round's goal rejected, and the next, until Nav2 let go -- 38 of 60
            # rounds red, each one preceded by exactly this line (2026-09-23).
            if not close_goal(node, GOAL_CLOSE_SEC):
                print(f"   leg {i}/{n}: Nav2 has not closed the goal {GOAL_CLOSE_SEC:.0f} s after "
                      f"the robot "
                      f"arrived; continuing. The next goal sent to this stack may be "
                      f"rejected while the old one is still running.")
            # A terminal status is bt_navigator answering, not bt_navigator
            # finished: the tree is still halting behind it. Every goal in this
            # tester went out ONE MILLISECOND after the previous cancel returned,
            # and the legs that came back SUCCEEDED in 2.9 s without moving are
            # the ones where that landed badly. Half a second against a 17 s leg
            # costs nothing and removes the race from the measurement.
            t_w = time.time()
            while time.time() - t_w < LEG_SETTLE_SEC:
                rclpy.spin_once(node, timeout_sec=0.05)
        # The worst map->odom gap goes on the PASSING line too. A failing leg
        # reporting 601 ms only says the stall happened; how close a healthy
        # run comes to the 0.5 s tolerance is what says whether the margin is
        # comfortable or whether every green leg was one hiccup from red.
        return verdict(f"NAV2 GOAL REACHED {n}/{n} legs: {round_trips} round trip(s) behind "
                       f"the obstacle wall and back home (within {goal_tolerance:.2f} m)"
                       f"{_map_odom_gap_note(node)}{_map_odom_offset_note(node)}{_scan_gap_note(node)}")

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

    def wait_for_tf() -> bool:
        """Do not dispatch anything until the transform tree can answer.

        This is a PRECONDITION, not another retry. What the pipeline waits for
        today is lifecycle activation -- every node configured and activated --
        and a goal sent on that signal alone has been accepted by a stack whose
        costmaps had no tree to look through yet. Three distinct complaints, all
        on 2026-09-23, all moments after "Nav2 stack active":

            Tf has two or more unconnected trees
            Invalid frame ID "base_link" ... frame does not exist
            Requested time ...581103 but the earliest data is at ...756795

        The existing remedy is reactive and narrow: retry once when a leg aborts
        inside 5 s having moved under 0.05 m with no plan. The leg that failed
        this way on mecanum had planned around the wall and driven 1.999 m, so
        it never matched -- a stack can be far enough up to plan and drive and
        still be short of the history a costmap reads back through.

        A failure here is reported as its own verdict rather than as a
        navigation failure, because it is one: nothing was ever asked of Nav2.
        """
        # No tf2_ros at all is a different thing from a tree that never came
        # up, and it is not this gate's to fail: world_xy already falls back to
        # the odometry pose in that build, so there is no map frame to be ready.
        # Refusing here would turn a working configuration into a red leg.
        if getattr(node, "tf_buffer", None) is None:
            print("   TF readiness not checked: no tf2_ros in this build, so the "
                  "verdict is measured from /odom and there is no map frame to wait for.")
            return True
        t0 = time.time()
        detail = "no attempt"
        while time.time() - t0 < TF_READY_TIMEOUT_SEC:
            rclpy.spin_once(node, timeout_sec=0.2)
            ready, detail = tf_history_ready(node.tf_buffer, node.goal_frame, node.base_frame,
                                             node.get_clock().now().nanoseconds,
                                             TF_READY_HISTORY_SEC)
            if ready:
                print(f"   TF ready after {time.time() - t0:.1f} s: {detail}")
                return True
        print(f"❌ NAV2 TF NEVER READY: {node.goal_frame} <- {node.base_frame} could not be "
              f"resolved for {TF_READY_HISTORY_SEC:.1f} s ago within {TF_READY_TIMEOUT_SEC:.0f} s "
              f"of the stack reporting active. No goal was sent, so this is not a navigation "
              f"result.\n   tf2 says: {detail}")
        return False

    try:
        if not wait_for_tf():
            return False

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

            # SUCCEEDED is Nav2's account of itself, and a goal demanded with
            # --require-goal is judged on the distance instead: see
            # reached_goal(). Without --require-goal this is the old, looser
            # contract -- the caller asked whether Nav2 finished, not whether
            # the robot is on the spot.
            if node.goal_completed and not require_goal \
                    and (moved() or not require_motion):
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
        if node.goal_completed and not require_goal:
            return verdict("NAV2 GOAL COMPLETED (after the window)")
        if node.goal_completed and require_goal and not reached_goal():
            print(f"❌ NAV2 GOAL NOT REACHED: Nav2 reported SUCCEEDED, but the robot is "
                  f"{node.goal_dist_now:.3f} m from ({goal_x:.2f}, {goal_y:.2f}) and this gate "
                  f"needs {goal_tolerance:.2f} m. The status explains; the distance decides.")
            return False
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
        # Never hand the stack back with a goal still running: the next process
        # to send one gets rejected, and a soak reads that as a robot fault.
        try:
            if not close_goal(node, GOAL_CLOSE_EXIT_SEC):
                print("   note: this run exited with a Nav2 goal still open; the next goal "
                      "sent to this stack may be rejected until bt_navigator lets go.")
        except Exception as exc:                  # a shutdown must not eat the verdict
            print(f"   note: could not close the goal on exit: {exc}")
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
                             "a sim-mode board at rest has been seen reporting 0.036")
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
