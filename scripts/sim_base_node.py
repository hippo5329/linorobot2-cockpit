#!/usr/bin/env python3
# ==============================================================================
# sim_base_node.py — the robot's base, simulated on the robot computer
#
# The other half of scripts/sim_laser_node.py. That one already raycasts the
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
# carries a transcription of SimEncoder::integrate(), the shared battery, and
# PID::compute() with its anti-windup, all checked against the firmware by
# tests/test_test_acc_simulation.py and tests/test_pid_auto_tune.py. This node
# imports them. A second copy of the wheel model is the one thing that would
# make this instrument lie.
#
#   ros2 run ... sim_base_node.py --ros-args -p params:=<robot>_config.yaml
# ==============================================================================
import math
import os
import random
import sys
import time

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


def _noise(peak):
    """simWheelNoise(): uniform on +/-peak, which is what the board uses."""
    return random.uniform(-peak, peak)


def _clamp_bias(v, limit):
    limit = abs(limit)
    return max(min(v, limit), -limit)


class SimIMU:
    """The board's simulated IMU, output and all.

    This matters more than it looks. The first version of this node published a
    PERFECT gyro, and a perfect gyro is a different robot to fuse: the board's
    has a fixed 0.004 rad/s bias and a bounded random walk on top, which over a
    four-round-trip run is minutes of integration for the EKF to fight. An
    instrument that leaves that out will happily pass runs the board fails, and
    then the absence of a reproduction means nothing.

    Bounded, like the firmware's: a free random walk has no limit, and an
    unbounded one let uptime decide whether SLAM worked -- a stationary gyro
    read 4.5x its nominal bias after a long session, which the EKF turned into
    52 degrees of yaw drift in a minute.
    """

    def __init__(self, d):
        self.d = d
        self.gyro_bias = 0.0
        self.k = 1.0 + d["scale_error"]

    def read(self, wz, ax, dt):
        rw = math.sqrt(max(dt, 0.0))
        self.gyro_bias += _noise(self.d["gyro_drift"]) * rw
        self.gyro_bias = _clamp_bias(self.gyro_bias, self.d["gyro_bias"])
        return (wz * self.k + self.gyro_bias + _noise(self.d["gyro_noise"]),
                ax * self.k + _noise(self.d["accel_noise"]))


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
        # getRPM() adds +/-SIM_WHEEL_NOISE_RPM to what it REPORTS, so the
        # odometry -- and therefore SLAM and Nav2 -- sees a noisy wheel. The PID
        # above reads wheel.rpm directly, which is what the board does too:
        # feed() and getRPM() are separate calls and only the latter is noisy.
        return [w.rpm + _noise(self.d["noise_rpm"]) for w in self.wheels]


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


class Wire:
    """The link between the board and the computer: latency, and jitter on it.

    Why this exists. The boardless stack passed an eight-leg Nav2 run that three
    boards were failing, and the difference is not the robot -- the model, the
    kinematics, the limits and the planner are all the same code. What the host
    does not have is micro-ROS over a serial or Wi-Fi link: on a board, `/odom`
    is STAMPED when the wheels were read and ARRIVES some milliseconds later,
    and that gap is what the TF buffer, the EKF and the controller actually
    contend with. A publisher with zero latency is a robot nobody can build.

    Two properties of a real link are modelled, and one is deliberately not:

      delay    the stamp stays the SAMPLE time and the message is held back.
               Publishing late with a late stamp would hide the very thing this
               reproduces -- an extrapolation request into a buffer whose newest
               entry is older than the controller expects, which is the shape of
               every 102/103 this project has chased.
      jitter   the gap between arrivals varies. Bus contention, the agent's
               scheduling, the board's loop competing with the radio.
      ORDER    is preserved, always. A serial stream and an XRCE session deliver
               in order; a link that reorders is a different fault and modelling
               one here would invent a failure the bench cannot have. Jitter
               therefore moves each arrival no earlier than the one before it.

    Defaults to zero -- no delay, no jitter, publish immediately -- so it changes
    nothing until somebody asks for it. The number to put here is a MEASURED
    round-trip on a board, and nobody has measured one yet; sweeping it is how
    you find out how much latency the stack tolerates before the gate goes red.
    """

    # WHAT A MESSAGE COSTS ON THE WIRE, in bytes, counted off the message
    # definitions rather than guessed:
    #
    #   Odometry        header + child_frame_id + 7 doubles of pose + 36 of its
    #                   covariance + 6 doubles of twist + 36 of its covariance
    #   Imu             header + 4 doubles of orientation + 9 of covariance,
    #                   then 3 + 9 for angular velocity and 3 + 9 for linear
    #                   acceleration
    #   MagneticField   header + 3 doubles + 9 of covariance
    #
    # Plus CDR alignment and the XRCE framing around each one, which is why these
    # are round numbers slightly above the arithmetic rather than exact: the
    # point is the RATIO between the three, and that a 50 Hz triple does not fit
    # in a 115200 baud budget.
    SIZES = {"odom": 720, "imu": 330, "mag": 120, "tf": 130}

    def __init__(self, delay_s=0.0, jitter_s=0.0, bytes_per_s=0.0):
        self.delay = max(float(delay_s), 0.0)
        self.jitter = max(float(jitter_s), 0.0)
        self.queue = []
        self.last_due = 0.0
        # THE LINK'S BANDWIDTH, and the drops that follow from running out of it.
        #
        # Latency alone is not what a serial link does to a robot. A micro-ROS
        # session over 8N1 carries baudrate/10 bytes per second, and the 50 Hz
        # triple (odom + imu + mag) is about 1170 bytes per cycle, i.e. 58 kB/s.
        # That fits in 921600 baud (92 kB/s) and does not fit in 115200
        # (11.5 kB/s) -- and the firmware publishes BEST EFFORT, so what does not
        # fit is not delayed, it is GONE.
        #
        # Which matters more than it looks, because madgwick pairs imu/data_raw
        # with imu/mag through an ApproximateTime synchroniser five deep: an
        # unpaired IMU sample produces no imu/data at all. So a link that drops
        # 30% of messages costs far more than 30% of the EKF's orientation
        # input, and the bench has already shown the asymmetry it predicts --
        # /odom at 33 Hz beside /imu/data at 10 Hz on the same leg.
        #
        # 0 is unlimited, which is what every existing leg gets.
        self.bytes_per_s = max(float(bytes_per_s), 0.0)
        self.budget = 0.0
        self.budget_time = None
        self.dropped = {}
        self.sent = {}

    @property
    def instant(self):
        # Delay and jitter only. The budget decides WHETHER a message goes, the
        # delay decides WHEN -- two independent properties of a link, and folding
        # the budget in here made a rate-limited leg with no delay queue every
        # message behind a pump it did not need.
        return self.delay <= 0.0 and self.jitter <= 0.0

    def afford(self, now, kind):
        """Token bucket, one cycle deep. True when this message fits.

        The bucket holds one control cycle's worth and no more: a real UART has
        no backlog to spend later, and a deeper bucket would let a quiet second
        pay for a burst the link could never actually carry.
        """
        if self.bytes_per_s <= 0.0:
            return True
        if self.budget_time is None:
            self.budget_time = now
            self.budget = self.bytes_per_s * 0.02
        else:
            self.budget += (now - self.budget_time) * self.bytes_per_s
            self.budget_time = now
            cap = self.bytes_per_s * 0.02
            if self.budget > cap:
                self.budget = cap
        cost = self.SIZES.get(kind, 200)
        if self.budget >= cost:
            self.budget -= cost
            self.sent[kind] = self.sent.get(kind, 0) + 1
            return True
        self.dropped[kind] = self.dropped.get(kind, 0) + 1
        return False

    def send(self, now, publish, kind=None):
        if kind is not None and not self.afford(now, kind):
            return              # best effort: what does not fit is gone
        if self.instant:
            publish()
            return
        due = now + self.delay + random.uniform(-self.jitter, self.jitter)
        # In order, never earlier than the previous message. This is the whole
        # difference between "a slow link" and "a link that shuffles packets",
        # and only the first one is real here.
        due = max(due, self.last_due, now)
        self.last_due = due
        self.queue.append((due, publish))

    def pump(self, now):
        while self.queue and self.queue[0][0] <= now:
            _, publish = self.queue.pop(0)
            publish()


class SimBaseNode(Node):
    def __init__(self):
        super().__init__("sim_base_node")
        self.declare_parameter("params", "")
        self.declare_parameter("stamped_cmd_vel", False)
        self.declare_parameter("rate", 50.0)          # CONTROL_TIMER, 50 Hz
        # The firmware steps the model on micros() DELTAS. This node stepped it on
        # the nominal 1/rate instead, which is deterministic and therefore lovely
        # for CI -- and it is also the one property that makes this instrument
        # blind to a whole class of defect. A late timer callback on a loaded box
        # is exactly the long interval that breaks an explicit Euler step, and a
        # model told "20 ms" when 400 ms passed cannot reproduce it; worse, the
        # pose then advances slower than wall time while Nav2 plans in wall time,
        # so the simulated robot goes sluggish under load for a reason no board
        # has. Measured by default; set false for a bit-reproducible run.
        self.declare_parameter("measured_dt", True)
        # A STARVED CONTROL LOOP, on demand.
        #
        # The defect this instrument exists to reproduce needs a long interval,
        # and a20 at load 2.4 on 32 cores never produces one: across six 40-goal
        # legs on 2026-09-25, not a single tick passed 100 ms. Waiting for a fast
        # machine to stutter is not an experiment. So the stall is a parameter --
        # every `control_stall_every_s` seconds the callback blocks for
        # `control_stall_ms`, which is what a board whose loop is starved does to
        # the interval, and to the publishing, at the same time.
        #
        # Sweeping it answers the question the bench cannot: with the model
        # bounded, how long a stall can the stack absorb before Nav2 loses the
        # robot -- and is that anywhere near what a starved board actually shows?
        # -1 means "take it from the config", the same convention the transport
        # delay uses, so a sweep can write it into the YAML it already writes.
        self.declare_parameter("control_stall_ms", -1.0)
        self.declare_parameter("control_stall_every_s", -1.0)
        # WHERE the stall lands decides whether it is an experiment at all.
        #
        # Measured 2026-09-25 on the unsliced model (gendrv/mecanum, ceiling
        # 140 rpm): one 920 ms interval taken at CRUISE duty peaks at 70.7 rpm --
        # no divergence whatsoever, because an explicit Euler step of a
        # first-order system is only unstable where the driving term is large, and
        # at equilibrium (no_load - rpm) is nearly zero. The same interval taken at
        # a full-duty step peaks at 324 rpm, and from rest at 574.
        #
        # So a stall on a fixed 5 s cadence mostly fires at cruise and proves
        # nothing, which is what a sweep of them returning all-green was about to
        # be read as. `on_command_change` fires it where the fault actually lives:
        # a starved loop crossing a command change, which is what a Nav2 stack
        # hands a board several times per goal.
        self.declare_parameter("control_stall_on_command_change", False)
        # OFF, because the board does not publish a transform either.
        #
        # main.cpp has no TransformBroadcaster at all: it publishes
        # odom/unfiltered as a TOPIC and the EKF owns `odom -> base_link`
        # (`publish_tf: true` in every reference config). A base that also
        # broadcasts it puts two publishers on one transform -- the exact
        # collision bringup.launch.py already avoids by forcing madgwick's
        # publish_tf to false.
        #
        # It is not a harmless duplicate. On 2026-09-24 it silently invalidated
        # a latency sweep: the EKF's FRESH transform masked this node's delayed
        # one, so legs at 800 and 1200 ms appeared to navigate when the config's
        # transform tolerances are 0.2-0.5 s and should have refused them. The
        # instrument was reporting a robot that could not exist.
        #
        # Left as a parameter for a stack run without an EKF, which is the only
        # case where something has to publish it.
        self.declare_parameter("publish_tf", False)
        # Milliseconds, because that is the unit anybody reasoning about a
        # serial link thinks in. -1 means "take it from the config".
        self.declare_parameter("transport_delay_ms", -1.0)
        self.declare_parameter("transport_jitter_ms", -1.0)

        path = str(self.get_parameter("params").value)
        if not path or not os.path.isfile(path):
            raise SystemExit("sim_base_node: -p params:=<robot>_config.yaml is required "
                             f"(got {path!r})")
        with open(path, encoding="utf-8") as fh:
            params = yaml.safe_load(fh) or {}
        self.d = dr.drivetrain(params)
        if self.d["circ"] <= 0 or self.d["max_rpm"] <= 0:
            raise SystemExit("sim_base_node: kinematics.wheel_diameter and max_rpm "
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

        # The link, from the ROS parameter if given, else from the robot's own
        # `simulation` block, else zero -- the same precedence every other knob
        # in this project follows.
        sim = (params.get("base_controller") or {}).get("simulation") or {}

        def _link(param, key):
            got = float(self.get_parameter(param).value)
            if got < 0.0:
                got = float(sim.get(key, 0.0) or 0.0)
            return max(got, 0.0) / 1000.0

        # The link's bandwidth, from the BAUD RATE the config already states --
        # 8N1, so ten bits per byte. A board that says `baudrate: 921600` is
        # telling us its budget; there is no second number to invent.
        # `transport_bytes_per_s` overrides it directly for a sweep, and 0 (the
        # default everywhere until asked) means unlimited, as before.
        baud = float((params.get("base_controller") or {}).get("baudrate") or 0.0)
        bps = float(sim.get("transport_bytes_per_s", 0.0) or 0.0)
        if bps <= 0.0 and sim.get("transport_rate_limit"):
            bps = baud / 10.0
        self.wire = Wire(_link("transport_delay_ms", "transport_delay_ms"),
                         _link("transport_jitter_ms", "transport_jitter_ms"),
                         bps)
        if bps > 0.0:
            self.get_logger().info(
                f"link budget {bps / 1000.0:.1f} kB/s"
                + (f" (from baudrate {baud:.0f}, 8N1)" if baud else "")
                + "; best effort, so what does not fit is dropped")
        self.imu = SimIMU(self.d)
        self.cmd = (0.0, 0.0, 0.0)
        self.cmd_time = self.get_clock().now()
        self.x = self.y = self.yaw = 0.0
        self.prev_vx = 0.0
        rate = float(self.get_parameter("rate").value)
        self.dt = 1.0 / rate
        self.measured_dt = bool(self.get_parameter("measured_dt").value)
        self.prev_mono = None
        self.max_dt = 0.0          # the worst interval this run, reported below
        self.stall = _link("control_stall_ms", "control_stall_ms")
        # Seconds, not milliseconds, so it does not go through _link's /1000.
        self.stall_every = float(self.get_parameter("control_stall_every_s").value)
        if self.stall_every < 0.0:
            self.stall_every = float(sim.get("control_stall_every_s", 0.0) or 0.0)
        if self.stall_every <= 0.0:
            self.stall_every = 5.0
        self.stall_next = None
        self.stalls = 0
        # Config fallback like the rest of the simulation block, so a sweep that
        # writes the YAML can aim the stall without a launch argument.
        self.stall_on_change = bool(
            self.get_parameter("control_stall_on_command_change").value
            or sim.get("control_stall_on_command_change", False))
        self.prev_cmd = None
        if self.stall > 0.0 and not self.measured_dt:
            # Otherwise the stall is invisible to the model -- it would be told
            # the nominal period while a quarter of a second passed, which is the
            # exact blindness this was added to remove.
            raise SystemExit("control_stall_ms needs measured_dt:=true to mean anything")
        self.create_timer(self.dt, self._tick)
        self.get_logger().info(
            f"simulated base: {self.d['base']}, {self.d['wheels']} wheels, "
            f"{self.d['mass']:.1f} kg, turns on {self.d['radius']:.4f} m, "
            f"budget {self.d['command_rpm']:.0f} rpm, at {rate:.0f} Hz"
            + ("" if self.wire.instant else
               f", link {self.wire.delay * 1000:.0f} +/- "
               f"{self.wire.jitter * 1000:.0f} ms"))

    def _cmd(self, msg):
        self.cmd = (msg.linear.x, msg.linear.y, msg.angular.z)
        self.cmd_time = self.get_clock().now()

    def _cmd_stamped(self, msg):
        self.cmd = (msg.twist.linear.x, msg.twist.linear.y, msg.twist.angular.z)
        self.cmd_time = self.get_clock().now()

    def _tick(self):
        now = self.get_clock().now()
        # The wire runs on a monotonic clock, not the ROS one: it models wall
        # time on a wire, and a simulated or stepped ROS clock would stall the
        # link rather than the robot.
        mono = time.monotonic()
        # Starve the loop BEFORE the interval is measured, so the stall lands in
        # dt exactly as a board's would -- a stall measured afterwards is a stall
        # the model never sees.
        if self.stall > 0.0:
            if self.stall_on_change:
                # A change in the commanded twist is a change in the speed the
                # wheels are driving towards -- the term that makes the step
                # unstable. Two qualifications, both needed:
                #
                #  * a THRESHOLD. Nav2's smoothed cmd_vel changes by a little on
                #    almost every 20 Hz message, so "any change" would stall on
                #    nearly every tick, run the loop at about 1 Hz for the whole
                #    leg, and trip the firmware's own 200 ms command timeout --
                #    which stops the robot and hides the very thing being tested.
                #  * the same MINIMUM SPACING the fixed cadence uses, so a burst
                #    of changes cannot become sustained starvation by accident.
                #    The two mechanisms then differ only in WHERE the stall lands.
                # The threshold was 0.05 m/s per tick and that was wrong by an
                # order of magnitude: Nav2's velocity smoother ramps at about
                # 2.5 m/s2, which is 0.05 m/s per 20 ms tick EXACTLY, so the test
                # sat on its own boundary and, with the minimum spacing on top,
                # fired essentially never. Measured 2026-09-25: the 900 ms leg
                # logged ZERO intervals over 100 ms across sixteen goals, and
                # came back green -- a treatment that was never administered,
                # about to be read as evidence the model was fine.
                big = (self.prev_cmd is not None and
                       (abs(self.cmd[0] - self.prev_cmd[0]) > 0.005 or
                        abs(self.cmd[1] - self.prev_cmd[1]) > 0.005 or
                        abs(self.cmd[2] - self.prev_cmd[2]) > 0.010))
                self.prev_cmd = self.cmd
                due = big and (self.stall_next is None or mono >= self.stall_next)
            else:
                if self.stall_next is None:
                    self.stall_next = mono + self.stall_every
                    due = False
                else:
                    due = mono >= self.stall_next
            if due:
                time.sleep(self.stall)
                self.stalls += 1
                self.stall_next = mono + self.stall_every
                mono = time.monotonic()
        self.wire.pump(mono)
        # SimEncoder::integrate()'s own two guards, for the same two reasons: the
        # first call has no previous reading to subtract, and an interval longer
        # than a second is a suspended process rather than a robot.
        dt = self.dt
        if self.measured_dt:
            if self.prev_mono is None:
                self.prev_mono = mono
                return
            dt = mono - self.prev_mono
            self.prev_mono = mono
            if dt <= 0.0 or dt > 1.0:
                return
            if dt > self.max_dt:
                self.max_dt = dt
                # The interval is the whole point of measuring it: past 2*tau an
                # explicit Euler step of this plant diverges, and the slicing in
                # drivetrain_report is what keeps it honest. Leave a trail of the
                # worst one so a boardless run that DOES misbehave can be read
                # against the intervals it actually saw, rather than assumed to
                # have run at its nominal rate.
                if dt > 0.1:
                    self.get_logger().warn(
                        f"control interval {dt * 1000:.0f} ms "
                        f"(nominal {self.dt * 1000:.0f}); the model is sliced at "
                        "a quarter of tau, so this is integrated, not skipped")
        # The firmware's 200 ms command timeout. Without it a node that stops
        # publishing leaves the simulated robot driving for ever, which is a
        # failure mode the board does not have and would send someone hunting a
        # controller bug that is not there.
        if (now - self.cmd_time).nanoseconds > 200_000_000:
            self.cmd = (0.0, 0.0, 0.0)

        vx, vy, wz = self.cmd
        rpm = self.wheels.step(target_rpm(self.d, vx, vy, wz), dt)
        # Read the wheels BACK through the kinematics, never integrate the
        # command: the same radius on both sides is what made a wrong mecanum
        # turn radius invisible on the bench for months.
        meas_x, meas_wz = dr._velocities(self.d, rpm)
        meas_y = 0.0
        if self.d["base"] == "mecanum":
            r1, r2, r3, r4 = rpm
            meas_y = ((-r1 + r2 + r3 - r4) / float(self.d["wheels"]) / 60.0) * self.d["circ"]

        self.yaw += meas_wz * dt
        cos_h, sin_h = math.cos(self.yaw), math.sin(self.yaw)
        self.x += (meas_x * cos_h - meas_y * sin_h) * dt
        self.y += (meas_x * sin_h + meas_y * cos_h) * dt

        # THE DOSE. An experiment that does not report how much treatment it
        # applied cannot be read at all: a leg that passes because the stall
        # never fired is indistinguishable, in the verdict, from one that passed
        # despite it. Reported on a slow cadence so it costs nothing, and only
        # when a stall was actually asked for.
        if self.stall > 0.0:
            self.dose_n = getattr(self, "dose_n", 0) + 1
            if self.dose_n % 500 == 0:
                self.get_logger().info(
                    f"stall dose: {self.stalls} applied of "
                    f"{self.stall * 1000:.0f} ms"
                    + (" (aimed at command changes)" if self.stall_on_change
                       else f" every {self.stall_every:.1f} s")
                    + f"; worst interval so far {self.max_dt * 1000:.0f} ms")

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
        self.wire.send(mono, lambda m=odom: self.odom_pub.publish(m), "odom")

        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = "imu_link"
        imu.orientation.x, imu.orientation.y = qx, qy
        imu.orientation.z, imu.orientation.w = qz, qw
        gyro_z, accel_x = self.imu.read(meas_wz, (meas_x - self.prev_vx) / dt, dt)
        imu.angular_velocity.z = gyro_z
        imu.linear_acceleration.x = accel_x
        imu.linear_acceleration.z = 9.81
        self.wire.send(mono, lambda m=imu: self.imu_pub.publish(m), "imu")
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
            # Through the SAME link. A TF that arrives instantly while the
            # odometry it describes is late is not a robot -- it is a robot
            # whose transform predicts the future, and it would paper over
            # exactly the extrapolation the delay is here to reproduce.
            self.wire.send(mono, lambda m=t: self.tf.sendTransform(m))


def main():
    if ROS_IMPORT_ERROR is not None:
        print(f"Error: cannot import the ROS 2 Python stack: {ROS_IMPORT_ERROR}",
              file=sys.stderr)
        print(f"  interpreter: {sys.executable}", file=sys.stderr)
        return 1
    rclpy.init()
    node = SimBaseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main() or 0)
