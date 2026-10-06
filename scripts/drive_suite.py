#!/usr/bin/env python3
"""Eight manoeuvres, checked for sign and magnitude against odometry.

Rates prove the board talks. Only this proves it MOVES, and moves the way it
was told: the simulated-wheel invert bug and the PID integral windup both produced
perfect 50 Hz topics on a robot that was spinning on the spot or pinned at
both rails. Every hardware run ends with this, after the topic gate.

Each manoeuvre is 5 s of a constant command, preceded by 2.5 s of zero so the
base settles. The check is the extreme of the reported twist in the commanded
direction, against the command, with a tolerance loose enough that a base
which tracks badly still passes: this is a sign-and-magnitude test, not a
performance one (see scripts/test_nav2_goal.py for the same reasoning).

    python3 scripts/drive_suite.py                       # plain Twist
    python3 scripts/drive_suite.py geometry_msgs/msg/TwistStamped
    python3 scripts/drive_suite.py --config ~/linorobot2-config/lino1_config.yaml
    python3 scripts/drive_suite.py --prefix lino1
    python3 scripts/drive_suite.py --base-type mecanum

Six of the eight are forward/backward, two turns and two spins. The last two are
a sideways pair, and they are run on EVERY drivetrain because the command is the
same and only the right answer differs: a mecanum base must strafe at the
commanded speed without turning, and a 2wd or skid base must do nothing at all.
Strafing is the only thing mecanum does that the others cannot, so without the
pair a mecanum base passes with its vy channel dead; and a differential base
that reports vy has a broken odometry model, which nothing else here would
catch. The drivetrain comes from --base-type, or from kinematics.base_type in
--config.

A namespaced robot has to be addressed by its namespace or the suite drives
nothing and then reports the silence as the board's fault. `--config` reads
`base_controller.topic_prefix` through the same normaliser the launchers and
mcu_env use, so the suite reaches the robot the config describes; `--prefix`
says it outright. Neither given, the topics are the unprefixed ones and the
behaviour is exactly what it always was.

Each line also says WHERE the robot was, because the suite now runs after the
Nav2 legs and its position is therefore uncontrolled. The simulated room clamps
the pose at its walls and at the obstacle wall (x = 2.0, |y| <= 1.5), and a
clamp acting during a manoeuvre is differentiated into a velocity the base was
never commanded: measured 2026-09-22 on the GenDrv, both spins reported
vx ~ +0.15 m/s while commanded (0.00, +/-1.50), and both were positive. Without
the pose that reads as a base fault; with it, a "near obstacle wall" note says
what it is. The room's geometry is mirrored here from firmware/common/lib/lidar/
sim_ld19.h -- if it changes there, change it here.

Each line also judges how FAR the base went, not only how fast it said it was going:
the path the board's own odometry travelled during the window, against commanded speed
x time (a spin: the yaw it turned). The speed is the median of the /odom samples that
ARRIVED, and a link that drops samples drops the stopped ones too: on 2026-10-04 an m21
GenDrv on Wi-Fi reported forward at +0.250 m/s and passed while it covered 0.67 m of the
1.25 m commanded -- its 200 ms command watchdog (main.cpp) stopped it every time Wi-Fi
stalled. And each line names the longest stretch with no /odom at all, judged against
GAP_LIMIT_S: a link that silent has left the base uncommanded.

Exit status 0 when all pass; the verdict line says how many did.
"""
import math
import os
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import cockpit_paths  # noqa: E402  -- the prefix rule lives in one place


# The simulated world the clamp holds the robot in, as wall segments, so a manoeuvre can say
# whether the pose it reports was being held. It used to be the default room only (the box
# and the obstacle wall at x = 2.0): in the four-room world an exploration leg ended at
# (+0.65, +0.18), its forward manoeuvre was held 0.31 m short of the interior wall at x = 1.5,
# and the suite called the pose "clear" and failed the base on distance -- every mecanum
# exploration leg, rc63 and rc64. set_world() replaces these from the robot's config, through
# the same depth_camera functions the emulator's env and the host laser use.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import depth_camera  # noqa: E402

ROBOT_R = 0.30
SEGMENTS = depth_camera.room_segments(dict(depth_camera.ROOM_DEFAULTS))
OUTER = 4            # the first four segments are the room's outer box
OBSTACLE = True      # the fifth, when present, is the single test wall
NEAR = 0.05          # "against" a surface: within this of where the clamp holds

# Distance and gap. The travel tolerance is the speed check's 45 % (a sign-and-magnitude
# test, not a performance one); on serial the bench covers ~90 % of the command. The gap
# limit is 2.5x the firmware's 200 ms cmd_vel watchdog: a link silent that long has left
# the base uncommanded, whatever the median speed of the samples that did arrive says.
TRAVEL_TOL = 0.45
GAP_LIMIT_S = 0.5


def _path_length(xs: list, ys: list) -> float:
    """Distance travelled along the reported poses. Summed segment by segment, so a
    dropped sample loses no distance: the board's pose integrates through the gap."""
    return sum(math.hypot(xs[i] - xs[i - 1], ys[i] - ys[i - 1]) for i in range(1, len(xs)))


def _yaw(qz: float, qw: float) -> float:
    return 2.0 * math.atan2(qz, qw)


def _turned(yaws: list) -> float:
    """Signed yaw turned, each step wrapped to (-pi, pi]."""
    total = 0.0
    for i in range(1, len(yaws)):
        d = yaws[i] - yaws[i - 1]
        total += math.atan2(math.sin(d), math.cos(d))
    return total


def _max_gap(stamps: list) -> float:
    """The longest hole between consecutive /odom samples, by the samples' OWN header
    stamps (the board's clock). Timing the callbacks instead measured this process:
    on a loaded bench the suite's own loop stalled ~1 s while a serial base drove
    1.13 of 1.25 m without a pause (rc-20261004.2 campaign). A link that drops
    samples still leaves a hole in the board's stamps."""
    ts = sorted(stamps)
    if len(ts) < 2:
        return float("inf")
    return max(ts[i] - ts[i - 1] for i in range(1, len(ts)))


def _travel_ok(want: float, got: float) -> bool:
    """Covered at least (1 - TRAVEL_TOL) of the command, in its direction."""
    return want * got > 0 and abs(got) >= abs(want) * (1.0 - TRAVEL_TOL)


# Set from the drivetrain before the manoeuvres run. The closures read them, so
# they are module-level rather than threaded through every call.
BASE_TYPE, MECANUM, STRAFES = "2wd", False, True


def _seg_dist(x: float, y: float, seg) -> float:
    x1, y1, x2, y2 = seg
    sx, sy = x2 - x1, y2 - y1
    l2 = sx * sx + sy * sy
    t = 0.0 if l2 < 1e-12 else max(0.0, min(1.0, ((x - x1) * sx + (y - y1) * sy) / l2))
    return math.hypot(x - (x1 + t * sx), y - (y1 + t * sy))


def _where(x: float, y: float) -> str:
    """Which surface, if any, the robot is being held against: within NEAR of the distance the
    clamp keeps its centre from a wall (the robot radius)."""
    if x != x or y != y:                       # NaN: no pose seen
        return "pose unknown"
    notes = []
    for i, seg in enumerate(SEGMENTS):
        if abs(_seg_dist(x, y, seg) - ROBOT_R) < NEAR:
            name = ("room wall" if i < OUTER else
                    "OBSTACLE WALL" if (OBSTACLE and i == OUTER) else
                    "interior wall (%g,%g)-(%g,%g)" % tuple(seg))
            if name not in notes:
                notes.append(name)
    return ", ".join(notes) if notes else "clear"


def set_world(cfg: dict, world: str = None) -> None:
    """The walls and radius of the robot's simulated world (base_controller.simulation), as the
    firmware's emulator and the host laser have them. A saved-map world has no collision."""
    global SEGMENTS, ROBOT_R, OBSTACLE
    cfg = cfg or {}
    if world:
        cfg.setdefault("base_controller", {}).setdefault("simulation", {})["world"] = world
    try:
        if depth_camera.sim_world(cfg) == "map":
            SEGMENTS, OBSTACLE = [], False
        else:
            room = depth_camera.sim_room(cfg)
            OBSTACLE = bool(room.get("wall_obstacle", True))
            SEGMENTS = depth_camera.room_segments(room, depth_camera.sim_walls(cfg))
        sim = (cfg.get("base_controller") or {}).get("simulation") or {}
        radius = sim.get("robot_radius")
        if radius is None:
            import gen_firmware_header
            radius = gen_firmware_header.nav2_robot_radius(cfg)
        ROBOT_R = float(radius)
    except (ValueError, TypeError, KeyError) as exc:
        print(f"[drive_suite] world not read from the config ({exc}); the default room", flush=True)


def _peak(samples: list, want: float) -> float:
    """The extreme in the commanded direction -- did the base ever reach it.

    Reported, never judged. It was judged, against a SYMMETRIC tolerance, and
    that combination cannot be satisfied by a base with a transient: a peak can
    only be further from the command than the sustained speed is, so the more a
    base overshoots on the way up, the worse it scored on a statistic chosen to
    be generous about tracking. On 2026-09-25 the GenDrv mecanum jazzy leg was
    marked BAD for `vy -0.464 (want -0.20)` while its pose moved 0.92 m in the
    5 s command -- 0.184 m/s, the commanded speed almost exactly -- and the
    Yahboom on the same drivetrain, same firmware, reported -0.225 and passed.
    """
    if not samples:
        return 0.0
    if want > 0:
        return max(samples)
    if want < 0:
        return min(samples)
    # A zero command has no commanded direction, so the informative extreme is
    # the largest excursion either way. Printing 0.000 here would claim the base
    # never moved, which is the opposite of what the column is for: the 1.5 rad/s
    # spin whose vx peaked at +0.15 m/s while the pose held still is exactly the
    # row a reader needs to see the excursion on.
    return max(samples, key=abs)


def _statistic(samples: list, want: float) -> float:
    """What to compare against the command: the SUSTAINED speed.

    Asked for motion, the median -- it is the speed the base held for most of
    the window, which is what "did it do what it was told" means, and one
    startup overshoot or one dropped sample cannot move it. Asked for zero, the
    question is whether it stayed still, and there the mean is right: it
    reports the behaviour rather than the worst sample.
    """
    if not samples:
        return 0.0
    if want == 0:
        return sum(samples) / len(samples)
    ordered = sorted(samples)
    mid = len(ordered) // 2
    if len(ordered) % 2:
        return ordered[mid]
    return (ordered[mid - 1] + ordered[mid]) / 2.0


def _topics(prefix: str) -> tuple:
    """(cmd_vel, odom) for a robot whose namespace is `prefix` ("" = plain)."""
    ns = f"/{prefix}" if prefix else ""
    return f"{ns}/cmd_vel", f"{ns}/odom/unfiltered"


# The sideways command used for the strafe pair. Well under the 0.4 m/s the Nav2
# legs drive at, so a base that ignores vy is obvious rather than marginal.
STRAFE_SPEED = 0.20


def main() -> int:
    global BASE_TYPE, MECANUM, STRAFES
    argv = sys.argv[1:]
    prefix = ""
    base_type = "2wd"
    world_cfg, world_arg = None, None
    rest = []
    i = 0
    while i < len(argv):
        if argv[i] == "--base-type" and i + 1 < len(argv):
            base_type = argv[i + 1].strip().lower()
            i += 2
        elif argv[i] == "--world" and i + 1 < len(argv):
            world_arg = argv[i + 1].strip().lower()
            i += 2
        elif argv[i] == "--prefix" and i + 1 < len(argv):
            prefix = argv[i + 1].strip().strip("/")
            i += 2
        elif argv[i] == "--config" and i + 1 < len(argv):
            import yaml
            with open(os.path.expanduser(argv[i + 1])) as fh:
                _cfg = yaml.safe_load(fh) or {}
            prefix = cockpit_paths.robot_namespace(_cfg)
            world_cfg = _cfg
            base_type = str((_cfg.get("kinematics") or {}).get(
                "base_type", base_type)).strip().lower()
            i += 2
        else:
            rest.append(argv[i])
            i += 1
    if world_cfg is not None or world_arg:
        set_world(world_cfg or {}, world_arg)
    BASE_TYPE = base_type
    MECANUM = base_type == "mecanum"
    # Every drivetrain runs the strafe pair; only the expected answer differs.
    STRAFES = True
    cmd_topic, odom_topic = _topics(prefix)

    rclpy.init()
    node = Node("drive_suite")
    tname = rest[0].strip() if rest else "geometry_msgs/msg/Twist"
    stamped = "TwistStamped" in tname
    if stamped:
        from geometry_msgs.msg import TwistStamped as T
    else:
        from geometry_msgs.msg import Twist as T
    if prefix:
        print(f"[drive_suite] robot namespace /{prefix}: {cmd_topic} -> {odom_topic}",
              flush=True)
    pub = node.create_publisher(T, cmd_topic, 10)

    seen = {"vx": [], "vy": [], "wz": [], "x": float("nan"), "y": float("nan"),
            "t": [], "px": [], "py": [], "yaw": []}

    def _odom(m):
        seen["vx"].append(m.twist.twist.linear.x)
        seen["vy"].append(m.twist.twist.linear.y)
        seen["wz"].append(m.twist.twist.angular.z)
        seen["x"] = m.pose.pose.position.x
        seen["y"] = m.pose.pose.position.y
        st = m.header.stamp
        seen["t"].append(st.sec + st.nanosec * 1e-9)
        seen["px"].append(m.pose.pose.position.x)
        seen["py"].append(m.pose.pose.position.y)
        seen["yaw"].append(_yaw(m.pose.pose.orientation.z, m.pose.pose.orientation.w))

    node.create_subscription(Odometry, odom_topic, _odom, qos_profile_sensor_data)

    def command(lin: float, ang: float, secs: float, lat: float = 0.0) -> None:
        m = T()
        if stamped:
            m.twist.linear.x = lin
            m.twist.linear.y = lat
            m.twist.angular.z = ang
        else:
            m.linear.x = lin
            m.linear.y = lat
            m.angular.z = ang
        t0 = time.time()
        while time.time() - t0 < secs:
            if stamped:
                m.header.stamp = node.get_clock().now().to_msg()
            pub.publish(m)
            # Drain every waiting callback until the next command is due, rather than
            # one spin_once() and a sleep: spin_once runs ONE callback, so the old loop
            # took ~20 of /odom's 50 messages a second, and the one it took was the
            # oldest left in the depth-5 queue -- up to ~100 ms stale, and the gap
            # column measured this loop, not the link. (The same one-take-per-pass
            # limit halved the LD19 node's /scan on the Arduino UNO Q.)
            due = time.time() + 0.05
            while time.time() < due:
                rclpy.spin_once(node, timeout_sec=max(0.0, due - time.time()))

    def run(label: str, lin: float, ang: float, secs: float = 5.0,
            lat: float = 0.0) -> bool:
        command(0.0, 0.0, 2.5)
        x0, y0 = seen["x"], seen["y"]
        for k in ("vx", "vy", "wz", "t", "px", "py", "yaw"):
            seen[k].clear()
        command(lin, ang, secs, lat)
        vx = seen["vx"] or [0.0]
        vy = seen["vy"] or [0.0]
        wz = seen["wz"] or [0.0]
        # The sustained speed, judged -- and the peak, reported beside it.
        # A zero command has no direction to peak in, and judging it by its
        # largest positive excursion fails on noise by construction: measured on
        # the GenDrv, a 1.5 rad/s spin reported vx peaks of +0.15 m/s while the
        # pose moved 0.05 m in 5 s, which is standing still. The question for a
        # zero command is whether the base STAYED there, so it is the mean.
        got_vx = _statistic(vx, lin)
        got_vy = _statistic(vy, lat)
        got_wz = _statistic(wz, ang)
        pk_vx, pk_vy, pk_wz = _peak(vx, lin), _peak(vy, lat), _peak(wz, ang)
        ok_vx = abs(got_vx - lin) < max(0.12, abs(lin) * 0.45)
        ok_vy = abs(got_vy - lat) < max(0.12, abs(lat) * 0.45)
        ok_wz = abs(got_wz - ang) < max(0.45, abs(ang) * 0.45)
        # How far, and how silent. A move is judged on the path travelled, a spin on
        # the yaw turned; a zero command has no distance to cover (its speed check
        # already asks whether it stayed still).
        gap = _max_gap(seen["t"])
        ok_gap = gap <= GAP_LIMIT_S
        if lin or lat:
            want_d = math.hypot(lin, lat) * secs
            got_d = _path_length(seen["px"], seen["py"])
            ok_d = _travel_ok(want_d, got_d)
            dist_col = "   dist %.2f/%.2f m %s" % (got_d, want_d, "ok" if ok_d else "BAD")
        elif ang:
            want_d = ang * secs
            got_d = _turned(seen["yaw"])
            ok_d = _travel_ok(want_d, got_d)
            dist_col = "   turn %+.2f/%+.2f rad %s" % (got_d, want_d, "ok" if ok_d else "BAD")
        else:
            ok_d, dist_col = True, ""
        gap_col = "   gap %.2f s %s" % (gap, "ok" if ok_gap else "BAD")
        x1, y1 = seen["x"], seen["y"]
        where = _where(x1, y1)
        # A move that ends held against a wall was stopped by the room, not the base: the
        # clamp holds the centre one radius off it, so the path is short by whatever lay
        # past the wall. Its distance is not judged; the speeds still are. A spin turns in
        # place and is judged whatever is around it.
        held = (lin or lat) and not ok_d and where not in ("clear", "pose unknown")
        if held:
            ok_d = True
            dist_col = "   dist %.2f/%.2f m held" % (got_d, want_d)
        # vy is only printed when it is part of the question: on a differential
        # base every line would carry a column that is always zero, and a column
        # that is always zero stops being read.
        # The peak goes on the line beside the number that was judged. It is the
        # only thing that separates "the base held the wrong speed" from "the
        # base overshot once on the way up", and the two call for opposite
        # responses -- so a line carrying just one of them cannot be read.
        lat_col = ("   vy %+.3f (pk %+.3f, want %+.2f) %s"
                   % (got_vy, pk_vy, lat, "ok" if ok_vy else "BAD")) if lat or STRAFES else ""
        print("%-12s cmd(%+.2f,%+.2f)  odom vx %+.3f (pk %+.3f, want %+.2f) %s%s"
              "   wz %+.3f (pk %+.3f, want %+.2f) %s%s%s"
              "   pose (%+.2f,%+.2f)->(%+.2f,%+.2f) %s"
              % (label, lin, ang, got_vx, pk_vx, lin, "ok" if ok_vx else "BAD", lat_col,
                 got_wz, pk_wz, ang, "ok" if ok_wz else "BAD", dist_col, gap_col,
                 x0, y0, x1, y1, where), flush=True)
        ok = ok_vx and ok_vy and ok_wz and ok_d and ok_gap
        if not ok_gap:
            print("             ^ /odom went silent for %.2f s (limit %.1f s): the link dropped "
                  "the base; it stops 200 ms after its last cmd_vel." % (gap, GAP_LIMIT_S),
                  flush=True)
        if held:
            print("             ^ held against %s: the room stopped the move short, which is "
                  "not a base fault; the distance is not judged." % where, flush=True)
        if not ok and where not in ("clear", "pose unknown"):
            print("             ^ held against %s: the clamp moves the pose every cycle and "
                  "that shows up as a velocity nobody commanded. Not a base fault." % where,
                  flush=True)
        return ok

    # Wait for the board before judging it: no odom in 10 s is its own failure.
    t0 = time.time()
    while not seen["vx"] and time.time() - t0 < 10:
        rclpy.spin_once(node, timeout_sec=0.1)
    if not seen["vx"]:
        print(f"VERDICT: nothing on {odom_topic} in 10 s -- is the agent up?")
        return 2

    results = [
        run("forward",     0.25,  0.00),
        run("backward",   -0.25,  0.00),
        run("left turn",   0.20,  0.80),
        run("right turn",  0.20, -0.80),
        run("left spin",   0.00,  1.50),
        run("right spin",  0.00, -1.50),
    ]
    # Strafing is the ONE thing a mecanum base does that the other two cannot, so
    # without these a mecanum leg goes green with its vy channel dead -- which is
    # exactly how a drivetrain axis can be added and test nothing (2026-09-23).
    #
    # The command is the same on every base; only the expected answer differs.
    # On 2wd and skid the right answer is that NOTHING happens: a differential
    # base that reports vy has a broken odometry model, and a differential base
    # that actually moves sideways has a broken inverse kinematic. Both are worth
    # a leg, and neither was tested before.
    if STRAFES:
        want = STRAFE_SPEED if MECANUM else 0.0
        results += [
            run("strafe left",  0.00, 0.00, lat=+want),
            run("strafe right", 0.00, 0.00, lat=-want),
        ]
        if not MECANUM:
            print("             ^ %s is not a mecanum base: a sideways command must "
                  "produce no vy and no motion." % BASE_TYPE, flush=True)
    command(0.0, 0.0, 2.5)
    print("VERDICT: %d/%d manoeuvres correct (%s)"
          % (sum(results), len(results), BASE_TYPE), flush=True)
    rclpy.shutdown()
    return 0 if all(results) else 1


if __name__ == "__main__":
    sys.exit(main())
