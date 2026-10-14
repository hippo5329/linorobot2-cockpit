#!/usr/bin/env python3
"""Room round a real robot before it moves, from its LiDAR -- and, when there is not
enough, inch it slowly to where there is.

    clearance.py check --need M [--save SNAP.json]
    clearance.py inch  --need M --radius R [--max-travel 0.6] [--stamped] [--save SNAP.json]
    clearance.py home  --snap SNAP.json [--stamped] [--tol 0.05] [--tol-deg 3]

The 1-Click runs this on a real robot before anything drives it (SLAM's mapping, Nav2,
test_acc). A simulated robot needs none of it: its room is the one the simulation draws.

check   N scans (the whole circle: scan_raw when a lidar.mask makes /scan a narrower
        view), the median return per beam, the chassis left out. Prints the nearest
        return in each quadrant (front, left, back, right; +-45 deg about 0 = ahead,
        counter-clockwise) and exits 0 when nothing is nearer than --need, 1 when
        something is. --save keeps the points as the start snapshot for `home`.
inch    check, and while something is nearer than --need: pick the 5 cm step that most
        increases the nearest distance -- forward or backward along a heading every
        15 deg, turned to first, with the path swept by the robot's circle (--radius)
        kept clear -- take it at 0.05 m/s, and look again. A turn in place sweeps that
        circle too, so with something inside radius + 2 cm only straight steps are tried.
        Gives up (exit 1) when no step helps, or after --max-travel metres. Every step
        is closed on /odom (the EKF: a tracked base's wheel yaw slips) with a timeout,
        and stops at once if a return comes within 0.08 m of the robot's circle in the
        direction of travel.
home    drive back to where `--save` took its snapshot: every scan matched against it
        (scan_match.icp, from the last pose), turn to face the start -- or away from it,
        to back up, whichever is the smaller turn -- drive, turn to the start heading,
        and settle: zero for a second and one more robust match before HOME is called.
        Refuses to move on a weak first fix (fewer than 150 pairs or rms over 0.025 m):
        from the wrong side of an obstacle a room can match in two places.

THE ROBOT MOVES in `inch` and `home`. /cmd_vel (TwistStamped with --stamped) at 10 Hz,
zero on every exit; the firmware's cmd_vel watchdog stops the wheels if this dies.
Nothing else may be driving the robot: it refuses when /cmd_vel already has a publisher.
"""
import argparse
import json
import math
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import scan_match  # noqa: E402

STEP_M = 0.05          # one inch
LIN_MPS = 0.05         # inching speed
ANG_RPS = 0.35         # turning speed while inching
HEADINGS_DEG = 15      # candidate step headings
STOP_MARGIN_M = 0.08   # stop when a return is this close to the robot's circle ahead
SPIN_MARGIN_M = 0.02   # a turn in place needs this beyond the circle
HOME_MIN_PAIRS = 150    # pairs a first fix needs -- or HOME_MIN_FRAC of a smaller snapshot,
HOME_MIN_FRAC = 0.40    # never under HOME_FLOOR_PAIRS: a LiDAR with a masked sector (a UNO Q
HOME_FLOOR_PAIRS = 100  # robot's ~290 points) moved 0.5 m sees 46 % of its snapshot at best
HOME_RIVAL_FRAC = 0.80  # another pose with this share of the best's inliers: ambiguous
TF_WAIT_TRIES = 100     # x 0.2 s: the base_link -> laser TF, before a snapshot is refused
HOME_MAX_RMS = 0.025
QUADRANTS = (("front", 0.0), ("left", 90.0), ("back", 180.0), ("right", 270.0))


# ---- geometry (no ROS) ----

def nearest(P) -> float:
    return float(np.min(np.hypot(P[:, 0], P[:, 1]))) if len(P) else math.inf


def quadrants(P) -> dict:
    """{name: nearest return within +-45 deg of that direction} (inf when none)."""
    out = {}
    if len(P):
        bearing = np.degrees(np.arctan2(P[:, 1], P[:, 0])) % 360.0
        d = np.hypot(P[:, 0], P[:, 1])
    for name, c in QUADRANTS:
        if not len(P):
            out[name] = math.inf
            continue
        m = ((bearing - c + 45.0) % 360.0) < 90.0
        out[name] = float(d[m].min()) if m.any() else math.inf
    return out


def segment_clearance(P, d) -> float:
    """The least distance from any point to the segment the centre sweeps, 0 -> d."""
    if not len(P):
        return math.inf
    L2 = float(d @ d)
    if L2 == 0.0:
        return nearest(P)
    t = np.clip((P @ d) / L2, 0.0, 1.0)
    return float(np.min(np.hypot(P[:, 0] - t * d[0], P[:, 1] - t * d[1])))


def turn_for(heading: float) -> tuple:
    """(turn, direction): reach a step along `heading` (base_link, rad) by turning `turn`
    and driving forward (+1) or backward (-1), whichever is the smaller turn."""
    h = scan_match.wrap(heading)
    if abs(h) <= math.pi / 2:
        return h, 1.0
    return scan_match.wrap(h + math.pi), -1.0


def best_step(P, radius: float, step: float = STEP_M, spin_ok: bool = True):
    """The step that most increases the nearest distance: (heading, nearest_after), or None
    when no step keeps the robot's circle off everything along its path or improves on
    standing still. Ties go to the smaller turn."""
    here = nearest(P)
    best, best_score = None, -math.inf
    for deg in range(0, 360, HEADINGS_DEG):
        h = math.radians(deg)
        turn, _ = turn_for(h)
        if not spin_ok and abs(turn) > 1e-6:
            continue
        d = np.array([step * math.cos(h), step * math.sin(h)])
        # the swept path: the circle's edge stays off everything (a point already inside
        # the circle only has to get no nearer)
        path = segment_clearance(P, d)
        if path < min(radius, here) - 1e-9:
            continue
        after = nearest(P - d)
        score = after - 0.002 * abs(turn)
        if score > best_score:
            best, best_score = (h, after), score
    if best is None or best[1] <= here + 0.005:
        return None
    return best


# ---- ROS ----

class Robot:
    """The scans, /odom and /cmd_vel, for a short-lived tool."""

    def __init__(self, stamped: bool = False, scan_topic: str = None):
        import rclpy
        from nav_msgs.msg import Odometry
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import LaserScan
        self.rclpy = rclpy
        rclpy.init()
        self.node = Node("clearance")
        self.stamped = stamped
        if stamped:
            from geometry_msgs.msg import TwistStamped as T
        else:
            from geometry_msgs.msg import Twist as T
        self.T = T
        self.pub = None
        self.scan, self.frame, self.odom = None, None, None
        self.topic = scan_match.whole_circle_topic(self.node, override=scan_topic)

        def on_scan(m):
            self.frame = m.header.frame_id
            self.scan = scan_match.scan_dict(m)

        def on_odom(m):
            p, q = m.pose.pose.position, m.pose.pose.orientation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            self.odom = (p.x, p.y, yaw)
        self.node.create_subscription(LaserScan, self.topic, on_scan, qos_profile_sensor_data)
        self.node.create_subscription(Odometry, "/odom", on_odom, 10)
        self.laser = None
        self.tf_ok = False

    def spin(self, secs: float):
        end = time.time() + secs
        while time.time() < end:
            self.rclpy.spin_once(self.node, timeout_sec=min(0.05, max(0.0, end - time.time())))

    def next_scan(self, timeout: float = 1.0):
        self.scan = None
        end = time.time() + timeout
        while self.scan is None and time.time() < end:
            self.rclpy.spin_once(self.node, timeout_sec=0.05)
        return self.scan

    def mount(self):
        # TF_WAIT_TRIES: a stack whose /scan is up can still be seconds from its static TF --
        # on a UNO Q, 6 s after the scan there was none, and a 5 s wait saved the home
        # snapshot in the scan frame (2026-10-14; the drive home then refused a weak fix).
        if self.laser is None:
            self.laser = scan_match.laser_pose(self.node, self.frame or "laser", tries=TF_WAIT_TRIES)
            self.tf_ok = self.laser is not None
            if self.laser is None:
                print(f"  ⚠️ no TF base_link -> {self.frame}; using the scan frame as base_link")
                self.laser = (0.0, 0.0, 0.0)
        return self.laser

    def points(self, n: int = 5, timeout: float = 15.0):
        """The median cloud of the next n scans, in base_link (empty when none came)."""
        scans, end = [], time.time() + timeout
        while len(scans) < n and time.time() < end:
            s = self.next_scan(min(1.0, end - time.time()))
            if s is not None:
                scans.append(s)
        if not scans:
            return None
        return scan_match.median_points(scans, self.mount())

    def claim_cmd_vel(self) -> bool:
        """Refuse to drive a robot something else is driving."""
        self.spin(1.0)
        others = len(self.node.get_publishers_info_by_topic("/cmd_vel"))
        if others:
            print(f"CLEARANCE REFUSED: /cmd_vel already has {others} publisher(s) -- "
                  f"something else is driving the robot")
            return False
        self.pub = self.node.create_publisher(self.T, "/cmd_vel", 10)
        return True

    def send(self, vx: float, wz: float, vy: float = 0.0):
        m = self.T()
        t = m.twist if self.stamped else m
        if self.stamped:
            m.header.stamp = self.node.get_clock().now().to_msg()
            m.header.frame_id = "base_link"
        t.linear.x, t.linear.y, t.angular.z = float(vx), float(vy), float(wz)
        self.pub.publish(m)

    def stop(self):
        if self.pub is not None:
            for _ in range(10):
                self.send(0.0, 0.0)
                time.sleep(0.02)

    def wait_odom(self, timeout: float = 5.0):
        end = time.time() + timeout
        while self.odom is None and time.time() < end:
            self.rclpy.spin_once(self.node, timeout_sec=0.05)
        return self.odom

    def blocked(self, direction: float, radius: float) -> float:
        """The nearest return in the robot's corridor in the direction of travel, when it is
        inside the stop margin of the circle; 0 when the way is open."""
        if self.scan is None:
            return 0.0
        P = scan_match.scan_points(self.scan, self.mount())
        ahead = P[(np.sign(P[:, 0]) == np.sign(direction)) & (np.abs(P[:, 1]) < radius)]
        if len(ahead) and np.min(np.abs(ahead[:, 0])) < radius + STOP_MARGIN_M:
            return float(np.min(np.abs(ahead[:, 0])))
        return 0.0

    def turn_by(self, angle: float, timeout: float = 15.0) -> bool:
        start = self.wait_odom()
        if start is None:
            print("  ❌ no /odom: cannot close a turn")
            return False
        end = time.time() + timeout
        while time.time() < end:
            self.rclpy.spin_once(self.node, timeout_sec=0.05)
            left = scan_match.wrap(angle - scan_match.wrap(self.odom[2] - start[2]))
            if abs(left) < math.radians(3):
                self.stop()
                return True
            self.send(0.0, max(0.15, min(ANG_RPS, abs(left))) * (1 if left > 0 else -1))
            time.sleep(0.05)
        self.stop()
        print("  ❌ the turn timed out")
        return False

    def drive(self, dist: float, direction: float, radius: float, timeout: float = 10.0) -> bool:
        start = self.wait_odom()
        if start is None:
            print("  ❌ no /odom: cannot close a step")
            return False
        end = time.time() + timeout
        while time.time() < end:
            self.rclpy.spin_once(self.node, timeout_sec=0.05)
            if math.hypot(self.odom[0] - start[0], self.odom[1] - start[1]) >= dist:
                self.stop()
                return True
            b = self.blocked(direction, radius)
            if b:
                self.stop()
                print(f"  ⚠️ stopped: a return {b:.2f} m {'ahead' if direction > 0 else 'behind'}")
                return False
            self.send(direction * LIN_MPS, 0.0)
            time.sleep(0.05)
        self.stop()
        print("  ❌ the step timed out")
        return False

    def close(self):
        self.stop()
        self.node.destroy_node()
        self.rclpy.shutdown()


def report(P, need: float) -> bool:
    q = quadrants(P)
    near = nearest(P)
    fmt = lambda v: "none" if v == math.inf else f"{v:.2f} m"
    print("  quadrants (+-45 deg): " + ", ".join(f"{k} {fmt(v)}" for k, v in q.items())
          + f"; nearest {fmt(near)} (need {need:.2f} m)")
    return near >= need


def save(path: str, P, r: "Robot") -> bool:
    """The start snapshot, in base_link, with the laser mount it was taken with. Refused
    without the TF: points in the scan frame are not where the robot is, and `home` would
    later match them against base_link points."""
    if not r.tf_ok:
        print(f"  no start snapshot: no TF base_link -> {r.frame} in {TF_WAIT_TRIES * 0.2:.0f} s")
        return False
    odom = r.wait_odom(2.0)
    with open(path, "w") as fh:
        json.dump({"points": P.tolist(), "quadrants": quadrants(P), "laser": list(r.laser),
                   "odom": list(odom) if odom else None}, fh)
    print(f"  start snapshot: {len(P)} points -> {path}")
    return True


def odom_since(then, now) -> tuple:
    """(yaw, x, y): the robot's pose now in its own frame at `then`, from two /odom poses
    (x, y, yaw) -- the same quantity the scan match returns, as the odometry has it."""
    c, s = math.cos(then[2]), math.sin(then[2])
    dx, dy = now[0] - then[0], now[1] - then[1]
    return scan_match.wrap(now[2] - then[2]), c * dx + s * dy, -s * dx + c * dy


def snapshot_points(snap: dict, laser) -> "np.ndarray":
    """The snapshot's points in base_link as the laser is mounted NOW. A snapshot saved with
    another mount (an old one in the scan frame, laser (0, 0, 0)) is taken back to the
    scan frame with its own mount and out again with this one."""
    P = np.asarray(snap["points"], float)
    then = snap.get("laser") or [0.0, 0.0, 0.0]
    if all(abs(a - b) < 1e-6 for a, b in zip(then, laser)):
        return P
    c, s_ = math.cos(-then[2]), math.sin(-then[2])
    d = P - np.asarray(then[:2], float)
    q = np.c_[c * d[:, 0] - s_ * d[:, 1], s_ * d[:, 0] + c * d[:, 1]]       # the scan frame
    c, s_ = math.cos(laser[2]), math.sin(laser[2])
    print(f"  the snapshot's laser mount ({then[0]:+.3f}, {then[1]:+.3f}, "
          f"{math.degrees(then[2]):+.1f} deg) differs from now's: re-expressed in base_link")
    return np.c_[laser[0] + c * q[:, 0] - s_ * q[:, 1], laser[1] + s_ * q[:, 0] + c * q[:, 1]]


def cmd_check(a, r: Robot) -> int:
    P = r.points(a.scans)
    if P is None:
        print(f"CLEARANCE FAIL: no scan on {r.topic}")
        return 2
    print(f"  reading {r.topic}; laser at x={r.laser[0]:.3f} y={r.laser[1]:.3f} "
          f"yaw={math.degrees(r.laser[2]):.1f} deg in base_link")
    ok = report(P, a.need)
    if a.save and ok and not save(a.save, P, r):
        print("CLEARANCE_REFUSED: no start snapshot, so no way home (nothing moved)")
        return 1
    print("CLEARANCE_OK" if ok else "CLEARANCE_REFUSED")
    return 0 if ok else 1


def cmd_inch(a, r: Robot) -> int:
    P = r.points(a.scans)
    if P is None:
        print(f"CLEARANCE FAIL: no scan on {r.topic}")
        return 2
    print(f"  reading {r.topic}")
    if report(P, a.need):
        r.mount()
        if a.save and not save(a.save, P, r):
            print("CLEARANCE_REFUSED: no start snapshot, so no way home (nothing moved)")
            return 1
        print("CLEARANCE_OK (no inching needed)")
        return 0
    if not r.claim_cmd_vel():
        return 1
    travel, k = 0.0, 0
    try:
        while travel + STEP_M <= a.max_travel + 1e-9:
            here = nearest(P)
            step = best_step(P, a.radius, spin_ok=here >= a.radius + SPIN_MARGIN_M)
            if step is None:
                print(f"CLEARANCE_REFUSED: no step improves on {here:.2f} m -- move the robot by hand")
                return 1
            h, after = step
            turn, direction = turn_for(h)
            k += 1
            print(f"  inch {k}: heading {math.degrees(h):+.0f} deg -> turn {math.degrees(turn):+.0f} deg, "
                  f"{'forward' if direction > 0 else 'back'} {STEP_M:.2f} m (nearest {here:.2f} -> {after:.2f} m)",
                  flush=True)
            if abs(turn) > 1e-6 and not r.turn_by(turn):
                return 1
            if not r.drive(STEP_M, direction, a.radius):
                # a stop on the guard is a fact about the room, not a failure: look again
                pass
            travel += STEP_M
            r.spin(0.5)
            P = r.points(a.scans)
            if P is None:
                print("CLEARANCE FAIL: the scan stopped")
                return 2
            if report(P, a.need):
                if a.save and not save(a.save, P, r):
                    print("CLEARANCE_REFUSED: no start snapshot, so no way home")
                    return 1
                print(f"CLEARANCE_OK after {k} inch(es), {travel:.2f} m")
                return 0
        print(f"CLEARANCE_REFUSED: still under {a.need:.2f} m after {travel:.2f} m of inching "
              f"(--max-travel {a.max_travel:.2f})")
        return 1
    finally:
        r.stop()


def home_min_pairs(snapshot_points: int) -> int:
    """HOME_MIN_PAIRS, or HOME_MIN_FRAC of a smaller snapshot, never under HOME_FLOOR_PAIRS."""
    return int(max(HOME_FLOOR_PAIRS, min(HOME_MIN_PAIRS, HOME_MIN_FRAC * snapshot_points)))


def first_fix_fault(cands, snapshot_points: int) -> str:
    """Why the first fix is not good enough to drive on, or ''. Weak (too few pairs for this
    snapshot, or a loose fit), or AMBIGUOUS: a different pose that fits almost as well -- a
    tracked robot's fix put it 0.3 m from home, turned the wrong way, and another strong fit
    1.2 m behind home, while it stood beside the boxes (2026-10-14)."""
    k_best, (th, x, y, rms, n) = cands[0]
    need = home_min_pairs(snapshot_points)
    if n < need or not rms <= HOME_MAX_RMS:
        return "the first fix is weak (%d pairs, need %d of a %d-point snapshot; rms %.3f)" % (
            n, need, snapshot_points, rms)
    other = scan_match.rival(cands)
    if other and other[0] > HOME_RIVAL_FRAC * k_best:
        k, (oth, ox, oy, orms, on) = other
        return ("the first fix is ambiguous: x %+.2f y %+.2f yaw %+.0f deg fits %d inliers against "
                "the best's %d" % (ox, oy, math.degrees(oth), k, k_best))
    return ""


# The drive home loses its match when this many scans in a row fit badly; it then stops and
# looks again from a standstill (RELOCATE_TRIES times at most) before it calls the robot LOST.
TRACK_BAD_SCANS = 5
RELOCATE_TRIES = 3
TRACK_MIN_PAIRS = 60
TRACK_MAX_RMS = 0.08


def home_step(pose, dist: float, holonomic: bool):
    """(vx, vy, wz) toward home from `pose` = (yaw, x, y) of the robot in the home frame.

    A holonomic base (mecanum) slides there with its heading held: turning first swung a
    masked LiDAR sector (a UNO Q robot's 95 deg rear) across the snapshot, the match fell under
    TRACK_MIN_PAIRS mid-turn and the run ended LOST 0.22 m from home (2026-10-14). The rest
    turn to face the way and drive."""
    th, x, y = pose
    speed = min(0.12, 0.5 * dist + 0.04)
    if holonomic:
        # home's direction in base_link: (-x, -y) in the home frame, turned by -th
        hx = -x * math.cos(th) - y * math.sin(th)
        hy = x * math.sin(th) - y * math.cos(th)
        norm = math.hypot(hx, hy) or 1.0
        err = scan_match.wrap(-th)
        wz = max(-0.3, min(0.3, 1.0 * err)) if abs(err) > math.radians(3) else 0.0
        return speed * hx / norm, speed * hy / norm, wz
    face, d = turn_for(math.atan2(-y, -x) - th)
    if abs(face) > math.radians(8):
        return 0.0, 0.0, max(0.2, min(0.6, abs(face))) * (1 if face > 0 else -1)
    return d * speed, 0.0, max(-0.4, min(0.4, 1.5 * face))


ODOM_TURN_WZ = 0.30       # rad/s: a turn closed on /odom, as teleop_step's (-89.5 deg for -90)
ODOM_TURN_DONE_DEG = 2.0
ODOM_TURNS = 4            # odom turns per drive home before the LiDAR-closed turn takes over


def odom_turn(r, angle: float) -> bool:
    """Turn by `angle` (rad, + = CCW) closed on /odom (the EKF: the gyro's yaw at 50 Hz), then
    stop. False when there is no /odom. The last turn on the spot used to be closed on the
    LiDAR match, ~0.9 s behind the robot on a UNO Q (scan, ICP, stop): at 0.2 rad/s each
    correction carried ~10 deg past and the robot hunted for 25 s (2026-10-15)."""
    o = r.wait_odom(2.0)
    if o is None:
        return False
    target = scan_match.wrap(o[2] + angle)
    end = time.time() + abs(angle) / ODOM_TURN_WZ * 3 + 3
    while time.time() < end:
        r.spin(0.02)
        e = scan_match.wrap(target - r.odom[2])
        if abs(e) < math.radians(ODOM_TURN_DONE_DEG):
            break
        r.send(0.0, math.copysign(ODOM_TURN_WZ, e))
    r.stop()
    return True


def path_blocked(P, vx: float, vy: float, radius: float) -> float:
    """The nearest return in the robot's corridor along (vx, vy), when it is inside the stop
    margin of its circle; 0 when the way is open or the robot is not translating."""
    v = math.hypot(vx, vy)
    if v == 0 or len(P) == 0:
        return 0.0
    ux, uy = vx / v, vy / v
    along = P[:, 0] * ux + P[:, 1] * uy
    across = np.abs(P[:, 1] * ux - P[:, 0] * uy)
    ahead = along[(along > 0) & (across < radius)]
    if len(ahead) and ahead.min() < radius + STOP_MARGIN_M:
        return float(ahead.min())
    return 0.0


def cmd_home(a, r: Robot) -> int:
    snap = json.load(open(a.snap))
    if not r.claim_cmd_vel():
        return 1
    if r.next_scan(10.0) is None:
        print(f"HOME ABORT: no scan on {r.topic}")
        return 2
    laser = r.mount()
    home = snapshot_points(snap, laser)
    S = scan_match.scan_points(r.scan, laser)
    cands = scan_match.locate_candidates(S, home)
    # The odometry's account of the move seeds more starts: from home's spot alone, every 30
    # degrees, a robot 0.32 m off and turned 15 deg fell into a wrong fit (2026-10-15). Only
    # more candidates under the same gate: odometry that drifted (the EKF while test_acc ran)
    # fits worse and loses.
    now = r.wait_odom(2.0) if snap.get("odom") else None
    if now:
        seed = odom_since(snap["odom"], now)
        print("  odometry since the snapshot: x %+.3f y %+.3f yaw %+.1f deg"
              % (seed[1], seed[2], math.degrees(seed[0])))
        cands = sorted(cands + scan_match.locate_candidates(S, home, guess=seed), key=lambda c: -c[0])
    k_best, (th, x, y, rms, n) = cands[0]
    pose = (scan_match.wrap(th), x, y)
    print("  first fix: x %+.3f y %+.3f yaw %+.1f deg -- %d pairs, rms %.3f"
          % (x, y, math.degrees(pose[0]), n, rms))
    why = first_fix_fault(cands, len(home))
    if why:
        print(f"HOME ABORT: {why} -- not driving on a guess")
        return 2
    bad, verdict, positioned, relocs, turns = 0, "TIMEOUT", False, 0, 0
    start = time.time()
    try:
        while time.time() - start < a.timeout:
            if r.next_scan(0.3) is None:
                r.send(0, 0)
                bad += 1
                if bad >= 10:
                    verdict = "LOST (no scans)"
                    break
                continue
            P = scan_match.scan_points(r.scan, laser)
            th, x, y, rms, n = scan_match.icp(P, home, pose[0], (pose[1], pose[2]))
            if a.trace:
                print("  trace %6.2f s  %3d pts  %3d pairs rms %.3f  x %+.3f y %+.3f yaw %+6.1f"
                      % (time.time() - start, len(P), n, rms, x, y, math.degrees(scan_match.wrap(th))),
                      flush=True)
            if n < TRACK_MIN_PAIRS or not rms < TRACK_MAX_RMS:
                r.send(0, 0)
                bad += 1
                if bad >= TRACK_BAD_SCANS:
                    # Stopped, the scan is sharp again: look from scratch, with the first fix's gate.
                    if relocs >= RELOCATE_TRIES:
                        verdict = "LOST (no match: %d pairs, rms %.3f)" % (n, rms)
                        break
                    relocs += 1
                    r.stop()
                    r.spin(1.0)
                    if r.next_scan(1.0) is None:
                        continue
                    cands = scan_match.locate_candidates(scan_match.scan_points(r.scan, laser), home)
                    why = first_fix_fault(cands, len(home))
                    k_best, (th, x, y, rms, n) = cands[0]
                    print("  relocate %d: x %+.3f y %+.3f yaw %+.1f deg -- %d pairs, rms %.3f%s"
                          % (relocs, x, y, math.degrees(th), n, rms, f" -- {why}" if why else ""))
                    if why:
                        verdict = f"LOST ({why})"
                        break
                    pose, bad = (scan_match.wrap(th), x, y), 0
                continue
            bad = 0
            pose = (scan_match.wrap(th), x, y)
            dist = math.hypot(x, y)
            vx = vy = wz = 0.0
            if positioned and dist > 2 * a.tol:
                positioned = False
            if not positioned and dist <= a.tol:
                positioned = True
            if positioned and abs(math.degrees(pose[0])) <= a.tol_deg:
                r.stop()
                r.spin(1.0)
                if r.next_scan(1.0) is not None:
                    P = scan_match.scan_points(r.scan, laser)
                    th, x, y, rms, n = scan_match.locate(P, home, pose)
                    pose = (scan_match.wrap(th), x, y)
                    print("  settle: x %+.3f y %+.3f yaw %+.1f (%d pairs, rms %.3f)"
                          % (x, y, math.degrees(pose[0]), n, rms))
                    if math.hypot(x, y) <= 2 * a.tol and abs(math.degrees(pose[0])) <= a.tol_deg:
                        verdict = "HOME"
                        break
                continue
            if not positioned:
                vx, vy, wz = home_step(pose, dist, a.holonomic)
            else:
                err = scan_match.wrap(-pose[0])
                if turns < ODOM_TURNS and odom_turn(r, err):
                    # on /odom by the whole error, then the LiDAR looks again from a standstill
                    turns += 1
                    pose = (scan_match.wrap(pose[0] + err), pose[1], pose[2])   # the match's seed
                    if a.trace:
                        print("  trace          odom turn %+.1f deg" % math.degrees(err), flush=True)
                    r.spin(0.5)
                    continue
                wz = max(0.2, min(0.6, abs(err))) * (1 if err > 0 else -1)
            near = path_blocked(P, vx, vy, a.radius)
            if near:
                verdict = "BLOCKED (%.2f m along the way home)" % near
                break
            if a.trace:
                print("  trace          cmd vx %+.3f vy %+.3f wz %+.3f" % (vx, vy, wz), flush=True)
            r.send(vx, wz, vy)
    finally:
        r.stop()
    print("%s after %.1f s: x %+.3f m y %+.3f m yaw %+.1f deg"
          % ("HOME_REACHED" if verdict == "HOME" else "HOME_FAILED", time.time() - start,
             pose[1], pose[2], math.degrees(pose[0]))
          + ("" if verdict == "HOME" else f" -- {verdict}"))
    return 0 if verdict == "HOME" else 1


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    for name in ("check", "inch", "home"):
        p = sub.add_parser(name)
        p.add_argument("--scans", type=int, default=5, help="scans per look (median per beam)")
        p.add_argument("--radius", type=float, default=0.20, help="the robot's circle, m (Nav2's robot_radius)")
        p.add_argument("--stamped", action="store_true", help="publish geometry_msgs/TwistStamped")
        p.add_argument("--scan-topic", default=None, help="default: /scan_raw when there is one, else /scan")
        if name in ("check", "inch"):
            p.add_argument("--need", type=float, required=True, help="nothing nearer than this, m")
            p.add_argument("--save", default="", help="keep the clear scan as the start snapshot")
        if name == "inch":
            p.add_argument("--max-travel", type=float, default=0.60, help="give up after this much inching, m")
        if name == "home":
            p.add_argument("--snap", required=True)
            p.add_argument("--tol", type=float, default=0.05)
            p.add_argument("--tol-deg", type=float, default=3.0)
            p.add_argument("--timeout", type=float, default=90.0)
            p.add_argument("--holonomic", action="store_true",
                           help="a mecanum base: slide home with the heading held, turn last")
            p.add_argument("--trace", action="store_true",
                           help="one line per scan: the match (pairs, rms), the pose and the command")
    a = ap.parse_args(argv)
    r = Robot(stamped=a.stamped, scan_topic=a.scan_topic)
    try:
        return {"check": cmd_check, "inch": cmd_inch, "home": cmd_home}[a.cmd](a, r)
    finally:
        r.close()


if __name__ == "__main__":
    sys.exit(main())
