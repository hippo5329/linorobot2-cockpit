#!/usr/bin/env python3
"""Room round a real robot before it moves, from its LiDAR -- and, when there is not
enough, inch it slowly to where there is.

    clearance.py check --need M [--save SNAP.json] [--blind START:WIDTH ... [--look-around]]
    clearance.py inch  --need M --radius R [--max-travel 0.6] [--stamped] [--save SNAP.json]
                       [--blind START:WIDTH ... [--look-around]]
    clearance.py home  --snap SNAP.json [--stamped] [--tol 0.05] [--tol-deg 3] [--blind START:WIDTH ...]

The 1-Click runs this on a real robot before anything drives it (SLAM's mapping, Nav2,
test_acc). A simulated robot needs none of it: its room is the one the simulation draws.

check   N scans (the whole circle: scan_raw when a lidar.mask makes /scan a narrower
        view), the median return per beam, the chassis left out. A LiDAR that cannot see
        all round (--blind: the robot's own mast or deck in the way, its lidar.mask
        sectors) leaves those bearings unseen, and an unseen bearing is not a clear one:
        with --look-around the robot first moves (forward; a --holonomic base in any seen
        direction, without turning) to the nearest spot where the whole circle a turn
        sweeps lies on ground the first look showed free, then turns in place in 30 deg
        steps until every bearing has been seen, merges the scans by the match (/odom as
        the guess), and turns back to that spot's heading, which is the start from then
        on. No such spot within --look-travel: refused, nothing moved. A return inside
        the circle stops the turn. Without --look-around the
        unseen bearings are reported, not judged. Prints the nearest
        return in each quadrant (front, left, back, right; +-45 deg about 0 = ahead,
        counter-clockwise) and exits 0 when nothing is nearer than --need, 1 when
        something is. --save keeps the points as the start snapshot for `home`.
inch    check; when something is nearer than --need, first one move straight to the open floor
        the look shows within --max-travel (room_spot: the most room, its whole circle seen free,
        a clear straight path), then while something is still nearer: pick the 5 cm step that most
        increases the nearest distance -- forward or backward along a heading every
        15 deg, turned to first, with the path swept by the robot's circle (--radius)
        kept clear -- take it at 0.05 m/s, and look again. A turn in place sweeps that
        circle too, so with something inside radius + 2 cm only straight steps are tried.
        Gives up (exit 1) when no step helps, or after --max-travel metres. With --blind,
        the look-around comes first and each new look keeps the remembered points in the
        unseen bearings (moved by /odom), so a step is never chosen into a blind sector
        just because nothing shows there. Every step
        is closed on /odom (the EKF: a tracked base's wheel yaw slips) with a timeout,
        and stops at once if a return comes within 0.08 m of the robot's circle in the
        direction of travel.
home    drive back to where `--save` took its snapshot: every scan matched against it
        (scan_match.icp, from the last pose), turn to face the start -- or away from it,
        to back up, whichever is the smaller turn -- drive, turn to the start heading,
        and settle: zero for a second and one more robust match before HOME is called.
        Refuses to move on a weak first fix (fewer than 150 pairs or rms over 0.025 m):
        from the wrong side of an obstacle a room can match in two places. With --blind, the
        path is judged in the blind bearings against the snapshot moved to where the robot is.

THE ROBOT MOVES in `inch` and `home`. /cmd_vel (TwistStamped with --stamped) at 10 Hz,
zero on every exit; the firmware's cmd_vel watchdog stops the wheels if this dies.
Nothing else may be driving the robot: it refuses when /cmd_vel already has a publisher.
Every closed-loop move stops and fails when its /odom (ODOM_STALE_S) or its guarding scan
(SCAN_STALE_S) stops coming. The robot's own body is every return within --body of the
centre (default: --radius less BODY_DAYLIGHT_M).
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
SEED_MAX_M = 3.0        # home: an odometry seed farther than this is a restarted count, not a move
HOME_MIN_FRAC = 0.40    # never under HOME_FLOOR_PAIRS: a LiDAR with a masked sector (a UNO Q
HOME_FLOOR_PAIRS = 100  # robot's ~290 points) moved 0.5 m sees 46 % of its snapshot at best
HOME_RIVAL_FRAC = 0.80  # another pose with this share of the best's inliers: ambiguous
TF_WAIT_TRIES = 100     # x 0.2 s: the base_link -> laser TF, before a snapshot is refused
HOME_MAX_RMS = 0.025
LOOK_STEP_DEG = 30.0    # one turn of the look-around
LOOK_MIN_PAIRS = 40     # a look-around step's match against the cloud so far; else /odom alone
LOOK_MAX_RMS = 0.05
THIN_M = 0.02           # merged clouds keep one point per cell this size
KNOWN_NO_RETURN_M = 1.0 # a beam with no return is taken as free only this far (a dark surface)
KNOWN_MARGIN_M = 0.05   # known-free ground stops this short of the return that bounds it
KNOWN_GRID_M = 0.03     # the turning circle is checked on a grid this fine
UNDER_BODY_M = 0.10     # ground this near the centre was under the robot: free even where blind
HOLD_GAIN = 1.5          # slide: rad/s of turn per rad off the start heading
HOLD_MAX_RPS = 0.3
# Every closed-loop move stops, and fails, when what it closes on or guards with goes stale.
# /odom (the EKF) comes at 40-50 Hz: 0.3 s without one is a dozen missed messages, not jitter,
# and at the inching speeds it is 1.5 cm or 6 deg of motion nobody measured. A move that kept
# closing on the last cached pose drove on for its whole timeout -- the unguarded retrace
# d / 0.05 + 8 s, a turn 15 s -- with nothing watching.
ODOM_STALE_S = 0.3
# The scan (10 Hz on the LiDARs we know) guards: 0.5 s is five missed scans, and at LIN_MPS the
# robot covers 2.5 cm in it, under a third of STOP_MARGIN_M, so the margin still holds when the
# last scan is that old. Older, and the guard is looking at a room the robot has left.
SCAN_STALE_S = 0.5
# The look-around's turn back: further off than this, the retrace is refused. Within it the
# retrace is aimed by the error /odom measured, so the line is the outbound one; what is left is
# the turn's own drift of the centre, and 5 deg is about what turn_by's 3 deg stop plus a coast
# leaves -- a failed turn (no /odom, a timeout) is far outside it.
BACK_TOL_DEG = 5.0
# The robot's own chassis, returns nearer the centre than --body. Its outline lies inside the
# circle the callers pass as --radius, which is Nav2's robot_radius plus 5 cm of daylight
# (gen_firmware_header.SIM_RADIUS_MARGIN_M): that daylight is room, not robot, so the default
# body is the radius less it. A fixed 0.20 m threw away a box 0.15 m from a 0.12 m robot, and
# read a 0.35 m robot's own posts as obstacles it could never get away from.
BODY_DAYLIGHT_M = 0.05
BODY_MIN_M = 0.05        # nearer than this a LiDAR's return is its own window or noise
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


def parse_blind(specs) -> list:
    """[(start_deg, width_deg)] from "START:WIDTH" strings (robot frame, counter-clockwise)."""
    out = []
    for spec in specs or []:
        a, w = (float(v) for v in str(spec).split(":"))
        if not 0.0 < w < 360.0:
            raise ValueError(f"--blind {spec}: the width must be between 0 and 360 deg")
        out.append((a % 360.0, w))
    return out


def in_blind(bearing_deg, blind) -> "np.ndarray":
    """True where a bearing (deg, base_link) lies in one of the blind sectors."""
    b = np.asarray(bearing_deg, float) % 360.0
    m = np.zeros(b.shape, bool)
    for a, w in blind:
        m |= ((b - a) % 360.0) < w
    return m


def bearings(P) -> "np.ndarray":
    return np.degrees(np.arctan2(P[:, 1], P[:, 0])) % 360.0 if len(P) else np.zeros(0)


def move_points(P, pose) -> "np.ndarray":
    """P, given in a frame at `pose` (yaw, x, y), expressed in the frame `pose` is given in."""
    c, s_ = math.cos(pose[0]), math.sin(pose[0])
    return np.c_[pose[1] + c * P[:, 0] - s_ * P[:, 1], pose[2] + s_ * P[:, 0] + c * P[:, 1]]


def into_frame(P, pose) -> "np.ndarray":
    """P, given in a frame, expressed in the frame at `pose` (yaw, x, y) in it."""
    c, s_ = math.cos(pose[0]), math.sin(pose[0])
    d = P - np.array([pose[1], pose[2]])
    return np.c_[c * d[:, 0] + s_ * d[:, 1], -s_ * d[:, 0] + c * d[:, 1]]


def thin(P, cell: float = THIN_M) -> "np.ndarray":
    """One point per grid cell: overlapping looks do not weigh a wall twice in the match."""
    if not len(P):
        return P
    _, i = np.unique(np.floor(P / cell).astype(np.int64), axis=0, return_index=True)
    return P[np.sort(i)]


def seen_bins(yaw_deg: float, blind) -> "np.ndarray":
    """The whole-degree bearings of the START frame that a look at heading yaw_deg sees."""
    w = np.arange(360) + 0.5
    return ~in_blind(w - yaw_deg, blind)


def carry(P_old, moved, P_new, blind, body: float = scan_match.BODY_M) -> "np.ndarray":
    """A new look with the old cloud's points kept where it is blind: P_old moved into the
    frame of the robot now (at `moved`, (yaw, x, y), in the old one); only the bearings the
    LiDAR cannot see keep remembered points."""
    if not blind or P_old is None or not len(P_old):
        return P_new
    Q = into_frame(P_old, moved)
    Q = Q[in_blind(bearings(Q), blind) & (np.hypot(Q[:, 0], Q[:, 1]) >= body)]
    return np.r_[P_new, Q] if len(Q) else P_new


def free_ranges(P, blind, origin=(0.0, 0.0)) -> "np.ndarray":
    """Per whole degree about the laser (`origin`, base_link): how far the ground is known to be
    free -- the nearest return in that degree or either neighbour, less KNOWN_MARGIN_M;
    KNOWN_NO_RETURN_M where there is none; 0 where the LiDAR is blind."""
    Q = np.asarray(P, float).reshape(-1, 2) - np.asarray(origin, float)
    rng = np.full(360, KNOWN_NO_RETURN_M + KNOWN_MARGIN_M)
    if len(Q):
        b = np.floor(bearings(Q)).astype(int) % 360
        np.minimum.at(rng, b, np.hypot(Q[:, 0], Q[:, 1]))
    rng = np.minimum(rng, np.minimum(np.roll(rng, 1), np.roll(rng, -1))) - KNOWN_MARGIN_M
    rng[in_blind(np.arange(360) + 0.5, blind)] = 0.0
    return np.maximum(rng, 0.0)


def disk(center, radius: float) -> "np.ndarray":
    """Grid points over a circle (and its edge), base_link."""
    g = np.arange(-radius, radius + 1e-9, KNOWN_GRID_M)
    X, Y = np.meshgrid(g, g)
    D = np.c_[X.ravel(), Y.ravel()]
    D = D[np.hypot(D[:, 0], D[:, 1]) <= radius]
    t = np.linspace(0.0, 2 * math.pi, 72, endpoint=False)
    return np.r_[D, radius * np.c_[np.cos(t), np.sin(t)]] + np.asarray(center, float)


def known_free(Q, ranges, origin=(0.0, 0.0)) -> "np.ndarray":
    """True for each point (base_link) on ground the look showed free; the ground under the
    robot (within UNDER_BODY_M of the centre) counts as free. Not BODY_M: that is where the
    scan's chassis returns end, and a 0.20 m allowance let a 0.25 m move pass with the
    turning circle still reaching into the blind sector."""
    D = np.asarray(Q, float) - np.asarray(origin, float)
    b = np.floor(bearings(D)).astype(int) % 360
    return (np.hypot(D[:, 0], D[:, 1]) <= ranges[b]) | (np.hypot(Q[:, 0], Q[:, 1]) < UNDER_BODY_M)


def open_spot(P, blind, circle: float, holonomic: bool, max_travel: float,
              origin=(0.0, 0.0), step: float = STEP_M):
    """(heading, distance): the nearest place where the whole circle a turn in place sweeps
    lies on ground already seen free, reached by a straight move that sweeps only seen ground
    -- forward only for a base that would have to turn to go anywhere else; None when there
    is none within max_travel. A blind sector behind the LiDAR means the robot's own spot is
    never wholly known, so the distance is never 0 while something is blind."""
    ranges = free_ranges(P, blind, origin)
    here = np.zeros(2)
    if not blind:
        return 0.0, 0.0
    heads = range(0, 360, HEADINGS_DEG) if holonomic else (0,)
    best = None
    for deg in heads:
        h = math.radians(deg)
        u = np.array([math.cos(h), math.sin(h)])
        for k in range(1, int(max_travel / step + 1e-9) + 1):
            c = k * step * u
            D = disk(c, circle)
            fresh = D[np.hypot(D[:, 0] - here[0], D[:, 1] - here[1]) > circle]   # swept on the way
            if not known_free(fresh, ranges, origin).all():
                break                                   # the way there leaves the known ground
            if known_free(D, ranges, origin).all():
                d = k * step
                score = (d, abs(scan_match.wrap(h)))
                if best is None or score < best[0]:
                    best = (score, h, d)
                break
    return None if best is None else (best[1], best[2])


ROOM_GRID_M = 0.10      # room_spot: candidate centres on a grid this fine
ROOM_MARGIN_M = 0.05    # room_spot: the room asked for, plus this
PATH_MARGIN_M = 0.10    # room_spot: the straight path keeps the robot's circle this far off returns


def seen_ranges(P, blind, origin=(0.0, 0.0)) -> "np.ndarray":
    """Per whole degree about the laser: how far the ground is known free -- up to the nearest
    return in that degree or either neighbour, less KNOWN_MARGIN_M; KNOWN_NO_RETURN_M where no
    beam came back; 0 where the LiDAR is blind. (free_ranges() caps every degree at
    KNOWN_NO_RETURN_M, returns or not: right for the look-around's 0.2 m turning circle, but a
    1 m room could then never count as seen.)"""
    Q = np.asarray(P, float).reshape(-1, 2) - np.asarray(origin, float)
    rng = np.full(360, np.inf)
    if len(Q):
        b = np.floor(bearings(Q)).astype(int) % 360
        np.minimum.at(rng, b, np.hypot(Q[:, 0], Q[:, 1]))
    rng = np.minimum(rng, np.minimum(np.roll(rng, 1), np.roll(rng, -1)))
    rng = np.where(np.isinf(rng), KNOWN_NO_RETURN_M + KNOWN_MARGIN_M, rng) - KNOWN_MARGIN_M
    rng[in_blind(np.arange(360) + 0.5, blind)] = 0.0
    return np.maximum(rng, 0.0)


def room_spot(P, need: float, radius: float, max_travel: float, blind=(), origin=(0.0, 0.0),
              spin_ok: bool = True):
    """(heading, distance, clearance) of the spot within max_travel with the most room, or
    None: nothing nearer than need (+ ROOM_MARGIN_M) all round, that whole circle on ground the
    look showed free (an unseen bearing is not a clear one), and a straight path there that
    keeps the robot's circle PATH_MARGIN_M off every return, on seen ground too. The score is
    the room less a quarter of the distance, so a nearer spot wins a near tie. Without
    `spin_ok` only straight ahead or behind (no turn in place).

    The 5 cm steps (best_step) climb the nearest distance greedily and stall in a corner: a
    tracked robot 1 m from boxes gave up "no room of 1.00 m" with 1.3 m of open floor 1-2 m
    away (2026-10-15, done by hand that day); this goes there in one move."""
    if not len(P):
        return None
    ranges = seen_ranges(P, blind, origin)
    cands = []
    g = np.arange(-max_travel, max_travel + 1e-9, ROOM_GRID_M)
    for x in g:                                  # the cheap tests first, on every candidate
        for y in g:
            d = math.hypot(x, y)
            if d < ROOM_GRID_M or d > max_travel:
                continue
            c = np.array([x, y])
            room = nearest(P - c)
            if room < need + ROOM_MARGIN_M:
                continue
            h = math.atan2(y, x)
            if not spin_ok and abs(turn_for(h)[0]) > 1e-6:
                continue
            if segment_clearance(P, c) < radius + PATH_MARGIN_M:
                continue
            cands.append((room - 0.25 * d, h, d, room, c))
    cands.sort(key=lambda t: -t[0])
    for score, h, d, room, c in cands:           # the seen-ground tests in score order
        if not known_free(disk(c, need), ranges, origin).all():
            continue
        n = max(2, int(d / 0.10))
        if all(known_free(disk(c * k / n, radius), ranges, origin).all() for k in range(1, n + 1)):
            return h, d, room
    return None


def turn_for(heading: float) -> tuple:
    """(turn, direction): reach a step along `heading` (base_link, rad) by turning `turn`
    and driving forward (+1) or backward (-1), whichever is the smaller turn."""
    h = scan_match.wrap(heading)
    if abs(h) <= math.pi / 2:
        return h, 1.0
    return scan_match.wrap(h + math.pi), -1.0


def best_step(P, radius: float, step: float = STEP_M, spin_ok: bool = True, unseen=()):
    """The step that most increases the nearest distance: (heading, nearest_after), or None
    when no step keeps the robot's circle off everything along its path or improves on
    standing still. Ties go to the smaller turn. No step heads into an `unseen` sector: an
    empty bearing there is a blind one, not a clear one."""
    here = nearest(P)
    best, best_score = None, -math.inf
    for deg in range(0, 360, HEADINGS_DEG):
        if unseen and in_blind([deg], unseen)[0]:
            continue
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

def body_for(radius: float, body: float = None) -> float:
    """The self-filter: returns nearer the centre than this are the robot itself. --body when
    given, else the --radius circle less BODY_DAYLIGHT_M (never under BODY_MIN_M)."""
    if body is not None and body > 0:
        return float(body)
    return max(BODY_MIN_M, float(radius) - BODY_DAYLIGHT_M)


class Robot:
    """The scans, /odom and /cmd_vel, for a short-lived tool."""

    def __init__(self, stamped: bool = False, scan_topic: str = None, body: float = scan_match.BODY_M):
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
        self.scan_t = self.odom_t = -math.inf
        self.body = body
        self.topic = scan_match.whole_circle_topic(self.node, override=scan_topic)

        def on_scan(m):
            self.take_scan(scan_match.scan_dict(m), m.header.frame_id)

        def on_odom(m):
            p, q = m.pose.pose.position, m.pose.pose.orientation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            self.take_odom((p.x, p.y, yaw))
        self.node.create_subscription(LaserScan, self.topic, on_scan, qos_profile_sensor_data)
        self.node.create_subscription(Odometry, "/odom", on_odom, 10)
        self.laser = None
        self.tf_ok = False

    # Arrival times on this host's monotonic clock: a board's or the EKF's header stamp is
    # another clock, and only "how long since one came" matters here.
    def take_scan(self, scan: dict, frame: str):
        self.frame = frame
        self.scan = scan
        self.scan_t = time.monotonic()

    def take_odom(self, pose):
        self.odom = tuple(pose)
        self.odom_t = time.monotonic()

    def odom_age(self) -> float:
        return time.monotonic() - self.odom_t

    def scan_age(self) -> float:
        return time.monotonic() - self.scan_t

    def pump(self, timeout: float = 0.05):
        """Wait up to `timeout` for a message, then take every one already queued: spin_once
        takes ONE callback per call, and a 10 Hz loop under 50 Hz /odom read a pose from a
        queue ten deep -- 0.2 s old, and stamped as if it had just come."""
        self.rclpy.spin_once(self.node, timeout_sec=timeout)
        for _ in range(30):
            self.rclpy.spin_once(self.node, timeout_sec=0.0)

    def spin(self, secs: float):
        end = time.time() + secs
        while time.time() < end:
            self.pump(min(0.05, max(0.0, end - time.time())))

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

    def scan_points(self, scan=None):
        """A scan's points in base_link, the robot's own body (self.body) left out."""
        return scan_match.scan_points(self.scan if scan is None else scan, self.mount(), body=self.body)

    def points(self, n: int = 5, timeout: float = 15.0):
        """The median cloud of the next n scans, in base_link (empty when none came)."""
        scans, end = [], time.time() + timeout
        while len(scans) < n and time.time() < end:
            s = self.next_scan(min(1.0, end - time.time()))
            if s is not None:
                scans.append(s)
        if not scans:
            return None
        self.last_scans = scans
        return scan_match.median_points(scans, self.mount(), body=self.body)

    def claim_cmd_vel(self) -> bool:
        """Refuse to drive a robot something else is driving."""
        if self.pub is not None:
            return True
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
        """The first /odom pose that ARRIVES after this call, or None: a pose cached from
        before is where the robot was, and a move closed on it closes on nothing."""
        since = time.monotonic()
        end = time.time() + timeout
        while time.time() < end:
            self.pump(0.05)
            if self.odom is not None and self.odom_t > since:
                return self.odom
        return None

    def stale(self, guarded: bool) -> str:
        """Why a closed-loop move must stop now, or '': the /odom it closes on, or the scan it
        guards with, has stopped coming."""
        if self.odom_age() > ODOM_STALE_S:
            return f"odometry stopped: the last /odom is {self.odom_age():.1f} s old"
        if guarded and self.scan_age() > SCAN_STALE_S:
            return f"scan stopped: the last scan is {self.scan_age():.1f} s old"
        return ""

    def blocked(self, direction: float, radius: float) -> float:
        """The nearest return in the robot's corridor in the direction of travel, when it is
        inside the stop margin of the circle; 0 when the way is open."""
        if self.scan is None:
            return 0.0
        P = self.scan_points()
        ahead = P[(np.sign(P[:, 0]) == np.sign(direction)) & (np.abs(P[:, 1]) < radius)]
        if len(ahead) and np.min(np.abs(ahead[:, 0])) < radius + STOP_MARGIN_M:
            return float(np.min(np.abs(ahead[:, 0])))
        return 0.0

    def turn_by(self, angle: float, timeout: float = 15.0, guard: float = 0.0) -> bool:
        """Turn in place by `angle`, closed on /odom; with a guard, stop the moment a return
        comes nearer the centre than that (the circle the turn sweeps, plus a margin)."""
        start = self.wait_odom()
        if start is None:
            print("  ❌ no /odom: cannot close a turn")
            return False
        end = time.time() + timeout
        while time.time() < end:
            self.pump(0.05)
            why = self.stale(guarded=bool(guard))
            if why:
                self.stop()
                print(f"  ❌ turn stopped: {why}")
                return False
            if guard and self.scan is not None:
                near = nearest(self.scan_points())
                if near < guard:
                    self.stop()
                    print(f"  ⚠️ turn stopped: a return {near:.2f} m from the centre")
                    return False
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
            self.pump(0.05)
            why = self.stale(guarded=True)
            if why:
                self.stop()
                print(f"  ❌ step stopped: {why}")
                return False
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

    def slide(self, dist: float, heading: float, radius: float, timeout: float = None,
              guard: bool = True) -> bool:
        """Move `dist` along `heading` (base_link) without turning -- vy for a mecanum base;
        heading 0 is a plain forward step. Closed on /odom, stopped by a return in the corridor
        unless `guard` is off (a retrace of ground just swept, into a blind sector). Either
        way it stops when /odom does: an unguarded retrace on a frozen pose drove on blind."""
        start = self.wait_odom()
        if start is None:
            print("  ❌ no /odom: cannot close a move")
            return False
        end = time.time() + (timeout or dist / LIN_MPS + 8.0)
        while time.time() < end:
            self.pump(0.05)
            why = self.stale(guarded=guard)
            if why:
                self.stop()
                print(f"  ❌ move stopped: {why}")
                return False
            if math.hypot(self.odom[0] - start[0], self.odom[1] - start[1]) >= dist:
                self.stop()
                return True
            # hold the heading, and the line in the start frame: a mecanum slide with no
            # correction turned MC200 -9.8 deg over 0.70 m (2026-10-15)
            err = scan_match.wrap(self.odom[2] - start[2])
            wz = max(-HOLD_MAX_RPS, min(HOLD_MAX_RPS, -HOLD_GAIN * err))
            vx = LIN_MPS * math.cos(heading - err)
            vy = LIN_MPS * math.sin(heading - err)
            if guard and self.scan is not None:
                b = path_blocked(self.scan_points(), vx, vy, radius)
                if b:
                    self.stop()
                    print(f"  ⚠️ stopped: a return {b:.2f} m in the way")
                    return False
            self.send(vx, wz, vy)
            time.sleep(0.05)
        self.stop()
        print("  ❌ the move timed out")
        return False

    def close(self):
        self.stop()
        self.node.destroy_node()
        self.rclpy.shutdown()


def report(P, need: float, unseen=()) -> bool:
    """Print the quadrants; True when nothing is nearer than `need`. `unseen` sectors (a
    blind LiDAR that did not look round) are named, never counted as clear or as blocked."""
    q = quadrants(P)
    near = nearest(P)
    fmt = lambda v: "none" if v == math.inf else f"{v:.2f} m"
    print("  quadrants (+-45 deg): " + ", ".join(f"{k} {fmt(v)}" for k, v in q.items())
          + f"; nearest {fmt(near)} (need {need:.2f} m)")
    for a, w in unseen:
        print(f"  ⚠️ unseen: {a:.0f}..{(a + w) % 360:.0f} deg (the LiDAR's blind sector; "
              f"--look-around turns to see it) -- not judged")
    return near >= need


def spin_match(P, cloud, guess, spread_deg: float = 10.0):
    """icp's (th, tx, ty, rms, pairs) of a look turned in place against the cloud so far: the
    heading searched first in half degrees about /odom's, then refined. ICP from /odom alone
    slid along the walls of a box room and kept 5-7 deg of a drifting yaw."""
    offs = np.arange(-spread_deg, spread_deg + 1e-9, 0.5)
    k = max(offs, key=lambda d: scan_match.inliers(P, cloud, guess[0] + math.radians(d), guess[1], guess[2]))
    return scan_match.icp(P, cloud, guess[0] + math.radians(k), (guess[1], guess[2]))


def look_around(r: "Robot", P0, blind, radius: float, scans: int, status: dict = None):
    """A blind LiDAR's whole circle: turn in place in LOOK_STEP_DEG steps until every bearing
    of the start frame has been seen, merge each look into the start frame (matched against
    the cloud so far, /odom as the guess), and turn back. None when it may not or could not
    turn: something inside the visible part of the circle the turn sweeps, no /odom, or a
    return that came inside it while turning. `status` gets "back": whether the robot is
    facing its start heading again (within BACK_TOL_DEG, by a fresh /odom), and "back_deg"."""
    status = {} if status is None else status
    status["back"], status["back_deg"] = True, 0.0
    guard = radius + SPIN_MARGIN_M
    if nearest(P0) < guard:
        print(f"  LOOK-AROUND REFUSED: a return {nearest(P0):.2f} m from the centre, inside the "
              f"{guard:.2f} m a turn in place sweeps")
        return None
    odom0 = r.wait_odom(5.0)
    if odom0 is None:
        print("  LOOK-AROUND REFUSED: no /odom to close the turns")
        return None
    status["back"] = False          # from here on, until a fresh /odom says it is
    seen = seen_bins(0.0, blind)
    cloud, turned, ok = P0, 0.0, True
    print(f"  looking around: {360 - int(seen.sum())} deg unseen, turning in "
          f"{LOOK_STEP_DEG:.0f} deg steps (THE ROBOT TURNS IN PLACE)", flush=True)
    try:
        while not seen.all() and turned < 360.0 - 1e-6:
            if not r.turn_by(math.radians(LOOK_STEP_DEG), guard=guard):
                ok = False
                break
            r.spin(0.5)
            P = r.points(scans)
            now = r.wait_odom(1.0)
            if P is None or now is None:
                print("  ❌ the scan or /odom stopped while looking around")
                ok = False
                break
            guess = odom_since(odom0, now)
            th, x, y, rms, n = spin_match(P, cloud, guess)
            pose = (th, x, y) if n >= LOOK_MIN_PAIRS and rms < LOOK_MAX_RMS else guess
            turned += LOOK_STEP_DEG
            cloud = thin(np.r_[cloud, move_points(P, pose)])
            seen |= seen_bins(math.degrees(pose[0]), blind)
            print(f"  look at {math.degrees(pose[0]):+.0f} deg: {len(P)} points "
                  + (f"({n} pairs, rms {rms:.3f})" if pose is not guess else "(by /odom: the match was weak)")
                  + f"; {360 - int(seen.sum())} deg still unseen", flush=True)
    finally:
        # back over ground the turn has just swept, so unguarded: a return that stopped the
        # turn would otherwise keep the robot facing away from its start. Whether it got there
        # is judged on a fresh /odom, never on the turn's say-so: the caller's retrace runs
        # along the start frame's line in the frame the robot faces NOW.
        now = r.wait_odom(1.0)
        if now is not None:
            back = -scan_match.wrap(now[2] - odom0[2])
            if abs(back) > math.radians(2.0):
                r.turn_by(back, timeout=30.0)
        r.stop()
        now = r.wait_odom(1.0)
        if now is None:
            status["back"], status["back_deg"] = False, math.nan
        else:
            off = math.degrees(scan_match.wrap(now[2] - odom0[2]))
            status["back"], status["back_deg"] = abs(off) <= BACK_TOL_DEG, off
    if not ok:
        return None
    return cloud


def save(path: str, P, r: "Robot", odom=None) -> bool:
    """The start snapshot, in base_link, with the laser mount it was taken with. Refused
    without the TF: points in the scan frame are not where the robot is, and `home` would
    later match them against base_link points. `odom`: the /odom pose the points are
    relative to (the start of a look-around), else /odom now."""
    if not r.tf_ok:
        print(f"  no start snapshot: no TF base_link -> {r.frame} in {TF_WAIT_TRIES * 0.2:.0f} s")
        return False
    odom = odom or r.wait_odom(2.0)
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


def warn_self_returns(r, blind) -> list:
    """One line when the body self-filter dropped returns that Nav2's collision monitor will
    NOT drop: inside the body, outside the blind (lidar.mask) sectors. This step passes them as
    the robot itself; the monitor counts them as a collision and holds every command at zero
    (a mecanum robot's mount, 0.03 m at 226-243 deg past a [130, 225] mask: "nearest 1.04 m"
    here, and Nav2 never moved it). The 1-Click refuses the Nav2 step on them
    (self_returns.py); this names them early."""
    import self_returns
    scans = getattr(r, "last_scans", None) or []
    arcs = self_returns.self_return_arcs(scans, body_of(r), 1, getattr(r, "laser", None) or (0.0, 0.0, 0.0), blind)
    if arcs:
        print(f"  ⚠️ left out as the robot's body, but not masked: {self_returns.arcs_text(arcs)} -- "
              f"Nav2's collision monitor sees them; add lidar.mask.sectors "
              f"{self_returns.sectors_text(arcs)}")
    return arcs


def first_look(a, r: Robot):
    """(P, unseen, odom0, rc): the first look, the whole circle when the LiDAR is blind
    somewhere and --look-around lets the robot turn to see it; P None (and the exit code)
    when it could not."""
    blind = parse_blind(a.blind)
    P = r.points(a.scans)
    if P is None:
        print(f"CLEARANCE FAIL: no scan on {r.topic}")
        return None, blind, None, 2
    r.mount()
    warn_self_returns(r, blind)
    if not (blind and a.look_around):
        return P, blind, None, 0
    if not r.claim_cmd_vel():
        return None, blind, None, 1
    # Turning sweeps the robot's whole circle, and behind the LiDAR that is ground nobody has
    # seen. So first move to where the circle lies on ground the look showed free, by a move
    # that sweeps only seen ground: the turn then sweeps nothing unknown.
    spot = open_spot(P, blind, a.radius + SPIN_MARGIN_M, a.holonomic, a.look_travel, r.laser[:2])
    if spot is None:
        print(f"CLEARANCE_REFUSED: no ground already seen free within {a.look_travel:.2f} m holds the "
              f"{a.radius + SPIN_MARGIN_M:.2f} m circle a turn sweeps -- move the robot by hand "
              f"(nothing moved)")
        return None, blind, None, 1
    h, d = spot
    if d > 0:
        print(f"  first {d:.2f} m toward {math.degrees(scan_match.wrap(h)):+.0f} deg, onto ground "
              f"already seen free (THE ROBOT MOVES)", flush=True)
        if not r.slide(d, h, a.radius):
            print("CLEARANCE_REFUSED: the move to open ground stopped -- move the robot by hand")
            return None, blind, None, 1
        r.spin(0.5)
    odom0 = r.wait_odom(5.0)
    P360, why, rc, back = None, "", 0, {"back": True, "back_deg": 0.0}
    P = r.points(a.scans) if d > 0 else P
    if P is None:
        why, rc = f"CLEARANCE FAIL: no scan on {r.topic}", 2
    else:
        P360 = look_around(r, P, blind, a.radius, a.scans, status=back)
        if P360 is None:
            why, rc = ("CLEARANCE_REFUSED: the LiDAR's blind sector could not be looked at -- "
                       "move the robot by hand"), 1
        else:
            print(f"  the whole circle: {len(P360)} points")
    if d > 0:
        # Back the way it came, which the move itself swept -- whether the look worked or not:
        # the robot ends where it was put, so a second look (the clearance step, then
        # test_acc's) does not creep it forward again. Two did, 0.70 m, onto boxes 0.51 m
        # ahead, and test_acc found no room; then a scan lost after the move left it out there.
        if not back["back"]:
            # The retrace is h + pi in the frame the robot faces now: turned 20 deg off, it
            # would back 0.35 m along a line nobody swept, unguarded, into the blind sector.
            # Guarded it is no better -- the guard is blind exactly there. Where it stands is
            # ground the first look showed free, wide enough to turn on: it stays there.
            off = ("no fresh /odom to say where it faces" if back["back_deg"] != back["back_deg"]
                   else f"{abs(back['back_deg']):.0f} deg off its start heading")
            print(f"CLEARANCE_REFUSED: the turn back did not finish ({off}), so the way back is "
                  f"not the way it came -- the robot is left where it is, {d:.2f} m toward "
                  f"{math.degrees(scan_match.wrap(h)):+.0f} deg from where it was put; move it by hand")
            return None, blind, None, 1
        print(f"  back {d:.2f} m to where it started, over ground just swept (THE ROBOT MOVES)",
              flush=True)
        # aimed by what is left of the turn back: h + pi in the start heading's frame
        if not r.slide(d, h + math.pi - math.radians(back["back_deg"]), a.radius, guard=False):
            print(why or "CLEARANCE_REFUSED: the move back stopped -- move the robot by hand")
            return None, blind, None, rc or 1
        r.spin(0.5)
        odom1 = r.wait_odom(5.0)
        if P360 is not None and (odom0 is None or odom1 is None):
            why, rc, P360 = "CLEARANCE FAIL: no /odom to carry the look back to the start", 2, None
        if P360 is not None:
            P360, odom0 = into_frame(P360, odom_since(odom0, odom1)), odom1
    if P360 is None:
        print(why)
        return None, blind, None, rc
    return P360, (), odom0, 0


def cmd_check(a, r: Robot) -> int:
    P, unseen, odom0, rc = first_look(a, r)
    if P is None:
        return rc
    print(f"  reading {r.topic}; laser at x={r.laser[0]:.3f} y={r.laser[1]:.3f} "
          f"yaw={math.degrees(r.laser[2]):.1f} deg in base_link")
    ok = report(P, a.need, unseen)
    if a.save and ok and not save(a.save, P, r, odom=odom0):
        print("CLEARANCE_REFUSED: no start snapshot, so no way home (nothing moved)")
        return 1
    print("CLEARANCE_OK" if ok else "CLEARANCE_REFUSED")
    return 0 if ok else 1


def cmd_inch(a, r: Robot) -> int:
    P, unseen, odom0, rc = first_look(a, r)
    if P is None:
        return rc
    blind = parse_blind(a.blind)
    print(f"  reading {r.topic}")
    if report(P, a.need, unseen):
        if a.save and not save(a.save, P, r, odom=odom0):
            print("CLEARANCE_REFUSED: no start snapshot, so no way home (nothing moved)")
            return 1
        print("CLEARANCE_OK (no inching needed)")
        return 0
    if not r.claim_cmd_vel():
        return 1
    travel, k = 0.0, 0
    last = odom0 or r.wait_odom(2.0)
    try:
        # First the open floor in one move, when the look shows some within reach; the 5 cm
        # steps below are the fallback (and finish the job if that move falls short).
        here = nearest(P)
        spot = room_spot(P, a.need, a.radius, a.max_travel, blind, r.laser[:2] if r.laser else (0.0, 0.0),
                         spin_ok=here >= a.radius + SPIN_MARGIN_M)
        if spot is not None:
            h, d, room = spot
            turn, direction = turn_for(h)
            print(f"  open floor: {room:.2f} m clear all round, {d:.2f} m away at {math.degrees(h):+.0f} deg "
                  f"-> turn {math.degrees(turn):+.0f} deg, {'forward' if direction > 0 else 'back'} "
                  f"{d:.2f} m (THE ROBOT MOVES)", flush=True)
            if abs(turn) > 1e-6 and not r.turn_by(turn):
                return 1
            r.drive(d, direction, a.radius, timeout=d / LIN_MPS + 10.0)
            moved = r.wait_odom(1.0)
            travel += (math.hypot(moved[0] - last[0], moved[1] - last[1])
                       if (moved is not None and last is not None) else d)
            r.spin(0.5)
            P_new = r.points(a.scans)
            if P_new is None:
                print("CLEARANCE FAIL: the scan stopped")
                return 2
            if blind and last is not None and moved is not None:
                P = carry(P, odom_since(last, moved), P_new, blind, body_of(r))
            else:
                P = P_new
            last = moved or last
            if report(P, a.need, unseen):
                if a.save and not save(a.save, P, r):
                    print("CLEARANCE_REFUSED: no start snapshot, so no way home")
                    return 1
                print(f"CLEARANCE_OK after a move to open floor, {travel:.2f} m")
                return 0
        while travel + STEP_M <= a.max_travel + 1e-9:
            here = nearest(P)
            step = best_step(P, a.radius, spin_ok=here >= a.radius + SPIN_MARGIN_M, unseen=unseen)
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
            P_new = r.points(a.scans)
            if P_new is None:
                print("CLEARANCE FAIL: the scan stopped")
                return 2
            now = r.wait_odom(1.0)
            if now is None:
                # the next step closes on it, and the blind bearings' memory moves by it
                print("CLEARANCE FAIL: odometry stopped (no /odom after the step)")
                return 2
            if blind and last is not None:
                # the blind bearings keep what was seen there, moved by the step
                P = carry(P, odom_since(last, now), P_new, blind, body_of(r))
            else:
                P = P_new
            last = now
            if report(P, a.need, unseen):
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
        if r.odom_age() > ODOM_STALE_S:
            # the LiDAR-closed turn takes over: it does not need /odom
            print(f"  ⚠️ odom turn stopped: odometry stopped ({r.odom_age():.1f} s since the last /odom)")
            r.stop()
            return False
        e = scan_match.wrap(target - r.odom[2])
        if abs(e) < math.radians(ODOM_TURN_DONE_DEG):
            break
        r.send(0.0, math.copysign(ODOM_TURN_WZ, e))
    r.stop()
    return True


def body_of(r) -> float:
    return getattr(r, "body", scan_match.BODY_M)


def guard_points(P, home, pose, blind, body: float = scan_match.BODY_M) -> "np.ndarray":
    """What the drive home judges its path by: the live scan, plus -- in the LiDAR's blind
    bearings -- the snapshot's points moved to where the robot is now (`pose` in the home
    frame), as carry() does for a look. A holonomic base slides home without turning, so the
    way back can lie in its blind sector; the live scan has nothing there, and "nothing" read
    as "clear". The snapshot is the whole circle the look-around saw at the start."""
    if not blind:
        return P
    return carry(home, pose, P, blind, body)


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
    blind = parse_blind(getattr(a, "blind", None))
    body = body_of(r)
    # Standing still: the median of a few scans, not the first one after the board came back
    # (test_acc's reboot) -- a partial first revolution matches nothing.
    S = r.points(3)
    if S is None or not len(S):
        print(f"HOME ABORT: no scan on {r.topic}")
        return 2
    cands = scan_match.locate_candidates(S, home)
    # The odometry's account of the move seeds more starts: from home's spot alone, every 30
    # degrees, a robot 0.32 m off and turned 15 deg fell into a wrong fit (2026-10-15). Only
    # more candidates under the same gate: odometry that drifted (the EKF while test_acc ran)
    # fits worse and loses.
    now = r.wait_odom(2.0) if snap.get("odom") else None
    seeds = []
    if now:
        seed = odom_since(snap["odom"], now)
        print("  odometry since the snapshot: x %+.3f y %+.3f yaw %+.1f deg"
              % (seed[1], seed[2], math.degrees(seed[0])))
        # A board that rebooted (test_acc's app=base) restarts its odometry: the difference
        # is then metres of nothing (x -7.9 y +9.0 on MC200), not a move.
        if math.hypot(seed[1], seed[2]) <= SEED_MAX_M:
            seeds.append(seed)
        else:
            print("  (not a move: the odometry restarted)")
    # Something else's count of the move -- test_acc's dead reckoning, by its gyro and by its
    # wheels: after a 45 deg turn none of the starts from home's spot found it (25 pairs).
    for g in getattr(a, "guess", None) or []:
        yaw, x, y = (float(v) for v in g.split(":"))
        seeds.append((math.radians(yaw), x, y))
    for seed in seeds:
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
            P = scan_match.scan_points(r.scan, laser, body=body)
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
                    cands = scan_match.locate_candidates(scan_match.scan_points(r.scan, laser, body=body), home)
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
                    P = scan_match.scan_points(r.scan, laser, body=body)
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
            near = path_blocked(guard_points(P, home, pose, blind, body), vx, vy, a.radius)
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
        p.add_argument("--body", type=float, default=None,
                       help="returns nearer the centre than this are the robot itself, m "
                            f"(default: --radius less {BODY_DAYLIGHT_M:.2f}, the callers' daylight)")
        p.add_argument("--blind", action="append", default=[], metavar="START:WIDTH",
                       help="a bearing sector the LiDAR cannot see (deg, base_link, counter-"
                            "clockwise; the robot's lidar.mask sectors); repeat for more")
        if name in ("check", "inch"):
            p.add_argument("--need", type=float, required=True, help="nothing nearer than this, m")
            p.add_argument("--save", default="", help="keep the clear scan as the start snapshot")
            p.add_argument("--look-around", action="store_true",
                           help="with --blind: move onto ground already seen free, then turn in "
                                "place until every bearing has been seen, and back (THE ROBOT MOVES)")
            p.add_argument("--look-travel", type=float, default=0.80,
                           help="the farthest the look-around moves to find open ground, m")
            p.add_argument("--holonomic", action="store_true",
                           help="a mecanum base: the look-around may slide sideways to open ground")
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
            p.add_argument("--guess", action="append", default=[], metavar="YAW:X:Y",
                           help="another first-fix start: the robot's pose in the snapshot's frame "
                                "(deg, m, m) as something else counted it -- test_acc's HOME line")
    a = ap.parse_args(argv)
    try:
        parse_blind(a.blind)
    except ValueError as e:
        ap.error(str(e))
    r = Robot(stamped=a.stamped, scan_topic=a.scan_topic, body=body_for(a.radius, a.body))
    try:
        return {"check": cmd_check, "inch": cmd_inch, "home": cmd_home}[a.cmd](a, r)
    finally:
        r.close()


if __name__ == "__main__":
    sys.exit(main())
