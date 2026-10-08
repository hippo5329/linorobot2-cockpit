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
HOME_MIN_PAIRS = 150
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
        if self.laser is None:
            self.laser = scan_match.laser_pose(self.node, self.frame or "laser")
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

    def send(self, vx: float, wz: float):
        m = self.T()
        t = m.twist if self.stamped else m
        if self.stamped:
            m.header.stamp = self.node.get_clock().now().to_msg()
            m.header.frame_id = "base_link"
        t.linear.x, t.angular.z = float(vx), float(wz)
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


def save(path: str, P):
    with open(path, "w") as fh:
        json.dump({"points": P.tolist(), "quadrants": quadrants(P)}, fh)
    print(f"  start snapshot: {len(P)} points -> {path}")


def cmd_check(a, r: Robot) -> int:
    P = r.points(a.scans)
    if P is None:
        print(f"CLEARANCE FAIL: no scan on {r.topic}")
        return 2
    print(f"  reading {r.topic}; laser at x={r.laser[0]:.3f} y={r.laser[1]:.3f} "
          f"yaw={math.degrees(r.laser[2]):.1f} deg in base_link")
    ok = report(P, a.need)
    if a.save and ok:
        save(a.save, P)
    print("CLEARANCE_OK" if ok else "CLEARANCE_REFUSED")
    return 0 if ok else 1


def cmd_inch(a, r: Robot) -> int:
    P = r.points(a.scans)
    if P is None:
        print(f"CLEARANCE FAIL: no scan on {r.topic}")
        return 2
    print(f"  reading {r.topic}")
    if report(P, a.need):
        if a.save:
            save(a.save, P)
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
                if a.save:
                    save(a.save, P)
                print(f"CLEARANCE_OK after {k} inch(es), {travel:.2f} m")
                return 0
        print(f"CLEARANCE_REFUSED: still under {a.need:.2f} m after {travel:.2f} m of inching "
              f"(--max-travel {a.max_travel:.2f})")
        return 1
    finally:
        r.stop()


def cmd_home(a, r: Robot) -> int:
    home = np.asarray(json.load(open(a.snap))["points"], float)
    if not r.claim_cmd_vel():
        return 1
    if r.next_scan(10.0) is None:
        print(f"HOME ABORT: no scan on {r.topic}")
        return 2
    laser = r.mount()
    th, x, y, rms, n = scan_match.locate(scan_match.scan_points(r.scan, laser), home)
    pose = (scan_match.wrap(th), x, y)
    print("  first fix: x %+.3f y %+.3f yaw %+.1f deg -- %d pairs, rms %.3f"
          % (x, y, math.degrees(pose[0]), n, rms))
    if n < HOME_MIN_PAIRS or not rms <= HOME_MAX_RMS:
        print("HOME ABORT: the first fix is weak (%d pairs, rms %.3f) -- not driving on a guess" % (n, rms))
        return 2
    bad, verdict, positioned = 0, "TIMEOUT", False
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
            if n < 60 or not rms < 0.08:
                r.send(0, 0)
                bad += 1
                if bad >= 10:
                    verdict = "LOST (no match: %d pairs, rms %.3f)" % (n, rms)
                    break
                continue
            bad = 0
            pose = (scan_match.wrap(th), x, y)
            dist = math.hypot(x, y)
            vx = wz = 0.0
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
                face, d = turn_for(math.atan2(-y, -x) - pose[0])
                err = face
                if abs(err) > math.radians(8):
                    wz = max(0.2, min(0.6, abs(err))) * (1 if err > 0 else -1)
                else:
                    vx = d * min(0.12, 0.5 * dist + 0.04)
                    wz = max(-0.4, min(0.4, 1.5 * err))
            else:
                err = scan_match.wrap(-pose[0])
                wz = max(0.2, min(0.6, abs(err))) * (1 if err > 0 else -1)
            if vx:
                ahead = P[(np.sign(P[:, 0]) == np.sign(vx)) & (np.abs(P[:, 1]) < a.radius)]
                if len(ahead) and np.min(np.abs(ahead[:, 0])) < a.radius + STOP_MARGIN_M:
                    verdict = "BLOCKED (%.2f m %s)" % (np.min(np.abs(ahead[:, 0])), "ahead" if vx > 0 else "behind")
                    break
            r.send(vx, wz)
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
    a = ap.parse_args(argv)
    r = Robot(stamped=a.stamped, scan_topic=a.scan_topic)
    try:
        return {"check": cmd_check, "inch": cmd_inch, "home": cmd_home}[a.cmd](a, r)
    finally:
        r.close()


if __name__ == "__main__":
    sys.exit(main())
