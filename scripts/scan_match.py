"""Where the robot is from its own LiDAR, with the room as the reference.

A real robot's wheels cannot say where it is: tracks slip, wheels skid, a turn in place
moves the centre. The room does not move, so a scan matched against a scan taken
earlier says how the robot moved between the two. This is the matcher the 1-Click's
clearance step (clearance.py) uses to find its way back to where it started.

  * scan_points()   a LaserScan as (N, 2) points in base_link, the chassis left out
  * icp()           2-D point-to-point ICP, trimmed: of the pairs inside the gate only the
                    best fraction is used, because a real room is not a set of walls --
                    chair legs, a person walking past, a cable on the floor -- and the
                    pairs those make are the worst ones
  * locate()        the robust match: ICP from several starting angles, and the one that
                    puts the MOST points within 5 cm wins. Not the lowest rms: a wrong
                    match that pairs a few points well beats the right one on rms.
  * median_points() one cloud from several scans of a robot standing still
  * track()         the path a moving robot took, scan by scan (speed_check.py)

Poses are (yaw, x, y): the pose of the scan's frame in the reference's frame,
p_ref = Rot(yaw) p + (x, y). Pure numpy; the ROS helpers import rclpy lazily so the
geometry is testable anywhere.
"""
import math
import time

import numpy as np

BODY_M = 0.20       # returns nearer the centre than this are the robot's own chassis
RANGE_MAX_M = 8.0   # beyond this an LD19-class return is too sparse to match


def wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def scan_points(scan: dict, laser=(0.0, 0.0, 0.0), body: float = BODY_M, rmax: float = RANGE_MAX_M):
    """(N, 2) points in base_link from a scan dict (scan_dict()), the chassis (< body m) left
    out. `laser` is the scan frame's (x, y, yaw) in base_link. NaN and inf beams (a masked
    sector, no return) are dropped."""
    r = np.asarray(scan["ranges"], float)
    th = scan["angle_min"] + np.arange(len(r)) * scan["angle_increment"] + laser[2]
    ok = np.isfinite(r) & (r >= scan["range_min"]) & (r <= min(scan["range_max"], rmax))
    x = laser[0] + r[ok] * np.cos(th[ok])
    y = laser[1] + r[ok] * np.sin(th[ok])
    keep = np.hypot(x, y) >= body
    return np.c_[x[keep], y[keep]]


def icp(S, R, th: float = 0.0, t=(0.0, 0.0), iters: int = 30, gate: float = 0.30, keep: float = 0.8):
    """(th, tx, ty, rms, pairs): the pose of S's frame in R's frame, from (th, t) as the guess."""
    t = np.asarray(t, float)
    rms, n = float("nan"), 0
    if len(S) == 0 or len(R) == 0:
        return th, float(t[0]), float(t[1]), rms, 0
    for k in range(iters):
        c, s = math.cos(th), math.sin(th)
        P = S @ np.array([[c, s], [-s, c]]) + t
        d2 = ((P[:, None, :] - R[None, :, :]) ** 2).sum(-1)
        j = d2.argmin(1)
        dist = np.sqrt(d2[np.arange(len(P)), j])
        g = gate if k < iters // 2 else min(gate, 0.10)
        m = dist < g
        if m.sum() >= 20 and keep < 1.0:
            m &= dist <= np.quantile(dist[m], keep)
        n = int(m.sum())
        if n < 20:
            break
        A, B = P[m], R[j[m]]
        ca, cb = A.mean(0), B.mean(0)
        H = (A - ca).T @ (B - cb)
        dth = math.atan2(H[0, 1] - H[1, 0], H[0, 0] + H[1, 1])
        c, s = math.cos(dth), math.sin(dth)
        Rm = np.array([[c, -s], [s, c]])
        t = Rm @ t + (cb - Rm @ ca)
        th += dth
        rms = float(np.sqrt((dist[m] ** 2).mean()))
        if abs(dth) < 1e-5 and np.abs(cb - Rm @ ca).max() < 1e-5:
            break
    return th, float(t[0]), float(t[1]), rms, n


def inliers(S, R, th: float, tx: float, ty: float, tol: float = 0.05) -> int:
    """How many of S's points land within `tol` of R once moved by (th, tx, ty)."""
    if len(S) == 0 or len(R) == 0:
        return 0
    c, s = math.cos(th), math.sin(th)
    P = S @ np.array([[c, s], [-s, c]]) + np.array([tx, ty])
    d2 = ((P[:, None, :] - R[None, :, :]) ** 2).sum(-1)
    return int((d2.min(1) < tol * tol).sum())


def locate_candidates(S, R, guess=None, spread=(0.0, -15.0, 15.0, -30.0, 30.0)):
    """Every ICP start's result, most inliers first: [(inliers, (th, tx, ty, rms, pairs))].
    From the guess and the guess turned by each `spread` degrees -- or, with no guess, from
    every 30 degrees round."""
    if guess is None:
        guess, spread = (0.0, 0.0, 0.0), tuple(range(0, 360, 30))
    out = []
    for d in spread:
        r = icp(S, R, guess[0] + math.radians(d), (guess[1], guess[2]))
        out.append((inliers(S, R, r[0], r[1], r[2]), (wrap(r[0]),) + tuple(r[1:])))
    out.sort(key=lambda c: -c[0])
    return out


def locate(S, R, guess=None, spread=(0.0, -15.0, 15.0, -30.0, 30.0)):
    """The candidate with the most inliers (locate_candidates). (th, tx, ty, rms, pairs)."""
    return locate_candidates(S, R, guess, spread)[0][1]


def rival(cands, min_m: float = 0.25, min_deg: float = 25.0):
    """The best candidate that is a DIFFERENT pose from the first (farther than min_m or
    turned more than min_deg): (inliers, pose), or None. A mirror room or a symmetric
    corner fits nearly as well as the truth from somewhere else, and only this says so."""
    if not cands:
        return None
    th0, x0, y0 = cands[0][1][:3]
    for k, c in cands[1:]:
        th, x, y = c[:3]
        if math.hypot(x - x0, y - y0) > min_m or abs(math.degrees(wrap(th - th0))) > min_deg:
            return (k, c)
    return None


def median_points(scans: list, laser=(0.0, 0.0, 0.0), min_frac: float = 0.5, body: float = BODY_M):
    """One cloud from several scans of a robot standing still: per beam, the median of the
    returns seen in most of them -- a passing foot or a flicker does not make it in."""
    beams = {}
    for s in scans:
        r = np.asarray(s["ranges"], float)
        th = s["angle_min"] + np.arange(len(r)) * s["angle_increment"] + laser[2]
        ok = np.isfinite(r) & (r >= s["range_min"]) & (r <= min(s["range_max"], RANGE_MAX_M))
        for i in np.nonzero(ok)[0]:
            beams.setdefault(int(i), []).append((laser[0] + r[i] * math.cos(th[i]),
                                                 laser[1] + r[i] * math.sin(th[i])))
    need = max(2, int(len(scans) * min_frac)) if len(scans) > 1 else 1
    pts = [np.median(np.asarray(v), 0) for v in beams.values() if len(v) >= need]
    P = np.asarray(pts, float).reshape(-1, 2)
    return P[np.hypot(P[:, 0], P[:, 1]) >= body]



def compose(a, b):
    """Pose a (yaw, x, y), then b expressed in a's frame."""
    c, s = math.cos(a[0]), math.sin(a[0])
    return (a[0] + b[0], a[1] + c * b[1] - s * b[2], a[2] + s * b[1] + c * b[2])


def relative(a, b):
    """Pose b in pose a's frame."""
    c, s = math.cos(a[0]), math.sin(a[0])
    dx, dy = b[1] - a[1], b[2] - a[2]
    return (b[0] - a[0], c * dx + s * dy, -s * dx + c * dy)


def track(scans: list, laser=(0.0, 0.0, 0.0), kf_yaw: float = math.radians(30), kf_dist: float = 0.15,
          progress=None):
    """Where the robot went, scan by scan, with the room as the reference.

    [(stamp, x, y, yaw, rms, pairs)] -- base_link in the frame of the first scan, yaw
    unwrapped (a full turn reads 2 pi) -- and `end`, the last scan matched directly against
    the first. Every scan is matched against a keyframe from the previous pose, a new
    keyframe every kf_yaw or kf_dist. Keyframe tracking drifts a little (each match slides
    along the walls, ~1.7 % of a turn); the end match does not, so when the robot turned
    more than 45 deg the yaw is scaled to agree with it -- a loop closure for a turn in
    place. A scan with too few returns, or a weak match, keeps the last pose.

    `progress(done, total)`, if given, is called every 50 scans: matching a few hundred scans
    takes a minute on a small robot computer, and a console silent that long reads as hung."""
    if not scans:
        return [], None
    pts = [scan_points(s, laser) for s in scans]
    kf_pose, kf_pts = (0.0, 0.0, 0.0), pts[0]
    pose = (0.0, 0.0, 0.0)
    out = [(scans[0]["stamp"], 0.0, 0.0, 0.0, 0.0, len(pts[0]))]
    for i, (sc, P) in enumerate(zip(scans[1:], pts[1:]), 2):
        if progress and i % 50 == 0:
            progress(i, len(scans))
        if len(P) < 30:
            continue
        guess = relative(kf_pose, pose)
        th, x, y, rms, n = icp(P, kf_pts, guess[0], (guess[1], guess[2]))
        if n < 30:
            continue
        pose = compose(kf_pose, (th, x, y))
        out.append((sc["stamp"], pose[1], pose[2], pose[0], rms, n))
        rel = relative(kf_pose, pose)
        if abs(rel[0]) > kf_yaw or math.hypot(rel[1], rel[2]) > kf_dist:
            kf_pose, kf_pts = pose, P
    turns = round(pose[0] / (2 * math.pi))
    g = pose[0] - turns * 2 * math.pi
    th, x, y, rms, n = locate(pts[-1], pts[0], (g, pose[1], pose[2]))
    th = g + wrap(th - g)          # locate() wraps; keep it on the tracked side of +-pi
    end = {"yaw": th + turns * 2 * math.pi, "x": x, "y": y, "rms": rms, "pairs": n,
           "tracked_yaw": pose[0], "tracked_x": pose[1], "tracked_y": pose[2]}
    if n >= 50 and abs(pose[0]) > math.radians(45):
        k = end["yaw"] / pose[0]
        out = [(t, px, py, yaw * k, r, m) for t, px, py, yaw, r, m in out]
    return out, end

# ---- ROS helpers (rclpy imported lazily) ----

def scan_dict(m, stamp: float = None) -> dict:
    return {"stamp": stamp if stamp is not None else m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
            "angle_min": m.angle_min, "angle_increment": m.angle_increment,
            "range_min": m.range_min, "range_max": m.range_max, "ranges": list(m.ranges)}


def whole_circle_topic(node, wait: float = 8.0, override: str = None) -> str:
    """The scan with the LiDAR's whole circle. A lidar.mask (lidar_mask.py) makes the driver
    publish scan_raw and laser_filters makes /scan the masked view SLAM and Nav2 get; a
    clearance check must see everything round the robot, so it reads scan_raw when there
    is one."""
    if override:
        return override
    import rclpy
    end = time.time() + wait
    while time.time() < end:
        names = {n for n, _ in node.get_topic_names_and_types()}
        if "/scan_raw" in names:
            return "/scan_raw"
        if "/scan" in names and time.time() > end - wait / 2:
            return "/scan"
        rclpy.spin_once(node, timeout_sec=0.2)
    return "/scan"


# How long a reader waits for the base_link -> laser TF, in SECONDS of wall time.
#
# It used to be a count of rclpy.spin_once() calls at timeout_sec=0.2, read as "x 0.2 s".
# spin_once runs ONE callback and returns at once when one is ready, and the node doing
# the waiting is subscribed to /scan, often /odom, and through the listener to /tf at the
# EKF's rate -- so ten "tries" were ten callbacks, a few tens of milliseconds, and
# clearance's "100 x 0.2 s = 20 s" about a second. The listener was also created only
# after the scans had been collected, so that window was all the time DDS had to match
# the new /tf_static reader with robot_state_publisher's writer and deliver its latched
# sample; /tf's frequent samples got through, the one static sample did not. A real
# robot whose robot_state_publisher did publish base_link -> laser was judged "no TF",
# in the scan frame, and a probe built the same way saw only map, odom and base_link.
TF_WAIT_S = 5.0


def wait_until(attempt, spin, timeout: float, clock=time.monotonic):
    """attempt() until it returns a value, spin(seconds) between tries; None after
    `timeout` seconds by `clock`. The bound is time, never a count of spins: a spin that
    runs one ready callback returns in microseconds on a busy node."""
    end = clock() + timeout
    while True:
        try:
            return attempt()
        except Exception:
            pass
        left = end - clock()
        if left <= 0:
            return None
        spin(min(0.1, left))


def tf_frame(frame: str) -> str:
    """tf2 refuses a frame id with a leading '/' (ROS 1 style, still sent by some
    drivers) as invalid, which a lookup reports like a missing transform."""
    return (frame or "").lstrip("/")


def frames_in(yaml_text: str) -> list:
    """Every frame named in Buffer.all_frames_as_yaml(): children and their parents."""
    import yaml
    try:
        tree = yaml.safe_load(yaml_text) or {}
    except Exception:
        return []
    if not isinstance(tree, dict):
        return []
    names = set(tree)
    names |= {v.get("parent") for v in tree.values() if isinstance(v, dict) and v.get("parent")}
    return sorted(str(n) for n in names)


class TfWatch:
    """A TF buffer fed on `node`'s own subscriptions (/tf and the latched /tf_static).

    Start it BEFORE waiting for anything else -- the scans, the odometry -- so DDS
    discovery and the static sample's delivery overlap that wait; then look up with
    laser_pose(), and close() it: a listener left on a node keeps taking /tf callbacks
    from the node's own spin_once() loop for as long as the node lives."""

    def __init__(self, node):
        from tf2_ros import Buffer, TransformListener
        self.node = node
        self.buf = Buffer()
        self.listener = TransformListener(self.buf, node)

    def laser_pose(self, frame: str, base_frame: str = "base_link", timeout: float = TF_WAIT_S):
        """The scan frame's (x, y, yaw) in base_frame; None when TF has none in `timeout` s."""
        import rclpy

        def attempt():
            tf = self.buf.lookup_transform(tf_frame(base_frame), tf_frame(frame), rclpy.time.Time())
            q = tf.transform.rotation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            return (tf.transform.translation.x, tf.transform.translation.y, yaw)
        return wait_until(attempt, lambda t: rclpy.spin_once(self.node, timeout_sec=t), timeout)

    def frames(self) -> list:
        """The frames this buffer has heard of, for a message that says why a lookup failed."""
        try:
            return frames_in(self.buf.all_frames_as_yaml())
        except Exception:
            return []

    def close(self):
        try:
            self.listener.unregister()
        except Exception:
            pass


def laser_pose(node, frame: str, base_frame: str = "base_link", timeout: float = TF_WAIT_S):
    """The scan frame's (x, y, yaw) in base_link, from TF; None when TF has none in
    `timeout` seconds. A caller that can, starts a TfWatch earlier and asks it instead."""
    watch = TfWatch(node)
    try:
        return watch.laser_pose(frame, base_frame, timeout)
    finally:
        watch.close()
