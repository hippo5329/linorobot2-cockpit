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


def locate(S, R, guess=None, spread=(0.0, -15.0, 15.0, -30.0, 30.0)):
    """ICP from the guess and from the guess turned by each `spread` degrees -- or, with no
    guess, from every 30 degrees round -- keeping the one with the most inliers.
    (th, tx, ty, rms, pairs)."""
    if guess is None:
        guess, spread = (0.0, 0.0, 0.0), tuple(range(0, 360, 30))
    best, best_in = None, -1
    for d in spread:
        r = icp(S, R, guess[0] + math.radians(d), (guess[1], guess[2]))
        k = inliers(S, R, r[0], r[1], r[2])
        if k > best_in:
            best, best_in = r, k
    return (wrap(best[0]),) + tuple(best[1:])


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


def laser_pose(node, frame: str, base_frame: str = "base_link", tries: int = 25):
    """The scan frame's (x, y, yaw) in base_link, from TF; None when TF has none."""
    import rclpy
    from tf2_ros import Buffer, TransformListener
    buf = Buffer()
    TransformListener(buf, node)
    for _ in range(tries):
        try:
            tf = buf.lookup_transform(base_frame, frame, rclpy.time.Time())
            q = tf.transform.rotation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            return (tf.transform.translation.x, tf.transform.translation.y, yaw)
        except Exception:
            rclpy.spin_once(node, timeout_sec=0.2)
    return None
