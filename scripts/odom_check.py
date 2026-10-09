"""Odometry against the LiDAR: how far the base REALLY went and turned.

The drive suite compares /odom with the command, and the base's speed loop steers by the
same odometry -- so a wrong wheel diameter, a wrong track width, a track that slips in a
turn, all leave /odom agreeing with the command while the robot covers another distance
or turns another angle. The room does not move: scans taken during a manoeuvre, matched
one to the next (scan_match.locate, the matcher the clearance step uses), say how the
robot moved, independently of the wheels.

  * Sample      a scan and the odom pose (x, y, yaw) seen at the same moment
  * chain()     the LiDAR's increments between consecutive samples, each match seeded
                with odom's own increment (so ICP starts near the answer) and refused
                when too few points agree
  * compare()   path length and yaw turned, LiDAR against odom, over the SAME intervals
  * corrections()  what the ratios say about the config: wheel diameter from the straight
                runs, kinematics.angular_scale (lr + fr on mecanum) from the spins

In simulation the LiDAR raycasts from the simulated pose, which is what the odometry
integrates, so the two agree by construction there -- a disagreement in simulation is a
fault in the instrument (or the matcher), and that is worth catching too. On a real
robot this is the calibration check nothing else on the bench can do.

Pure numpy; scan_match supplies the geometry.
"""
import math
from typing import List, NamedTuple, Optional, Sequence

import numpy as np

import scan_match

# A match counts when this many points land within 5 cm, and this share of the
# newer scan does. Below either, the room is too bare (or the scan too sparse) to say.
MIN_INLIERS = 80
MIN_INLIER_FRAC = 0.5
# The share the two may differ by: past WARN it is reported, past FAIL it is judged.
WARN_FRAC = 0.05
FAIL_FRAC = 0.15
# Below these, a manoeuvre did not move enough for a ratio to mean anything; the two are
# then compared as absolutes (a commanded-zero strafe on a 2wd base must stay put).
MIN_DIST_M = 0.30
MIN_TURN_RAD = 0.50
ABS_DIST_M = 0.05
ABS_TURN_RAD = 0.08


class Sample(NamedTuple):
    points: np.ndarray        # (N, 2) in base_link, from scan_match.scan_points
    x: float                  # the odom pose when the scan arrived
    y: float
    yaw: float


class Increment(NamedTuple):
    lidar: Optional[tuple]    # (dyaw, dx, dy) of the later scan's frame in the earlier's, or None
    odom: tuple               # the same from odom, in the earlier pose's frame
    inliers: int


def odom_increment(a: Sample, b: Sample) -> tuple:
    """(dyaw, dx, dy): b's odom pose in a's odom frame."""
    c, s = math.cos(a.yaw), math.sin(a.yaw)
    wx, wy = b.x - a.x, b.y - a.y
    return (scan_match.wrap(b.yaw - a.yaw), c * wx + s * wy, -s * wx + c * wy)


def normals(R: np.ndarray, k: int = 6) -> np.ndarray:
    """Unit normal of the surface at each point of R: the least-spread direction of its k
    nearest neighbours. (N, 2)."""
    d2 = ((R[:, None, :] - R[None, :, :]) ** 2).sum(-1)
    nb = np.argsort(d2, 1)[:, :k]
    out = np.zeros_like(R)
    for i, idx in enumerate(nb):
        Q = R[idx] - R[idx].mean(0)
        w, v = np.linalg.eigh(Q.T @ Q)
        out[i] = v[:, 0]
    return out


def refine(S: np.ndarray, R: np.ndarray, guess: tuple, iters: int = 25,
           gate: float = 0.10, keep: float = 0.8) -> tuple:
    """Point-to-LINE ICP from `guess` (th, tx, ty): each pair's residual is measured along
    the reference surface's normal, so sliding along a wall costs nothing and only the
    surfaces ACROSS the motion pin it. Point-to-point ICP (scan_match.icp) pairs a point with
    the nearest SAMPLE of the other scan, and two scans never sample a wall at the same
    places: on a straight run it settled at 0.096 m of a true 0.110 from every seed below
    it -- biased toward whatever odom said, which is the one thing this check cannot be."""
    th, tx, ty = guess
    Nr = normals(R)
    for _ in range(iters):
        c, s = math.cos(th), math.sin(th)
        P = S @ np.array([[c, s], [-s, c]]) + np.array([tx, ty])
        d2 = ((P[:, None, :] - R[None, :, :]) ** 2).sum(-1)
        j = d2.argmin(1)
        n = Nr[j]
        r = ((P - R[j]) * n).sum(1)
        m = np.sqrt(d2[np.arange(len(P)), j]) < gate
        if m.sum() < 20:
            break
        if keep < 1.0:
            m &= np.abs(r) <= np.quantile(np.abs(r[m]), keep)
        # d(residual)/d(th, tx, ty) at the current pose: n . (dRot/dth p_rotated-about-origin)
        Pr = P[m] - np.array([tx, ty])
        J = np.c_[n[m, 0] * -Pr[:, 1] + n[m, 1] * Pr[:, 0], n[m, 0], n[m, 1]]
        delta, *_ = np.linalg.lstsq(J, -r[m], rcond=None)
        th, tx, ty = th + delta[0], tx + delta[1], ty + delta[2]
        if np.abs(delta).max() < 1e-6:
            break
    return th, tx, ty


def match(a: Sample, b: Sample) -> Increment:
    """The LiDAR's increment from a to b: the robust match (scan_match.locate, seeded with
    odom's increment) to get near, then point-to-line refinement to get it right."""
    guess = odom_increment(a, b)
    if len(a.points) < MIN_INLIERS or len(b.points) < MIN_INLIERS:
        return Increment(None, guess, 0)
    th, tx, ty, _rms, _pairs = scan_match.locate(b.points, a.points, guess=guess,
                                                 spread=(0.0, -10.0, 10.0))
    th, tx, ty = refine(b.points, a.points, (th, tx, ty))
    th = scan_match.wrap(th)
    k = scan_match.inliers(b.points, a.points, th, tx, ty)
    if k < MIN_INLIERS or k < MIN_INLIER_FRAC * len(b.points):
        return Increment(None, guess, k)
    return Increment((th, tx, ty), guess, k)


def chain(samples: Sequence[Sample]) -> List[Increment]:
    return [match(samples[i - 1], samples[i]) for i in range(1, len(samples))]


class Comparison(NamedTuple):
    kind: str                 # "dist" or "turn"
    odom: float               # path length (m) or yaw turned (rad), over the matched steps
    lidar: float
    matched: int              # steps the LiDAR could measure
    steps: int
    verdict: str              # "ok", "warn", "BAD" or "n/a"

    @property
    def ratio(self) -> Optional[float]:
        if self.verdict == "n/a" or abs(self.odom) < 1e-9:
            return None
        return self.lidar / self.odom


def compare(incs: Sequence[Increment], turning: bool) -> Comparison:
    """LiDAR against odom over the steps the LiDAR measured. A step it could not match
    is left out of BOTH sums, so the two always cover the same motion."""
    used = [i for i in incs if i.lidar is not None]
    kind = "turn" if turning else "dist"
    if not incs or len(used) < max(2, int(0.7 * len(incs))):
        return Comparison(kind, 0.0, 0.0, len(used), len(incs), "n/a")
    if turning:
        o = sum(i.odom[0] for i in used)
        lv = sum(i.lidar[0] for i in used)
        small, absol = MIN_TURN_RAD, ABS_TURN_RAD
    else:
        o = sum(math.hypot(i.odom[1], i.odom[2]) for i in used)
        lv = sum(math.hypot(i.lidar[1], i.lidar[2]) for i in used)
        small, absol = MIN_DIST_M, ABS_DIST_M
    if max(abs(o), abs(lv)) < small:
        verdict = "ok" if abs(lv - o) <= absol else "BAD"
    else:
        err = abs(lv - o) / max(abs(o), 1e-9)
        verdict = "ok" if (err <= WARN_FRAC and lv * o > 0) else \
                  "warn" if (err <= FAIL_FRAC and lv * o > 0) else "BAD"
    return Comparison(kind, o, lv, len(used), len(incs), verdict)


def column(c: Comparison) -> str:
    """The suite line's LiDAR column."""
    if c.verdict == "n/a":
        return "   lidar n/a (%d/%d steps matched)" % (c.matched, c.steps)
    unit = "rad" if c.kind == "turn" else "m"
    r = c.ratio
    return "   lidar %s %+.2f %s vs odom %+.2f%s %s" % (
        c.kind, c.lidar, unit, c.odom,
        (" (x%.3f)" % r) if r is not None and abs(c.odom) >= (MIN_TURN_RAD if c.kind == "turn"
                                                             else MIN_DIST_M) else "",
        c.verdict)


def _median(v: List[float]) -> Optional[float]:
    v = sorted(v)
    if not v:
        return None
    m = len(v) // 2
    return v[m] if len(v) % 2 else 0.5 * (v[m - 1] + v[m])


def corrections(straight: List[Comparison], spins: List[Comparison], kinematics: dict) -> dict:
    """What the measured ratios say the kinematics are. The straight runs give the linear
    scale (true / odom distance = true / configured wheel diameter). The spins give the
    angular one; odom's yaw rate is (wheel speed difference) / (2 x rotation radius), so
    once the wheels' own scale is taken out, the radius is off by linear / angular:

      * 2wd and skid steer: kinematics.angular_scale (Kinematics::rotationRadius, the
        firmware's scrub factor -- "the angle the base actually turned / the angle odom
        claims") -- the measured track stays what the tape says
      * mecanum: the radius is (lr + fr) / 2 with no scale, so the sum lr + fr

    Empty ratios when nothing was measured."""
    lin = _median([c.ratio for c in straight if c.ratio is not None and abs(c.odom) >= MIN_DIST_M])
    ang = _median([c.ratio for c in spins if c.ratio is not None and abs(c.odom) >= MIN_TURN_RAD])
    out = {"linear_ratio": lin, "angular_ratio": ang}
    k = kinematics or {}
    if lin is not None and k.get("wheel_diameter"):
        out["wheel_diameter"] = float(k["wheel_diameter"]) * lin
    if ang is not None and ang > 0:
        scale = (lin if lin is not None else 1.0) / ang
        base = str(k.get("base_type", "2wd")).lower()
        if base == "mecanum":
            lr, fr = float(k.get("lr_wheels_distance") or 0.0), float(k.get("fr_wheels_distance") or 0.0)
            if lr + fr > 0:
                out["lr_plus_fr"] = (lr + fr) * scale
        else:
            out["angular_scale"] = float(k.get("angular_scale", 1.0)) * scale
    return out


def summary(corr: dict, kinematics: dict) -> List[str]:
    """Lines for the verdict: the ratios, and a suggested value only where it is off."""
    lines = []
    lin, ang = corr.get("linear_ratio"), corr.get("angular_ratio")
    if lin is None and ang is None:
        return ["LIDAR: no manoeuvre could be measured -- the room is too bare for the matcher, "
                "or no scan arrived; odometry is NOT verified against the room."]
    k = kinematics or {}
    if lin is not None:
        lines.append("LIDAR: straight runs went x%.3f of what odom reported%s" % (
            lin, "" if abs(lin - 1) <= WARN_FRAC else
            " -- wheel_diameter %.4f m would match (now %.4f)" % (corr["wheel_diameter"],
                                                                  float(k.get("wheel_diameter", 0)))
            if "wheel_diameter" in corr else ""))
    if ang is not None:
        tail = ""
        if abs(ang - 1) > WARN_FRAC or (lin is not None and abs(lin - 1) > WARN_FRAC):
            if "angular_scale" in corr:
                tail = " -- kinematics.angular_scale %.3f would match (now %.3f)" % (
                    corr["angular_scale"], float(k.get("angular_scale", 1.0)))
            elif "lr_plus_fr" in corr:
                tail = " -- lr_wheels_distance + fr_wheels_distance %.4f m would match (now %.4f)" % (
                    corr["lr_plus_fr"],
                    float(k.get("lr_wheels_distance") or 0) + float(k.get("fr_wheels_distance") or 0))
        lines.append("LIDAR: spins turned x%.3f of what odom reported%s" % (ang, tail))
    return lines
