"""Odometry against the room (odom_check.py), with no ROS: a synthetic room, a known
true path, and odometry that is wrong by a known factor -- the matcher must find the
factor, and the corrections must name the config values that would remove it."""
import math
import os
import sys

import numpy as np
import pytest

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO, "scripts"))

import odom_check  # noqa: E402


# A room with something in it, as segments: a bare rectangle lets a straight run slide
# along itself. Scans are RAY-CAST from the pose, one beam every 0.8 degrees as an LD19's
# 450, so two scans sample a wall at different places as a real LiDAR's do (a fixed grid
# of points would hand ICP correspondences no real scan has).
SEGS = [(-2.5, -2.0, 3.0, -2.0), (3.0, -2.0, 3.0, 2.0), (3.0, 2.0, -2.5, 2.0), (-2.5, 2.0, -2.5, -2.0),
        (1.0, 0.8, 1.6, 0.8), (-1.2, -1.4, -1.2, -0.6), (0.4, -1.0, 0.9, -1.5), (-0.6, 1.1, -0.2, 1.6)]
BEAMS = np.radians(np.arange(0.0, 360.0, 0.8))


def seen_from(x, y, th, rmax=8.0):
    """What scan_points would give at a pose: each beam's nearest wall hit, in base_link."""
    pts = []
    for b in BEAMS:
        dx, dy = math.cos(th + b), math.sin(th + b)
        best = rmax
        for x1, y1, x2, y2 in SEGS:
            ex, ey = x2 - x1, y2 - y1
            den = dx * ey - dy * ex
            if abs(den) < 1e-12:
                continue
            t = ((x1 - x) * ey - (y1 - y) * ex) / den
            u = ((x1 - x) * dy - (y1 - y) * dx) / den
            if 0 <= u <= 1 and 0.2 < t < best:
                best = t
        if best < rmax:
            pts.append((best * math.cos(b), best * math.sin(b)))
    return np.asarray(pts)[::2]          # thinned as the suite thins (<= 300 points)


def run(path, lin_scale=1.0, ang_scale=1.0):
    """Samples along a true path; odom reports distance / lin_scale and yaw / ang_scale
    (a wheel diameter lin_scale too small, a turn ang_scale more than odom claims)."""
    out, ox, oy, oth = [], path[0][0], path[0][1], path[0][2]
    for i, (x, y, th) in enumerate(path):
        if i:
            px, py, pth = path[i - 1]
            c, s = math.cos(pth), math.sin(pth)
            dx, dy = c * (x - px) + s * (y - py), -s * (x - px) + c * (y - py)
            dth = math.atan2(math.sin(th - pth), math.cos(th - pth))
            dx, dy, dth = dx / lin_scale, dy / lin_scale, dth / ang_scale
            c, s = math.cos(oth), math.sin(oth)
            ox, oy, oth = ox + c * dx - s * dy, oy + s * dx + c * dy, oth + dth
        out.append(odom_check.Sample(seen_from(x, y, th), ox, oy, oth))
    return out


def straight(n=12, step=0.11):
    return [(-0.5 + i * step, 0.1, 0.0) for i in range(n)]


def spin(n=12, step=0.6):
    return [(0.2, -0.1, i * step) for i in range(n)]


def test_a_true_odometry_agrees_with_the_room():
    c = odom_check.compare(odom_check.chain(run(straight())), turning=False)
    assert c.verdict == "ok" and c.ratio == pytest.approx(1.0, abs=0.02)
    c = odom_check.compare(odom_check.chain(run(spin())), turning=True)
    assert c.verdict == "ok" and c.ratio == pytest.approx(1.0, abs=0.02)


def test_a_wrong_wheel_diameter_is_found_on_the_straight():
    # the robot went 10 % further than odom says: the configured wheel is 10 % small
    c = odom_check.compare(odom_check.chain(run(straight(), lin_scale=1.10)), turning=False)
    assert c.ratio == pytest.approx(1.10, abs=0.02)
    assert c.verdict == "warn"
    c = odom_check.compare(odom_check.chain(run(straight(), lin_scale=1.25)), turning=False)
    assert c.verdict == "BAD"


def test_a_slipping_track_is_found_on_the_spin():
    # a tracked base turns 0.81 of what its tracks say (the test robot, 2026-10-08)
    c = odom_check.compare(odom_check.chain(run(spin(), ang_scale=0.81)), turning=True)
    assert c.ratio == pytest.approx(0.81, abs=0.02)
    assert c.verdict == "BAD"


def test_corrections_name_the_config_values():
    lin = odom_check.compare(odom_check.chain(run(straight(), lin_scale=1.10)), turning=False)
    ang = odom_check.compare(odom_check.chain(run(spin(), ang_scale=1.10 * 0.81)), turning=True)
    kine = {"base_type": "2wd", "wheel_diameter": 0.060, "lr_wheels_distance": 0.20, "angular_scale": 1.0}
    corr = odom_check.corrections([lin], [ang], kine)
    assert corr["wheel_diameter"] == pytest.approx(0.066, abs=0.0015)
    # with the wheel fixed, the turn is still 0.81 short: the scrub factor, not the track
    assert corr["angular_scale"] == pytest.approx(1 / 0.81, abs=0.04)
    assert "lr_wheels_distance" not in corr, "the measured track stays what the tape says"
    text = "\n".join(odom_check.summary(corr, kine))
    assert "wheel_diameter 0.066" in text and "kinematics.angular_scale 1.2" in text

    mec = odom_check.corrections([lin], [ang], {"base_type": "mecanum", "wheel_diameter": 0.06,
                                                "lr_wheels_distance": 0.2, "fr_wheels_distance": 0.16})
    assert mec["lr_plus_fr"] == pytest.approx(0.36 / 0.81, abs=0.02) and "angular_scale" not in mec


def test_a_still_base_must_stay_still():
    path = [(0.2, 0.3, 0.1)] * 6
    c = odom_check.compare(odom_check.chain(run(path)), turning=False)
    assert c.verdict == "ok" and abs(c.lidar) < 0.02


def test_no_room_no_verdict():
    empty = [odom_check.Sample(np.zeros((0, 2)), 0.1 * i, 0.0, 0.0) for i in range(8)]
    c = odom_check.compare(odom_check.chain(empty), turning=False)
    assert c.verdict == "n/a"
    assert "lidar n/a" in odom_check.column(c)
    lines = odom_check.summary(odom_check.corrections([c], [], {}), {})
    assert "NOT verified" in lines[0]


def test_the_suite_and_the_pipeline_use_it():
    src = open(os.path.join(REPO, "scripts", "drive_suite.py"), encoding="utf-8").read()
    assert "odom_check.compare(odom_check.chain(samples), turning)" in src
    assert "ok = ok_vx and ok_vy and ok_wz and ok_d and ok_gap and lidar_ok" in src
    pipe = open(os.path.join(REPO, "scripts", "one_click_pipeline.py"), encoding="utf-8").read()
    assert 'drive_lidar = scan_from == "lidar" and has_lidar' in pipe
    assert '+ (" --lidar" if drive_lidar else "")' in pipe


def test_range_noise_does_not_move_the_verdict():
    # sigma 1 cm per return, about an LD19's at a few metres
    rng = np.random.default_rng(7)

    def noisy(samples):
        return [s._replace(points=s.points + rng.normal(0, 0.01, s.points.shape)) for s in samples]
    c = odom_check.compare(odom_check.chain(noisy(run(straight()))), turning=False)
    assert c.verdict == "ok" and c.ratio == pytest.approx(1.0, abs=0.03)
    c = odom_check.compare(odom_check.chain(noisy(run(spin(), ang_scale=0.81))), turning=True)
    assert c.verdict == "BAD" and c.ratio == pytest.approx(0.81, abs=0.03)
