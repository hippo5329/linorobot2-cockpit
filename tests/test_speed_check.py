"""speed_check.py: the arithmetic that turns the room's account and the wheels' into a
config change, and the LiDAR tracker it stands on -- no ROS, no robot."""
import math
import os
import sys

import numpy as np
import pytest

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO, "scripts"))

import scan_match  # noqa: E402
import speed_check as sc  # noqa: E402


# ---- the steps ----

@pytest.mark.parametrize("strafe", [False, True])
def test_every_step_is_followed_by_its_mirror(strafe):
    s = sc.steps(strafe)
    assert len(s) % 2 == 0
    for (k1, v1), (k2, v2) in zip(s[0::2], s[1::2]):
        assert k1 == k2 and v2 == -v1          # the robot ends where it began


def test_strafe_only_with_the_flag():
    assert not any(k == "lat" for k, _ in sc.steps(False))
    assert sum(k == "lat" for k, _ in sc.steps(True)) == 4


def test_no_step_goes_further_than_half_a_metre():
    assert max(abs(v) for k, v in sc.steps(True) if k != "ang") * sc.RUN_S <= 0.4 + 1e-9


@pytest.mark.parametrize("bt,want", [("mecanum", True), ("Mecanum", True), ("2wd", False),
                                     ("4wd", False), ("skid_steer", False)])
def test_strafe_is_on_for_a_mecanum_base(bt, want):
    assert sc.wants_strafe({"kinematics": {"base_type": bt}}) is want


def test_no_params_means_no_strafe():
    assert sc.wants_strafe({}) is False


# ---- measure(): one step's speeds ----

def _poses(t0, t1, vx=0.0, vy=0.0, wz=0.0, yaw0=0.0, hz=10.0):
    """The room's account: base_link driving at (vx, vy) in its own frame and turning wz."""
    out, x, y, yaw = [], 0.0, 0.0, yaw0
    t = t0
    while t <= t1 + 1e-9:
        out.append((t, x, y, yaw, 0.01, 300))
        x += (vx * math.cos(yaw) - vy * math.sin(yaw)) / hz
        y += (vx * math.sin(yaw) + vy * math.cos(yaw)) / hz
        yaw += wz / hz
        t += 1.0 / hz
    return out


def _odom(t0, t1, vx=0.0, wz=0.0, vy=0.0, hz=50.0):
    return [(t0 + i / hz, vx, wz, vy) for i in range(int((t1 - t0) * hz) + 1)]


def test_linear_along_the_heading():
    m = sc.measure("lin", _poses(0, 2, vx=0.18, yaw0=0.7), _odom(0, 2, vx=0.20), [], 0.0, 2.0)
    assert m["lidar"] == pytest.approx(0.18, rel=1e-6)
    assert m["odom"] == pytest.approx(0.20)


def test_backwards_reads_negative():
    m = sc.measure("lin", _poses(0, 2, vx=-0.1), _odom(0, 2, vx=-0.1), [], 0.0, 2.0)
    assert m["lidar"] == pytest.approx(-0.1, rel=1e-6)


def test_strafe_across_the_heading_left_is_positive():
    m = sc.measure("lat", _poses(0, 2, vy=0.15, yaw0=-1.2), _odom(0, 2, vy=0.2), [], 0.0, 2.0)
    assert m["lidar"] == pytest.approx(0.15, rel=1e-6)
    assert m["odom"] == pytest.approx(0.2)


def test_a_spin_from_the_unwrapped_yaw_and_the_gyro():
    gyro = [(i / 100, 0.95) for i in range(201)]
    m = sc.measure("ang", _poses(0, 2, wz=0.8, yaw0=3.0), _odom(0, 2, wz=1.0), gyro, 0.0, 2.0)
    assert m["lidar"] == pytest.approx(0.8, rel=1e-6)
    assert m["gyro"] == pytest.approx(0.95)


def test_the_first_part_of_a_step_is_not_counted():
    # The robot is still speeding up for the first SETTLE_S: only the steady part counts.
    poses = _poses(0, sc.SETTLE_S, vx=0.02) + [
        (t, 0.02 * sc.SETTLE_S + 0.2 * (t - sc.SETTLE_S), 0.0, 0.0) for t in np.arange(sc.SETTLE_S, 2.0, 0.1)]
    m = sc.measure("lin", poses, _odom(0, 2, vx=0.2), [], 0.0, 2.0)
    assert m["lidar"] == pytest.approx(0.2, rel=1e-3)


def test_too_few_samples_is_no_measurement():
    assert sc.measure("lin", _poses(0, 0.2, vx=0.1), _odom(0, 2, vx=0.1), [], 0.0, 2.0) is None


# ---- ratios() and advice() ----

def _rows(lin=None, ang=None, lat=None, gyro=1.0):
    rows = []
    for kind, r in (("lin", lin), ("ang", ang), ("lat", lat)):
        if r is None:
            continue
        for v in (0.1, -0.1, 0.2, -0.2):
            rows.append({"kind": kind, "cmd": v, "odom": v, "lidar": v * r,
                         "gyro": v * r * gyro if kind == "ang" else float("nan")})
    return rows


def test_ratios_are_medians_per_motion():
    r = sc.ratios(_rows(lin=0.95, ang=0.81, lat=0.9, gyro=1.0))
    assert r["lin"] == pytest.approx(0.95)
    assert r["ang"] == pytest.approx(0.81)
    assert r["lat"] == pytest.approx(0.9)
    assert r["gyro"] == pytest.approx(1.0)


def test_within_three_percent_changes_nothing():
    lines = sc.advice({"lin": 1.02, "ang": 0.99}, {"kinematics": {"base_type": "2wd", "wheel_diameter": 0.1}})
    assert all("nothing to change" in s for s in lines)


def test_linear_sets_the_wheel_diameter():
    lines = sc.advice({"lin": 0.9}, {"kinematics": {"wheel_diameter": 0.1}})
    assert "kinematics.wheel_diameter 0.1000 -> 0.0900" in lines[0]
    assert "over-read" in lines[0]


def test_a_skid_steer_turn_sets_angular_scale_after_the_wheel_diameter():
    # The tracked test robot turned 0.81 of what its tracks said: angular_scale ~1.24.
    p = {"kinematics": {"base_type": "2wd", "wheel_diameter": 0.0542, "angular_scale": 1.0}}
    lines = sc.advice({"lin": 1.0, "ang": 0.81}, p)
    ang = [s for s in lines if s.startswith("ANGULAR")][0]
    assert "kinematics.angular_scale 1.000 -> 1.235" in ang
    # With the wheels 10 % over-reading, the same body turn is less of a turn error.
    ang2 = [s for s in sc.advice({"lin": 0.9, "ang": 0.81}, p) if s.startswith("ANGULAR")][0]
    assert "-> 1.111" in ang2


def test_an_existing_angular_scale_is_corrected_not_replaced():
    p = {"kinematics": {"base_type": "skid_steer", "angular_scale": 1.24}}
    ang = [s for s in sc.advice({"ang": 0.96}, p) if s.startswith("ANGULAR")][0]
    assert "1.240 -> 1.292" in ang


def test_a_mecanum_turn_names_no_key_that_does_not_exist():
    p = {"kinematics": {"base_type": "mecanum", "angular_scale": 1.0}}
    ang = [s for s in sc.advice({"lin": 1.0, "ang": 0.8}, p) if s.startswith("ANGULAR")][0]
    assert "no angular_scale" in ang and "lr_wheels_distance" in ang and "->" not in ang


def test_strafe_has_no_config_key():
    lat = [s for s in sc.advice({"lin": 1.0, "lat": 0.85}, {"kinematics": {"base_type": "mecanum"}})
           if s.startswith("STRAFE")][0]
    assert "no config key" in lat and "->" not in lat


def test_the_gyro_is_named_only_when_it_is_off():
    assert not any(s.startswith("GYRO") for s in sc.advice({"gyro": 1.01}, {}))
    assert any(s.startswith("GYRO") for s in sc.advice({"gyro": 1.08}, {}))


def test_the_keys_it_names_are_the_keys_the_firmware_reads():
    """A verdict that names a config key nobody reads is worse than none."""
    src = open(os.path.join(REPO, "scripts", "mcu_env.py")).read()
    for key in ("wheel_diameter", "angular_scale"):
        assert f'"{key}"' in src
    k = open(os.path.join(REPO, "firmware", "common", "lib", "kinematics", "kinematics.cpp")).read()
    assert "if (robot_base == MECANUM)\n        return (wheels_y_distance + wheels_x_distance) / 2.0;" in k


# ---- the tracker ----

ROOM = (-2.0, 3.0, -1.5, 2.5)          # xmin, xmax, ymin, ymax
BOX = (0.8, 1.2, 0.9, 1.4)


def _ray(px, py, a, rect, inside):
    xmin, xmax, ymin, ymax = rect
    c, s = math.cos(a), math.sin(a)
    best = math.inf
    for x0, horizontal in ((xmin, False), (xmax, False), (ymin, True), (ymax, True)):
        d = (x0 - (py if horizontal else px)) / (s if horizontal else c) if (s if horizontal else c) else -1
        if d <= 1e-9:
            continue
        hx, hy = px + c * d, py + s * d
        if horizontal and xmin - 1e-9 <= hx <= xmax + 1e-9 or not horizontal and ymin - 1e-9 <= hy <= ymax + 1e-9:
            best = min(best, d)
    return best


def _scan(x, y, yaw, stamp, n=360):
    ranges = []
    for i in range(n):
        a = yaw - math.pi + i * 2 * math.pi / n
        ranges.append(min(_ray(x, y, a, ROOM, True), _ray(x, y, a, BOX, False)))
    return {"stamp": stamp, "angle_min": -math.pi, "angle_increment": 2 * math.pi / n,
            "range_min": 0.05, "range_max": 12.0, "ranges": ranges}


def test_the_tracker_follows_a_straight_drive():
    scans = [_scan(0.03 * i, 0.0, 0.0, i * 0.1) for i in range(11)]
    poses, end = scan_match.track(scans)
    assert poses[-1][1] == pytest.approx(0.30, abs=0.01)
    assert abs(poses[-1][2]) < 0.01
    assert end["x"] == pytest.approx(0.30, abs=0.01)


def test_the_tracker_follows_a_turn_past_pi():
    scans = [_scan(0.0, 0.0, math.radians(15 * i), i * 0.1) for i in range(15)]   # 210 deg
    poses, end = scan_match.track(scans)
    assert math.degrees(poses[-1][3]) == pytest.approx(210, abs=2)
    assert math.degrees(end["yaw"]) == pytest.approx(210, abs=2)


def test_no_scans_no_track():
    assert scan_match.track([]) == ([], None)


def test_importing_needs_no_ros():
    src = open(os.path.join(REPO, "scripts", "speed_check.py")).read()
    head = src.split("# ---- ROS ----")[0]
    assert "import rclpy" not in head
