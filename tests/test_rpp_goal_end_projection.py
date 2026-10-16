"""Near the goal, RPP must not chase the goal point itself.

2026-10-16, the tracked robot, jazzy: it stopped 0.18 m from a goal (xy_goal_tolerance
0.15) and turned in place at -0.46 rad/s for over three minutes until it was cancelled.
PoseProgressChecker (cbe9e34) counts a turn as progress, so the controller never gave
up and the BT never recovered.

The mechanism, in Nav2 1.3.13's regulated_pure_pursuit_controller.cpp: once the path's
end is nearer than lookahead_dist (0.6), getLookAheadPoint() returns the LAST pose -- the
carrot is the goal itself, 0.18 m away. shouldRotateToPath() turns in place until the
goal's bearing is under rotate_to_heading_min_angle (0.785) and then hands over to pure
pursuit, whose arc through a point at distance L and bearing phi has radius
L / (2 sin phi): 0.13 m here, tighter than the robot's own track, at the approach crawl
(min_approach_linear_velocity 0.05 m/s). A tracked base cannot drive that arc; it pivots,
its pivot drifts the base, the goal stays outside the tolerance, and the next cycle turns
again. Every turn is progress to the checker.

interpolate_curvature_after_goal (with use_fixed_curvature_lookahead) projects the
curvature point past the goal along the path's last segment, curvature_lookahead_dist
from the robot, and uses it for the in-place turn too: the arc's radius can never be
under half that distance, so the robot drives through the goal. Nav2 documents it as the
remedy for oscillation at the end of the path.
"""
import glob
import math
import os
import re

import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _refs():
    found = sorted(glob.glob(os.path.join(ROOT, "config", "reference", "*_config.yaml")))
    assert found, "no reference configs: this test checks nothing"
    for f in found:
        with open(f, encoding="utf-8") as fh:
            yield os.path.basename(f), yaml.safe_load(fh)


def _fp(d):
    return d["nav2"]["controller_server"]["ros__parameters"]["FollowPath"]


# ---- Nav2 1.3.13 RPP, the two functions that decide the carrot, ported line for line

def _circle_segment_intersection(p1, p2, r):
    x1, y1 = p1
    x2, y2 = p2
    dx, dy = x2 - x1, y2 - y1
    dr2 = dx * dx + dy * dy
    D = x1 * y2 - x2 * y1
    dd = (x2 * x2 + y2 * y2) - (x1 * x1 + y1 * y1)
    sq = math.sqrt(r * r * dr2 - D * D)
    s = math.copysign(1.0, dd)
    return ((D * dy + s * dx * sq) / dr2, (-D * dx + s * dy * sq) / dr2)


def _look_ahead_point(L, path, interpolate_after_goal=False):
    for i, p in enumerate(path):
        if math.hypot(*p) >= L:
            if i == 0:
                return p
            return _circle_segment_intersection(path[i - 1], p, L)
    if interpolate_after_goal:
        last, prev = path[-1], path[-2]
        a = math.atan2(last[1] - prev[1], last[0] - prev[0])
        proj = (last[0] + math.cos(a) * L, last[1] + math.sin(a) * L)
        return _circle_segment_intersection(last, proj, L)
    return path[-1]


def _arc_radius(pt):
    x, y = pt
    k = 2.0 * y / (x * x + y * y)
    return math.inf if k == 0 else 1.0 / abs(k)


def _straight_path(dist, bearing, step=0.05):
    n = max(2, int(math.ceil(dist / step)) + 1)
    return [(dist * i / (n - 1) * math.cos(bearing), dist * i / (n - 1) * math.sin(bearing))
            for i in range(n)]


def _arc_near_goal(fp, dist, bearing):
    """The radius RPP's pure pursuit asks for, robot at the origin facing +x, the goal
    `dist` away at `bearing` -- what the controller computes once the in-place turn has
    brought the bearing under rotate_to_heading_min_angle."""
    path = _straight_path(dist, bearing)
    if fp.get("use_fixed_curvature_lookahead"):
        pt = _look_ahead_point(fp.get("curvature_lookahead_dist", 0.6), path,
                               fp.get("interpolate_curvature_after_goal", False))
    else:
        pt = _look_ahead_point(fp.get("lookahead_dist", 0.6), path)
    return _arc_radius(pt)


def test_the_incident_without_the_projection_asks_for_an_arc_tighter_than_the_robot():
    """The arithmetic of the spin, so the test below means something: the goal 0.18 m
    off at just under the 0.785 rad hand-over, the template's lookahead 0.6."""
    r = _arc_near_goal({"lookahead_dist": 0.6}, 0.18, 0.78)
    assert 0.12 < r < 0.135, r       # under the tracked robot's 0.216 m track, at 0.05 m/s


def test_every_reference_projects_the_curvature_point_past_the_goal():
    for name, d in _refs():
        fp = _fp(d)
        assert fp.get("use_fixed_curvature_lookahead") is True, name
        assert fp.get("interpolate_curvature_after_goal") is True, (
            f"{name}: without interpolate_curvature_after_goal RPP's carrot is the goal itself "
            f"once it is nearer than lookahead_dist, and a near goal asks for an arc the base "
            f"cannot drive -- the endless turn of 2026-10-16")
        # Equal, and a fixed lookahead: mid-path the curvature point is then the carrot
        # itself, so the setting changes nothing but the end of the path.
        assert fp.get("curvature_lookahead_dist") == fp.get("lookahead_dist"), name
        assert not fp.get("use_velocity_scaled_lookahead_dist", False), name
        assert fp.get("use_rotate_to_heading", True) is not False, name


def test_no_goal_near_the_robot_asks_for_an_arc_under_half_the_lookahead():
    for name, d in _refs():
        fp = _fp(d)
        floor = fp["curvature_lookahead_dist"] / 2.0
        tol = d["nav2"]["controller_server"]["ros__parameters"]["general_goal_checker"][
            "xy_goal_tolerance"]
        min_angle = fp.get("rotate_to_heading_min_angle", 0.785)
        for dist in (0.10, 0.15, 0.18, tol, 0.3, 0.45, 0.59):
            for bearing in (0.05, 0.3, 0.6, min_angle - 0.005, -(min_angle - 0.005)):
                r = _arc_near_goal(fp, dist, bearing)
                assert r >= floor - 1e-9, (
                    f"{name}: a goal {dist:.2f} m off at {bearing:+.2f} rad asks for a "
                    f"{r:.3f} m arc (floor {floor:.2f} m)")


def test_the_comment_says_why_beside_the_values():
    for f in sorted(glob.glob(os.path.join(ROOT, "config", "reference", "*_config.yaml"))):
        text = open(f, encoding="utf-8").read()
        at = text.index("interpolate_curvature_after_goal: true")
        assert "carrot IS the goal" in text[max(0, at - 1200):at], os.path.basename(f)


def test_lyrical_carries_the_projection_and_rejects_a_one_pose_plan():
    """Lyrical nests RPP under FollowPath.primary_controller; the nesting must carry the
    three keys (tests/test_lyrical_nesting_keeps_the_config.py checks they are popped,
    not hardcoded). Jazzy's RPP rejects a one-pose plan whenever the projection is on
    (transformGlobalPlan's reject_unit_path is that flag); Lyrical moved the check to the
    controller server's path handler, default off, and its getLookAheadPoint would then
    step before the only pose. The launcher turns it on there."""
    src = open(os.path.join(ROOT, "launchers", "nav2.launch.py"), encoding="utf-8").read()
    lyr = src[src.index('if distro == "lyrical":'):]
    lyr = lyr[:lyr.index("# 3. bt_navigator")]
    for key in ("use_fixed_curvature_lookahead", "curvature_lookahead_dist",
                "interpolate_curvature_after_goal"):
        assert re.search(rf'"{key}":\s*follow_path\.pop\(', lyr), key
    assert 'ph.setdefault("reject_unit_path", True)' in lyr
    assert 'ctrl_params.setdefault("PathHandler", {})' in lyr
