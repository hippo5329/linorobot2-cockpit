"""The simulated wall must push the robot back the way it came.

`pushOffSegment` moved the robot to FAKE_ROBOT_RADIUS from the wall along the
direction from the wall to WHERE IT IS NOW. A centre that has just crossed the
line -- 8 mm of travel per 50 Hz cycle at 0.4 m/s -- is then on the far side, so
the "push off" put it down BEHIND the wall. Measured on the bench: a GenDrv
rounding the wall's end at y = -1.40 was placed at x = 2.83, and Nav2 then drove
to a goal it should not have been able to reach.

The side now comes from the previous pose. These cases are a transcription of
the C++ (firmware/common/lib/lidar/fake_ld19.h) rather than a run of it, so the
first test checks that the source still has the shape they describe.
"""

import math
import os

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
HEADER = os.path.join(REPO_ROOT, "firmware", "common", "lib", "lidar", "fake_ld19.h")
R, X1, Y1, X2, Y2 = 0.30, 2.0, -1.5, 2.0, 1.5


def _push(x, y, px, py):
    sx, sy = X2 - X1, Y2 - Y1
    len2 = sx * sx + sy * sy
    t = ((x - X1) * sx + (y - Y1) * sy) / len2
    interior = 0.0 < t < 1.0
    t = min(1.0, max(0.0, t))
    cx, cy = X1 + t * sx, Y1 + t * sy
    nx, ny = x - cx, y - cy
    d = math.hypot(nx, ny)
    if d >= R:
        return x, y
    if interior:
        ux, uy = -sy, sx
        prev_side = (px - X1) * ux + (py - Y1) * uy
        if abs(prev_side) > 1e-6:
            s = 1.0 if prev_side > 0 else -1.0
            ul = math.sqrt(len2)
            return cx + ux / ul * s * R, cy + uy / ul * s * R
    if d < 1e-6:
        nx, ny, d = -sy, sx, math.sqrt(len2)
    return cx + nx / d * R, cy + ny / d * R


def test_the_clamp_takes_the_side_from_the_previous_pose():
    src = open(HEADER).read()
    sig = src[src.index("static void pushOffSegment"):][:400]
    assert "float px, float py)" in sig, "the previous pose is an argument"
    body = src[src.index("static void pushOffSegment"):src.index("rangeAheadM")]
    assert "prev_side" in body and "(px - x1) * ux + (py - y1) * uy" in body
    call = src[src.index("if (wall_on_)"):src.index("if (wall_on_)") + 700]
    assert "pose_x_, pose_y_" in call, "clampToRoom passes where the robot was"


def test_a_centre_just_past_the_line_is_put_back_on_its_own_side():
    x, y = _push(2.01, -1.40, 1.99, -1.42)
    assert math.isclose(x, X1 - R, abs_tol=1e-6), x
    assert math.isclose(y, -1.40, abs_tol=1e-6)


def test_pressed_on_the_face_it_stops_one_radius_short():
    x, y = _push(1.95, 0.0, 1.90, 0.0)
    assert math.isclose(x, 1.70, abs_tol=1e-6) and math.isclose(y, 0.0, abs_tol=1e-6)


def test_rounding_the_end_it_is_pushed_radially_off_the_corner():
    x, y = _push(2.0, -1.55, 1.95, -1.60)
    assert math.isclose(x, 2.0, abs_tol=1e-6)
    assert math.isclose(y, -1.80, abs_tol=1e-6), "a disc of radius R rounds a corner wide"


def test_a_robot_clear_of_the_wall_is_left_alone():
    assert _push(1.5, 0.0, 1.45, 0.0) == (1.5, 0.0)
