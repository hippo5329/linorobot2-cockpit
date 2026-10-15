"""Before test_acc the room check goes to the open floor it sees, in one planned move.

The 5 cm steps climb the nearest distance greedily and stalled in a corner: a tracked robot
1 m from boxes gave up "no room of 1.00 m" with 1.3 m of open floor 1-2 m away (2026-10-15).
room_spot() finds the spot with the most room whose whole circle the look saw free, reached by
a straight path that keeps the robot's circle clear.
"""
import math
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "scripts"))
import clearance  # noqa: E402


def wall(x0, y0, x1, y1, step=0.03):
    n = int(math.hypot(x1 - x0, y1 - y0) / step) + 1
    return np.c_[np.linspace(x0, x1, n), np.linspace(y0, y1, n)]


def room(corner_gap=0.8):
    """a 5 x 4 m room; the robot 0.8 m from the left wall and the bottom wall (a corner)"""
    return np.r_[wall(-corner_gap, -corner_gap, 4.2, -corner_gap), wall(-corner_gap, 3.2, 4.2, 3.2),
                 wall(-corner_gap, -corner_gap, -corner_gap, 3.2), wall(4.2, -corner_gap, 4.2, 3.2)]


def test_a_corner_robot_is_sent_to_the_open_floor():
    P = room()
    assert clearance.nearest(P) < 1.0                       # the corner: no room here
    spot = clearance.room_spot(P, 1.0, 0.16, 2.5)
    assert spot is not None
    h, d, rm = spot
    assert rm >= 1.05 and d <= 2.5
    c = np.array([d * math.cos(h), d * math.sin(h)])
    assert clearance.nearest(P - c) >= 1.05
    assert 0 < h < math.pi / 2                                # out of the corner, into the room


def test_no_spot_when_the_room_is_too_small():
    P = room()
    assert clearance.room_spot(P, 2.5, 0.16, 2.5) is None    # 5 x 4 m holds no 2.5 m circle


def test_the_path_must_be_clear():
    P = np.r_[room(), wall(0.4, -0.8, 0.4, 3.2)]             # a wall right beside the robot, full height
    spot = clearance.room_spot(P, 1.0, 0.16, 2.5)
    if spot is not None:                                      # it may only go where the wall is not between
        h, d, _ = spot
        assert d * math.cos(h) < 0.4 - 0.16


def test_an_unseen_circle_is_not_a_clear_one():
    P = room()
    blind = clearance.parse_blind(["0:120"])                  # everything ahead-left unseen
    spot = clearance.room_spot(P, 1.0, 0.16, 2.5, blind)
    if spot is not None:
        assert not (0 <= math.degrees(spot[0]) % 360 <= 120)


def test_inch_tries_the_open_floor_before_the_steps():
    src = open(os.path.join(ROOT, "scripts", "clearance.py")).read()
    body = src[src.index("def cmd_inch("):]
    assert body.index("room_spot(") < body.index("best_step(")
    pipe = open(os.path.join(ROOT, "scripts", "one_click_pipeline.py")).read()
    assert "max_travel=TEST_ACC_TRAVEL_M" in pipe
