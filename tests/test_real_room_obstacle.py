"""A real robot's legs are judged against the room's own /map, not the sim's wall.

With --no-wall the x=2.0 wall of the simulated room means nothing, yet every real-robot
leg still warned that its pose "crossed x=2.0 inside the obstacle", and nothing
checked that the goal really sat behind one (2026-10-10). A real-room leg is behind
an obstacle when the straight line from its start to its goal crosses occupied
cells of /map, and its driven track should cross none.
"""

import os

from test_nav2_goal_motion import _load_module

tng = _load_module()
REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC = open(os.path.join(REPO_ROOT, "scripts", "test_nav2_goal.py")).read()


def _room(boxes, res=0.05, size=6.0):
    """A size x size m grid centred on the origin, `boxes` = (x0, y0, x1, y1) occupied."""
    n = int(size / res)
    ox = oy = -size / 2
    data = [0] * (n * n)
    for x0, y0, x1, y1 in boxes:
        for j in range(n):
            for i in range(n):
                x, y = ox + (i + 0.5) * res, oy + (j + 0.5) * res
                if x0 <= x <= x1 and y0 <= y <= y1:
                    data[j * n + i] = 100
    return (res, ox, oy, n, n, data)


BOXES = _room([(1.0, -0.3, 1.4, 0.5)])          # a box between home and (2.3, 0.1)


def test_a_box_on_the_straight_line_makes_the_leg_behind_an_obstacle():
    behind, words = tng.room_route_note(BOXES, (0.0, 0.0), (2.3, 0.1), [(0.0, 0.0), (1.2, 0.9), (2.3, 0.1)])
    assert behind is True
    assert "0.4" in words and "driven track stayed clear" in words, words
    assert 0.35 <= tng.blocked_length(BOXES, (0.0, 0.0), (2.3, 0.1)) <= 0.45


def test_a_clear_line_says_the_leg_tested_no_detour():
    behind, words = tng.room_route_note(BOXES, (0.0, 1.0), (2.3, 1.0), [])
    assert behind is False and "NOTHING IN THE WAY" in words


def test_a_track_through_the_box_is_reported():
    behind, words = tng.room_route_note(BOXES, (0.0, 0.0), (2.3, 0.1), [(1.2, 0.1), (1.25, 0.1)])
    assert behind is True and "occupied cell 2 time(s)" in words, words


def test_no_map_is_unjudged_not_a_pass():
    behind, words = tng.room_route_note(None, (0.0, 0.0), (2.3, 0.1), [])
    assert behind is None and "no /map" in words


def test_a_point_off_the_map_is_free():
    assert tng.grid_occupied(BOXES, 50.0, 50.0) is False


def test_the_sim_wall_warning_is_off_in_a_real_room():
    body = SRC[SRC.index("def estimator_cut_the_corner"):SRC.index("def wall_path_ok")]
    assert "if NO_WALL:\n            return False" in body


def test_the_sim_plan_check_is_off_in_a_real_room():
    assert "if routes_around and reaches_behind and not NO_WALL:" in SRC


def test_only_a_real_room_run_subscribes_to_the_map():
    i = SRC.index('self.create_subscription(OccupancyGrid, "/map"')
    assert "if NO_WALL:" in SRC[i - 600:i], "the sim legs would wait on /map too"
    assert "TRANSIENT_LOCAL" in SRC[i - 600:i], "slam_toolbox's /map is latched; volatile misses it"


def test_the_headline_counts_the_legs_behind_an_obstacle():
    assert "judged legs behind an obstacle on /map" in SRC
    assert "NOT EVERY LEG HAD AN OBSTACLE IN THE WAY" in SRC
