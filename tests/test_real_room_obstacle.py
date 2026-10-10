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
    assert "driven track stayed clear" in words, words
    # the box is 0.4 m deep; the 0.16 m disc meets it for that plus a radius each side
    assert 0.65 <= tng.blocked_length(BOXES, (0.0, 0.0), (2.3, 0.1)) <= 0.80


def test_a_clear_line_says_the_leg_tested_no_detour():
    behind, words = tng.room_route_note(BOXES, (0.0, 1.2), (2.3, 1.2), [])
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


def test_a_single_goal_gets_the_room_verdict_too():
    """Only the round-trip legs printed it; a single goal with nothing in the way said nothing."""
    assert SRC.count('verdict(f"NAV2 GOAL REACHED (within {goal_tolerance:.2f} m){_room_note()}")') == 2


def _panel(value):
    """A 0.1 m panel of cells at `value`, as a real tracked robot's box panel ended up in /map."""
    g = _room([])
    res, ox, oy, w, h, data = g
    data = list(data)
    for j in range(h):
        for i in range(w):
            x, y = ox + (i + 0.5) * res, oy + (j + 0.5) * res
            if 1.6 <= x <= 1.7 and -0.25 <= y <= 0.35:
                data[j * w + i] = value
    return (res, ox, oy, w, h, data)


def test_a_thin_panel_that_slam_let_fade_still_counts():
    """From leg 12 of a 20/20 run the panel's cells sat below 65 % and every line read
    "nothing in the way" with the panel standing (camera, 2026-10-10)."""
    behind, words = tng.room_route_note(_panel(55), (0.0, 0.0), (2.3, 0.1), [])
    assert behind is True, words


def test_a_line_that_grazes_the_panels_end_counts_but_a_clear_one_does_not():
    assert tng.room_route_note(_panel(100), (0.0, 0.45), (2.3, 0.45), [])[0] is True   # 0.10 m off its end
    assert tng.room_route_note(_panel(100), (0.0, 0.60), (2.3, 0.60), [])[0] is False  # 0.25 m off


def test_free_and_unknown_cells_are_not_in_the_way():
    assert tng.room_route_note(_panel(25), (0.0, 0.0), (2.3, 0.1), [])[0] is False
    assert tng.room_route_note(_panel(-1), (0.0, 0.0), (2.3, 0.1), [])[0] is False


def test_the_track_is_still_judged_strictly():
    """A pose that only passes NEAR a faded panel is not a pose inside an obstacle."""
    assert tng.track_hits(_panel(55), [(1.65, 0.0)]) == 0
    assert tng.track_hits(_panel(100), [(1.65, 0.0)]) == 1
