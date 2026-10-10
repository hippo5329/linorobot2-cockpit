"""The leg home is judged on heading as well as position.

The verdict used to be position alone, and it cancelled each goal the moment the robot
was inside the gate's radius -- before Nav2's goal checker had turned it to the goal's
heading. The TS100 came home on 2026-10-10 within 0.15 m and facing -120 deg, and the run
passed; the next run would have started SLAM in a frame turned by 120 deg.
"""
import math
import os

from test_nav2_goal_motion import StubNode, _run

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PIPELINE = open(os.path.join(REPO_ROOT, "scripts", "one_click_pipeline.py")).read()


def _home_node(yaw, status=4):
    node = StubNode(odom_lin=0.25, dist=3.1, goal_dist=0.1, planned=True, goal_status=status)
    node.world_yaw = yaw
    return node


def test_home_facing_the_start_heading_passes_and_says_so(monkeypatch, capsys):
    node = _home_node(math.radians(4.0))
    assert _run(monkeypatch, node, timeout=0.3, round_trips=1, goal_x=3.0, goal_y=0.0,
                home_yaw_tolerance=0.20) is True
    out = capsys.readouterr().out
    assert "leg 2/2 -> (0.00, 0.00): reached" in out and "heading +4.0 deg" in out, out
    assert "heading within 11.5 deg" in out, out
    # the leg out is judged on position only
    assert "leg 1/2 -> (3.00, 0.00): reached" in out
    assert out.split("leg 2/2")[0].count("heading +") == 0, out


def test_home_by_position_but_turned_fails(monkeypatch, capsys):
    """The TS100's case: at home, facing -120 deg, the goal closed."""
    node = _home_node(math.radians(-120.0), status=5)
    assert _run(monkeypatch, node, timeout=3.0, round_trips=1, goal_x=3.0, goal_y=0.0,
                home_yaw_tolerance=0.20) is False
    out = capsys.readouterr().out
    assert "HOME WITHOUT ITS HEADING" in out and "-120.0 deg" in out, out


def test_a_heading_wrapping_past_pi_is_measured_the_short_way(monkeypatch):
    node = _home_node(2 * math.pi - math.radians(3.0))
    assert _run(monkeypatch, node, timeout=0.3, round_trips=1, goal_x=3.0, goal_y=0.0,
                home_yaw_tolerance=0.20) is True


def test_without_a_tolerance_the_old_position_verdict_stands(monkeypatch):
    node = _home_node(math.radians(-120.0))
    assert _run(monkeypatch, node, timeout=0.3, round_trips=1, goal_x=3.0, goal_y=0.0) is True


def test_no_heading_measured_is_not_a_pass(monkeypatch):
    node = StubNode(odom_lin=0.25, dist=3.1, goal_dist=0.1, planned=True, goal_status=4)
    assert _run(monkeypatch, node, timeout=3.0, round_trips=1, goal_x=3.0, goal_y=0.0,
                home_yaw_tolerance=0.20) is False


def test_the_pipeline_derives_it_from_the_robots_goal_checker_and_passes_it():
    assert 'checker.get("yaw_goal_tolerance", 0.25)) + 0.05' in PIPELINE
    assert '--home-yaw-tolerance {args.home_yaw_tolerance}' in PIPELINE
