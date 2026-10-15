"""The motion guard's judgements (scripts/motion_guard.py) and where the 1-Click uses them.

A tracked robot at 8 % charge explored on 6.9 Hz scans of 456 beams (its LiDAR gives 10 Hz),
toward a frontier beyond a garage wall; it reached the wall, the scan stopped, and the map
lost it. Nothing then judged the scan's rate, or whether a goal lay inside the room.
"""
import os
import sys

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO_ROOT, "scripts"))

import motion_guard as mg  # noqa: E402

PIPELINE = open(os.path.join(REPO_ROOT, "scripts", "one_click_pipeline.py")).read()


def _stamps(hz, seconds, end=100.0):
    n = int(hz * seconds)
    return [end - seconds + i / hz for i in range(n + 1)]


def test_a_healthy_scan_passes():
    assert mg.scan_fault(_stamps(10.0, 5), 100.0, 10.0, started=90.0) == ""


def test_the_tracked_robots_scan_rate_is_refused():
    why = mg.scan_fault(_stamps(6.9, 5), 100.0, 10.0, started=90.0)
    assert "6.9 Hz" in why and "8.0 Hz" in why, why


def test_a_busy_robot_computer_s_short_dip_is_not_a_starved_lidar():
    # MC200's UNO Q while Nav2 came up: a few seconds near 8.4 Hz inside a 10 Hz stream
    dip = [t for t in _stamps(10.0, 5) if t < 97.0] + _stamps(8.4, 3)[1:]
    assert mg.scan_fault(dip, 100.0, 10.0, started=90.0) == ""
    assert mg.scan_fault(_stamps(7.5, 5), 100.0, 10.0, started=90.0) != ""


def test_the_ready_check_waits_out_a_whole_rate_window():
    assert mg.READY_WINDOW_S >= mg.SCAN_WINDOW_S


def test_a_stopped_scan_is_refused_whatever_its_rate_was():
    st = _stamps(10.0, 5, end=97.0)
    assert "stopped" in mg.scan_fault(st, 100.0, 10.0, started=90.0)


def test_no_scan_at_all_is_refused_once_the_grace_has_passed():
    assert mg.scan_fault([], 90.5, 10.0, started=90.0) == ""
    assert "no /scan" in mg.scan_fault([], 92.0, 10.0, started=90.0)


def test_an_unknown_lidar_model_is_judged_on_staleness_only():
    assert mg.scan_fault(_stamps(3.0, 5), 100.0, 0.0, started=90.0) == ""


def test_the_rate_is_not_judged_before_a_full_window():
    assert mg.scan_fault(_stamps(6.0, 1), 100.0, 10.0, started=99.0) == ""


def test_battery_fraction_and_unmeasured():
    assert "8 %" in mg.battery_fault(0.08, 0.10)
    assert mg.battery_fault(0.30, 0.10) == ""
    assert mg.battery_fault(float("nan"), 0.10) == ""     # NaN = the board does not measure it
    assert mg.battery_fault(None, 0.10) == ""


ROOM = [(-3.0, -2.5), (3.5, -2.5), (3.5, 2.0), (-3.0, 2.0)]


def test_the_room_outline_is_the_hull_of_the_returns():
    pts = ROOM + [(0.0, 0.0), (1.0, 1.0), (-2.0, 1.5)]
    assert sorted(mg.hull(pts)) == sorted(ROOM)


def test_a_plan_inside_the_room_passes_and_one_past_the_wall_is_named():
    poly = mg.hull(ROOM)
    assert mg.plan_outside(poly, (1.6, -1.35), 0.30) == ""
    # the explorer's frontier beyond the garage wall
    why = mg.plan_outside(poly, (-0.59, 3.14), 0.30)
    assert "beyond the room's outline" in why, why


def test_a_plan_hugging_the_wall_is_named_too():
    poly = mg.hull(ROOM)
    why = mg.plan_outside(poly, (0.0, 1.9), 0.30)
    assert "0.10 m inside" in why, why


def test_the_robot_outside_the_outline_reads_negative():
    poly = mg.hull(ROOM)
    assert mg.inside_by(poly, -3.95, 2.30) < 0
    assert mg.inside_by(poly, 0.0, 0.0) > 1.9


def test_the_pipeline_starts_the_guard_for_a_real_robot_before_any_motion():
    # as soon as Nav2 is up -- before test_acc, before exploration or the goals
    first = PIPELINE.index("if is_real and nav2_ok:\n                guard_start()")
    assert first < PIPELINE.index("nav2_ok, nav2_detail, nav2_log = measure_and_restart(")
    assert first < PIPELINE.index('explore = launch_bg(f"ros2 launch linorobot2_cockpit explore.launch.py')
    assert "if nav2_ok and motion_ok:\n                    explore = launch_bg" in PIPELINE
    assert "if nav2_ok and motion_ok:\n                    test_res = run_ros" in PIPELINE


def test_a_trip_fails_the_run_with_its_reason():
    assert PIPELINE.count('motion_guard_tripped(mg["trip"])') == 3
    assert PIPELINE.count('failures.append(f"Motion guard: {tripped}")') == 2
    assert 'failures.append(f"Motion guard: {why_not}")' in PIPELINE


def test_the_room_bound_is_on_unless_asked_off():
    assert "not args.explore_beyond_room" in PIPELINE
    assert '"--explore-beyond-room", action="store_true"' in PIPELINE
