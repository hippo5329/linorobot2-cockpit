"""The motion guard's judgements (scripts/motion_guard.py) and where the 1-Click uses them.

A tracked robot at 8 % charge explored on 6.9 Hz scans of 456 beams (its LiDAR gives 10 Hz),
toward a frontier beyond a garage wall; it reached the wall, the scan stopped, and the map
lost it. Nothing then judged the scan's rate, or whether a goal lay inside the room.
"""
import os
import sys

import pytest

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
    first = PIPELINE.index("if real_checks and nav2_ok:\n                guard_start()")
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


# ---- one rate floor for bring-up and the run ----

import lidar_check  # noqa: E402


def _scans(stamps):
    import math
    return [{"stamp": t, "angle_min": 0.0, "angle_increment": 2 * math.pi / 450, "range_min": 0.02,
             "range_max": 12.0, "ranges": [2.0] * 450} for t in stamps]


def test_a_lidar_bring_up_passes_never_trips_the_guard_and_one_it_fails_does():
    # a 7 Hz LD19 passed bring-up at 60 % and then stopped every run at 80 %
    seven = _stamps(7.0, 10)
    assert not lidar_check.analyse(_scans(seven), 10.0, 10.0)["pass"]
    assert mg.scan_fault(seven, 100.0, 10.0, started=90.0) != ""
    healthy = _stamps(9.0, 10)
    assert lidar_check.analyse(_scans(healthy), 10.0, 10.0)["pass"]
    assert mg.scan_fault(healthy, 100.0, 10.0, started=90.0) == ""
    assert mg.SCAN_RATE_SHARE == lidar_check.RATE_FLOOR_SHARE
    assert mg.SCAN_WINDOW_S == lidar_check.RATE_WINDOW_S


def test_bring_up_judges_the_worst_window_as_the_guard_would():
    # 10 s that average 8.5 Hz, but 5 s of them at 7: the guard trips there, so bring-up fails
    st = _stamps(10.0, 5, end=95.0) + _stamps(7.0, 5, end=100.0)[1:]
    res = lidar_check.analyse(_scans(st), 10.0, 10.0)
    assert res["hz"] > 8.0 and res["worst_hz"] < 8.0 and not res["pass"]


def test_an_unknown_model_is_held_to_the_rate_it_gave_at_the_start():
    ready = _stamps(6.0, 6, end=96.0)
    hz = mg.judged_rate(0.0, ready, 96.0)
    assert hz == pytest.approx(6.0, abs=0.05)
    assert mg.judged_rate(10.0, ready, 96.0) == 10.0
    # later it falls to half: tripped, where want_hz 0 judged nothing at all
    later = ready + _stamps(3.0, 5, end=101.0)[1:]
    assert "measured at the start" in mg.scan_fault(later, 101.0, hz, started=90.0, model_hz=0.0)


def test_a_forwarded_lidar_rides_out_one_wifi_gap():
    gap = _stamps(10.0, 5, end=98.5)                       # then 1.5 s of nothing
    assert "stopped" in mg.scan_fault(gap, 100.0, 10.0, started=90.0, stale_s=mg.scan_stale_s(10.0))
    assert mg.scan_fault(gap, 100.0, 10.0, started=90.0,
                         stale_s=mg.scan_stale_s(10.0, forwarded=True)) == ""
    assert mg.scan_stale_s(10.0) == 1.0 and mg.scan_stale_s(2.0) == 1.5
    assert lidar_check.forwarded({"base_controller": {"lidar": {"model": "ld19", "comm_mode": "udp"}}})
    assert not lidar_check.forwarded({"base_controller": {"lidar": {"comm_mode": "serial"}}})


# ---- the battery: by its voltage when the percentage is NaN, judged on a median ----

def test_a_nan_percentage_is_read_from_the_voltage():
    assert mg.battery_fraction(float("nan"), 7.0, 7.2, 9.9) == 0.0      # under min_v: empty
    assert mg.battery_fraction(float("nan"), 10.5, 7.2, 9.9) == 1.0     # clamped
    mid = mg.battery_fraction(float("nan"), 8.5, 7.2, 9.9)
    assert 0.2 < mid < 0.4                                             # the firmware's own curve
    assert mg.battery_fraction(0.42, 7.0, 7.2, 9.9) == 0.42             # a real percentage wins
    assert mg.battery_fraction(float("nan"), 8.5) is None               # no min_v/max_v
    assert mg.battery_fraction(float("nan"), float("nan"), 7.2, 9.9) is None


def test_the_limits_come_from_the_robots_battery_pins():
    import yaml
    cfg = yaml.safe_load(open(os.path.join(REPO_ROOT, "config", "reference", "makerspet_mini_config.yaml")))
    assert mg.battery_limits(cfg) == (7.2, 9.9)
    assert mg.battery_limits({"base_controller": {"pins": {"battery": 29}}}) == (0.0, 0.0)
    assert mg.battery_limits({}) == (0.0, 0.0)


def test_a_low_pack_with_nan_percentage_trips_where_it_went_unseen():
    b = mg.Battery(7.2, 9.9)
    for i in range(10):
        b.add(100.0 + 0.3 * i, float("nan"), 7.0)
    assert "battery at 0 %" in mg.battery_fault(b.fraction(103.0), 0.05)


def test_one_sag_while_accelerating_is_not_the_pack():
    b = mg.Battery(7.2, 9.9)
    for i, v in enumerate([9.0, 9.0, 6.9, 9.0, 9.0]):          # one sample under min_v
        b.add(100.0 + 0.5 * i, float("nan"), v)
    assert mg.battery_fault(b.fraction(102.0), 0.10) == ""
    assert b.fraction(102.0) == pytest.approx(mg.battery_fraction(None, 9.0, 7.2, 9.9))


def test_a_battery_without_percentage_or_limits_is_not_judged():
    b = mg.Battery()
    b.add(100.0, float("nan"), 11.1)
    assert b.fraction(100.5) is None and not b.judgeable()
    assert mg.Battery(7.2, 9.9).judgeable()


# ---- after a trip a new goal is cancelled the moment it is accepted ----

def test_a_goal_accepted_or_running_is_one_to_cancel():
    assert mg.goal_running([1]) and mg.goal_running([4, 2])
    assert not mg.goal_running([4, 5, 6]) and not mg.goal_running([])


def test_the_battery_is_read_best_effort():
    # the firmware publishes /battery best effort: a reliable (depth 10) reader matches nothing
    # and the guard reported "battery: NOT JUDGED" on MC200 (rc-20261015.2)
    import re
    src = open(os.path.join(os.path.dirname(__file__), "..", "scripts", "motion_guard.py")).read()
    assert re.search(r'create_subscription\(BatteryState, "battery", on_battery, qos_profile_sensor_data\)', src)
