"""Returns inside the robot's footprint on the scan Nav2's collision monitor reads.

A mecanum robot's LiDAR saw its own mount at 0.03-0.05 m, bearings 226-243 deg, just past
its lidar.mask.sectors [[130, 225]]. The collision monitor (FootprintApproach, min_points 6)
counted a collision in progress and held /cmd_vel at zero for every leg while Nav2 reported
"active", the clearance step passed ("nearest 1.04 m": it drops returns that near as the
robot's body), and the LiDAR bring-up passed ("nearest 0.03 m"). self_returns.py finds
them; the 1-Click refuses the Nav2 step on them, lidar_check.py fails on them, clearance.py
names them. No ROS here.
"""
import math
import os
import re
import sys

import pytest
import yaml

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO, "scripts"))

import lidar_check  # noqa: E402
import self_returns  # noqa: E402

N_BEAMS = 456                       # an LD19: 0.79 deg a beam
INC = 2 * math.pi / N_BEAMS


def ld19_scan(far=1.5, near=None, range_min=0.02, range_max=12.0, angle_min=0.0):
    """A whole-circle scan, the LD driver's 0..2 pi; `near` maps beam index -> range."""
    ranges = [far] * N_BEAMS
    for i, r in (near or {}).items():
        ranges[i] = r
    return {"angle_min": angle_min, "angle_increment": INC, "range_min": range_min,
            "range_max": range_max, "ranges": ranges}


def beams_between(a_deg, b_deg):
    return [i for i in range(N_BEAMS) if a_deg <= math.degrees(i * INC) <= b_deg]


def mc200_scan():
    """21 returns at 0.03-0.05 m across 226-243 deg, as the robot's /scan held them."""
    idx = beams_between(226.0, 243.0)
    assert len(idx) == 21
    return ld19_scan(near={i: 0.03 + 0.02 * (k % 3) / 2 for k, i in enumerate(idx)})


MC200_BC = {"lidar": {"mask": {"sectors": [[130, 225]]}}}


# ---- the pure arc finder ----

def test_mc200_case_refuses_with_the_arc_and_the_sector_to_add():
    arcs = self_returns.self_return_arcs([mc200_scan()] * 10, 0.26, 6)
    assert len(arcs) == 1
    a = arcs[0]
    assert (a["start"], a["end"]) == (226, 243)
    assert a["count"] == 21
    assert a["rmin"] == pytest.approx(0.03) and a["rmax"] == pytest.approx(0.05)
    assert self_returns.suggest(a) == [221, 248]
    text = "\n".join(self_returns.refusal(arcs, 0.26, 6, "/scan", MC200_BC))
    assert "21 LiDAR returns inside the robot's 0.26 m footprint" in text
    assert "226-243 deg" in text and "0.03-0.05 m" in text
    assert "add lidar.mask.sectors [221, 248] (5 deg past the returns) or move what the LiDAR " \
           "sees; until then Nav2's collision monitor holds the robot still" in text
    # pasteable whole value, joined with the config's own sector
    assert "lidar.mask.sectors: [[130, 248]]" in text


def test_clean_scan_passes():
    assert self_returns.self_return_arcs([ld19_scan()] * 10, 0.26, 6) == []


def test_single_stray_close_return_passes():
    # one beam, every scan: a real thing but under min_points
    assert self_returns.self_return_arcs([ld19_scan(near={10: 0.05})] * 10, 0.26, 6) == []
    # a flicker: a few close beams in one scan of ten
    scans = [ld19_scan() for _ in range(10)]
    scans[3] = ld19_scan(near={i: 0.04 for i in range(100, 110)})
    assert self_returns.self_return_arcs(scans, 0.26, 6) == []


def test_returns_in_a_minority_of_scans_do_not_count():
    scans = [mc200_scan() if k < 4 else ld19_scan() for k in range(10)]
    assert self_returns.self_return_arcs(scans, 0.26, 6) == []
    scans = [mc200_scan() if k < 7 else ld19_scan() for k in range(10)]
    assert self_returns.self_return_arcs(scans, 0.26, 6)


def test_inf_nan_and_zero_are_not_returns():
    near = {i: v for i, v in zip(range(50, 80), [math.inf, math.nan, 0.0] * 10)}
    assert self_returns.self_return_arcs([ld19_scan(near=near)] * 10, 0.26, 6) == []
    # a simulated LiDAR's "nothing out there" is inf everywhere
    assert self_returns.self_return_arcs([ld19_scan(far=math.inf)] * 10, 0.26, 6) == []


def test_returns_below_range_min_or_above_range_max_ignored():
    near = {i: 0.03 for i in range(50, 80)}
    assert self_returns.self_return_arcs([ld19_scan(near=near, range_min=0.05)] * 10, 0.26, 6) == []
    assert self_returns.self_return_arcs([ld19_scan(near=near, range_min=0.02)] * 10, 0.26, 6)
    assert self_returns.self_return_arcs([ld19_scan(far=0.1, range_max=0.08)] * 10, 0.26, 6) == []


def test_bearings_follow_the_mount_yaw_and_a_minus_pi_scan():
    # the host's simulated LD19 publishes -pi..pi; same returns, same robot-frame arc
    idx = beams_between(226.0, 243.0)
    shifted = {(i - N_BEAMS // 2) % N_BEAMS: 0.04 for i in idx}
    scan = ld19_scan(near=shifted, angle_min=-math.pi)
    a = self_returns.self_return_arcs([scan] * 5, 0.26, 6)[0]
    assert (a["start"], a["end"]) == (226, 243)
    # a LiDAR mounted turned 180 deg: the scan's 46-63 deg is the robot's 226-243
    near = {i: 0.04 for i in beams_between(46.0, 63.0)}
    a = self_returns.self_return_arcs([ld19_scan(near=near)] * 5, 0.26, 6, laser=(0.0, 0.0, math.pi))[0]
    assert (a["start"], a["end"]) == (226, 243)


def test_inside_is_judged_from_base_link_not_the_lidar():
    # the LiDAR 0.20 m ahead of the centre: a 0.10 m return ahead is 0.30 m out, one behind
    # it 0.10 m in
    ahead = {i: 0.10 for i in beams_between(355.5, 360)} | {i: 0.10 for i in beams_between(0, 4.5)}
    assert self_returns.self_return_arcs([ld19_scan(near=ahead)] * 5, 0.26, 6, laser=(0.2, 0.0, 0.0)) == []
    behind = {i: 0.10 for i in beams_between(176, 184)}
    assert self_returns.self_return_arcs([ld19_scan(near=behind)] * 5, 0.26, 6, laser=(0.2, 0.0, 0.0))


def test_an_arc_across_zero_is_one_arc():
    near = {i: 0.04 for i in beams_between(352, 360)} | {i: 0.04 for i in beams_between(0, 8)}
    arcs = self_returns.self_return_arcs([ld19_scan(near=near)] * 5, 0.26, 6)
    assert len(arcs) == 1
    assert (arcs[0]["start"], arcs[0]["end"]) == (352, 8)
    assert self_returns.suggest(arcs[0]) == [347, 13]


def test_masked_returns_are_left_out():
    masked = self_returns.widened_mask({"lidar": {"mask": {"sectors": [[130, 250]]}}})
    assert self_returns.self_return_arcs([mc200_scan()] * 10, 0.26, 6, masked=masked) == []
    masked = self_returns.widened_mask(MC200_BC)          # [130, 225] + 1 deg: 226-243 stays
    assert self_returns.self_return_arcs([mc200_scan()] * 10, 0.26, 6, masked=masked)


def test_merged_sectors():
    assert self_returns.merged_sectors([[130, 225]], [[221, 248]]) == [[130, 248]]
    assert self_returns.merged_sectors([[350, 10]], [[5, 20], [100, 120]]) == [[100, 120], [350, 20]]


# ---- what the collision monitor is configured with ----

def test_collision_settings_from_a_reference_config():
    with open(os.path.join(REPO, "config", "reference", "unoq_config.yaml")) as fh:
        p = yaml.safe_load(fh)
    cs = self_returns.collision_settings(p)
    assert cs == {"topic": "scan", "min_points": 6, "radius": pytest.approx(0.26)}


def test_collision_settings_launcher_defaults_and_a_footprint():
    p = {"nav2": {"local_costmap": {"local_costmap": {"ros__parameters": {
        "footprint": "[[0.2, 0.15], [0.2, -0.15], [-0.2, -0.15], [-0.2, 0.15]]"}}}}}
    cs = self_returns.collision_settings(p)
    assert cs["topic"] == "scan" and cs["min_points"] == 6
    assert cs["radius"] == pytest.approx(0.25)


def test_collision_settings_without_a_holding_polygon_or_scan_source():
    base = {"observation_sources": ["scan"], "scan": {"type": "scan", "topic": "scan"},
            "polygons": ["Slow"], "Slow": {"type": "polygon", "action_type": "slowdown", "min_points": 6}}
    p = {"nav2": {"collision_monitor": {"ros__parameters": base}}}
    assert self_returns.collision_settings(p)["topic"] is None
    base = dict(base, polygons=["F"], F={"action_type": "approach", "min_points": 4},
                observation_sources=["sonar"], sonar={"type": "range", "topic": "sonar"})
    p = {"nav2": {"collision_monitor": {"ros__parameters": base}}}
    assert self_returns.collision_settings(p)["topic"] is None


# ---- the 1-Click refuses the Nav2 step on them ----

PIPE = open(os.path.join(REPO, "scripts", "one_click_pipeline.py"), encoding="utf-8").read()


def test_pipeline_checks_after_nav2_and_before_any_motion():
    start = PIPE.index("nav2_ok, nav2_detail, nav2_log = start_nav2(nav2_cmd(nav2_cfg)")
    check = PIPE.index("nav2_held = self_return_step(args, nav2_cfg)", start)
    for later in ("guard_start()\n            if do_test_acc and nav2_ok",
                  "measure_and_restart((nav2_ok, nav2_detail, nav2_log))",
                  "explore.launch.py", "test_nav2_goal.py'"):
        assert check < PIPE.index(later, start), later
    block = PIPE[check:PIPE.index("motion_ok = mg[\"ok\"]", check)]
    # a refusal fails the Nav2 step: nothing downstream sends a goal, explores or runs test_acc
    assert "nav2_ok, nav2_detail = False," in block
    assert 'failures.append(f"Nav2: held by its collision monitor' in block


def test_pipeline_says_why_no_goal_was_sent():
    assert "elif nav2_held:" in PIPE
    assert "NAV2 GOAL NOT SENT: Nav2 is {nav2_detail}" in PIPE
    assert "EXPLORE NOT STARTED: Nav2 is {nav2_detail}" in PIPE


def test_self_return_step_reads_the_verdict(monkeypatch):
    import subprocess
    import types
    import one_click_pipeline as ocp
    out = "\n".join(self_returns.refusal(self_returns.self_return_arcs([mc200_scan()] * 10, 0.26, 6),
                                         0.26, 6, "/scan", MC200_BC))
    reason = self_returns.short_reason(self_returns.self_return_arcs([mc200_scan()] * 10, 0.26, 6))
    for rc, stdout, want in ((1, out + "\nSELF_RETURNS REFUSED: " + reason, reason),
                             (0, "SELF_RETURNS OK", ""), (2, "SELF_RETURNS NO_SCAN: nothing", "")):
        monkeypatch.setattr(ocp, "run_ros", lambda *a, **k: subprocess.CompletedProcess([], rc, stdout, ""))
        assert ocp.self_return_step(types.SimpleNamespace(distro="jazzy"), "/x.yaml") == want
    assert reason == ("21 LiDAR returns inside the footprint at 226-243 deg; "
                      "add lidar.mask.sectors [221, 248]")


# ---- the LiDAR bring-up step ----

def test_lidar_check_fails_on_unmasked_self_returns():
    # scan_raw: the whole circle, the mask's own self-returns (147-225) too
    near = {i: 0.04 for i in beams_between(147, 243)}
    scans = [ld19_scan(near=near)] * 10
    with open(os.path.join(REPO, "config", "reference", "unoq_config.yaml")) as fh:
        p = yaml.safe_load(fh)
    p["base_controller"]["lidar"]["mask"] = {"sectors": [[130, 225]]}
    r = lidar_check.self_return_check(scans, p)
    assert r["fail"] and "226-243 deg" in r["fail"] and "[221, 248]" in r["fail"]
    assert any("lidar.mask.sectors [221, 248]" in ln for ln in r["lines"])
    assert any("[[130, 248]]" in ln for ln in r["lines"])


def test_lidar_check_passes_masked_self_returns():
    near = {i: 0.04 for i in beams_between(147, 243)}
    scans = [ld19_scan(near=near)] * 10
    with open(os.path.join(REPO, "config", "reference", "unoq_config.yaml")) as fh:
        p = yaml.safe_load(fh)
    p["base_controller"]["lidar"]["mask"] = {"sectors": [[130, 250]]}
    r = lidar_check.self_return_check(scans, p)
    assert r == {"arcs": [], "lines": [], "fail": ""}


def test_lidar_check_names_but_passes_a_few():
    scans = [ld19_scan(near={i: 0.04 for i in beams_between(300, 302)})] * 10
    r = lidar_check.self_return_check(scans, {})
    assert not r["fail"] and r["arcs"] and "⚠️" in r["lines"][0]


# ---- clearance names what its body filter drops ----

def test_clearance_warns_on_unmasked_body_returns(capsys):
    import types
    import clearance
    r = types.SimpleNamespace(last_scans=[mc200_scan()] * 5, body=0.26, laser=(0.0, 0.0, 0.0))
    arcs = clearance.warn_self_returns(r, [(129.0, 97.0)])          # --blind 129:97 = [130, 225] + 1
    assert arcs and re.search(r"not masked: 226-243 deg .* \[221, 248\]", capsys.readouterr().out)
    r.last_scans = [ld19_scan()] * 5
    assert clearance.warn_self_returns(r, []) == [] and capsys.readouterr().out == ""
