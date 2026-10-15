"""The 1-Click's motion guard around test_acc and exploration (one_click_pipeline.py).

A guard that has tripped (a starved LiDAR, a flat pack) must keep test_acc's full-power runs
from starting, and must stop exploration at once rather than when the explorer next returns.
"""
import os
import subprocess
import sys
import time
import types

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO_ROOT, "scripts"))

import one_click_pipeline as ocp  # noqa: E402

PIPELINE = open(os.path.join(REPO_ROOT, "scripts", "one_click_pipeline.py")).read()


def test_a_trip_stops_the_exploration_watch_at_once(tmp_path, monkeypatch):
    logs = {}

    def fake_launch(cmd, log_tag="launch", distro="lyrical"):
        path = tmp_path / f"{log_tag}.log"
        logs[log_tag] = path
        fh = open(path, "w")
        fh.write("exploring\n")
        fh.flush()
        return subprocess.Popen(["sleep", "30"], stdout=fh, preexec_fn=os.setsid)

    monkeypatch.setattr(ocp, "launch_bg", fake_launch)
    monkeypatch.setattr(ocp, "run_log_path", lambda tag: str(tmp_path / f"{tag}.log"))
    trip = tmp_path / "trip"
    args = types.SimpleNamespace(explore_timeout=30, explore_min_area=0.0, distro="jazzy")
    t0 = time.time()
    trip.write_text("MOTION_GUARD_TRIPPED: /scan stopped: the last one is 1.2 s old\n")
    res = ocp.watch_exploration(args, str(trip), poll_s=0.05)
    assert res.returncode == 3 and time.time() - t0 < 10
    assert "exploring" in res.stdout


def test_the_exploration_watch_returns_its_own_exit_untripped(tmp_path, monkeypatch):
    def fake_launch(cmd, log_tag="launch", distro="lyrical"):
        fh = open(tmp_path / f"{log_tag}.log", "w")
        return subprocess.Popen(["bash", "-c", "echo EXPLORE_DONE; exit 0"], stdout=fh)

    monkeypatch.setattr(ocp, "launch_bg", fake_launch)
    monkeypatch.setattr(ocp, "run_log_path", lambda tag: str(tmp_path / f"{tag}.log"))
    args = types.SimpleNamespace(explore_timeout=30, explore_min_area=0.0, distro="jazzy")
    res = ocp.watch_exploration(args, str(tmp_path / "no_trip"), poll_s=0.05)
    assert res.returncode == 0 and "EXPLORE_DONE" in res.stdout


def test_test_acc_is_not_run_after_a_trip_and_the_guard_is_restarted_after_it():
    body = PIPELINE[PIPELINE.index("def measure_and_restart(nav2_state):"):
                    PIPELINE.index('print(f"\\n[6/6] [NAV2] Launching Nav2')]
    # refused on a trip or a guard that never said ready, before Nav2 is stopped
    refuse = body.index('print(f"  ❌ TEST_ACC NOT RUN: {why}")')
    assert refuse < body.index("_stop_group_and_wait(proc)")
    # the guard is stopped before the board reboots into test_acc and started after Nav2
    assert body.index("guard_stop()") < body.index("test_acc_phase(")
    assert body.index("test_acc_phase(") < body.index("guard_start()")
    # the explore branch measures only while the guard is happy
    assert "if do_test_acc and motion_ok:\n" in PIPELINE


def test_the_ekf_goes_to_the_snapshot_pose_not_the_pose_before_inching():
    body = PIPELINE[PIPELINE.index("def test_acc_phase("):PIPELINE.index("def _test_acc_body(")]
    assert 'pose_home = state.get("pose_home") or pose0' in body
    assert "ekf_set_pose(pose_home, args.distro)" in body
    inner = PIPELINE[PIPELINE.index("def _test_acc_body("):]
    assert inner.index('state["pose_home"]') > inner.index("if not clearance_step(args, params, controller_cfg, need, save=snap")
