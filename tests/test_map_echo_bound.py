"""The /map wait gives each `ros2 topic echo --once` longer than a slow robot computer needs.

UNO Q (QRB2210, Cortex-A53), 2026-10-05: a cold echo of a latched topic took
3.5-5.1 s on an IDLE board. The old 5 s bound timed out under Nav2 load on a
slam_toolbox that was publishing, and the leg failed "SLAM: no map was
published" while Nav2 reached 8/8 on that very map.
"""
import os
import sys
import time

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO_ROOT, "scripts"))

import one_click_pipeline as ocp  # noqa: E402


class Res:
    returncode, stdout = 1, ""


def _echo_bounds(monkeypatch, timeout_sec):
    seen = []

    def run_ros(cmd, timeout=None, distro=None):
        seen.append((int(cmd.split()[1]), timeout))
        return Res()

    clock = [1000.0]
    monkeypatch.setattr(ocp, "run_ros", run_ros)
    monkeypatch.setattr(ocp.time, "time", lambda: clock[0])
    monkeypatch.setattr(ocp.time, "sleep", lambda s: clock.__setitem__(0, clock[0] + s + 15))
    assert not ocp.wait_for_topic("/map", timeout_sec=timeout_sec, require_message="info.width")
    return seen


def test_each_echo_outlasts_the_measured_cold_start(monkeypatch):
    seen = _echo_bounds(monkeypatch, 40)
    assert seen and seen[0][0] >= 10, seen      # past the 5.1 s worst idle measurement
    assert all(t > b for b, t in seen), seen    # the subprocess outlives the echo's own timeout


def test_the_echo_never_runs_far_past_the_wait(monkeypatch):
    seen = _echo_bounds(monkeypatch, 40)
    assert all(5 <= b <= 15 for b, _ in seen), seen
