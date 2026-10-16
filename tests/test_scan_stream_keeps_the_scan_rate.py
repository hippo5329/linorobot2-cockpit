"""The LiDAR viewer keeps up with the LiDAR.

The page drew ~6.7 fps from a 10 Hz /scan (web UI walkthrough, 2026-10-16).
scan_stream.py capped at 10 Hz with `now - last_emit < 0.1 s -> drop`, and a
LiDAR's revolutions are not exactly 100 ms apart: every scan that came a few ms
early was dropped. The gate now schedules slots from when the last was due and
admits a frame up to half a period early; the cap went to 20 Hz.
"""
import ast
import os
import random
import re

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCRIPT = os.path.join(REPO_ROOT, "scripts", "scan_stream.py")


def _rate_gate():
    """RateGate alone: the module itself exits without rclpy."""
    src = open(SCRIPT).read()
    tree = ast.parse(src)
    node = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "RateGate")
    ns = {}
    exec(compile(ast.Module(body=[node], type_ignores=[]), SCRIPT, "exec"), ns)
    return ns["RateGate"]


def _jittered(hz, seconds, jitter, seed=1):
    rnd = random.Random(seed)
    return [k / hz + rnd.uniform(-jitter, jitter) for k in range(int(hz * seconds))]


def _passed(gate, times):
    return sum(1 for t in sorted(times) if gate.admit(t))


def test_a_jittered_10hz_scan_passes_whole_at_the_default_cap():
    RateGate = _rate_gate()
    default = float(re.search(r'"--max-hz", type=float, default=([\d.]+)', open(SCRIPT).read()).group(1))
    times = _jittered(10, 60, 0.015)
    assert _passed(RateGate(default), times) == len(times)


def test_even_a_cap_at_the_scan_rate_no_longer_drops_early_frames():
    RateGate = _rate_gate()
    times = _jittered(10, 60, 0.02)
    assert _passed(RateGate(10.0), times) >= 0.99 * len(times)


def test_the_old_gate_was_the_bottleneck():
    """What the viewer measured: ~2/3 of a jittered 10 Hz stream under a 10 Hz strict gate."""
    times = _jittered(10, 60, 0.02)
    last, n = -1.0, 0
    for t in times:
        if t - last >= 0.1:
            last, n = t, n + 1
    assert n < 0.8 * len(times)


def test_a_faster_lidar_is_still_capped():
    RateGate = _rate_gate()
    times = _jittered(40, 30, 0.003)
    n = _passed(RateGate(20.0), times)
    assert 0.95 * 20 * 30 <= n <= 1.02 * 20 * 30


def test_a_stall_does_not_release_a_burst():
    RateGate = _rate_gate()
    gate = RateGate(10.0)
    assert gate.admit(0.0)
    # nothing for 2 s, then frames 10 ms apart: only the pace of the cap gets through
    burst = [2.0 + 0.01 * k for k in range(30)]
    assert _passed(gate, burst) <= 4


def test_zero_means_uncapped():
    RateGate = _rate_gate()
    assert _passed(RateGate(0), [0.0, 0.001, 0.002]) == 3


def test_the_viewer_reports_its_frame_rate():
    js = open(os.path.join(REPO_ROOT, "web", "frontend", "app-nav-viz.js")).read()
    body = js[js.index('lidarSource.addEventListener("scan"'):]
    body = body[:body.index("});")]
    assert "fps" in body
