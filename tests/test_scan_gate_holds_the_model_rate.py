"""The topic gate holds /scan to 80 % of the LiDAR model's own rate, not a flat 5 Hz.

The LD driver polled a 10 Hz LiDAR at 10 Hz and dropped one scan in four at times: the
real tracked robot ran at 6.6-8.8 Hz and its motion guard (80 % of the model's rate) stopped Nav2,
while every simulation leg passed the 5 Hz gate (2026-10-15).
"""
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "scripts"))


def test_the_verifier_takes_the_floor_and_the_pipeline_passes_the_models():
    vt = open(os.path.join(ROOT, "scripts", "verify_topics.py")).read()
    assert '"--scan-min-hz"' in vt and 'self.thresholds["/scan"] = scan_min_hz' in vt
    assert "scan_min_hz=args.scan_min_hz" in vt
    ocp = open(os.path.join(ROOT, "scripts", "one_click_pipeline.py")).read()
    assert 'verify_flag += f" --scan-min-hz {lidar_check.RATE_FLOOR_SHARE * want_hz:.1f}"' in ocp


def test_an_ld19_is_held_to_eight_hz():
    import lidar_check
    hz = lidar_check.expected_hz({"base_controller": {"lidar": {"model": "ld19"}}})
    assert round(lidar_check.RATE_FLOOR_SHARE * hz, 1) == 8.0
    assert lidar_check.expected_hz({"base_controller": {"lidar": {"model": "nothing_known"}}}) == 0.0
