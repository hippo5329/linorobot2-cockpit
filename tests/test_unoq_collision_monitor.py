"""The collision monitor takes the newest transform: base_shift_correction false, in every design.

MC200 (2026-10-14): scans ~50 ms ahead of the newest odom TF on a CPU that ran the 20 Hz
controller at 10.4 Hz. With the shift on, the monitor's lookup at each scan's stamp failed
("extrapolation into the future") and every failure stopped the robot, so a Nav2 soak chunk
logged 26 "Failed to make progress". One template for every design (test_default_chassis), so
the flag is off everywhere: at 0.25 m/s and 10 Hz scans the shift is <= 2.5 cm.
"""
import glob
import os

import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def test_every_design_takes_the_newest_transform():
    seen = 0
    for p in glob.glob(os.path.join(ROOT, "config", "reference", "*_config.yaml")):
        cm = ((yaml.safe_load(open(p)).get("nav2") or {}).get("collision_monitor") or {}).get("ros__parameters")
        if cm is None:
            continue
        seen += 1
        assert cm["base_shift_correction"] is False, p
        assert cm["source_timeout"] <= 1.0, p          # a quiet LiDAR still stops the robot
    assert seen >= 5
