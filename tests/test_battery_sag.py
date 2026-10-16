"""battery_sag.py: the pack's rest voltage, its worst sag and where it fell, from the
logger's samples, the 1-Click's step timeline and test_acc's own figures (user, 2026-10-16:
"always monitor battery sag, and report worst sag in web ui")."""
import json
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "scripts"))
import battery_sag  # noqa: E402

PHASES = [(0, "bringup (at rest)"), (10, "clearance"), (20, "SLAM and Nav2 start-up (at rest)"),
          (30, "exploration"), (60, "drive suite"), (80, "map save (at rest)")]


def test_rest_is_the_median_before_any_load_and_worst_is_the_lowest_under_load():
    samples = [(1, 11.00), (2, 11.02), (3, 10.98), (12, 10.90), (25, 10.95), (35, 10.50),
               (40, 10.61), (65, 10.70), (85, 10.80)]
    s = battery_sag.summarize(samples, PHASES)
    assert s["rest"] == 11.00
    assert s["worst"] == 10.50 and s["where"] == "exploration"
    assert abs(s["sag"] - 0.50) < 1e-9
    # at-rest stretches after a load (recovery) are neither the rest nor a load
    assert s["phases"] == {"clearance": 10.90, "exploration": 10.50, "drive suite": 10.70}


def test_test_acc_counts_when_it_is_the_worst():
    t = battery_sag.test_acc_figures("MAX VEL ...\nBATTERY worst 10.31 V  sag  0.67 V from 10.98 V at rest\n")
    assert t == {"worst": 10.31, "sag": 0.67, "rest": 10.98}
    s = battery_sag.summarize([(1, 11.0), (35, 10.5)], PHASES, t)
    assert s["worst"] == 10.31 and s["where"] == "test_acc"
    line = battery_sag.sag_line(s)
    assert line.startswith("BATTERY_SAG: worst 10.31 V during test_acc, 0.69 V")
    assert "below 11.00 V at rest" in line


def test_no_reading_before_the_first_load_takes_the_later_rest():
    # MC200 2026-10-16 run 2: the logger's first sample came 8 s into the clearance step, and
    # the line said "not measured" over 775 samples
    samples = [(12, 10.90), (22, 10.80), (24, 10.82), (35, 10.50), (65, 10.70), (85, 10.78)]
    s = battery_sag.summarize(samples, PHASES)
    assert s["rest"] == 10.80                  # median of 10.80, 10.82, 10.78
    assert s["worst"] == 10.50 and s["where"] == "exploration"


def test_the_pipeline_waits_for_the_first_readings_before_the_first_motion():
    src = open(os.path.join(os.path.dirname(battery_sag.__file__), "one_click_pipeline.py"),
               encoding="utf-8").read()
    start = src[src.index("def battery_start("):src.index("def battery_report(")]
    assert "time.time() + 15.0" in start and ">= 3" in start


def test_nothing_under_load_is_not_measured():
    assert battery_sag.summarize([(1, 11.0)], PHASES) is None
    assert battery_sag.sag_line(None).startswith("BATTERY_SAG: not measured")


def test_the_logs_torn_lines_and_zero_volts_are_skipped(tmp_path):
    p = tmp_path / "b.jsonl"
    p.write_text(json.dumps({"t": 1, "v": 11.0}) + "\n{\"t\": 2, \n" + json.dumps({"t": 3, "v": 0.0}) + "\n")
    assert battery_sag.read_samples(str(p)) == [(1.0, 11.0)]
