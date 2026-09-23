"""The transform complaint has to reach the transcript whole.

_nav2_complaints exists so error_code=102/103/203 arrives with the transform
name and "by how much it was late" attached, because the container holding
nav2.log is destroyed by the next leg. It then truncated every line at 160
characters, which on the tf2 message lands mid-word:

    Requested time 1790121566.891555 but t

-- cutting off the latest-data stamp, i.e. the only half that answers "by how
much". It also counted the raw text, so one recurring stall with a moving
timestamp was reported as N distinct complaints, 1x each.

Seen on a 2wd serial board, both distros, 2026-09-23: leg 7/8 aborted with 102 and
the transcript could not say whether the tree was 30 ms or 30 s behind.
"""
import os
import sys

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO_ROOT, "scripts"))

import one_click_pipeline as ocp  # noqa: E402

TF_LINE = (
    "[controller_server-4] [WARN] [1790121567.201752301] [RPPPathHandler]: Exception in "
    "transformPose: Lookup would require extrapolation into the future.  Requested time "
    "1790121566.891555 but the latest data is at time 1790121566.575000, when looking up "
    "transform from frame [odom] to frame [map]\n"
)


def _report(tmp_path, lines):
    log = tmp_path / "nav2.log"
    log.write_text("".join(lines))
    return ocp._nav2_complaints(str(log))


def test_the_latest_data_stamp_survives_the_report(tmp_path):
    out = _report(tmp_path, [TF_LINE])
    assert "the latest data is at time 1790121566.575000" in out
    assert "frame [odom] to frame [map]" in out


def test_the_gap_is_computed_so_a_stall_is_told_from_a_skew(tmp_path):
    out = _report(tmp_path, [TF_LINE])
    # 1790121566.891555 - 1790121566.575000 = 0.316555 s
    assert "TF tree was 317 ms behind the request" in out


def test_one_recurring_stall_counts_as_one_complaint(tmp_path):
    lines = [TF_LINE.replace("1790121566.891555", f"179012156{n}.891555") for n in range(6)]
    out = _report(tmp_path, lines)
    rows = [r for r in out.splitlines() if "x " in r and "extrapolation" in r]
    assert len(rows) == 1, out
    assert rows[0].strip().startswith("6x"), rows[0]


def test_a_line_without_two_stamps_is_left_alone(tmp_path):
    line = ("[planner_server-3] [INFO] [1790121447.525125131] [global_costmap]: Timed out "
            "waiting for transform from base_link to map to become available, tf error: x\n")
    out = _report(tmp_path, [line])
    assert "Timed out waiting for transform from base_link to map" in out
    assert "behind the request" not in out
