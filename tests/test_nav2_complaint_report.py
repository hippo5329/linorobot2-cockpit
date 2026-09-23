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


PAST_LINE = (
    "[planner_server-3] [INFO] [1790127735.742273636] [global_costmap.global_costmap]: Timed "
    "out waiting for transform from base_link to map to become available, tf error: Lookup "
    "would require extrapolation into the past.  Requested time 1790127734.581103 but the "
    "earliest data is at time 1790127734.756795, when looking up transform from frame "
    "[base_link] to frame [map]\n"
)


def test_the_past_direction_says_the_buffer_was_still_filling(tmp_path):
    """Two directions, two meanings, and only one of them is "late".

    "into the future" means the tree had not published recently enough.
    "into the past" means the request predates the OLDEST entry -- the buffer
    had not filled yet, which is a startup race, not a stall. tf2 prints the
    earliest stamp rather than the latest for this case, and the first version
    of this reporter matched only "the latest data is at" and silently added
    nothing to the line that mattered most on a leg 1/8.
    """
    out = _report(tmp_path, [PAST_LINE])
    # 1790127734.756795 - 1790127734.581103 = 0.175692 s
    assert "the TF buffer began 176 ms after the request: it was still filling" in out
    assert "earliest data is at time 1790127734.756795" in out


def test_the_note_survives_a_line_long_enough_to_be_truncated(tmp_path):
    """The annotation is appended AFTER the cut, never inside it.

    Computing the note and then truncating the result put it past the limit on
    a long global_costmap line and discarded the number the function exists to
    produce -- the same fault, one level up, as the 160-character cut this
    reporter was written to fix.
    """
    padded = PAST_LINE.replace("[global_costmap.global_costmap]",
                               "[global_costmap.global_costmap" + "X" * 300 + "]")
    out = _report(tmp_path, [padded])
    assert "it was still filling" in out, out


def test_a_line_without_two_stamps_is_left_alone(tmp_path):
    line = ("[planner_server-3] [INFO] [1790121447.525125131] [global_costmap]: Timed out "
            "waiting for transform from base_link to map to become available, tf error: x\n")
    out = _report(tmp_path, [line])
    assert "Timed out waiting for transform from base_link to map" in out
    assert "behind the request" not in out


def test_a_braked_robot_is_visible_in_the_transcript(tmp_path):
    """The collision monitor sits between cmd_vel_smoothed and cmd_vel and can
    zero the command every cycle. When it does, the rest of the stack reports
    only "Failed to make progress" -- which reads like a controller fault, and
    on 2026-09-23 sent an afternoon into the wrong layer.

    Its lines are INFO, not WARN or ERROR, and notifyActionState() fires only
    on a CHANGE of state, so a robot held from the first second to the last
    produces exactly ONE line. It has to be in the pattern or it is not in the
    transcript, and the nav2 log dies with the leg's container.
    """
    log = tmp_path / "nav2.log"
    log.write_text(
        "[collision_monitor-9] [WARN] [123.4] [collision_monitor]: Robot to stop due to "
        "invalid source. Either due to data not published yet, or to lack of new data\n"
        "[controller_server-5] [WARN] [124.0] [controller_server]: Failed to make progress\n"
        "[controller_server-5] [WARN] [125.0] [controller_server]: Failed to make progress\n")
    out = ocp._nav2_complaints(str(log))
    assert "Robot to stop due to" in out, out
    assert "Failed to make progress" in out, out
    # the single monitor line must not be crowded out by the frequent one
    assert out.index("Robot to stop") >= 0


def test_every_monitor_action_is_extracted_not_just_the_stop(tmp_path):
    """A slowdown or a speed limit is the same class of evidence: the robot was
    commanded one thing and given another."""
    for phrase in ("Robot to stop due to invalid source",
                   "Robot to slowdown for 50.000000 percents due to Foo polygon",
                   "Robot to limit speed due to Foo polygon",
                   "Robot to approach for 1.200000 seconds away from collision",
                   "Robot to continue normal operation"):
        log = tmp_path / "one.log"
        log.write_text(f"[collision_monitor-9] [INFO] [1.0] [collision_monitor]: {phrase}\n")
        out = ocp._nav2_complaints(str(log))
        assert phrase.split(" due to")[0].split(" for ")[0] in out, (phrase, out)


def test_the_frames_survive_the_truncation(tmp_path):
    """tf2 puts the frames at the END of the message, so the 400-character cut
    that keeps the timestamps throws them away.

    On 2026-09-23 a 102 came back as a 20.9 ms future request "when looking "
    and the two 50 Hz publishers -- slam_toolbox for map->odom, the EKF for
    odom->base_link -- could not be told apart. That is the difference between
    a SLAM fault and a filter fault, and it was the one thing the line was
    being read for.
    """
    long_prefix = "x" * 380
    msg = ("[controller_server-5] [ERROR] [1.0] [RPPPathHandler]: " + long_prefix +
           " Exception in transformPose: Lookup would require extrapolation into the "
           "future.  Requested time 1790144829.902377 but the latest data is at time "
           "1790144829.881487, when looking up transform from frame [odom] to frame [map]")
    log = tmp_path / "nav2.log"
    log.write_text(msg + "\n")
    out = ocp._nav2_complaints(str(log))
    assert "odom -> map" in out, out
    assert "21 ms behind the request" in out, out


def test_no_frame_note_when_tf2_did_not_name_them():
    assert ocp._tf_frames("Failed to make progress") == ""
