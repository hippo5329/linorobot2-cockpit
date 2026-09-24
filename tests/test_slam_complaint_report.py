""""No map was published" has to say WHICH failure it was.

Three different faults produce that one sentence -- a lifecycle node stuck in
`inactive`, a solver plugin that would not load, and scans dropped because
odom->laser was not in the TF buffer yet -- and they are three different
repairs. The pipeline used to print "check logs/slam.log for 'Activating'",
naming a file inside the leg's container, which the next leg destroys.

Seen on the Yahboom skid_steer jazzy leg of the 2026-09-24 gate: /odom 50.2 Hz,
/imu/data 50.0 Hz, /scan 10.0 Hz, all green, then no map, then planner_server
failing to configure with Invalid frame ID "map". The Nav2 error was the
symptom; the log that named the layer was already gone.
"""
import os
import sys

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO_ROOT, "scripts"))

import one_click_pipeline as ocp  # noqa: E402


def _report(tmp_path, text):
    log = tmp_path / "slam.log"
    log.write_text(text)
    return ocp._slam_complaints(str(log))


def test_configuring_without_activating_names_the_stuck_node(tmp_path):
    """The case the manual `lifecycle set activate` exists to rescue."""
    out = _report(tmp_path, "[slam_toolbox-6] [INFO] [1.0] [slam_toolbox]: Creating\n"
                            "[slam_toolbox-6] [INFO] [1.1] [slam_toolbox]: Configuring\n")
    assert "Creating -> Configuring" in out, out
    assert "stuck in `inactive`" in out, out


def test_reaching_activating_is_not_reported_as_stuck(tmp_path):
    """Activated and still no map is a DIFFERENT fault, and saying "stuck in
    inactive" about it sends the reader to the wrong layer."""
    out = _report(tmp_path, "[slam_toolbox-6] [INFO] [1.0] [slam_toolbox]: Configuring\n"
                            "[slam_toolbox-6] [INFO] [1.2] [slam_toolbox]: Activating\n")
    assert "Configuring -> Activating" in out, out
    assert "stuck in" not in out, out


def test_dropped_scans_are_counted_as_one_complaint(tmp_path):
    """The classic no-map-with-a-healthy-/scan: the transform is not there yet,
    so the message filter discards every scan. Its timestamp moves, so counting
    the raw text turns one fault into a page of one-of-each rows."""
    lines = "".join(
        f"[slam_toolbox-6] [WARN] [{t}.0] [slam_toolbox]: Message Filter dropping message: "
        f"frame 'laser' at time {t}.000 for reason 'discarding message because the queue is full'\n"
        for t in range(100, 112))
    out = _report(tmp_path, lines)
    rows = [r for r in out.splitlines() if "x " in r and "dropping message" in r]
    assert len(rows) == 1, out
    assert rows[0].strip().startswith("12x"), rows[0]


def test_a_missing_log_says_so_rather_than_claiming_nothing_was_logged(tmp_path):
    """No file and an empty file mean different things: the first is the
    container already gone, the second is slam_toolbox never speaking."""
    gone = ocp._slam_complaints(str(tmp_path / "gone" / "slam.log"))
    assert "no slam.log to explain it" in gone, gone
    out = _report(tmp_path, "")
    assert "logged no lifecycle transition at all" in out, out


def test_the_frames_survive_a_transform_complaint(tmp_path):
    """Reuses the nav2 reporter's frame note: which transform was missing is
    the difference between a SLAM fault and a filter fault."""
    out = _report(tmp_path, "[slam_toolbox-6] [INFO] [1.0] [slam_toolbox]: Configuring\n"
                            "[slam_toolbox-6] [ERROR] [2.0] [slam_toolbox]: Lookup would require "
                            "extrapolation into the future.  Requested time 1790144829.902377 but "
                            "the latest data is at time 1790144829.881487, when looking up "
                            "transform from frame [odom] to frame [base_link]\n")
    assert "odom -> base_link" in out, out
    assert "21 ms behind the request" in out, out
