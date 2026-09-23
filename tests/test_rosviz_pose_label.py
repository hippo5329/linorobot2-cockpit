"""The map viewer must say WHICH pose it is drawing.

The filtered pose (EKF, corrected by SLAM) and the base's own odometry are
different claims, and on a long leg they differ by more than a robot radius --
the Nav2 gate measures wall crossings on the raw pose precisely because the
filtered one had already drifted inside the obstacle. A viewer that shows one
and is read as the other gets trusted and is wrong.
"""
import pathlib
import re

FRONTEND = pathlib.Path(__file__).resolve().parents[1] / "web" / "frontend"


def _rosviz():
    return (FRONTEND / "rosviz.js").read_text()


def test_the_panel_has_somewhere_to_say_which_pose():
    assert 'id="rosviz-pose"' in (FRONTEND / "index.html").read_text()


def test_the_label_distinguishes_amcl_from_the_composed_odom_pose():
    js = _rosviz()
    assert "updatePoseLabel" in js
    body = js[js.index("function updatePoseLabel"):]
    body = body[: body.index("\n    }")]
    assert "/amcl_pose" in body
    assert "/odom" in body


def test_an_uncorrected_pose_is_not_labelled_as_being_in_the_map_frame():
    """Before any map->odom arrives the marker is in the ODOM frame.

    Labelling it "in map" then is the original defect in words rather than in
    pixels: the viewer would be claiming a localisation it does not have.
    """
    js = _rosviz()
    body = js[js.index("function updatePoseLabel"):]
    body = body[: body.index("\n    }")]
    assert "counts.tf" in body, "the label must depend on whether a correction has arrived"
    # the branch taken with no correction must not claim the map frame
    no_corr = body[body.index("counts.tf"):]
    assert "not localised" in no_corr or "odom frame" in no_corr


def test_the_label_is_updated_wherever_the_pose_or_the_correction_changes():
    js = _rosviz()
    for handler in ("counts.tf++", "counts.odom++", "counts.amcl++"):
        line = [l for l in js.splitlines() if handler in l]
        assert line, f"no handler for {handler}"
        assert "updatePoseLabel()" in line[0], f"{handler} does not refresh the pose label"
