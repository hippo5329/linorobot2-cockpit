"""Every 1-Click keeps its whole output in logs/<run>/oneclick.log.

A run started from the web UI streamed only to the page; progress.log keeps the stages and
the failures, so once the page was gone the room check's own lines were too, and a UNO Q's
test_acc "no room of 1.06 m" could not be traced (2026-10-15).
"""
import io
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "scripts"))
import one_click_pipeline as ocp  # noqa: E402


def test_the_tee_writes_everything_to_the_output_log(tmp_path):
    out = open(tmp_path / "oneclick.log", "w")
    shown = io.StringIO()
    tee = ocp._Tee(shown, None, out)
    tee.write("  open floor: 1.13 m clear all round\n")
    tee.write("CLEARANCE_OK after a move to open floor, 1.12 m\n")
    out.close()
    text = (tmp_path / "oneclick.log").read_text()
    assert "open floor: 1.13" in text and "CLEARANCE_OK" in text
    assert shown.getvalue() == text


def test_start_progress_log_opens_the_output_log(tmp_path, monkeypatch):
    monkeypatch.setattr(ocp, "LOG_DIR", str(tmp_path / "run"))
    monkeypatch.setattr(ocp, "LOG_ROOT", str(tmp_path))
    monkeypatch.setattr(ocp, "PROGRESS", None)
    old_out, old_err = sys.stdout, sys.stderr
    try:
        p = ocp.start_progress_log()
        print("[4.8/6] [CLEARANCE] Nothing within 1.06 m")
        print("  inch 1: heading +30 deg", file=sys.stderr)
        sys.stdout.flush()
    finally:
        sys.stdout, sys.stderr = old_out, old_err
        try:
            p.end("test")
        except Exception:
            pass
    text = open(os.path.join(str(tmp_path / "run"), "oneclick.log")).read()
    assert "[CLEARANCE] Nothing within 1.06 m" in text and "inch 1: heading +30 deg" in text
