"""A 1-Click run keeps its stack for the person who pressed Start; the next Start
launched a second one beside it. On 2026-09-25 the old launch respawned its agent
onto the tty the flasher had released, two agents shared one serial port, and the
old Nav2 drove the new run's robot. The new run must stop the kept stack first."""
import os
import subprocess
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))

import one_click_pipeline as ocp  # noqa: E402
import robot_stack  # noqa: E402

SRC = open(os.path.join(os.path.dirname(__file__), "..", "scripts",
                        "one_click_pipeline.py")).read()


def _kept(tmp_path, tag):
    proc = subprocess.Popen(["sleep", "60"], start_new_session=True)
    robot_stack.record(tag, proc.pid, state_dir=str(tmp_path))
    return proc


def test_the_kept_stack_is_stopped(tmp_path, capsys):
    procs = [_kept(tmp_path, t) for t in ("bringup", "slam", "nav2")]
    try:
        stopped = ocp.stop_previous_stack(state_dir=str(tmp_path))
        assert sorted(stopped) == ["bringup", "nav2", "slam"]
        for p in procs:
            assert p.wait(timeout=5) is not None
        assert robot_stack.load(str(tmp_path)) == []
        assert "previous 1-Click run" in capsys.readouterr().out
    finally:
        for p in procs:
            if p.poll() is None:
                p.kill()


def test_nothing_kept_is_quiet(tmp_path, capsys):
    assert ocp.stop_previous_stack(state_dir=str(tmp_path)) == []
    assert capsys.readouterr().out == ""


def test_it_runs_before_the_board_is_touched():
    call = SRC.index("    stop_previous_stack()\n")
    assert call < SRC.index("[1/6] [CONFIG]")
    assert call < SRC.index("[2/6] [PROBE]")
