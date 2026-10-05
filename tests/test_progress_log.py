"""logs/<run>/progress.log tells where a 1-Click run is, and that it is still moving.

A UNO Q run sat at "[3/6] [FLASH]" for a quarter of an hour (2026-10-06): an lsof that did
not return. Nothing showed whether it was flashing slowly or stuck. The progress log takes
the run's own stage lines with times, and repeats "still at <stage>" while one lasts.
"""
import os
import re
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "scripts"))

import one_click_pipeline as ocp  # noqa: E402


def _log(tmp_path, beat=3600):
    return ocp.ProgressLog(str(tmp_path / "progress.log"), beat=beat)


def test_stages_and_failures_are_logged_with_times(tmp_path):
    p = _log(tmp_path)
    p.feed("🚀 banner\n\n[2/6] [PROBE] Asking the UNO Q's STM32 over SWD what its flash holds...\n")
    p.feed("    STM32U5 DEV_ID 0x482\n[3/6] [FLASH] Updating the firmware on 'unoq'")
    p.feed(" (/dev/ttyHS1) — auto-update.\n❌ [FLASH FAILED] Microcontroller firmware flash failed\n")
    p.end("exit 1")
    lines = (tmp_path / "progress.log").read_text().splitlines()
    body = [re.sub(r"^\S+ \+\s*[\d.]+s  ", "", l) for l in lines]
    assert body[0].startswith("START")
    assert any("[2/6] PROBE  Asking the UNO Q" in l for l in lines)
    assert any("[3/6] FLASH  Updating the firmware on 'unoq' (/dev/ttyHS1)" in l for l in lines)
    assert any("❌ [FLASH FAILED]" in l for l in lines)
    assert not any("DEV_ID" in l for l in lines)          # ordinary output is not copied
    assert body[-1].startswith("END exit 1")
    assert all(re.match(r"\d{4}-\d\d-\d\dT\d\d:\d\d:\d\dZ \+\s*\d+\.\ds  ", l) for l in lines)


def test_a_stage_that_lasts_gets_a_heartbeat(tmp_path):
    p = _log(tmp_path, beat=0.2)
    p.feed("[3/6] [FLASH] Updating the firmware\n")
    time.sleep(0.9)
    p.end("interrupted")
    text = (tmp_path / "progress.log").read_text()
    assert "still at [3/6] FLASH for" in text


def test_a_half_stage_like_4_7_counts_and_the_tee_passes_output_through(tmp_path, capsys):
    p = _log(tmp_path)
    tee = ocp._Tee(sys.stdout, p)
    tee.write("\n[4.7/6] [POSE] Starting from (0, 0)\n")
    p.end("exit 0")
    assert "[4.7/6] [POSE]" in capsys.readouterr().out
    assert "[4.7/6] POSE  Starting from (0, 0)" in (tmp_path / "progress.log").read_text()
