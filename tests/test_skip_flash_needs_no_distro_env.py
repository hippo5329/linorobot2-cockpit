"""A run that flashes nothing does not need a `<env>_<distro>` PlatformIO env.

UNO Q lyrical legs, 2026-10-05: the firmware is Zephyr, built outside PlatformIO
with its micro-ROS distro chosen in build.sh, and flashed before the run over SWD.
The pipeline still resolved `unoq` to `unoq_lyrical` and stopped every leg at
"No PlatformIO env 'unoq_lyrical'", although --skip-flash builds and flashes nothing.
The guard is for what this run builds or flashes, and must stay on that path.
"""
import os
import re
import sys

import pytest

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO_ROOT, "scripts"))

import one_click_pipeline as ocp  # noqa: E402

SRC = open(os.path.join(REPO_ROOT, "scripts", "one_click_pipeline.py")).read()


def test_the_guard_still_stops_a_lyrical_flash_with_no_env():
    with pytest.raises(SystemExit):
        ocp.resolve_pio_env("unoq", "lyrical")


def test_a_declared_lyrical_env_resolves():
    assert ocp.resolve_pio_env("pico2", "lyrical") == "pico2_lyrical"


def test_resolution_runs_only_when_this_run_flashes():
    call = SRC.index("pio_env = resolve_pio_env(pio_env, args.distro)")
    head = SRC[:call].rsplit("\n", 3)
    assert re.search(r"elif not args\.skip_flash:\s*$", head[-2] + "\n"), head[-3:]
    assert SRC.count("resolve_pio_env(") == 2      # the definition and this one call
