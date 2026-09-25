"""The Start 1-Click passes no --goal-timeout. It inherited 25 s for legs that
must arrive (round trips default to 4, which forces --require-goal) and failed
every run mid-detour, 3.7 m from the goal. The default must follow what the
leg has to prove."""
import os
import re
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))

import one_click_pipeline as ocp  # noqa: E402

SRC = open(os.path.join(os.path.dirname(__file__), "..", "scripts",
                        "one_click_pipeline.py")).read()


def test_a_leg_that_must_arrive_gets_the_gate_timeout():
    assert ocp.goal_timeout_default(True, 0) == 180
    assert ocp.goal_timeout_default(False, 4) == 180


def test_a_plan_only_leg_keeps_the_short_timeout():
    assert ocp.goal_timeout_default(False, 0) == 25


def test_the_argument_has_no_fixed_default():
    m = re.search(r'"--goal-timeout", type=int, default=(\w+)', SRC)
    assert m and m.group(1) == "None"
    assert "args.goal_timeout = goal_timeout_default(" in SRC


def test_round_trips_default_is_still_a_must_arrive_run():
    m = re.search(r'"--goal-round-trips", type=int, default=(\d+)', SRC)
    assert m and int(m.group(1)) > 0
