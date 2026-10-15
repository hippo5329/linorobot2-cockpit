"""--real-checks puts a simulated run through the real robot's safety steps.

The room check and its inching, the motion guard round Nav2 and test_acc, and the guard's
verdicts ran only on a real robot, so their faults reached one first: a false guard trip at
teardown, inching that stalled in a corner (2026-10-15). The simulation gate passes
--real-checks; a real robot has them anyway.
"""
import os

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC = open(os.path.join(ROOT, "scripts", "one_click_pipeline.py")).read()


def test_the_flag_exists_and_widens_is_real():
    assert '"--real-checks", action="store_true"' in SRC
    assert 'real_checks = is_real or bool(getattr(args, "real_checks", False))' in SRC


def test_every_safety_step_keys_on_real_checks():
    assert "args.clearance = clearance_real(params) if real_checks else 0.0" in SRC
    assert "if real_checks and nav2_ok:\n                guard_start()" in SRC
    assert "if real_checks and restarted[0]:\n                    guard_start()" in SRC
    body = SRC[SRC.index("def measure_and_restart(nav2_state):"):]
    body = body[:body.index("test_acc_phase(")]
    assert "if real_checks:" in body and "guard_stop()" in body
