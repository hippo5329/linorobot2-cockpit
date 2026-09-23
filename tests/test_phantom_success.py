"""Nav2 can report a goal SUCCEEDED with the robot nowhere near it.

reached_goal() has refused to believe the status over the position since the
Yahboom leg 2/8 of 2026-09-22 -- "reached in 2 s, closest 2.830 m" against a
0.40 m tolerance. What it did NOT do is act on the disagreement: the leg went
on spinning for the whole 180 s window waiting for an arrival the stack had
already stopped driving towards, and then failed the run.

Twice in the 2026-09-23 mecanum slice, both on lyrical:

    GenDrv serial leg 4/8  SUCCEEDED 2.9 s after dispatch, 2.84 m away,
                           0.114 m travelled, NO nav2 complaint at all
    YB-EET01     leg 3/8   the same, 13 s, after error_code 105

Both boards then scored 8/8 on the drive suite in the same container, so the
bases drove. Every goal in this tester goes out one MILLISECOND after the
previous leg's cancel returns a terminal status -- on the passing legs too --
which is the shape of a race against bt_navigator's tree still halting, not of
a robot that cannot move.

So the tester now confirms the disagreement is not its own pose lagging, stops
waiting, and asks once more per run. The retry is the experiment: if the second
ask drives, the first answer was a phantom; if it does not, the leg fails on
its own evidence. Neither outcome is hidden -- both print.
"""
import ast
import os

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
GOAL = os.path.join(ROOT, "scripts", "test_nav2_goal.py")


def _src():
    with open(GOAL, encoding="utf-8") as fh:
        return fh.read()


def _func(name):
    for node in ast.walk(ast.parse(_src())):
        if isinstance(node, ast.FunctionDef) and node.name == name:
            return node
    raise AssertionError(f"{name}() is gone from {GOAL}")


def _body(name):
    return ast.get_source_segment(_src(), _func(name))


def test_a_phantom_success_ends_the_wait():
    """Status 4 with the robot elsewhere must leave the loop, not sit out the
    window. 180 s per occurrence, and the leg fails either way."""
    body = _body("await_arrival")
    assert "node.goal_status == 4" in body, "SUCCEEDED is not distinguished from running"
    assert "PHANTOM_CONFIRM_SEC" in body


def test_the_disagreement_is_confirmed_before_it_is_believed():
    """The mirror of the bug: a real arrival whose last /odom has not landed
    yet would be failed if the first sample decided it."""
    body = _body("await_arrival")
    assert "phantom_since" in body
    src = _src()
    assert "PHANTOM_CONFIRM_SEC = " in src
    ns = {}
    exec(compile(ast.Module([n for n in ast.parse(src).body
                             if isinstance(n, ast.Assign)
                             and getattr(n.targets[0], "id", "") == "PHANTOM_CONFIRM_SEC"], []),
                 GOAL, "exec"), ns)
    assert 0.5 <= ns["PHANTOM_CONFIRM_SEC"] <= 10.0


def test_the_leg_is_asked_once_more_and_the_answer_is_printed():
    src = _src()
    assert "retried_phantom" in src
    assert "was a phantom" in src, "a successful retry must say the first answer was false"
    # once per RUN, like the startup retry -- a systematic failure must not be
    # retried thirty times into a green run
    assert src.count("retried_phantom = True") == 1


def test_the_retry_cannot_pass_a_leg_the_robot_did_not_drive():
    """The retry re-dispatches and waits on the SAME arrival test. If it were
    allowed to pass on the status alone it would launder the very fault it is
    diagnosing."""
    body = _body("await_arrival")
    assert "reached_goal()" in body
    assert "wall_path_ok()" in body
    # the only `return True` in the loop is the full arrival test
    tree = ast.parse(body.strip())
    returns = [n for n in ast.walk(tree)
               if isinstance(n, ast.Return) and isinstance(n.value, ast.Constant)
               and n.value.value is True]
    assert len(returns) == 1, "more than one way to call a leg arrived"


def test_the_next_goal_is_not_dispatched_into_a_halting_tree():
    """The race itself: a terminal status is bt_navigator answering, not
    bt_navigator finished."""
    src = _src()
    assert "LEG_SETTLE_SEC" in src
    ns = {}
    exec(compile(ast.Module([n for n in ast.parse(src).body
                             if isinstance(n, ast.Assign)
                             and getattr(n.targets[0], "id", "") == "LEG_SETTLE_SEC"], []),
                 GOAL, "exec"), ns)
    assert 0.0 < ns["LEG_SETTLE_SEC"] <= 2.0, "a settle long enough to matter to a 17 s leg"
    run = _body("run_legs")
    assert run.index("close_goal(node, GOAL_CLOSE_SEC)") < run.index("LEG_SETTLE_SEC"), \
        "the settle must follow the cancel, not precede it"


def test_one_wait_loop_serves_every_dispatch():
    """The first attempt and both retries must be measured by the same
    instrument. The retry's private copy sampled no map->odom gap and watched
    for no runaway, so a retried leg was silently measured more weakly."""
    run = _body("run_legs")
    assert run.count("await_arrival(t0)") == 3, "a dispatch is waiting on its own loop again"
    body = _body("await_arrival")
    # exactly one wait loop in the whole function, and it is await_arrival's own
    assert run.count("while time.time() - t0 < timeout:") == 1, "an inlined copy is back"
    assert body.count("while time.time() - t0 < timeout:") == 1
    assert "_sample_map_odom(node)" in body
    assert "_runaway(node)" in body
