"""How far SLAM's correction wanders, measured before it causes an abort.

map->odom is SLAM's correction to the wheels. On a simulated robot in a
10 x 6 m room it should stay small: the sim LiDAR sees a room that matches the
wheels exactly, so there is nothing for the correction to correct.

On the 2026-09-23 mecanum slice (stamp 20260923-170733) a leg aborted with

    error_code=203, error_msg='GridBasedplugin failed to plan from (3.80, -3.11)

while the drive suite moments later showed the base at (+0.01, -0.00) driving
8/8. So the base was at the origin and the correction had walked about 4.9 m,
taking the robot's MAP pose outside a 6 m-tall room. The planner refused a start
it could not see. Nothing measured the correction, so this was attributable only
because the drive suite happened to print the base's position straight after.

The lookup in _sample_map_odom already had the translation and discarded it.
Reported on passing legs too, for the same reason the stamp gap is: a leg that
passes with a 3 m correction is one leg from that abort, and a gate that only
shows the number after it has failed cannot see it coming.
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


def _note():
    fn = _func("_map_odom_offset_note")
    ns = {}
    exec(compile(ast.Module([fn], []), GOAL, "exec"), ns)
    return ns["_map_odom_offset_note"]


class N:
    map_odom_max_offset = 0.0


def test_the_sampler_keeps_the_translation_it_already_looked_up():
    body = ast.get_source_segment(_src(), _func("_sample_map_odom"))
    assert "tr.transform.translation" in body, "the correction's size is discarded again"
    assert "map_odom_max_offset" in body


def test_a_small_correction_is_reported_without_alarm():
    n = N()
    n.map_odom_max_offset = 0.08
    out = _note()(n)
    assert "0.08 m" in out
    assert "outside" not in out


def test_a_correction_bigger_than_the_room_says_what_it_implies():
    """4.9 m in a room whose y half-extent is 2.69 m: the map pose can be outside
    a room the base is still inside, which is the 203."""
    n = N()
    n.map_odom_max_offset = 4.9
    out = _note()(n)
    assert "4.90 m" in out
    assert "outside a room the BASE is still inside" in out
    assert "2.69" in out


def test_nothing_measured_is_not_reported_as_zero():
    """A leg with no TF at all must not read as a leg with a perfect correction."""
    assert _note()(N()) == ""


def test_it_reaches_the_passing_line_too():
    src = _src()
    reached = src[src.index('f"NAV2 GOAL REACHED {n}/{n} legs'):]
    assert "_map_odom_offset_note(node)" in reached[:800], \
        "the passing line drops the correction size"
    assert src.count("_map_odom_offset_note(node)") >= 2, "the failing line drops it"
