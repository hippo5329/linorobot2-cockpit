"""The recurring error_code=102 is a SLAM stall, and the message hides it.

    [transformPoseInTargetFrame]: Extrapolation Error looking up target frame:
    Lookup would require extrapolation into the future.  Requested time
    1790147334.319486 but the latest data is at time 1790147334.301742

Eighteen milliseconds. It reads like the request landed between two 50 Hz
publishes, and that reading cost two days: the reference configs carry the
archaeology of planner_server.transform_tolerance going 0.3 -> 0.5 -> 1.0, each
raise commented with this exact message, each buying nothing.

The message is not a race. nav2_util::transformPoseInTargetFrame calls
tf_buffer.transform(input, target, timeout), and tf2 reaches
ExtrapolationException from that call only AFTER canTransform has waited the
whole timeout -- had it merely run out of patience it would be TimeoutException,
which that function catches separately and reports differently. The tolerance in
force is the local costmap's (controller_server.cpp: transform_tolerance_ =
costmap_ros_->getTransformTolerance()), 0.5 s in our configs.

So map->odom did not arrive for at least half a second, from a publisher running
at 50 Hz. The 18 ms is the age of the last stamp before the stall, not the gap.

Measuring the longest gap is therefore the whole diagnosis, and it has to be
sampled rather than awaited: a transform that has stopped arriving fires no
callback to notice it by.
"""
import ast
import os

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
GOAL = os.path.join(ROOT, "scripts", "test_nav2_goal.py")


def _src():
    with open(GOAL, encoding="utf-8") as fh:
        return fh.read()


def _func(name):
    tree = ast.parse(_src())
    for node in ast.walk(tree):
        if isinstance(node, ast.FunctionDef) and node.name == name:
            return node
    raise AssertionError(f"{name}() is gone from {GOAL}")


def test_the_gap_is_sampled_inside_the_leg_loop():
    """Not in a callback: silence is the thing being measured."""
    src = _src()
    assert "_sample_map_odom(node)" in src
    # it must sit in the per-leg spin loop, not only at leg end
    loop = src[src.index("while time.time() - t0 < timeout:"):]
    assert "_sample_map_odom" in loop[:600], "not sampled while the leg runs"


def test_a_failed_leg_reports_the_worst_gap():
    src = _src()
    assert "_map_odom_gap_note(node)" in src


def test_the_note_names_the_tolerance_when_the_gap_reaches_it():
    """A 30 ms gap is normal jitter; 500 ms is the fault, and the line has to
    say which one it saw or the next reader repeats the tolerance-raising."""
    fn = _func("_map_odom_gap_note")
    body = ast.get_source_segment(_src(), fn)
    assert "transform tolerance" in body
    assert "SLAM stalled" in body
    assert "0.5" in body or "tolerance: float = 0.5" in body


def test_the_gap_tracks_stamps_not_wall_clock():
    """Wall-clock age would also grow while the robot is simply idle between
    legs; the stamp interval is what says the publisher stopped."""
    fn = _func("_sample_map_odom")
    body = ast.get_source_segment(_src(), fn)
    assert "header.stamp" in body
    assert "map_odom_max_gap" in body


def test_the_gap_logic_is_monotonic():
    """Replayed against a stalled publisher, the note must fire; against a
    healthy 50 Hz one it must stay silent.

    Module-level and taking `node`, like _why/_gap/_where/_runaway: a method
    would break every test that drives this file with a FakeNode, which is
    exactly how it broke when first written.
    """
    class Node:
        map_odom_max_gap = 0.0
        _last = None
    fn = _func("_map_odom_gap_note")
    src = ast.get_source_segment(_src(), fn)
    ns = {}
    exec(compile(ast.Module([fn], []), GOAL, "exec"), ns)
    note = ns["_map_odom_gap_note"]

    healthy = Node(); healthy.map_odom_max_gap = 0.021
    out = note(healthy)
    assert "transform tolerance" not in out
    assert "kept up" in out, out          # below the sampling resolution

    stalled = Node(); stalled.map_odom_max_gap = 0.63
    out = note(stalled)
    assert "630 ms" in out and "SLAM stalled" in out, out

    quiet = Node(); quiet.map_odom_max_gap = 0.0
    assert note(quiet) == ""


def test_the_sampler_is_rate_limited_so_it_cannot_perturb_the_leg():
    """A diagnostic in the tester's hot loop is not free.

    Written without a limit, this ran a TF lookup between the tester and every
    callback it processes. The 2wd slice went 10/10 -> 6/10 on identical
    firmware and images the first time it ran, two legs driving out of the
    room. Causal or coincidental, it stopped being answerable -- which is the
    failure: a diagnostic that can perturb what it measures poisons every
    result after it.
    """
    body = ast.get_source_segment(_src(), _func("_sample_map_odom"))
    assert "_last_map_odom_sample" in body, "the sampler is unlimited"
    assert "0.1" in body, "no sampling interval"


def test_a_gap_under_the_sampling_resolution_is_not_reported_as_one():
    """At 10 Hz a 30 ms gap cannot be seen; reporting one as measured would
    invent precision the sampler does not have."""
    fn = _func("_map_odom_gap_note")
    ns = {}
    exec(compile(ast.Module([fn], []), GOAL, "exec"), ns)
    note = ns["_map_odom_gap_note"]

    class N:
        map_odom_max_gap = 0.03
    assert "kept up" in note(N()), note(N())
    N.map_odom_max_gap = 0.63
    assert "SLAM stalled" in note(N())


def test_a_passing_run_reports_its_worst_gap_too():
    """A failing leg saying 601 ms only says the stall happened. How close a
    HEALTHY run comes to the 0.5 s tolerance is what says whether the margin is
    comfortable or whether every green leg was one hiccup from red -- and that
    decides whether raising the tolerance is evidence-based or another blind
    raise like the three already in the reference configs."""
    src = _src()
    reached = src[src.index('f"NAV2 GOAL REACHED {n}/{n} legs'):]
    assert "_map_odom_gap_note(node)" in reached[:600], "the passing line drops the gap"
