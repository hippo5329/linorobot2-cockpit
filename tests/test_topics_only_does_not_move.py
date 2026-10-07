"""A --topics-only --no-drive-test run commands no motion.

On 2026-10-07 the first real robot (a GenDrv with a real LD19, no robot computer)
was flashed and checked with `--topics-only --no-drive-test`, and its wheels
turned for several seconds: the "mapping rotation" -- `ros2 topic pub /cmd_vel`
at 0.4 rad/s for --explore-sec -- ran anyway. Its `else:` had come to hang off
`if args.drive_test:` instead of the step-6 chain it describes (topics-only /
Nav2 / plain exploration spin), so every run WITHOUT the drive suite spun the
base, and a run with it never did the spin it meant to.

The rotation belongs to the step-6 chain: Nav2 skipped by --no-nav2 or for want
of a LiDAR, with SLAM running to map what the spin shows. --topics-only runs no
SLAM, so it never spins; --no-drive-test skips the drive suite. Together they
command nothing.
"""
import ast
import os

SRC = open(os.path.join(os.path.dirname(__file__), "..", "scripts",
                        "one_click_pipeline.py")).read()
TREE = ast.parse(SRC)

PARENT = {}
for node in ast.walk(TREE):
    for child in ast.iter_child_nodes(node):
        PARENT[child] = node


def _is_attr(test, name):
    return isinstance(test, ast.Attribute) and test.attr == name \
        and isinstance(test.value, ast.Name) and test.value.id == "args"


def _publishes_drive_cmd(node):
    return isinstance(node, ast.Call) and getattr(node.func, "id", None) == "launch_bg" \
        and node.args and isinstance(node.args[0], ast.Name) and node.args[0].id == "drive_cmd"


def _guards(node):
    """(If node, True when `node` is in its body, False when in its orelse) up the tree."""
    out = []
    while node in PARENT:
        parent = PARENT[node]
        if isinstance(parent, ast.If):
            out.append((parent, node in parent.body))
        node = parent
    return out


def _reachable_with(guards, topics_only, drive_test):
    for if_node, in_body in guards:
        if _is_attr(if_node.test, "topics_only") and in_body != topics_only:
            return False
        if _is_attr(if_node.test, "drive_test") and in_body != drive_test:
            return False
    return True


def test_some_call_publishes_the_spin():
    assert [n for n in ast.walk(TREE) if _publishes_drive_cmd(n)], \
        "the test lost its subject: no launch_bg(drive_cmd, ...) left"


def test_no_spin_on_a_topics_only_run_without_the_drive_suite():
    for call in [n for n in ast.walk(TREE) if _publishes_drive_cmd(n)]:
        assert not _reachable_with(_guards(call), topics_only=True, drive_test=False), \
            f"line {call.lineno}: /cmd_vel is published on a --topics-only --no-drive-test run"


def test_the_drive_suite_has_no_else():
    ifs = [n for n in ast.walk(TREE) if isinstance(n, ast.If) and _is_attr(n.test, "drive_test")]
    assert ifs
    for n in ifs:
        assert not n.orelse, f"line {n.lineno}: `if args.drive_test:` has an else -- what runs when the suite is skipped?"


def test_the_mapping_spin_is_the_step_six_fallback():
    spin = [n for n in ast.walk(TREE) if isinstance(n, ast.Constant) and isinstance(n.value, str)
            and "Simulating a mapping rotation" in n.value]
    assert len(spin) == 1
    guards = _guards(spin[0])
    top = [g for g in guards if _is_attr(g[0].test, "topics_only")]
    assert top and top[0][1] is False, "the spin must sit in the else of `if args.topics_only:`"
