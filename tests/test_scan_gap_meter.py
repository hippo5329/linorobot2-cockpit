"""map->odom standing still names a symptom; it cannot name the cause.

With restamp_tf false, slam_toolbox stamps map->odom from the scan, so the
map->odom meter and the /scan stamp are the SAME number read at different
points. It says "no scan for 1.2 s" and stops there -- and "no scan" has two
entirely different causes that call for opposite fixes:

  * the board stopped producing them. The scan STAMPS gap. Firmware loop,
    emulator, micro-ROS executor. Nothing on the host touches it.
  * the board produced them on time and they arrived in a burst. The stamps
    stay 100 ms apart and the ARRIVALS gap. Transport, agent, DDS.

The first run to carry this meter already had the evidence that the obvious
suspect is wrong: on 2026-09-23 the GenDrv jazzy skid_steer leg gapped 1200 ms
on the SERIAL transport while the Wi-Fi leg on the same board, same image, same
run gapped 200 ms. Wi-Fi was the standing explanation for scan latency. It is
not the explanation for these.
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
    fn = _func("_scan_gap_note")
    ns = {}
    exec(compile(ast.Module([fn], []), GOAL, "exec"), ns)
    return ns["_scan_gap_note"]


class N:
    scan_count = 100
    scan_max_stamp_gap = 0.0
    scan_max_arrival_gap = 0.0


def test_the_tester_subscribes_to_scan_itself():
    """Second-hand through map->odom is how one measurement had to stand for
    two causes."""
    src = _src()
    assert "LaserScan" in src
    assert '"/scan", self._scan_cb' in src


def test_the_meter_is_a_callback_not_a_lookup_in_the_leg_loop():
    """The previous diagnostic put a TF lookup between the tester and every
    callback it processes and took the 2wd slice from 10/10 to 6/10. This one
    does arithmetic in a subscription and must stay that cheap."""
    body = ast.get_source_segment(_src(), _func("_scan_cb"))
    for forbidden in ("lookup_transform", "spin_once", "open(", "print("):
        assert forbidden not in body, f"{forbidden} in the scan callback"


def test_both_intervals_are_recorded():
    body = ast.get_source_segment(_src(), _func("_scan_cb"))
    assert "header.stamp" in body, "no stamp interval: cannot see a production stall"
    assert "time.time()" in body, "no arrival interval: cannot see a delivery stall"


def test_a_steady_scan_says_so_rather_than_saying_nothing():
    n = N()
    n.scan_max_stamp_gap = 0.104
    n.scan_max_arrival_gap = 0.11
    out = _note()(n)
    assert "steady" in out and "100 scans" in out, out


def test_a_stamp_gap_is_blamed_upstream_of_the_wire():
    """Stamps 1.2 s apart mean there was nothing in flight to delay."""
    n = N()
    n.scan_max_stamp_gap = 1.2
    n.scan_max_arrival_gap = 1.25
    out = _note()(n)
    assert "1200 ms" in out, out
    assert "upstream of the wire" in out, out
    assert "transport" not in out.split("--")[1], out


def test_on_time_but_late_names_the_ambiguity_rather_than_a_culprit():
    """Stamps at nominal, arrivals bunched: the board did its job -- but this
    measurement cannot say who delayed them.

    The arrival interval is timed in the TESTER, whose executor is
    single-threaded and spins with a 0.2 s timeout while running the leg logic
    and a 10 Hz TF lookup. "Produced on time and delivered late, so this is the
    transport" was the first wording, and on the 2026-09-23 run it printed that
    verdict on every leg -- against arrival gaps of 400 ms that this process
    could have caused entirely by itself. A diagnostic that names the wrong
    subsystem confidently is worse than one that says it does not know.
    """
    n = N()
    n.scan_max_stamp_gap = 0.1
    n.scan_max_arrival_gap = 1.2
    out = _note()(n)
    assert "cannot" in out and "told apart" in out, out
    assert "so this is the transport" not in out, out
    assert "executor" in out, "the reader is not told why the number is ambiguous"


def test_no_scan_at_all_is_not_reported_as_steady():
    """An empty meter reads as a perfect run, which is the one reading that
    must never be silent -- a leg whose /scan never reached the tester is the
    strongest evidence there is, not the absence of evidence."""
    n = N()
    n.scan_count = 0
    assert "no /scan seen" in _note()(n)


def test_the_note_reaches_both_verdict_lines():
    src = _src()
    reached = src[src.index('f"NAV2 GOAL REACHED {n}/{n} legs'):]
    assert "_scan_gap_note(node)" in reached[:700], "the passing line drops it"
    assert src.count("_scan_gap_note(node)") >= 2, "the failing line drops it"


def test_an_unsynced_clock_does_not_register_as_one_huge_gap():
    """micro-ROS stamps 0 until the agent's time sync lands; taking that as the
    previous stamp turns the first real scan into a 56-year interval."""
    body = ast.get_source_segment(_src(), _func("_scan_cb"))
    assert "stamp > 0.0" in body, "a zero stamp is accepted as a baseline"
