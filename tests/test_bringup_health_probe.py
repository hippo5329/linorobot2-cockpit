"""The bringup health card must measure rates the way the pipeline's gate does.

It shelled out to `ros2 topic hz` per topic under a 3 s `timeout`. That CLI
first looks the publisher up in the graph to copy its QoS, and under the
cockpit's Fast DDS profile that lookup does not find the micro-ROS agent's
publishers: `ros2 topic list` showed /odom/unfiltered and /imu/data, `hz` on
either said "does not appear to be published yet", and the card drew two red
rows on a robot whose own 1-Click gate had just measured both at 50 Hz -- via
a direct sensor-data subscription, which needs no graph lookup. Measured
2026-09-22 on a bare Pico 2 launched from the UI.
"""
import ast
import os
import re
import sys

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
RUNNERS = os.path.join(REPO_ROOT, "web", "backend", "runners.py")


def _src():
    return open(RUNNERS, encoding="utf-8").read()


def _func(name, code_only=False):
    """Source of a top-level function; with code_only, its docstring removed,
    so a test about what the code DOES cannot be tripped by a comment that
    explains what it used to do."""
    src = _src()
    tree = ast.parse(src)
    for node in tree.body:
        if isinstance(node, ast.FunctionDef) and node.name == name:
            seg = ast.get_source_segment(src, node)
            doc = ast.get_docstring(node, clean=False)
            if code_only and doc:
                seg = seg.replace(doc, "", 1)
            return seg
    raise AssertionError(f"{name} not found")


def test_probe_no_longer_shells_out_to_topic_hz():
    body = _func("check_bringup_health", code_only=True)
    assert "ros2 topic hz" not in body, (
        "check_bringup_health went back to `ros2 topic hz`, whose graph lookup "
        "does not see micro-ROS publishers under the Fast DDS profile"
    )


def test_probe_subscribes_with_sensor_data_qos():
    assert "qos_profile_sensor_data" in _src(), (
        "the probe must subscribe with the sensor-data QoS so best-effort "
        "micro-ROS topics match"
    )


def test_every_health_topic_has_a_message_type():
    ns = {}
    src = _src()
    for name in ("BRINGUP_HEALTH_TOPICS", "BRINGUP_HEALTH_TYPES"):
        m = re.search(rf"^{name}\s*=\s*(\[.*?\]|\{{.*?\}})", src, re.S | re.M)
        assert m, name
        ns[name] = ast.literal_eval(m.group(1))
    keys = {t[0] for t in ns["BRINGUP_HEALTH_TOPICS"]}
    missing = keys - set(ns["BRINGUP_HEALTH_TYPES"])
    assert not missing, f"health topics with no message type, so never rated: {missing}"
    for typ in ns["BRINGUP_HEALTH_TYPES"].values():
        assert re.fullmatch(r"[a-z_]+/msg/[A-Z][A-Za-z0-9]+", typ), typ


def test_rate_from_stamps():
    sys.path.insert(0, os.path.join(REPO_ROOT, "web", "backend"))
    src = _func("rate_from_stamps")
    ns = {"List": list, "Optional": object}
    exec(src, ns)
    f = ns["rate_from_stamps"]
    assert f([]) is None and f([1.0]) is None
    assert abs(f([0.0, 0.02, 0.04, 0.06, 0.08]) - 50.0) < 1e-6
    assert f([5.0, 5.0]) is None                      # zero span, no divide
