"""The Topic monitor (imported from the Robot Config Engine's viewer, user 2026-10-06).

A rate is only ever a measurement: the engine showed one guessed from the topic's name, and
ran the ros2 CLI, which cannot see the micro-ROS publishers under the cockpit's Fast DDS
profile. Both endpoints go through scripts/topic_stream.py's rclpy subscription.
"""
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "scripts"))
sys.path.insert(0, os.path.join(ROOT, "web", "backend"))


def test_the_rate_is_the_arrivals():
    import importlib.util
    spec = importlib.util.spec_from_file_location("ts_src", os.path.join(ROOT, "scripts", "topic_stream.py"))
    src = open(spec.origin).read()
    ns = {}
    # rate_of without rclpy: the function alone
    start = src.index("def rate_of(")
    exec(src[start:src.index("\n\n\nclass ", start)], ns)
    rate_of = ns["rate_of"]
    assert rate_of([0.0]) == (None, None, None)
    hz, gmin, gmax = rate_of([0.0, 0.02, 0.04, 0.06, 0.26])
    assert abs(hz - 4 / 0.26) < 1e-9 and abs(gmin - 0.02) < 1e-9 and abs(gmax - 0.2) < 1e-9


def test_no_endpoint_runs_the_ros2_cli_for_a_rate_or_an_echo():
    src = open(os.path.join(ROOT, "web", "backend", "routes_hardware.py")).read()
    import re
    assert not re.search(r"ros2 topic (hz|echo) [{$-]", src), "a command string runs the CLI"
    assert '"/api/ros2/stream"' in src and "topic_stream.py" in src


def test_the_stream_is_authorised_by_a_ticket():
    import access
    assert access.is_stream_path("/api/ros2/stream")


def test_the_page_shows_no_guessed_rate():
    js = open(os.path.join(ROOT, "web", "frontend", "app-agent-bringup.js")).read()
    html = open(os.path.join(ROOT, "web", "frontend", "index.html")).read()
    assert 'id="card-topic-monitor"' in html
    assert "/api/ros2/stream?" in js and "/api/ros2/hz_single?" in js
    assert '"50.0 Hz"' not in js     # the engine's placeholder rates


def test_the_monitor_shows_the_mcus_topics_only():
    """User, 2026-10-06: "we need only the mcu's topic view"."""
    js = open(os.path.join(ROOT, "web", "frontend", "app-agent-bringup.js")).read()
    assert "/api/ros2/mcu_topics?" in js and "/api/ros2/topics?" not in js
    src = open(os.path.join(ROOT, "scripts", "topic_stream.py")).read()
    env = open(os.path.join(ROOT, "scripts", "mcu_env.py")).read()
    assert 'MCU_NODE_SUFFIX = "_base_node"' in src and 'f"{robot_name}_base_node"' in env
