"""The bringup health check gives the ROS start-up time on a slow robot computer, and no more.

On an Arduino UNO Q a login shell, the ROS setup files and `ros2 topic list` took 5-14 s; the
listing's limit was the measuring window + 2 s (6 s), so a healthy MC200 read "Could not query
ROS 2 graph" in its own web UI (2026-10-15). The limit then went to 25 s for the listing, with the
measuring probe after it, and a second check took more than 30 s (walkthrough 2026-10-16). The
probe now reads the graph itself: one start-up, one window, one limit.
"""
import os

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _src():
    return open(os.path.join(ROOT, "web", "backend", "runners.py")).read()


def test_the_startup_has_a_generous_allowance():
    val = float(_src().split("LIST_TIMEOUT_S = ", 1)[1].split()[0])
    assert val >= 20.0


def test_one_process_lists_and_measures_within_one_bound():
    src = _src()
    body = src[src.index("def check_bringup_health"):]
    body = body[:body.index("\ndef ")] if "\ndef " in body else body
    code = body.split('"""', 2)[2]          # past the docstring
    assert "ros2 topic list" not in code, "a separate CLI listing doubles the start-up"
    assert code.count("subprocess.run(") == 1
    assert "timeout=window + LIST_TIMEOUT_S" in code
    window_max = 8.0
    assert "min(8.0, timeout + 4.0)" in code
    assert window_max + float(src.split("LIST_TIMEOUT_S = ", 1)[1].split()[0]) <= 30.0
    assert "get_topic_names_and_types" in src
