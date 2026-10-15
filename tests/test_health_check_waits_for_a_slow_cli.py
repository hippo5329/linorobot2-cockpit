"""The bringup health check gives the ros2 CLI time to start on a slow robot computer.

On an Arduino UNO Q a login shell, the ROS setup files and `ros2 topic list` took 5-14 s; the
listing's limit was the measuring window + 2 s (6 s), so a healthy MC200 read "Could not query
ROS 2 graph" in its own web UI (2026-10-15).
"""
import os

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def test_the_listing_has_its_own_generous_limit():
    src = open(os.path.join(ROOT, "web", "backend", "runners.py")).read()
    assert "timeout=max(timeout + 2, LIST_TIMEOUT_S)" in src
    val = float(src.split("LIST_TIMEOUT_S = ", 1)[1].split()[0])
    assert val >= 20.0
