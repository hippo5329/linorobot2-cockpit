"""A run's background steps stop in the reverse of their start order.

The motion guard starts after the bringup whose LiDAR it watches. Stopped in start order, the
bringup went first and the guard, still watching, latched "/scan stopped: the last one is
2.0 s old" two seconds after a real robot's run had passed every leg (2026-10-15).
"""
import importlib.util
import os

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _load():
    spec = importlib.util.spec_from_file_location("ocp_teardown", os.path.join(ROOT, "scripts", "one_click_pipeline.py"))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def test_the_last_started_stops_first():
    ocp = _load()
    bringup, slam, nav2, guard = object(), object(), object(), object()
    stopped = []
    ocp.stop_all_bg([bringup, slam, nav2, guard], stop=stopped.append)
    assert stopped == [guard, nav2, slam, bringup]


def test_kept_processes_are_left_running():
    ocp = _load()
    bringup, nav2, guard = object(), object(), object()
    stopped = []
    ocp.stop_all_bg([bringup, nav2, guard], kept={id(bringup), id(nav2)}, stop=stopped.append)
    assert stopped == [guard]


def test_main_tears_down_through_it():
    src = open(os.path.join(ROOT, "scripts", "one_click_pipeline.py")).read()
    finally_part = src[src.rindex("Cleaning up background ROS 2 launch processes"):]
    assert "stop_all_bg(bg_processes, kept)" in finally_part.split("open_log_files")[0]
