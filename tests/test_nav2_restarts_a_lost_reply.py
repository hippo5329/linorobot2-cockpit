"""A Nav2 whose lifecycle reply was lost is restarted, once; a Nav2 that reports a failure is not.

Gate 20260926-disp8, RP2040 jazzy 2wd, attempt 1: the drive suite 8/8, a map,
then lifecycle_manager configured route_server and rmw logged "failed to send
response to /route_server/change_state (timeout)". The manager waited for that
reply for the whole 240 s window and the leg failed with "the stack never
activated". The line is in 26 of 1683 kept Nav2 starts, none of which activated.
"""
import os
import sys
import time

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO_ROOT, "scripts"))

import one_click_pipeline as ocp  # noqa: E402

LOST = ("[route_server-4] [WARN] [1790374202.779713236] [route_server.rclcpp]: failed to send "
        "response to /route_server/change_state (timeout): client will not receive response, "
        "at ./src/rmw_response.cpp:153, at ./src/rcl/service.c:400\n")
REPORTED = "[lifecycle_manager-11] [ERROR] [1.0] [lifecycle_manager_navigation]: Failed to bring up all requested nodes. Aborting bringup.\n"
ACTIVE = "[lifecycle_manager-11] [INFO] [1.0] [lifecycle_manager_navigation]: Managed nodes are active\n"


def test_a_lost_reply_is_reported_at_once_not_after_the_window(monkeypatch, tmp_path):
    monkeypatch.setattr(ocp, "LOG_DIR", str(tmp_path))
    (tmp_path / "nav2.log").write_text("[lifecycle_manager]: Configuring route_server\n" + LOST)
    t0 = time.time()
    ok, detail = ocp.wait_for_nav2_activation(timeout_sec=60)
    assert not ok and time.time() - t0 < 5, "the lost reply must end the wait now"
    assert "route_server" in detail and "change_state" in detail, detail


def test_a_verdict_wins_over_the_line(monkeypatch, tmp_path):
    """An activated stack is active, whatever was logged on the way."""
    monkeypatch.setattr(ocp, "LOG_DIR", str(tmp_path))
    (tmp_path / "nav2.log").write_text(LOST + ACTIVE)
    assert ocp.wait_for_nav2_activation(timeout_sec=5)[0] is True


class Launches:
    """One scripted nav2 log per launch; the pipeline's process calls stubbed."""

    def __init__(self, monkeypatch, tmp_path, logs):
        self.logs, self.launched, self.stopped = list(logs), [], []
        self.tmp = tmp_path
        monkeypatch.setattr(ocp, "LOG_DIR", str(tmp_path))
        monkeypatch.setattr(ocp, "launch_bg", self.launch_bg)
        monkeypatch.setattr(ocp, "_stop_group_and_wait", lambda proc: self.stopped.append(proc))
        real_wait = ocp.wait_for_nav2_activation
        monkeypatch.setattr(ocp, "wait_for_nav2_activation",
                            lambda log_tag="nav2": real_wait(timeout_sec=2, log_tag=log_tag))

    def launch_bg(self, cmd, log_tag="launch", distro="jazzy"):
        (self.tmp / f"{log_tag}.log").write_text(self.logs[len(self.launched)])
        self.launched.append(log_tag)
        return object()


def run(g):
    bg, stack = [], []
    ok, detail, log = ocp.start_nav2("ros2 launch x", "jazzy", bg, stack)
    return ok, detail, log, bg, stack


def test_a_lost_reply_is_restarted_once(monkeypatch, tmp_path):
    g = Launches(monkeypatch, tmp_path, [LOST, ACTIVE])
    ok, _, log, bg, stack = run(g)
    assert ok and g.launched == ["nav2", "nav2_retry"] and len(g.stopped) == 1
    assert log.endswith("nav2_retry.log"), "the reports must read the run that is up"
    assert len(bg) == 1 and [t for t, _ in stack] == ["nav2"]


def test_a_silent_hang_is_restarted_too(monkeypatch, tmp_path):
    g = Launches(monkeypatch, tmp_path, ["[lifecycle_manager]: Configuring planner_server\n", ACTIVE])
    ok, _, _, _, _ = run(g)
    assert ok and g.launched == ["nav2", "nav2_retry"]


def test_a_reported_failure_is_not_hidden_by_a_restart(monkeypatch, tmp_path):
    g = Launches(monkeypatch, tmp_path, [REPORTED])
    ok, _, log, _, _ = run(g)
    assert not ok and g.launched == ["nav2"] and g.stopped == []
    assert log.endswith("nav2.log")


def test_the_restart_happens_once(monkeypatch, tmp_path):
    g = Launches(monkeypatch, tmp_path, [LOST, LOST])
    ok, _, _, _, _ = run(g)
    assert not ok and g.launched == ["nav2", "nav2_retry"] and len(g.stopped) == 1
