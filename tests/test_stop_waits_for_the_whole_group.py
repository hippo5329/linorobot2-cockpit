"""Stopping a step stops its whole process group, not just `ros2 launch`.

On SIGINT `ros2 launch` exits within a second, while Nav2's composed container,
in the same group, is still tearing down eleven servers. stop_bg stopped
escalating once the launch had exited, and robot_stack.is_alive called a
group with a zombie leader dead, so the container outlived the run as an orphan.
Measured 2026-09-26 on lyrical: after a run with --shutdown-when-done, a
nav2_container from that run was still alive and shared a node name with the
next run's.

The stand-in: a leader that exits on SIGINT, and a child that ignores SIGINT and
SIGTERM, as a container stuck in shutdown does in effect.
"""
import os
import subprocess
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))

import one_click_pipeline as ocp  # noqa: E402
import robot_stack  # noqa: E402

LEADER = 'trap "exit 0" INT; (trap "" INT TERM; exec sleep 60) & wait'


def _group():
    proc = subprocess.Popen(["sh", "-c", LEADER], start_new_session=True)
    pgid = os.getpgid(proc.pid)
    deadline = time.time() + 5
    while len(robot_stack.live_members(pgid)) < 2 and time.time() < deadline:
        time.sleep(0.05)
    assert len(robot_stack.live_members(pgid)) == 2
    return proc, pgid


def test_stop_bg_escalates_until_the_group_is_empty():
    proc, pgid = _group()
    ocp.stop_bg(proc, step_s=0.5)
    assert robot_stack.live_members(pgid) == [], "a member that ignores SIGINT and SIGTERM survived"


def test_a_group_with_a_dead_leader_is_still_alive(tmp_path):
    proc, pgid = _group()
    robot_stack.record("nav2", proc.pid, state_dir=str(tmp_path))
    os.kill(proc.pid, 2)               # the leader leaves, the child stays
    deadline = time.time() + 5
    while proc.pid in robot_stack.live_members(pgid) and time.time() < deadline:
        time.sleep(0.05)
    (entry,) = robot_stack.load(str(tmp_path))
    assert robot_stack.is_alive(entry), "the child is still running: the stack is alive"
    robot_stack.stop(state_dir=str(tmp_path))
    assert robot_stack.live_members(pgid) == []
    proc.poll()
