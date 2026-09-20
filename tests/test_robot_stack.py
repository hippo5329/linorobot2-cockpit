"""A 1-Click run leaves a robot RUNNING, and Stop can still reach it.

The pipeline used to tear bringup, SLAM and Nav2 down in its `finally` block
the moment it finished. That is right for an automated run and wrong for a
person: they press Start 1-Click to GET a robot, and were handed one that had
just been switched off -- nothing on /scan, nothing to drive.

Keeping the stack alive means the pipeline EXITS while its children keep
running, so something has to remember them or the Stop buttons have nothing to
signal. That record is scripts/robot_stack.py, in the shared state directory,
because the pipeline runs as container-root and the backend as the container
user (see cockpit_paths.state_dir).

Process groups, never names: every launch is started with os.setsid(), so one
signal reaches `ros2 launch` and everything it spawned, and a pgid cannot match
the wrong process the way a name can.
"""
import os
import signal
import subprocess
import sys
import time

import pytest

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO_ROOT, "scripts"))

import robot_stack  # noqa: E402


@pytest.fixture
def state(tmp_path):
    return str(tmp_path)


def _spawn():
    """A process in its own group, like launch_bg starts one."""
    return subprocess.Popen([sys.executable, "-c", "import time; time.sleep(120)"],
                            start_new_session=True)


def test_a_recorded_stack_survives_the_pipeline(state):
    proc = _spawn()
    try:
        robot_stack.record("bringup", proc.pid, state_dir=state)
        entries = robot_stack.load(state_dir=state)
        assert [e["tag"] for e in entries] == ["bringup"]
        assert entries[0]["pgid"] == os.getpgid(proc.pid)
    finally:
        proc.kill()
        proc.wait(timeout=5)


def test_stop_signals_the_group_and_forgets_it(state):
    proc = _spawn()
    robot_stack.record("nav2", proc.pid, state_dir=state)
    assert robot_stack.stop("nav2", state_dir=state) == ["nav2"]
    proc.wait(timeout=5)
    assert proc.poll() is not None, "the process group was not stopped"
    assert robot_stack.load(state_dir=state) == []


def test_stopping_one_part_leaves_the_others(state):
    a, b = _spawn(), _spawn()
    try:
        robot_stack.record("bringup", a.pid, state_dir=state)
        robot_stack.record("nav2", b.pid, state_dir=state)
        robot_stack.stop("nav2", state_dir=state)
        b.wait(timeout=5)
        tags = [e["tag"] for e in robot_stack.load(state_dir=state)]
        assert tags == ["bringup"], "stopping Nav2 must not stop the robot"
        assert a.poll() is None
    finally:
        for p in (a, b):
            try:
                p.kill()
                p.wait(timeout=5)
            except Exception:
                pass


def test_a_dead_entry_is_dropped_rather_than_reported_running(state):
    """Otherwise the header claims a robot that is not there, and Stop signals
    a pgid that may since belong to somebody else."""
    proc = _spawn()
    robot_stack.record("slam", proc.pid, state_dir=state)
    proc.kill()
    proc.wait(timeout=5)
    time.sleep(0.2)
    assert robot_stack.load(state_dir=state) == []


def test_describe_says_nothing_when_nothing_runs(state):
    assert "nothing" in robot_stack.describe(state_dir=state)


def test_the_pipeline_keeps_the_stack_by_default():
    """--shutdown-when-done is opt-in; the UI passes nothing and must get a
    robot that stays up."""
    import one_click_pipeline as ocp
    parser = None
    src = open(os.path.join(REPO_ROOT, "scripts", "one_click_pipeline.py")).read()
    assert '"--shutdown-when-done"' in src
    assert 'dest="keep_running"' in src and 'default=True' in src
    # and the teardown is actually skipped for the stack
    assert "if id(p) in kept:" in src


def test_the_ui_pipeline_does_not_ask_for_shutdown():
    src = open(os.path.join(REPO_ROOT, "web", "backend", "main.py")).read()
    i = src.index("pipeline_script = os.path.join")
    window = src[i:i + 2000]
    assert "--shutdown-when-done" not in window, (
        "the 1-Click button must leave the robot running")


def test_a_zombie_launch_is_not_a_running_robot(state):
    """The pipeline exits on purpose, so its `ros2 launch` children are
    reparented and sit as <defunct> until init collects them -- and their
    process GROUP still exists, so killpg(pgid, 0) succeeds and the corpse
    reads as alive. Measured on the bench: all three launches were
    `[ros2] <defunct>` after being stopped."""
    proc = subprocess.Popen([sys.executable, "-c", "pass"], start_new_session=True)
    pgid = os.getpgid(proc.pid)
    # Do NOT wait(): leaving it uncollected is exactly the case under test.
    deadline = time.time() + 5
    while time.time() < deadline:
        with open(f"/proc/{proc.pid}/stat") as fh:
            data = fh.read()
        if data[data.rindex(")") + 1:].split()[0] == "Z":
            break
        time.sleep(0.05)
    else:
        pytest.skip("could not produce a zombie on this system")

    robot_stack.record("bringup", proc.pid, pgid=pgid, state_dir=state)
    assert robot_stack.load(state_dir=state) == [], (
        "a defunct launch was reported as a running robot")
    proc.wait(timeout=5)
