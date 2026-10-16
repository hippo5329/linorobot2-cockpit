"""run_ros on a timeout takes its whole process group with it. subprocess.run's timeout killed
only the `bash -c`, and the ros2 under it lived on: a `topic echo --once /map` held 200 MB of
the MC200's 3.6 GB ten minutes later while Nav2 failed to start (2026-10-16)."""
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "scripts"))
import one_click_pipeline as ocp  # noqa: E402


def _alive(pid: int) -> bool:
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    try:   # a zombie is gone for this purpose
        with open(f"/proc/{pid}/stat") as fh:
            return fh.read().split()[2] != "Z"
    except OSError:
        return False


def test_a_timed_out_command_leaves_no_child(tmp_path, monkeypatch):
    monkeypatch.setattr(ocp, "get_ros_env", lambda distro: "true")
    pidfile = tmp_path / "child.pid"
    # a child that outlives bash, as ros2 does under `bash -c "... && ros2 topic echo"`
    res = ocp.run_ros(f"sleep 60 & echo $! > {pidfile}; wait", timeout=1, distro="jazzy")
    assert res.returncode == 124
    pid = int(pidfile.read_text())
    for _ in range(50):
        if not _alive(pid):
            break
        time.sleep(0.1)
    assert not _alive(pid), "the timed-out command's child is still running"


def test_a_command_that_finishes_returns_its_output(monkeypatch):
    monkeypatch.setattr(ocp, "get_ros_env", lambda distro: "true")
    res = ocp.run_ros("echo hello; echo oops >&2; exit 3", timeout=10, distro="jazzy")
    assert (res.returncode, res.stdout.strip(), res.stderr.strip()) == (3, "hello", "oops")
