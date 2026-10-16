"""A TF wait is bounded by seconds, and the listener starts before the scans.

self_returns.py (the 1-Click's "LiDAR sees the robot" check) printed "no TF base_link ->
laser; judging in the scan frame" on a real robot whose robot_state_publisher did publish
base_link -> laser on the latched /tf_static, and a probe built the same way saw only
map, odom and base_link.

scan_match.laser_pose() waited `tries` x rclpy.spin_once(timeout_sec=0.2), read as
"tries x 0.2 s". spin_once runs ONE callback and returns as soon as one is ready, and the
node is subscribed to /scan and, through the listener, to /tf at the EKF's rate: in the
jazzy image beside a 100 Hz /tf, "10 tries" gave up after 0.11 s (2026-10-16). The
listener was also created only after the scans had been read, so those milliseconds were
all DDS had to match the new /tf_static reader to robot_state_publisher's writer and hand
over its one latched sample, while /tf kept coming. clearance's "100 x 0.2 s = 20 s" was
about a second for the same reason.
"""
import os
import sys
import types

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "scripts"))
import scan_match  # noqa: E402


class _Clock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t


def test_the_wait_is_seconds_not_spins():
    """A busy node: every spin returns after 1 ms. The old bound (10 spins) is 10 ms."""
    clock, spins = _Clock(), []

    def spin(timeout):
        spins.append(timeout)
        clock.t += 0.001

    def attempt():
        if clock.t < 1.5:
            raise LookupError("not yet")
        return "pose"
    assert scan_match.wait_until(attempt, spin, 5.0, clock=clock) == "pose"
    assert len(spins) > 1000
    assert all(0 < s <= 0.1 for s in spins)


def test_the_wait_gives_up_on_time():
    clock = _Clock()

    def spin(timeout):
        clock.t += timeout          # an idle node: each spin waits its whole timeout

    def never():
        raise LookupError("no such frame")
    assert scan_match.wait_until(never, spin, 2.0, clock=clock) is None
    assert 2.0 <= clock.t < 2.11


def test_a_leading_slash_is_not_a_frame():
    assert scan_match.tf_frame("/laser") == "laser"
    assert scan_match.tf_frame("tracked/laser") == "tracked/laser"


def test_the_frames_tf_knows_are_listed():
    text = ("laser: \n  parent: 'base_link'\n  broadcaster: 'x'\n"
            "base_link: \n  parent: 'odom'\n  broadcaster: 'y'\n")
    assert scan_match.frames_in(text) == ["base_link", "laser", "odom"]
    assert scan_match.frames_in("[]") == []          # an empty Buffer says this
    assert scan_match.frames_in(": : :") == []


@pytest.fixture
def fake_ros(monkeypatch):
    """rclpy and tf2_ros, just enough for TfWatch: the static transform appears after
    40 callbacks -- four times what the old ten-try loop allowed."""
    st = {"spins": 0, "lookups": [], "unregistered": 0, "listeners": []}
    rclpy = types.ModuleType("rclpy")
    rclpy.time = types.SimpleNamespace(Time=lambda: 0)
    rclpy.spin_once = lambda node, timeout_sec=None: st.__setitem__("spins", st["spins"] + 1)
    tf2_ros = types.ModuleType("tf2_ros")

    class Buffer:
        def lookup_transform(self, target, source, when):
            st["lookups"].append((target, source))
            if st["spins"] < 40:
                raise LookupError(f'"{source}" passed to lookupTransform does not exist')
            q = types.SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0)
            t = types.SimpleNamespace(x=0.095, y=0.0, z=0.1)
            return types.SimpleNamespace(transform=types.SimpleNamespace(translation=t, rotation=q))

        def all_frames_as_yaml(self):
            return "base_link: \n  parent: 'odom'\n"

    class TransformListener:
        def __init__(self, buf, node):
            st["listeners"].append(node)

        def unregister(self):
            st["unregistered"] += 1

    tf2_ros.Buffer, tf2_ros.TransformListener = Buffer, TransformListener
    monkeypatch.setitem(sys.modules, "rclpy", rclpy)
    monkeypatch.setitem(sys.modules, "tf2_ros", tf2_ros)
    return st


def test_laser_pose_outwaits_a_busy_node_and_unsubscribes(fake_ros):
    pose = scan_match.laser_pose(object(), "/laser")
    assert pose == pytest.approx((0.095, 0.0, 0.0))
    assert fake_ros["spins"] >= 40
    assert fake_ros["lookups"][-1] == ("base_link", "laser")
    # a listener left behind keeps taking /tf callbacks from the caller's own spin loop
    assert fake_ros["unregistered"] == 1


def test_a_watch_started_early_names_what_tf_has(fake_ros):
    w = scan_match.TfWatch(object())
    assert w.frames() == ["base_link", "odom"]
    w.close()
    assert fake_ros["unregistered"] == 1


def _src(name):
    with open(os.path.join(ROOT, "scripts", name), encoding="utf-8") as fh:
        return fh.read()


def test_self_returns_listens_before_it_reads_the_scans():
    main = _src("self_returns.py")
    main = main[main.index("def main("):]
    assert main.index("scan_match.TfWatch(node)") < main.index("create_subscription(LaserScan")
    assert "tf_watch.laser_pose(" in main and "tf_watch.close()" in main
    assert "tf_watch.frames()" in main          # the warning says what TF did have


def test_lidar_check_listens_before_it_reads_the_scans():
    src = _src("lidar_check.py")
    assert src.index("scan_match.TfWatch(node)") < src.index("create_subscription(LaserScan")
    assert "tf_watch.close()" in src


def test_no_caller_counts_spins():
    for f in sorted(os.listdir(os.path.join(ROOT, "scripts"))):
        if f.endswith(".py"):
            assert "laser_pose(" not in _src(f) or "tries=" not in _src(f), (
                f"{f}: a TF wait counted in spin_once() calls is milliseconds on a busy node")
