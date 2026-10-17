"""A base that went silent is not a robot that ran away.

2026-10-17, a GenDrv on udp4 (mecanum, lyrical): the agent session dropped 13 s
before the first Nav2 goal. /odom/unfiltered and imu/data stopped together, and
the EKF -- fusing the IMU's ax and ay -- kept predicting from its last
acceleration with nothing to correct it: x = a*t^2/2, 5.33 m at 13 s and 6.25 m
at 14 s (a = 0.063 m/s2). The leg was printed as "RAN AWAY ... the base was
still taking commands, so this is the controller driving it away", which was
false on every count.

Two fixes, each pinned here:
  * the shipped configs do not fuse linear acceleration (upstream linorobot2's
    ekf.yaml never did), so a filter left alone holds its last velocity instead
    of accelerating away;
  * the Nav2 tester hears the base's own odometry and names a silent base for
    what it is, before any verdict that reads the (extrapolated) pose.
"""
import glob
import os
import re
import sys
import types

import yaml

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCRIPTS = os.path.join(REPO_ROOT, "scripts")


def _load_module():
    """Import test_nav2_goal with the ROS 2 imports stubbed out."""
    stubs = {
        "rclpy": ["init", "shutdown", "spin_once"],
        "rclpy.action": ["ActionClient"],
        "rclpy.node": ["Node"],
        "rclpy.qos": ["QoSProfile", "ReliabilityPolicy", "HistoryPolicy"],
        "action_msgs.msg": ["GoalStatus"],
        "geometry_msgs.msg": ["PoseStamped", "Twist", "TwistStamped"],
        "nav_msgs.msg": ["Path", "Odometry"],
        "sensor_msgs.msg": ["LaserScan"],
        "nav2_msgs.action": ["NavigateToPose"],
        "action_msgs": [], "geometry_msgs": [], "nav_msgs": [], "nav2_msgs": [],
        "sensor_msgs": [],
    }
    saved = {name: sys.modules.get(name) for name in stubs}
    for name, attrs in stubs.items():
        mod = types.ModuleType(name)
        for a in attrs:
            setattr(mod, a, type(a, (object,), {}))
        sys.modules[name] = mod
    sys.path.insert(0, SCRIPTS)
    try:
        sys.modules.pop("test_nav2_goal", None)
        import test_nav2_goal
        return test_nav2_goal
    finally:
        for name, mod in saved.items():
            if mod is None:
                sys.modules.pop(name, None)
            else:
                sys.modules[name] = mod


MOD = _load_module()


def _imu0_config(path):
    def find(o):
        if isinstance(o, dict):
            for k, v in o.items():
                if k == "imu0_config":
                    return v
                r = find(v)
                if r is not None:
                    return r
        return None
    with open(path, encoding="utf-8") as fh:
        return find(yaml.safe_load(fh))


def test_no_shipped_config_fuses_linear_acceleration():
    files = sorted(glob.glob(os.path.join(REPO_ROOT, "config", "reference", "*_config.yaml")))
    assert files
    for f in files:
        cfg = _imu0_config(f)
        assert cfg and len(cfg) == 15, f
        assert cfg[12] is False and cfg[13] is False and cfg[14] is False, \
            f"{os.path.basename(f)} fuses linear acceleration: a silent base runs off as a*t^2/2"
        assert cfg[11] is True, f"{os.path.basename(f)}: yaw rate is what carries rotation"


class Node:
    """The attributes _runaway and _base_silent_for read, nothing else."""
    def __init__(self, last_xy=(6.69, -0.30), filtered=7.0, raw=None, raw_dist=0.0):
        self._last_xy = last_xy
        self.odom_max_dist = filtered
        self.raw_last_arrival = raw
        self.raw_max_dist = raw_dist


def test_a_base_never_heard_is_not_silent():
    assert MOD._base_silent_for(Node(raw=None), now=1000.0) is None


def test_a_base_heard_just_now_is_not_silent():
    assert MOD._base_silent_for(Node(raw=1000.0 - MOD.BASE_SILENT_S + 0.5), now=1000.0) is None


def test_a_base_quiet_past_the_bound_is_silent_and_says_for_how_long():
    age = MOD._base_silent_for(Node(raw=987.0), now=1000.0)
    assert age is not None and abs(age - 13.0) < 1e-9


def test_the_silence_bound_sits_past_the_firmware_own_loss_verdict():
    # A Wi-Fi gap the board rides out must never end a leg: the firmware itself
    # gives up only after AGENT_LOSS_UDP_MS of unanswered pings.
    with open(os.path.join(REPO_ROOT, "firmware", "src", "main.cpp"), encoding="utf-8") as fh:
        ms = int(re.search(r"AGENT_LOSS_UDP_MS\s*=\s*(\d+)", fh.read()).group(1))
    assert MOD.BASE_SILENT_S > ms / 1000.0


def test_the_2026_10_17_shape_is_not_a_runaway():
    # Filtered pose 6.7 m out and 7 m of filtered "travel"; the base's own pose
    # moved 0.02 m before it went quiet.
    assert MOD._runaway(Node(raw=0.0, raw_dist=0.02)) is False


def test_a_base_that_really_drove_off_is_still_a_runaway():
    assert MOD._runaway(Node(raw=0.0, raw_dist=6.5)) is True


def test_without_the_base_own_odometry_the_old_rule_stands():
    assert MOD._runaway(Node(raw=None)) is True


def test_the_lost_base_verdict_comes_before_every_pose_verdict():
    with open(os.path.join(SCRIPTS, "test_nav2_goal.py"), encoding="utf-8") as fh:
        src = fh.read()
    cascade = src[src.index("            if not arrived:\n"):]
    assert cascade.index("LOST THE BASE") < cascade.index("IS VACUOUS") < cascade.index("RAN AWAY")
    loop = src[src.index("def await_arrival"):]
    assert loop.index("_base_silent_for(node)") < loop.index("if _runaway(node):")
