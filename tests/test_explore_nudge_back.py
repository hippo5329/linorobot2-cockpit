"""explore_nudge.py: "back" undoes what "out" drove, not what it was asked for.

MC200 2026-10-20: a wall 1.14 m ahead stopped "out" at 0.00 m (0.46 m guard), and
"back" still reversed the full 0.8 m: the robot ended 0.82 m behind home and
Nav2's first goal began vacuous."""
import math
import os
import runpy
import sys
import types

SCRIPT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "scripts", "explore_nudge.py")


def _run(monkeypatch, wall_ahead, wall_behind=3.0, distance=0.8, side_walls=None):
    robot = {"x": 0.0, "y": 0.0, "yaw": 0.0, "vx": 0.0, "wz": 0.0, "t": 0.0}
    subs = {}

    class Msg:
        def __init__(self):
            self.linear = types.SimpleNamespace(x=0.0)
            self.angular = types.SimpleNamespace(z=0.0)

    class Node:
        def __init__(self, name): pass
        def create_publisher(self, typ, topic, depth):
            def publish(m):
                robot["vx"], robot["wz"] = m.linear.x, m.angular.z
            return types.SimpleNamespace(publish=publish)
        def create_subscription(self, typ, topic, cb, qos):
            subs[topic] = cb
        def destroy_node(self): pass

    def odom_msg():
        y = robot["yaw"]
        q = types.SimpleNamespace(x=0.0, y=0.0, z=math.sin(y / 2), w=math.cos(y / 2))
        return types.SimpleNamespace(pose=types.SimpleNamespace(pose=types.SimpleNamespace(
            position=types.SimpleNamespace(x=robot["x"], y=robot["y"]), orientation=q)))

    def spin_once(node, timeout_sec=0.0):
        dt = 0.05
        robot["x"] += robot["vx"] * math.cos(robot["yaw"]) * dt
        robot["y"] += robot["vx"] * math.sin(robot["yaw"]) * dt
        robot["yaw"] += robot["wz"] * dt
        robot["t"] += dt
        subs["odom"](odom_msg())
        if "scan" in subs:
            subs["scan"](types.SimpleNamespace(header=types.SimpleNamespace(frame_id="laser")))

    def scan_points(_d, laser=None):
        # Walls fixed in the room, returned in the robot's frame.
        c, s = math.cos(robot["yaw"]), math.sin(robot["yaw"])
        pts = []
        walls = [(wall_ahead, 0.0), (-wall_behind, 0.0)]
        if side_walls is not None:      # a ring of returns: no heading has room
            walls += [(side_walls * math.cos(k * math.pi / 18), side_walls * math.sin(k * math.pi / 18))
                      for k in range(36)]
        for wx, wy in walls:
            dx, dy = wx - robot["x"], wy - robot["y"]
            pts.append((c * dx + s * dy, -s * dx + c * dy))
        return pts

    clock = types.SimpleNamespace(time=lambda: robot["t"], sleep=lambda s: None)
    mods = {
        "rclpy": types.SimpleNamespace(init=lambda: None, shutdown=lambda: None, spin_once=spin_once),
        "rclpy.node": types.SimpleNamespace(Node=Node),
        "rclpy.qos": types.SimpleNamespace(qos_profile_sensor_data=None),
        "geometry_msgs.msg": types.SimpleNamespace(Twist=Msg, TwistStamped=Msg),
        "nav_msgs.msg": types.SimpleNamespace(Odometry=object),
        "sensor_msgs.msg": types.SimpleNamespace(LaserScan=object),
        "scan_match": types.SimpleNamespace(laser_pose=lambda n, f: (0.0, 0.0, 0.0),
                                            scan_points=scan_points, scan_dict=lambda m: {}),
        "time": clock,
    }
    for k, v in mods.items():
        monkeypatch.setitem(sys.modules, k, v)
    monkeypatch.setattr(sys, "argv", ["explore_nudge.py", "--distance", str(distance),
                                      "--guard", "0.46", "--radius", "0.20", "--yaw-rate", "2.0"])
    try:
        runpy.run_path(SCRIPT, run_name="__main__")
    except SystemExit as e:
        assert e.code == 0
    return robot


def test_a_blocked_out_leg_does_not_send_the_robot_backwards(monkeypatch, capsys):
    robot = _run(monkeypatch, wall_ahead=0.6)
    assert math.hypot(robot["x"], robot["y"]) < 0.05


def test_a_blocked_start_heading_drives_out_the_clearest_way(monkeypatch, capsys):
    """Yahboom 2026-10-22: a box 1.22 m ahead cut "out" to 0.00 m and the nudge only spun;
    camera-only SLAM drifted 0.6 m. The look finds a clear heading and "out" drives there."""
    robot = _run(monkeypatch, wall_ahead=0.6)
    out = capsys.readouterr().out
    assert "the clearest way is" in out
    assert "out, turn, back, turn" in out              # the whole 0.8 m, not a warning
    # faces its start, within one simulated step (2 rad/s x 0.05 s)
    assert abs(math.remainder(robot["yaw"], 2 * math.pi)) <= 0.1 + 1e-9


def test_boxed_in_all_round_says_so(monkeypatch, capsys):
    robot = _run(monkeypatch, wall_ahead=0.6, side_walls=0.6)
    out = capsys.readouterr().out
    assert "the map may not grow" in out
    assert math.hypot(robot["x"], robot["y"]) < 0.05


def test_a_part_driven_out_leg_comes_back_to_the_start(monkeypatch):
    robot = _run(monkeypatch, wall_ahead=1.2)       # stops ~0.54 m out
    assert math.hypot(robot["x"], robot["y"]) < 0.05


def test_an_open_room_still_drives_the_whole_pattern(monkeypatch, capsys):
    robot = _run(monkeypatch, wall_ahead=5.0)
    assert "out, turn, back, turn" in capsys.readouterr().out
    assert math.hypot(robot["x"], robot["y"]) < 0.05
