#!/usr/bin/env python3
"""Check a real robot's speeds against its LiDAR: what was commanded, what the wheels say
(odom/unfiltered) and what the room says (the LiDAR's own track of the robot).

    speed_check.py --params ROBOT_config.yaml [--strafe | --no-strafe] [--json OUT.json]
                   [--radius R] [--need M] [--stamped | --unstamped]

The wheels count what they turned, not what the floor gave back. A tyre a millimetre
smaller than the config says, a track that skids sideways in a turn, a mecanum roller that
slips -- the wheels report the same speed either way, so does odometry, and Nav2 overshoots
a goal or never finishes its final turn. The room does not slip: every scan is matched
against the ones before it (scan_match.track), and the robot's speed over the floor comes
out of that.

What moves: forward then back at 0.10 and 0.20 m/s, a spin left then right at 0.5 and
1.0 rad/s, and on a mecanum base (on by default from kinematics.base_type, or --strafe)
left then right at 0.10 and 0.20 m/s. Each step is 2 s, then a second at rest; every step
is followed by its mirror image, so the robot ends about where it began, and it is never
more than ~0.4 m from there. /cmd_vel is set to zero on every exit, and the firmware's
cmd_vel timeout stops the wheels if this program dies.

It refuses to move when anything is nearer than the robot's circle (--radius, default Nav2's
robot_radius plus its margin) plus REACH_M all round (--need may ask for more), or when
something else already publishes /cmd_vel (a teleop, Nav2). While it moves, every step stops
the run when a return comes into the corridor ahead (clearance.path_blocked), into the circle
a spin sweeps, or when the scan stops. Run it with the robot brought up on its
own -- no SLAM, no Nav2 -- in the cockpit's container; it runs itself under the stack's own
ROS environment (one_click_pipeline.get_ros_env).

What it prints: a row per step (commanded, odom, LiDAR, gyro, LiDAR/odom, battery), then
one verdict per motion -- LINEAR, ANGULAR, STRAFE -- with the config change it implies.
A ratio within 3 % of 1.0 is fine: that is about what the LiDAR can resolve in 2 s. A
NEGATIVE ratio is a wiring or mounting fault, never a scale. The gyro's yaw rate against the
LiDAR's is judged too (GYRO_TOLERANCE). Faults go in the JSON's "faults" and exit 1.
"""
import argparse
import json
import math
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import scan_match  # noqa: E402

LINEAR = [("lin", 0.10), ("lin", -0.10), ("lin", 0.20), ("lin", -0.20)]
ANGULAR = [("ang", 0.5), ("ang", -0.5), ("ang", 1.0), ("ang", -1.0)]
STRAFE = [("lat", 0.10), ("lat", -0.10), ("lat", 0.20), ("lat", -0.20)]
RUN_S, PAUSE_S, SETTLE_S = 2.0, 1.0, 0.6
TOLERANCE = 0.03
# The gyro and the room measure the same turn with no slip in between: 10 % apart is an IMU
# fault (its model, its scale, its mounting), not something to tune around.
GYRO_TOLERANCE = 0.10
# How far a step goes from the start: 0.20 m/s for RUN_S, then its mirror, so 0.4 m, plus a
# coast and a margin. The room it needs is the robot's circle plus this: a 0.6 m check from
# the centre let a 0.31 m robot drive its front to within a few cm of a box.
REACH_M = 0.45
SCAN_STALE_S = 0.5      # a step's guard needs a scan this fresh (10 Hz: five missed)
SPIN_MARGIN_M = 0.02    # a spin needs this beyond the robot's circle (clearance.SPIN_MARGIN_M)
UNITS = {"lin": "m/s", "ang": "rad/s", "lat": "m/s"}


# ---- arithmetic (no ROS) ----

def steps(strafe: bool) -> list:
    """The steps in the order they are driven; each one followed by its mirror image."""
    return LINEAR + ANGULAR + (STRAFE if strafe else [])


def base_type(params: dict) -> str:
    return str(((params or {}).get("kinematics") or {}).get("base_type", "2wd")).strip().lower()


def wants_strafe(params: dict) -> bool:
    """A mecanum base moves sideways; nothing else does."""
    return base_type(params) == "mecanum"


def measure(kind: str, poses, odom, gyro, t_a: float, t_b: float, settle: float = SETTLE_S):
    """One step's speeds over its steady part (settle s in, to the end).

    poses: rows (stamp, x, y, yaw, ...) from scan_match.track -- the room's account;
    odom:  rows (stamp, vx, wz, vy) from odom/unfiltered -- the wheels' account;
    gyro:  rows (stamp, wz) from imu/data.
    Returns {"lidar", "odom", "gyro"} in the step's own unit, or None with too few samples.
    """
    a0, b0 = t_a + settle, t_b
    P = np.asarray([p[:4] for p in poses], float).reshape(-1, 4)
    O = np.asarray(odom, float).reshape(-1, 4)
    G = np.asarray(gyro, float).reshape(-1, 2)
    q = P[(P[:, 0] >= a0) & (P[:, 0] <= b0)]
    o = O[(O[:, 0] >= a0) & (O[:, 0] <= b0)]
    g = G[(G[:, 0] >= a0) & (G[:, 0] <= b0)]
    if len(q) < 3 or len(o) < 3:
        return None
    dt = q[-1, 0] - q[0, 0]
    if dt <= 0:
        return None
    hd = float(q[:, 3].mean())
    dx, dy = q[-1, 1] - q[0, 1], q[-1, 2] - q[0, 2]
    gy = float("nan")
    if kind == "lin":       # along the heading
        lid = (dx * math.cos(hd) + dy * math.sin(hd)) / dt
        od = float(o[:, 1].mean())
    elif kind == "lat":     # across it, + = left
        lid = (-dx * math.sin(hd) + dy * math.cos(hd)) / dt
        od = float(o[:, 3].mean())
    else:
        lid = (q[-1, 3] - q[0, 3]) / dt
        od = float(o[:, 2].mean())
        if len(g):
            gy = float(g[:, 1].mean())
    return {"lidar": float(lid), "odom": od, "gyro": gy}


def ratios(rows: list) -> dict:
    """Median LiDAR/odom per motion, and gyro/LiDAR for the spins."""
    out = {}
    for kind in ("lin", "ang", "lat"):
        r = [x["lidar"] / x["odom"] for x in rows if x["kind"] == kind and abs(x["odom"]) > 1e-6]
        if r:
            out[kind] = float(np.median(r))
    g = [x["gyro"] / x["lidar"] for x in rows
         if x["kind"] == "ang" and math.isfinite(x["gyro"]) and abs(x["lidar"]) > 1e-6]
    if g:
        out["gyro"] = float(np.median(g))
    return out


def fine(r: float, tol: float = TOLERANCE) -> bool:
    return abs(r - 1.0) <= tol


def room_need(radius: float, need: float = 0.0) -> float:
    """The clear room all round before anything moves: the circle plus REACH_M, or more."""
    return max(float(need or 0.0), float(radius) + REACH_M)


def step_fault(P, vx: float, vy: float, wz: float, radius: float, scan_age: float) -> str:
    """Why a step must stop now, or '': a return in the corridor ahead (translating), inside
    the circle a spin sweeps (turning), or a scan too old to say."""
    import clearance
    if scan_age > SCAN_STALE_S:
        return f"the scan stopped ({scan_age:.1f} s since the last one)"
    near = clearance.path_blocked(P, vx, vy, radius)
    if near:
        return f"a return {near:.2f} m in the way"
    if wz and clearance.nearest(P) < radius + SPIN_MARGIN_M:
        return f"a return {clearance.nearest(P):.2f} m from the centre, inside the circle a spin sweeps"
    return ""


WIRING = ("a wiring or mounting fault, not a scale: the LiDAR's yaw 180 deg off (geometry.laser), "
          "the LiDAR upside down (a mirrored scan), or the motors or encoders swapped or reversed")


def faults(rat: dict) -> list:
    """The ratios no config value corrects: a negative one, and a gyro more than
    GYRO_TOLERANCE off the room. Empty when there are none."""
    out = []
    names = {"lin": "LINEAR", "ang": "ANGULAR", "lat": "STRAFE"}
    for k, name in names.items():
        r = rat.get(k)
        if r is not None and r < 0:
            out.append(f"{name} LiDAR/odom {r:.3f} is negative: {WIRING}")
    r_g = rat.get("gyro")
    if r_g is not None:
        if r_g < 0:
            out.append(f"GYRO gyro/LiDAR {r_g:.3f} is negative: the IMU reads turns backwards -- "
                       f"mounted reversed (upside down) or geometry.imu's roll/yaw wrong")
        elif not fine(r_g, GYRO_TOLERANCE):
            out.append(f"GYRO gyro/LiDAR {r_g:.3f}: the IMU's turn rate is {abs(1 - r_g):.0%} "
                       f"{'high' if r_g > 1 else 'low'}, outside {GYRO_TOLERANCE:.0%} -- the IMU's "
                       f"model or scale, or its mounting")
    return out


def advice(rat: dict, params: dict, tol: float = TOLERANCE) -> list:
    """The verdict lines: what each ratio means and the config change it implies.

    The wheel diameter scales every speed odometry reports, turns and strafes included, so
    the turn and strafe ratios are judged after it: what is left once wheel_diameter is
    corrected is ratio / LINEAR."""
    kin = (params or {}).get("kinematics") or {}
    bt = base_type(params)
    lines = []
    r_lin = rat.get("lin")
    if r_lin is not None and r_lin < 0:
        lines.append(f"LINEAR  LiDAR/odom {r_lin:.3f}: NEGATIVE -- the room saw the robot go the other "
                     f"way: {WIRING}. No wheel_diameter fixes it")
    elif r_lin is not None:
        if fine(r_lin, tol):
            lines.append(f"LINEAR  LiDAR/odom {r_lin:.3f}: fine (within {tol:.0%}) -- nothing to change")
        else:
            wd = kin.get("wheel_diameter")
            what = "over-read" if r_lin < 1 else "under-read"
            new = (f"kinematics.wheel_diameter {float(wd):.4f} -> {float(wd) * r_lin:.4f}" if wd is not None
                   else f"kinematics.wheel_diameter x {r_lin:.3f}")
            lines.append(f"LINEAR  LiDAR/odom {r_lin:.3f}: the wheels {what} the floor by "
                         f"{abs(1 - r_lin):.1%} -- set {new}")
    base = r_lin if r_lin is not None and r_lin > 0 else 1.0
    r_ang = rat.get("ang")
    if r_ang is not None and r_ang < 0:
        lines.append(f"ANGULAR LiDAR/odom {r_ang:.3f}: NEGATIVE -- the room saw the robot turn the other "
                     f"way: {WIRING}. No angular_scale fixes it")
    elif r_ang is not None:
        left = r_ang / base
        if fine(left, tol):
            lines.append(f"ANGULAR LiDAR/odom {r_ang:.3f} ({left:.3f} once wheel_diameter is right): "
                         f"fine (within {tol:.0%}) -- nothing to change")
        elif bt == "mecanum":
            lines.append(f"ANGULAR LiDAR/odom {r_ang:.3f} ({left:.3f} once wheel_diameter is right): the body "
                         f"turns {left:.0%} of what the wheels say. A mecanum base has no angular_scale (the "
                         f"firmware turns on (lr_wheels_distance + fr_wheels_distance) / 2 and ignores it): "
                         f"measure those two again; what a tape measure does not explain is roller slip, "
                         f"which no config key corrects")
        else:
            sc = float(kin.get("angular_scale", 1.0))
            lines.append(f"ANGULAR LiDAR/odom {r_ang:.3f} ({left:.3f} once wheel_diameter is right): the body "
                         f"turns {left:.0%} of what the wheels say -- set kinematics.angular_scale "
                         f"{sc:.3f} -> {sc / left:.3f}")
    r_lat = rat.get("lat")
    if r_lat is not None and r_lat < 0:
        lines.append(f"STRAFE  LiDAR/odom {r_lat:.3f}: NEGATIVE -- the room saw the robot slide the other "
                     f"way: {WIRING} (on a mecanum base, also the wheels' X pattern mirrored)")
    elif r_lat is not None:
        left = r_lat / base
        if fine(left, tol):
            lines.append(f"STRAFE  LiDAR/odom {r_lat:.3f} ({left:.3f} once wheel_diameter is right): "
                         f"fine (within {tol:.0%}) -- nothing to change")
        else:
            lines.append(f"STRAFE  LiDAR/odom {r_lat:.3f} ({left:.3f} once wheel_diameter is right): the body "
                         f"moves {left:.0%} of the sideways speed the wheels say -- roller slip; there is no "
                         f"config key for it (sideways odometry comes from wheel_diameter alone). Check the "
                         f"rollers turn freely and the wheels sit in the right X pattern")
    r_g = rat.get("gyro")
    if r_g is not None and r_g < 0:
        lines.append(f"GYRO    gyro/LiDAR {r_g:.3f}: NEGATIVE -- FAIL: the IMU reads turns backwards; it is "
                     f"mounted reversed (upside down) or geometry.imu's roll/yaw is wrong")
    elif r_g is not None and not fine(r_g, tol):
        verdict = "FAIL" if not fine(r_g, GYRO_TOLERANCE) else f"within {GYRO_TOLERANCE:.0%}"
        lines.append(f"GYRO    gyro/LiDAR {r_g:.3f}: the IMU's turn rate is {abs(1 - r_g):.1%} "
                     f"{'high' if r_g > 1 else 'low'} ({verdict}) -- the IMU, not the wheels; check its "
                     f"mounting and model")
    if not lines:
        lines.append("no verdict: no step had enough samples")
    return lines


class StepStopped(Exception):
    """A step's guard stopped the robot (step_fault)."""


def robot_radius(params: dict, given: float = None) -> float:
    """The robot's circle: --radius, else what the 1-Click's clearance step passes (Nav2's
    robot_radius plus its margin, gen_firmware_header.nav2_robot_radius)."""
    if given:
        return float(given)
    import gen_firmware_header
    return gen_firmware_header.nav2_robot_radius(params or {})


# ---- ROS ----

def _cmd_type(node, topic: str, stamped):
    """Twist or TwistStamped: what the board subscribes to, else by distro (TwistStamped
    from kilted on), unless --stamped / --unstamped says."""
    from geometry_msgs.msg import Twist, TwistStamped
    if stamped is not None:
        return TwistStamped if stamped else Twist
    for info in node.get_subscriptions_info_by_topic(topic):
        if info.topic_type.endswith("TwistStamped"):
            return TwistStamped
        if info.topic_type.endswith("/Twist"):
            return Twist
    return Twist if os.environ.get("ROS_DISTRO", "lyrical") in ("humble", "iron", "jazzy") else TwistStamped


def _whole_circle(node, pre: str, wait: float = 8.0) -> str:
    """The scan with the LiDAR's whole circle, under the robot's namespace: scan_raw when a
    lidar.mask makes /scan a narrower view (scan_match.whole_circle_topic, namespaced)."""
    import rclpy
    end = time.time() + wait
    while time.time() < end:
        names = {n for n, _ in node.get_topic_names_and_types()}
        if f"{pre}/scan_raw" in names:
            return f"{pre}/scan_raw"
        if f"{pre}/scan" in names and time.time() > end - wait / 2:
            return f"{pre}/scan"
        rclpy.spin_once(node, timeout_sec=0.2)
    return f"{pre}/scan"


def run(a, params: dict) -> int:
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import BatteryState, Imu, LaserScan
    import clearance
    import cockpit_paths

    ns = cockpit_paths.robot_namespace(params)
    pre = f"/{ns}" if ns else ""
    rclpy.init()
    node = Node("speed_check")
    st = lambda h: h.stamp.sec + h.stamp.nanosec * 1e-9   # noqa: E731
    scans, odom, gyro, batt, frame = [], [], [], [], [None]
    last_scan = [-math.inf]          # arrival, this host's monotonic clock
    scan_topic = a.scan_topic or _whole_circle(node, pre)
    radius = robot_radius(params, a.radius)
    body = clearance.body_for(radius)

    def on_scan(m):
        frame[0] = m.header.frame_id
        last_scan[0] = time.monotonic()
        scans.append(scan_match.scan_dict(m))
    node.create_subscription(LaserScan, scan_topic, on_scan, qos_profile_sensor_data)
    node.create_subscription(Odometry, f"{pre}/odom/unfiltered",
                             lambda m: odom.append((st(m.header), m.twist.twist.linear.x,
                                                    m.twist.twist.angular.z, m.twist.twist.linear.y)),
                             qos_profile_sensor_data)
    node.create_subscription(Imu, f"{pre}/imu/data",
                             lambda m: gyro.append((st(m.header), m.angular_velocity.z)), qos_profile_sensor_data)
    # stamped on arrival: a battery message may carry no stamp, and it moves slowly
    node.create_subscription(BatteryState, f"{pre}/battery",
                             lambda m: batt.append((time.time(), m.voltage, m.current)), qos_profile_sensor_data)
    pub, T = None, None

    def spin(secs):
        end = time.time() + secs
        while time.time() < end:
            rclpy.spin_once(node, timeout_sec=0.02)

    def send(vx, wz, vy=0.0):
        m = T()
        t = m.twist if hasattr(m, "twist") else m
        if hasattr(m, "header"):
            m.header.stamp = node.get_clock().now().to_msg()
            m.header.frame_id = f"{ns}/base_link" if ns else "base_link"
        t.linear.x, t.linear.y, t.angular.z = float(vx), float(vy), float(wz)
        pub.publish(m)

    def hold(vx, wz, vy, secs):
        end = time.time() + secs
        while time.time() < end:
            if vx or vy or wz:
                P = scan_match.scan_points(scans[-1], laser, body=body) if scans else np.zeros((0, 2))
                why = step_fault(P, vx, vy, wz, radius, time.monotonic() - last_scan[0])
                if why:
                    send(0.0, 0.0)
                    raise StepStopped(why)
            send(vx, wz, vy)
            spin(0.1)

    try:
        t0 = time.time()
        while (not scans or not odom) and time.time() - t0 < 15:
            rclpy.spin_once(node, timeout_sec=0.1)
        if not scans or not odom:
            print(f"SPEED_CHECK REFUSED: no data -- {scan_topic} {len(scans)}, {pre}/odom/unfiltered {len(odom)}. "
                  f"Bring the robot up first (no SLAM, no Nav2). Nothing moved.")
            return 2
        laser = scan_match.laser_pose(node, frame[0] or "laser")
        if laser is None:
            print(f"  ⚠️ no TF base_link -> {frame[0]}; using the scan frame as base_link")
            laser = (0.0, 0.0, 0.0)
        # The room first: the robot's circle plus the steps' reach, all round.
        need = room_need(radius, a.need)
        still = scans[-5:]
        P = scan_match.median_points(still, laser, body=body)
        near = clearance.nearest(P)
        q = clearance.quadrants(P)
        fmt = lambda v: "none" if v == math.inf else f"{v:.2f} m"   # noqa: E731
        print("  room (+-45 deg): " + ", ".join(f"{k} {fmt(v)}" for k, v in q.items())
              + f"; nearest {fmt(near)} (need {need:.2f} m: radius {radius:.2f} + {REACH_M:.2f})")
        if near < need:
            print(f"SPEED_CHECK REFUSED: something is {near:.2f} m away, nearer than {need:.2f} m -- "
                  f"give the robot room. Nothing moved.")
            return 2
        cmd_topic = f"{pre}/cmd_vel"
        others = len(node.get_publishers_info_by_topic(cmd_topic))
        if others:
            print(f"SPEED_CHECK REFUSED: {cmd_topic} already has {others} publisher(s) -- something else "
                  f"is driving the robot (Nav2, a teleop). Nothing moved.")
            return 2
        T = _cmd_type(node, cmd_topic, a.stamped)
        pub = node.create_publisher(T, cmd_topic, 10)
        print(f"  {cmd_topic} as {T.__name__}; steps: {'linear, angular, strafe' if a.strafe else 'linear, angular'}")
        print("--- THE ROBOT MOVES (about 0.4 m at most, back to where it started)", flush=True)
        scans.clear(); odom.clear(); gyro.clear()
        rest = [v for _, v, _ in batt[-10:]]
        windows = []
        hold(0, 0, 0, 1.0)
        for kind, v in steps(a.strafe):
            ta = time.time()
            hold(v if kind == "lin" else 0.0, v if kind == "ang" else 0.0, v if kind == "lat" else 0.0, RUN_S)
            tb = time.time()
            hold(0, 0, 0, PAUSE_S)
            windows.append((kind, v, ta, tb))
    except StepStopped as e:
        print(f"SPEED_CHECK STOPPED: {e} -- the robot was stopped mid-step; no verdict")
        if a.json:
            with open(a.json, "w") as fh:
                json.dump({"stopped": str(e), "ratios": {}, "faults": [f"stopped: {e}"]}, fh)
        return 1
    finally:
        if pub is not None:
            for _ in range(10):
                send(0.0, 0.0)
                time.sleep(0.02)
    # Stamps are ROS time, the windows wall time on this host: the same clock here (the agent
    # syncs the board to the host).
    poses, end = scan_match.track(sorted(scans, key=lambda s: s["stamp"]), laser)
    rows = []
    B = np.asarray(batt, float).reshape(-1, 3)
    if rest:
        print("battery at rest: %.2f V" % (sum(rest) / len(rest)))
    print("step               cmd     odom    LiDAR    gyro   LiDAR/odom  LiDAR/cmd  battery V (min)")
    for kind, v, ta, tb in windows:
        m = measure(kind, poses, odom, gyro, ta, tb)
        if m is None:
            print(f"{kind} {v:+.2f}: too few samples")
            continue
        mb = (B[:, 0] >= ta + SETTLE_S) & (B[:, 0] <= tb) if len(B) else np.zeros(0, bool)
        bv, bmin = (float(B[mb, 1].mean()), float(B[mb, 1].min())) if mb.any() else (float("nan"),) * 2
        rows.append(dict(kind=kind, cmd=v, volt=bv, volt_min=bmin, **m))
        print("%s %+.2f %-5s  %+6.3f  %+6.3f  %+6.3f  %+6.3f    %5.3f      %5.3f    %5.2f (%5.2f)"
              % (kind, v, UNITS[kind], v, m["odom"], m["lidar"], m["gyro"],
                 m["lidar"] / m["odom"] if m["odom"] else float("nan"), m["lidar"] / v, bv, bmin))
    if end:
        print("ended %.3f m, %.3f m, %.1f deg from the start (the LiDAR, %d pairs)"
              % (end["x"], end["y"], math.degrees(end["yaw"]), end["pairs"]))
    rat = ratios(rows)
    for line in advice(rat, params):
        print(line)
    bad = faults(rat)
    for f in bad:
        print(f"SPEED_CHECK FAULT: {f}")
    if a.json:
        with open(a.json, "w") as fh:
            json.dump({"rows": rows, "ratios": rat, "faults": bad, "end": end, "windows": windows,
                       "batt": batt}, fh)
    print("SPEED_CHECK_DONE")
    node.destroy_node()
    rclpy.shutdown()
    return 1 if bad else 0


def ros_env_command(argv: list, distro: str) -> str:
    """This program again, under the shell prefix every ROS step of the 1-Click runs under
    (one_click_pipeline.get_ros_env: ROS, the workspaces and the Fast DDS profile). A reader
    without the stack's DDS profile cannot see a stack that has it, so a plain
    `python3 speed_check.py` in the container would find no /odom at all."""
    import shlex
    import one_click_pipeline
    args = " ".join(shlex.quote(x) for x in argv)
    return (f"{one_click_pipeline.get_ros_env(distro or 'auto')} && SPEED_CHECK_IN_ROS_ENV=1 "
            f"exec python3 {shlex.quote(os.path.abspath(__file__))} {args}")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--params", default="", help="the robot's config YAML (base_type, wheel_diameter, namespace)")
    sx = ap.add_mutually_exclusive_group()
    sx.add_argument("--strafe", dest="strafe", action="store_true", default=None,
                    help="sideways steps too (default: on for a mecanum base)")
    sx.add_argument("--no-strafe", dest="strafe", action="store_false")
    ap.add_argument("--need", type=float, default=0.0,
                    help=f"refuse to move with anything nearer, m (at least --radius + {REACH_M:.2f})")
    ap.add_argument("--radius", type=float, default=0.0,
                    help="the robot's circle, m (default: Nav2's robot_radius plus its margin, from --params)")
    tx = ap.add_mutually_exclusive_group()
    tx.add_argument("--stamped", dest="stamped", action="store_true", default=None,
                    help="publish TwistStamped (default: what the board subscribes to)")
    tx.add_argument("--unstamped", dest="stamped", action="store_false")
    ap.add_argument("--scan-topic", default=None, help="default: scan_raw when there is one, else scan")
    ap.add_argument("--json", default="", help="write the rows, ratios and track here")
    a = ap.parse_args(argv)
    if os.environ.get("SPEED_CHECK_IN_ROS_ENV") != "1":
        cmd = ros_env_command(sys.argv[1:] if argv is None else list(argv), os.environ.get("ROS_DISTRO", "auto"))
        os.execvp("bash", ["bash", "-c", cmd])
    params = {}
    if a.params:
        import yaml
        with open(os.path.expanduser(a.params)) as fh:
            params = yaml.safe_load(fh) or {}
    if a.strafe is None:
        a.strafe = wants_strafe(params)
    return run(a, params)


if __name__ == "__main__":
    sys.exit(main())
