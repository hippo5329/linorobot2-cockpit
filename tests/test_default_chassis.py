"""One default chassis, everywhere.

A Nav2 fake-mode failure can only be attributed to a board if every default config
shares the same chassis: the same body and sensor mounts (laser at the base origin),
the same costmap radii, consistent frame names, the same bare kinematics, and no
inherited hardware quirks. The Yahboom reference failed the Nav2 goal on both
transports and both distros while every bare board passed, and the config was the
only variable: a laser 0.12 m ahead, a different Nav2 template (0.22/0.7,
base_footprint for EKF/SLAM against base_link in the costmaps) and a motor invert
copied from the vendor's firmware. 2026-09-22.
"""
import glob
import math
import os
import re
import sys

import yaml

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
REF = os.path.join(REPO_ROOT, "config", "reference")
sys.path.insert(0, os.path.join(REPO_ROOT, "scripts"))
import gen_robot_description  # noqa: E402
import gen_firmware_header    # noqa: E402

DEFAULT_KINEMATICS = {"wheel_diameter": 0.1, "lr_wheels_distance": 0.271,
                      "max_rpm": 140, "counts_per_rev": 4000}
ROBOT_RADIUS, INFLATION = 0.26, 0.55


def _refs():
    for path in sorted(glob.glob(os.path.join(REF, "*_config.yaml"))):
        with open(path) as fh:
            yield os.path.basename(path), yaml.safe_load(fh)


def _walk(node, key):
    """Every value of `key` anywhere in a nested dict."""
    if isinstance(node, dict):
        for k, v in node.items():
            if k == key:
                yield v
            yield from _walk(v, key)
    elif isinstance(node, list):
        for v in node:
            yield from _walk(v, key)


def test_no_default_config_inverts_a_motor_or_encoder():
    for name, params in _refs():
        pins = params.get("base_controller", {}).get("pins", {})
        for unit, cfg in pins.items():
            if isinstance(cfg, dict) and "invert" in cfg:
                assert cfg["invert"] is False, f"{name}: {unit}.invert is on -- the default is forward"
    presets = open(os.path.join(REPO_ROOT, "web", "frontend", "app-presets.js")).read()
    assert not re.search(r"invert:\s*true", presets), "a preset inverts a motor or encoder"


def test_every_reference_puts_the_laser_at_the_base_origin():
    for name, params in _refs():
        geo = gen_robot_description.effective_geometry(params)["laser"]
        assert (geo["x"], geo["y"], geo["yaw"]) == (0.0, 0.0, 0.0), \
            f"{name}: laser at ({geo['x']}, {geo['y']}, yaw {geo['yaw']}) -- same mounts everywhere"


def test_every_reference_shares_the_costmap_radii():
    for name, params in _refs():
        radii = set(_walk(params, "robot_radius"))
        infl = set(_walk(params, "inflation_radius"))
        assert radii <= {ROBOT_RADIUS}, f"{name}: robot_radius {radii}"
        assert infl <= {INFLATION}, f"{name}: inflation_radius {infl}"


def test_every_reference_agrees_on_the_base_frame():
    for name, params in _refs():
        costmap_base = set(_walk(params, "robot_base_frame"))
        for key in ("base_link_frame", "base_frame", "base_frame_id"):
            for v in _walk(params, key):
                assert {v} == costmap_base, \
                    f"{name}: {key}={v} but the costmaps use {costmap_base}"


def test_every_preset_and_reference_ships_the_same_wheel_motor_and_encoder():
    """One default wheel diameter, one motor, one encoder -- everywhere.

    A reference design on this bench is a bare module with extra pins: nothing
    has ever run on a real chassis, so a wheel diameter or an rpm that differs
    per preset is an unmeasured guess that makes two Nav2 results incomparable.
    A robot gets its measured numbers when the robot exists.
    """
    presets = open(os.path.join(REPO_ROOT, "web", "frontend", "app-presets.js")).read()
    for key, want in (("wheel_diameter", 0.1), ("lr_wheels_distance", 0.271),
                      ("max_rpm", 140), ("cpr", 4000)):
        values = {float(v) for v in re.findall(rf"\n      {key}: ([0-9.]+)", presets)}
        assert values == {float(want)}, f"presets disagree on {key}: {sorted(values)}"
    for name, params in _refs():
        k = params.get("kinematics", {})
        for key, want in DEFAULT_KINEMATICS.items():
            assert k.get(key) == want, f"{name}: {key}={k.get(key)}, not the default {want}"


def test_the_footprint_contains_every_default_body():
    """robot_radius must actually contain the body it is given.

    Pulling the mecanum reference onto the 2WD wheelbase once made its body
    0.499 m long, whose half-diagonal is 0.293 m -- larger than the shared
    robot_radius of 0.26, so no plan could start from inside its own footprint.
    """
    for name, params in _refs():
        b = gen_robot_description.effective_geometry(params)["body"]
        half = math.hypot(float(b["length"]), float(b["width"])) / 2
        assert half <= ROBOT_RADIUS + 1e-9, \
            f"{name}: body {b['length']}x{b['width']} has half-diagonal {half:.3f} > robot_radius {ROBOT_RADIUS}"


def test_bare_kinematics_are_the_same_in_the_presets_and_the_release_image():
    presets = open(os.path.join(REPO_ROOT, "web", "frontend", "app-presets.js")).read()
    for m in re.finditer(r'id: "(bare_[a-z0-9]+)"(.*?)pins:', presets, re.S):
        block = m.group(2)
        for key, want in (("wheel_diameter", 0.1), ("lr_wheels_distance", 0.271),
                          ("max_rpm", 140), ("cpr", 4000)):
            got = re.search(rf"\b{key}: ([0-9.]+)", block)
            assert got and float(got.group(1)) == want, f"{m.group(1)}: {key} {got and got.group(1)}"
    bare = gen_firmware_header.release_params("esp32") if hasattr(gen_firmware_header, "release_params") else None
    if bare is None:
        src = open(os.path.join(REPO_ROOT, "scripts", "gen_firmware_header.py")).read()
        for key, want in DEFAULT_KINEMATICS.items():
            assert re.search(rf'"{key}": {want}\b', src), f"release bare config: {key} != {want}"
    else:
        for key, want in DEFAULT_KINEMATICS.items():
            assert bare["kinematics"][key] == want


def test_a_failed_goal_re_runs_the_drive_suite():
    """A failed goal must say which half is at fault.

    The six manoeuvres run before SLAM and Nav2, so by the time a goal fails
    they are minutes and two lifecycle activations old. Asking the base again,
    in the state the failure happened in, separates "Nav2 cannot navigate" from
    "the base stopped answering" -- and those send the reader to opposite ends
    of the stack.
    """
    src = open(os.path.join(REPO_ROOT, "scripts", "one_click_pipeline.py")).read()
    fail_branch = src[src.index('failures.append(f"Nav2: goal test exit'):]
    assert "drive_suite.py" in fail_branch[:2000], "a failed goal does not re-run the manoeuvres"
    # and it must not depend on a name the earlier drive block owns: --no-drive-test
    # would otherwise turn the failure into a NameError inside its own diagnostic.
    assert "post_tname" in fail_branch[:2000] and "{tname}" not in fail_branch[:2000]


def test_the_simulated_pose_is_zeroed_before_slam_in_fake_mode():
    """A goal at fixed coordinates only means something from a known start.

    The flash zeroes the simulated pose and the six manoeuvres then move it
    (measured residual 0.46-0.48 m). The firmware resets the pose on a new agent
    session -- and bouncing only the agent was measured to wreck the EKF (it
    predicts through the reconnect gap, then wobbles for 15 s), so the pipeline
    restarts the whole bringup between the drive suite and SLAM and waits for
    base AND EKF to sit at the origin. It also catches an EKF that never
    followed the base at all, before Nav2 is asked to steer by it.
    """
    pipe = open(os.path.join(REPO_ROOT, "scripts", "one_click_pipeline.py")).read()
    step = pipe[pipe.index("[4.7/6] [POSE]"):pipe.index("# Step 5: SLAM")]
    assert "stop_bg(bringup_proc)" in step, "the bringup is restarted, not the agent alone"
    assert 'launch_bg(bringup_cmd, log_tag="bringup2"' in step
    assert "_ekf_xy" in step and "_odom_xy" in step, "both the base and the EKF are read"
    assert "EKF does not follow /odom/unfiltered" in step, "an EKF that ignores the base fails here"
    assert "pgrep" not in step and "os.kill" not in step, "no process is killed by name or pid"
    assert "if not is_real and args.pose_reset" in pipe, "a real base must never be touched"
    launch = open(os.path.join(REPO_ROOT, "launchers", "bringup.launch.py")).read()
    agent = launch[launch.index('name="micro_ros_agent"'):][:1400]
    assert "respawn=True" in agent, "an agent that dies mid-run still comes back"
