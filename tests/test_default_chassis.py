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


def _reference_names():
    """Discovered, never listed.

    A test that names its inputs goes red when the set legitimately changes
    (esp32s3 was a bare module shipped as a reference and was removed), and --
    worse -- stays green when a new reference is added that it never checks.
    """
    import glob
    return sorted(
        os.path.basename(f)[: -len("_config.yaml")]
        for f in glob.glob(os.path.join(REPO_ROOT, "config", "reference", "*_config.yaml"))
    )

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


def test_the_drive_suite_runs_after_the_goal_not_before_it():
    """The six manoeuvres used to run between the flash and SLAM and left a
    0.46-0.48 m residual that SLAM then anchored its map to. They run after the
    Nav2 goal instead: the flash has already zeroed the pose, so nothing has to
    be reset, and a udp4 board never loses its agent to a bringup restart."""
    pipe = open(os.path.join(REPO_ROOT, "scripts", "one_click_pipeline.py")).read()
    assert "[4.5/6] [DRIVE]" not in pipe, "the drive suite no longer runs before SLAM"
    assert pipe.index("[6/6] [NAV2]") < pipe.index("[6.5/6] [DRIVE]"), "drive comes after the goal"
    assert pipe.count("drive_suite.py") == 1, "exactly one drive suite call, not one per outcome"
    drive = pipe[pipe.index("[6.5/6] [DRIVE]"):]
    assert "EKF does not follow /odom/unfiltered" in drive, \
        "the suite reads /odom/unfiltered; Nav2 steers by /odom, so compare them here"


def test_slam_starts_from_a_known_pose():
    """A goal at fixed coordinates only means something from a known start, so
    the pose is checked before SLAM and the bringup is restarted (a new session
    zeroes a simulated pose) only if something actually moved the robot."""
    pipe = open(os.path.join(REPO_ROOT, "scripts", "one_click_pipeline.py")).read()
    step = pipe[pipe.index("[4.7/6] [POSE]"):pipe.index("# Step 5: SLAM")]
    assert "math.hypot(*start_xy) <= POSE_START_TOL" in step, "no restart when already at the origin"
    assert "stop_bg(bringup_proc)" in step and 'log_tag="bringup2"' in step
    assert "pgrep" not in step and "os.kill" not in step, "no process is killed by name or pid"
    assert "if not is_real and args.pose_reset" in pipe, "a real base must never be touched"
    launch = open(os.path.join(REPO_ROOT, "launchers", "bringup.launch.py")).read()
    agent = launch[launch.index('name="micro_ros_agent"'):][:1400]
    assert "respawn=True" in agent, "an agent that dies mid-run still comes back"


def test_the_generated_bare_config_is_the_default_chassis():
    """gen_bare_config.py had its own kinematics table and drifted: 0.152 m wheels,
    inverted even-numbered motors and encoders, LED -1. It now takes them from
    gen_firmware_header.bare_mcu_params, and the pipeline regenerates the bare
    config on every run, so a cell can no longer test a two-day-old file."""
    sys.path.insert(0, os.path.join(REPO_ROOT, "scripts"))
    import gen_bare_config
    leds = {"pico": 25, "pico2": 25, "picow": 32, "pico2w": 32, "esp32": 2, "esp32s3": 48}
    for mcu in sorted(gen_bare_config.BOARDS):
        cfg = gen_bare_config.bare_config(mcu)
        for key, want in DEFAULT_KINEMATICS.items():
            assert cfg["kinematics"][key] == want, f"bare_{mcu}: {key} {cfg['kinematics'][key]} != {want}"
        pins = cfg["base_controller"]["pins"]
        assert pins["led"] == leds[mcu], f"bare_{mcu}: LED {pins['led']} (rule: the onboard LED is on)"
        for n in range(1, 5):
            assert pins[f"motor{n}"]["invert"] is False and pins[f"encoder{n}"]["invert"] is False, \
                f"bare_{mcu}: invert flags default OFF"
        for key in ("use_fake_imu", "use_fake_wheel", "use_fake_ld19"):
            assert cfg["base_controller"]["sensors"][key] is True
    pipe = open(os.path.join(REPO_ROOT, "scripts", "one_click_pipeline.py")).read()
    assert "gen_bare_config.bare_config(bare_mcu.group(1))" in pipe, "bare configs are regenerated per run"


def test_every_reference_and_the_bare_config_share_one_nav2_ekf_slam_template():
    """Two lineages had grown: a flat DWB template (mecanum, and the bare configs
    generated from it) and a nested RotationShim/RPP one (GenDrv, ESP32-S3,
    Yahboom). Both stalled a metre from the wall on 2026-09-22 for different
    reasons, and a failure could not be compared across boards. One template now,
    keyed the same way; the mecanum may differ only in its lateral-velocity keys."""
    import yaml
    sys.path.insert(0, os.path.join(REPO_ROOT, "scripts"))
    import gen_bare_config
    refs = {n: yaml.safe_load(open(os.path.join(REPO_ROOT, "config", "reference", f"{n}_config.yaml")))
            for n in _reference_names()}
    refs["bare_pico"] = gen_bare_config.bare_config("pico")
    base = refs["gendrv"]
    lateral = {  # the mecanum's allowed differences: it strafes
        ("ekf", "ekf_filter_node", "ros__parameters", "odom0_config"),
        ("nav2", "controller_server", "ros__parameters", "min_y_velocity_threshold"),
        ("nav2", "velocity_smoother", "ros__parameters", "max_velocity"),
        ("nav2", "velocity_smoother", "ros__parameters", "min_velocity"),
        ("nav2", "velocity_smoother", "ros__parameters", "max_accel"),
        ("nav2", "velocity_smoother", "ros__parameters", "max_decel"),
    }
    # ...and its YAW limits, which are now derived per drivetrain rather than
    # shared. A mecanum turns on (lr + fr)/2 where a differential base turns on
    # lr/2, so the same rad/s costs it 66% more wheel speed. One template used to
    # mean one set of angular limits, sized for the worst base -- which made the
    # differential robots slower than they need to be AND, before the radius was
    # corrected, still left the mecanum over budget. These are the keys
    # scripts/drivetrain_report.py writes, and they are expected to differ
    # exactly as much as the geometry does.
    angular = {
        ("nav2", "controller_server", "ros__parameters", "FollowPath",
         "rotate_to_heading_angular_vel"),
        ("nav2", "controller_server", "ros__parameters", "FollowPath", "max_angular_accel"),
        # ...and the linear target with them. A mecanum's envelope is scaled DOWN
        # as a pair when the translate-plus-rotate demand does not fit its
        # budget -- shaving only the yaw would change the robot's character into
        # a base that turns but will not drive -- so its linear target moves too,
        # by the same factor and for the same geometric reason.
        ("nav2", "controller_server", "ros__parameters", "FollowPath",
         "desired_linear_vel"),
        ("nav2", "behavior_server", "ros__parameters", "max_rotational_vel"),
        ("nav2", "behavior_server", "ros__parameters", "min_rotational_vel"),
        ("nav2", "behavior_server", "ros__parameters", "rotational_acc_lim"),
    }
    # ... and it is the one reference that wires an HC-SR04, which the
    # collision monitor may then listen to. A sensor the robot has is exactly
    # the kind of difference this template permits -- the point of one template
    # is that two boards with the same hardware behave the same, not that a
    # board with more hardware must ignore it. The allowance is derived from
    # the pins rather than named, so it disappears if the sonar ever does.
    sonar_keys = {
        ("nav2", "collision_monitor", "ros__parameters", "observation_sources"),
        ("nav2", "collision_monitor", "ros__parameters", "sonar"),
    }
    def _sonar_fitted(d):
        pins = ((d.get("base_controller") or {}).get("pins") or {}).get("sonar") or {}
        try:
            return int(pins.get("trigger", -1)) >= 0 and int(pins.get("echo", -1)) >= 0
        except (TypeError, ValueError):
            return False
    def walk(a, b, path, out):
        if isinstance(a, dict) and isinstance(b, dict):
            for k in set(a) | set(b):
                walk(a.get(k), b.get(k), path + (k,), out)
        elif a != b:
            out.append(path)
    for name, d in refs.items():
        diffs = []
        for sec in ("ekf", "slam", "nav2"):
            walk(base.get(sec), d.get(sec), (sec,), diffs)
        allowed = set(lateral) | set(angular) if name == "pico2_mecanum" else set()
        if _sonar_fitted(d) and not _sonar_fitted(base):
            allowed |= sonar_keys
        bad = [p for p in diffs if p not in allowed]
        assert not bad, f"{name} drifts from the template at {bad[:6]}"


def test_every_run_writes_its_own_log_directory():
    """A second run must not erase the first one's evidence: a Wi-Fi leg
    overwrote a serial leg's nav2.log and the failure it held was gone."""
    pipe = open(os.path.join(REPO_ROOT, "scripts", "one_click_pipeline.py")).read()
    assert 'RUN_ID = time.strftime' in pipe and "LOG_DIR = os.path.join(LOG_ROOT, RUN_ID)" in pipe
    assert pipe.count('open(run_log_path(') == 2, "both launchers write through run_log_path"
    assert 'os.symlink(target, link)' in pipe, "logs/latest and logs/<tag>.log still resolve"


def test_none_is_not_a_chip_name():
    """Every config in this project spells "not fitted" as NONE. Read as a chip
    name it made a real-sensor run demand /battery from a board whose battery
    input has nothing connected, and abort before SLAM."""
    import importlib.util
    spec = importlib.util.spec_from_file_location(
        "ocp_sensor_topics", os.path.join(REPO_ROOT, "scripts", "one_click_pipeline.py"))
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    assert m.sensor_topics({"sensors": {"current": "INA219"}}) == ["/battery"]
    for absent in ("NONE", "none", " None ", "", "off"):
        assert m.sensor_topics({"sensors": {"current": absent}}) == [], absent
    assert m.sensor_topics({"sensors": {"current": "INA219", "use_fake_current": True}}) == []


def test_the_gate_never_asks_for_tighter_than_nav2_promises():
    """Nav2 stops when ITS goal checker is satisfied. A leg ended 0.340 m from
    home inside a 0.35 m checker, was failed by a 0.30 m gate, and the next leg
    then started on top of its goal: "Resulting plan has 0 poses in it"."""
    pipe = open(os.path.join(REPO_ROOT, "scripts", "one_click_pipeline.py")).read()
    assert 'parser.add_argument("--goal-tolerance", type=float, default=None' in pipe
    assert 'checker.get("xy_goal_tolerance", 0.25)) + 0.05' in pipe
    import yaml
    for name in _reference_names():
        d = yaml.safe_load(open(os.path.join(REPO_ROOT, "config", "reference", f"{name}_config.yaml")))
        checker = d["nav2"]["controller_server"]["ros__parameters"]["general_goal_checker"]
        assert checker["xy_goal_tolerance"] == 0.35, f"{name}: {checker['xy_goal_tolerance']}"


def test_topics_only_runs_no_slam_no_nav2_no_map():
    """A real-sensor run verifies topics. It does not navigate, map or SLAM:
    those need the wheels to be real, and on this bench they are not."""
    pipe = open(os.path.join(REPO_ROOT, "scripts", "one_click_pipeline.py")).read()
    assert 'parser.add_argument("--topics-only", action="store_true"' in pipe
    for guard in ('if args.topics_only:\n            print("\\n[5/6] [SLAM] Skipped per --topics-only',
                  'if args.topics_only:\n            print("\\n[6/6] [NAV2] Skipped per --topics-only',
                  'if args.topics_only:\n            print("\\n[MAP] Skipped per --topics-only'):
        assert guard in pipe, guard
    # the six manoeuvres are topics too -- rates prove it talks, driving proves it moves
    assert 'parser.add_argument("--drive-test", dest="drive_test", action="store_true", default=True' in pipe


def test_a_real_imu_on_simulated_wheels_is_called_out():
    """The EKF fuses vyaw from odom AND imu. A board bolted to a bench reports
    gyro=(0, 0, 0) while the fake wheels report a turn, so the filtered heading
    is dragged toward zero on every IMU sample. Measured on the GenDrv: 8/8 legs
    in fake mode, stalled 1.615 m (jazzy) and 1.625 m (lyrical) short of the same
    goal in auto mode, both reporting "Failed to make progress". A run that mixes
    them must say so, or the transcript reads like a navigation fault."""
    pipe = open(os.path.join(REPO_ROOT, "scripts", "one_click_pipeline.py")).read()
    assert "NOT_FITTED = {" in pipe, "the not-fitted sentinel is named once"
    assert 'use_fake_wheel' in pipe and "A real IMU with simulated wheels" in pipe
    import yaml
    for name in _reference_names():
        d = yaml.safe_load(open(os.path.join(REPO_ROOT, "config", "reference", f"{name}_config.yaml")))
        ekf = d["ekf"]["ekf_filter_node"]["ros__parameters"]
        # index 11 is vyaw: both sources fuse it, which is what makes the mix bite
        assert ekf["odom0_config"][11] is True, name
        assert ekf["imu0_config"][11] is True, name


def test_a_failed_goal_reports_what_nav2_complained_about():
    """`error_code=203` is a controller TF error and `103` a planner one, and
    neither says WHICH transform was missing or how late it was. On a bench the
    log that would say dies with the container the next leg destroys. Measured
    2026-09-22: five legs aborted with TF codes on the released image and there
    was nothing afterwards to diagnose them with."""
    import importlib.util
    spec = importlib.util.spec_from_file_location(
        "ocp_complaints", os.path.join(REPO_ROOT, "scripts", "one_click_pipeline.py"))
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)

    import tempfile
    with tempfile.NamedTemporaryFile("w", suffix=".log", delete=False) as fh:
        fh.write("[controller_server] [ERROR] Could not transform odom to base_link\n" * 3)
        fh.write("[planner_server] [WARN] Lookup would require extrapolation into the past\n")
        fh.write("[bt_navigator] nothing interesting here\n")
        path = fh.name
    out = m._nav2_complaints(path)
    assert "3x" in out and "Could not transform" in out
    assert "Lookup would require extrapolation" in out
    assert "nothing interesting" not in out
    os.unlink(path)

    # A missing log is a fact about the run, not a crash.
    assert "no nav2.log" in m._nav2_complaints("/nonexistent/nav2.log")


def test_the_pipeline_asks_for_them_on_a_failed_goal():
    pipe = open(os.path.join(REPO_ROOT, "scripts", "one_click_pipeline.py")).read()
    assert 'print(_nav2_complaints(os.path.join(LOG_DIR, "nav2.log")))' in pipe


# --- the EKF's heading source ------------------------------------------------

def _all_configs():
    """Every reference, plus every bare module the rule generates: the EKF row is
    part of the one default chassis, so a fix to it must reach both."""
    for name, cfg in _refs():
        yield name, cfg
    import gen_bare_config
    for mcu in sorted(gen_bare_config.BOARDS):
        yield f"bare_{mcu}", gen_bare_config.bare_config(mcu)


def _ekf(cfg):
    ekf = (cfg.get("ekf") or {}).get("ekf_filter_node", {}).get("ros__parameters", {})
    return ekf or (cfg.get("ekf") or {})


def test_every_config_fuses_absolute_yaw_from_the_imu():
    """linorobot2_hardware's wiki specifies imu0 as yaw, vyaw, ax, ay.

    Without index 5 the EKF integrates yaw rate with nothing to correct it: the
    magnetometer that exists to anchor the heading is fused into madgwick and
    then thrown away, and the error lands in map->odom where nobody looks.
    Measured 2026-09-23 with it off: madgwick -137.3 deg against wheels -129.1,
    and the map viewer drew the scan 5-8 deg off the walls it had just built.
    """
    for name, cfg in _all_configs():
        imu0 = _ekf(cfg).get("imu0_config")
        assert imu0, f"{name}: no imu0_config"
        assert imu0[5] is True, f"{name}: imu0_config[5] (absolute yaw) is not fused"
        assert imu0[11] is True, f"{name}: imu0_config[11] (yaw rate) is not fused"


def test_every_config_fuses_the_wheel_vy():
    """Both wiki pages specify odom0 as vx, vy, vyaw.

    On a differential base vy is not missing data -- the firmware publishes it as
    0 with TWIST_COV 1e-5, which is the non-holonomic constraint stated as a
    measurement. Dropping it lets the filter drift sideways for free. On a
    mecanum base it is a real velocity and dropping it loses a whole axis.
    """
    for name, cfg in _all_configs():
        odom0 = _ekf(cfg).get("odom0_config")
        assert odom0[6] is True, f"{name}: odom0_config[6] (vx) is not fused"
        assert odom0[7] is True, f"{name}: odom0_config[7] (vy) is not fused"


def test_only_one_source_supplies_absolute_yaw():
    """Two absolute headings that disagree make the filter split the difference."""
    for name, cfg in _all_configs():
        odom0 = _ekf(cfg).get("odom0_config")
        assert odom0, f"{name}: no odom0_config"
        assert odom0[5] is False, f"{name}: odom0_config[5] also fuses absolute yaw"


# --- the simulated sensors must be a typical real one -------------------------

def _fake_peak(macro):
    import re
    src = open(os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                            "firmware", "common", "lib", "encoder", "fake_wheel.h")).read()
    m = re.search(rf"#define {macro}\s+([0-9.eE+-]+)f?\b", src)
    assert m, f"{macro} not found in fake_wheel.h"
    return float(m.group(1))


def test_the_simulated_sensors_are_a_typical_real_one():
    """Sized to the MEDIAN of the datasheet variances mcu_env.py carries, so the
    bench is not tuned against a sensor nobody sells.

    fakeWheelNoise() is uniform on +/-peak, so var = peak^2/3. The placeholders
    said 1e-5 while the accelerometer produced 5.1e-4 -- the EKF was told the
    simulated IMU was 50x quieter than it was, on every fake-mode leg.
    """
    import statistics as st
    import mcu_env
    src = open(os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                            "scripts", "mcu_env.py")).read()
    # The tables are the authority; read the medians back out of them.
    import re
    accel = [float(v) for v in re.findall(r'"accel_cov":\s*([0-9.eE+-]+)', src)]
    gyro = [float(v) for v in re.findall(r'"gyro_cov":\s*([0-9.eE+-]+)', src)]
    assert len(accel) >= 8 and len(gyro) >= 8, "the per-chip table shrank; check this test"
    for macro, want in (("FAKE_IMU_ACCEL_NOISE", st.median(accel)),
                        ("FAKE_IMU_GYRO_NOISE", st.median(gyro))):
        got = _fake_peak(macro) ** 2 / 3.0
        assert abs(got - want) / want < 0.02, (
            f"{macro} produces variance {got:.4g}; the typical real sensor is {want:.4g}")


def test_the_fake_sensor_declares_the_covariance_it_produces():
    """The simulated sensor is the one sensor whose noise is known exactly, so
    it derives its covariance instead of restating it in a second constant that
    can drift."""
    src = open(os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                            "firmware", "common", "lib", "encoder", "fake_wheel.h")).read()
    assert "constexpr float fakeCov(float peak) { return peak * peak / 3.0f; }" in src
    for macro in ("FAKE_IMU_ACCEL_NOISE", "FAKE_IMU_GYRO_NOISE", "FAKE_MAG_NOISE_T"):
        assert f"fakeCov({macro})" in src, f"{macro} covariance is not derived from its noise"
    # and the generic placeholders must no longer stand in for the fake sensor
    head = src[src.index("void initMsgs("):src.index("for (int i = 0; i < 3; i++)")]
    for placeholder in ("float accel_cov[3] = ACCEL_COV", "float gyro_cov[3] = GYRO_COV",
                        "float mag_cov[3] = MAG_COV"):
        assert placeholder not in head, f"{placeholder}: the fake sensor is using a placeholder"


def test_the_simulated_magnetometer_ships_calibrated():
    """A real robot that has run magnetometer_calibration has its hard iron
    removed; the simulated one should start there too.

    The offset is still INJECTED by applyMag, so the calibration routine has a
    real one to find -- this only makes the default robot one that has already
    found it. Uncorrected, the simulated (6, -4, 2.5) uT drags madgwick's
    heading 7.4 deg at rest, and the EKF now fuses that heading as absolute.
    Writing `mag_bias 0,0,0` in the env puts it back to uncalibrated.
    """
    src = open(os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                            "firmware", "src", "main.cpp")).read()
    block = src[src.index("static bool mag_bias_read"):src.index('envFloatVec("mag_bias"')]
    assert "sim_mag" in block, "the default is not conditioned on the mag being simulated"
    for macro in ("FAKE_MAG_BIAS_X", "FAKE_MAG_BIAS_Y", "FAKE_MAG_BIAS_Z"):
        assert macro in block, f"{macro} is not used as the simulated calibration"
    # and it must not override a calibration somebody actually supplied
    assert "!mag_bias[0] && !mag_bias[1] && !mag_bias[2]" in block
