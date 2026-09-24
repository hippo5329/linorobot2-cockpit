"""The simulated base is the firmware's model, not a second one.

scripts/fake_base_node.py publishes odom/unfiltered and imu/data from this
computer, so SLAM and Nav2 run with no microcontroller. That is only worth
having if it behaves like the board -- an instrument that disagrees with the
thing it stands in for sends people hunting bugs that are not there.

So it imports the transcription in drivetrain_report.py (already checked against
the firmware by tests/test_test_acc_simulation.py and test_pid_auto_tune.py)
rather than carrying its own copy, and what is left to check here is the wiring:
the kinematics round trip, the command timeout, and the rule that the wheels are
read BACK through the kinematics rather than the command being integrated.
"""
import math
import os
import sys

import pytest
import yaml

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO_ROOT, "scripts"))

import drivetrain_report as dr  # noqa: E402
import fake_base_node as fb  # noqa: E402

REF = os.path.join(REPO_ROOT, "config", "reference")


def _code_only(path):
    """The file with comments and docstrings removed.

    Scanning raw text for a constant or a macro name matches the COMMENT that
    explains where the value comes from -- which is exactly the prose these
    checks want to encourage. Parsing and re-emitting keeps the check
    behavioural: what the code does, not what it says about itself.
    """
    import ast
    tree = ast.parse(open(path, encoding="utf-8").read())
    for node in ast.walk(tree):
        if not isinstance(node, (ast.Module, ast.ClassDef, ast.FunctionDef,
                                 ast.AsyncFunctionDef)):
            continue
        body = node.body
        if (body and isinstance(body[0], ast.Expr)
                and isinstance(body[0].value, ast.Constant)
                and isinstance(body[0].value.value, str)):
            node.body = body[1:] or [ast.Pass()]
    return ast.unparse(ast.fix_missing_locations(tree))


def _params(name="gendrv"):
    with open(os.path.join(REF, f"{name}_config.yaml"), encoding="utf-8") as fh:
        return yaml.safe_load(fh)


def _rig(name="gendrv"):
    params = _params(name)
    d = dr.drivetrain(params)
    return d, fb.Wheels(d, params["kinematics"]["pid"])


def _settle(d, wheels, vx, vy, wz, seconds=4.0):
    rpm = [0.0] * 4
    for _ in range(int(seconds / 0.02)):
        rpm = wheels.step(fb.target_rpm(d, vx, vy, wz), 0.02)
    return dr._velocities(d, rpm)


def test_it_imports_without_a_ros_environment():
    """The physics has to be testable in the hermetic suite. Exiting at import
    time would put the only part worth checking behind a sourced setup.bash."""
    assert hasattr(fb, "target_rpm") and hasattr(fb, "Wheels")


def test_the_base_reaches_the_speed_it_is_commanded():
    d, wheels = _rig()
    vx, _ = _settle(d, wheels, 0.25, 0.0, 0.0)
    assert vx == pytest.approx(0.25, abs=0.01)


def test_the_base_reaches_the_yaw_rate_it_is_commanded():
    d, wheels = _rig()
    _, wz = _settle(d, wheels, 0.0, 0.0, 1.0)
    assert wz == pytest.approx(1.0, abs=0.05)


def test_a_command_beyond_the_motors_comes_back_scaled_not_clipped():
    """Kinematics scales the whole request so an unreachable command is the SAME
    motion more slowly. Clipping individual wheels would change the ratio between
    them, which on a mecanum is the direction."""
    d, _ = _rig("pico2_mecanum")
    modest = fb.target_rpm(d, 0.2, 0.0, 0.5)
    huge = fb.target_rpm(d, 2.0, 0.0, 5.0)          # 10x, far past the budget
    # Same shape, smaller magnitude: every wheel scaled by one factor.
    ratios = [h / m for h, m in zip(huge, modest) if abs(m) > 1e-6]
    assert max(ratios) == pytest.approx(min(ratios), rel=0.02), huge
    assert max(abs(w) for w in huge) <= d["command_rpm"] + 1e-6


def test_the_wheels_are_read_back_through_the_kinematics():
    """The rule that makes this instrument able to see what the bench cannot.

    Integrating cmd_vel into a pose reproduces the board's blind spot exactly:
    the same radius on both sides cancels, and a wrong turn radius is invisible.
    Here the yaw rate must come from the WHEELS, so a mecanum's (lr+fr)/2 and a
    differential's lr/2 produce measurably different motion for the same wheels.
    """
    src = open(os.path.join(REPO_ROOT, "scripts", "fake_base_node.py"),
               encoding="utf-8").read()
    tick = src[src.index("def _tick"):]
    assert "dr._velocities(" in tick, "the base must read its wheels back"
    # The pose must be integrated from the MEASURED velocity, not the command.
    assert "self.x += (meas_x" in tick and "self.yaw += meas_wz" in tick
    assert "self.x += (vx" not in tick


def test_the_same_wheels_mean_different_motion_on_different_drivetrains():
    diff, _ = _rig("gendrv")
    mec, _ = _rig("pico2_mecanum")
    rpm = [60.0, -60.0, 60.0, -60.0]        # a pure spin
    _, wz_diff = dr._velocities(diff, rpm)
    _, wz_mec = dr._velocities(mec, rpm)
    assert abs(wz_mec) < abs(wz_diff), "the mecanum's bigger radius must turn slower"


def test_a_differential_base_ignores_a_lateral_command():
    d, wheels = _rig("gendrv")
    with_y = fb.target_rpm(d, 0.2, 0.3, 0.0)
    without = fb.target_rpm(d, 0.2, 0.0, 0.0)
    assert with_y == without, "vy must be dropped before the wheels see it"


def test_a_mecanum_does_not_ignore_it():
    d, _ = _rig("pico2_mecanum")
    assert fb.target_rpm(d, 0.2, 0.3, 0.0) != fb.target_rpm(d, 0.2, 0.0, 0.0)


def test_the_command_times_out_like_the_firmware():
    """moveBase() zeroes the twist after 200 ms with no command. Without it a
    publisher that stops leaves the simulated robot driving for ever -- a failure
    mode the board does not have, which would send someone hunting a controller
    bug that is not there."""
    src = open(os.path.join(REPO_ROOT, "scripts", "fake_base_node.py"),
               encoding="utf-8").read()
    assert "200_000_000" in src
    firmware = open(os.path.join(REPO_ROOT, "firmware", "src", "main.cpp"),
                    encoding="utf-8").read()
    assert "(millis() - prev_cmd_time) >= 200" in firmware, \
        "the firmware's timeout moved; this node's must follow"


def test_it_does_not_carry_its_own_wheel_model():
    """One copy of the model. A second would drift from fake_wheel.h silently,
    and this node exists to be believed."""
    src = _code_only(os.path.join(REPO_ROOT, "scripts", "fake_base_node.py"))
    assert "import drivetrain_report as dr" in src
    assert "dr._Wheel(" in src and "dr._Pack(" in src
    for token in ("FAKE_", "gear_eff", "battery_sag", "math.exp"):
        assert token not in src, f"the node appears to model the wheel itself: {token}"


def test_the_pid_is_the_firmwares_including_anti_windup():
    src = open(os.path.join(REPO_ROOT, "scripts", "fake_base_node.py"),
               encoding="utf-8").read()
    step = src[src.index("    def step(self, target_rpm"):src.index("def target_rpm")]
    assert "i_max = pwm_max / abs(self.ki)" in step
    assert "self.integral[i] += error" in step


def test_bringup_can_run_without_a_board():
    launch = open(os.path.join(REPO_ROOT, "launchers", "bringup.launch.py"),
                  encoding="utf-8").read()
    assert '"fake_base"' in launch
    assert "fake_base_node.py" in launch
    # ...and the agent must NOT start, or it holds a serial port nothing answers.
    agent = launch[launch.index("micro_ros_agent\"] + micro_ros_args") - 900:]
    agent = agent[:agent.index("micro_ros_agent\"] + micro_ros_args")]
    assert "fake_base" in agent, "the agent still starts when the base is simulated"


def test_the_node_is_installed_with_the_package():
    """It runs from share/, like fake_laser_node."""
    cmake = open(os.path.join(REPO_ROOT, "CMakeLists.txt"), encoding="utf-8").read()
    assert "scripts" in cmake
    assert os.path.isfile(os.path.join(REPO_ROOT, "scripts", "fake_base_node.py"))


# --- the sensors, not just the motion ---------------------------------------
#
# The first version of this node published a perfect gyro and perfect encoders,
# and passed an eight-leg Nav2 run that three boards were failing. That is the
# worst possible outcome for a diagnostic instrument: an absence of reproduction
# that means nothing. The board's simulated IMU carries a fixed 0.004 rad/s bias
# and a bounded random walk, and its encoders report +/-1 rpm of noise, so the
# EKF downstream is fusing a different robot entirely.

def test_the_reported_wheel_speed_carries_the_boards_noise():
    d, wheels = _rig()
    seen = set()
    for _ in range(50):
        seen.update(round(r, 6) for r in wheels.step([0.0] * 4, 0.02))
    assert len(seen) > 5, "the reported rpm is noiseless; getRPM() adds +/-1 rpm"


def test_the_gyro_has_a_bias_and_it_is_bounded():
    d, _ = _rig()
    imu = fb.FakeIMU(d)
    for _ in range(20000):                       # far longer than any run
        imu.read(0.0, 0.0, 0.02)
        assert abs(imu.gyro_bias) <= d["gyro_bias"] + 1e-9
    # ...and it actually wandered, rather than sitting at zero.
    walked = any(abs(fb.FakeIMU(d).read(0.0, 0.0, 0.02)[0]) > 0 for _ in range(5))
    assert walked


def test_the_random_walk_rate_does_not_depend_on_the_call_rate():
    """The step scales with sqrt(dt), so a node at 50 Hz and one at 200 Hz drift
    at the same rate. Getting this wrong makes the instrument's drift a function
    of its own timer."""
    src = open(os.path.join(REPO_ROOT, "scripts", "fake_base_node.py"),
               encoding="utf-8").read()
    assert "math.sqrt(max(dt, 0.0))" in src
    firmware = open(os.path.join(REPO_ROOT, "firmware", "common", "lib", "encoder",
                                 "fake_wheel.h"), encoding="utf-8").read()
    assert "const float rw = sqrtf(dt);" in firmware


def test_the_gyro_reading_is_the_firmwares_expression():
    """angular_z * k + bias + noise, with k the scale-factor error."""
    d, _ = _rig()
    imu = fb.FakeIMU(d)
    imu.gyro_bias = 0.0
    readings = [imu.read(1.0, 0.0, 0.0)[0] for _ in range(400)]   # dt 0 -> no walk
    mean = sum(readings) / len(readings)
    assert mean == pytest.approx(1.0 * (1.0 + d["scale_error"]), abs=0.002)


def test_the_noise_constants_come_from_the_firmware_header():
    """Parsed, not restated -- the same rule the rest of the model follows."""
    d = dr.model_defaults()
    for key in ("noise_rpm", "gyro_bias", "gyro_drift", "gyro_noise",
                "accel_noise", "scale_error"):
        assert key in d, key
    src = _code_only(os.path.join(REPO_ROOT, "scripts", "fake_base_node.py"))
    for literal in ("0.004", "0.0015", "0.003", "0.03912"):
        assert literal not in src, f"{literal} is restated instead of parsed"
    # ...and every one of them is reached through the parsed table.
    for key in ("gyro_bias", "gyro_drift", "gyro_noise", "accel_noise",
                "scale_error", "noise_rpm"):
        assert f"d['{key}']" in src, key


# --- the link ----------------------------------------------------------------
#
# The boardless stack passed an eight-leg run that three boards were failing,
# and the difference is not the robot: the model, the kinematics, the limits and
# the planner are the same code. What the host lacks is micro-ROS over a wire.
# On a board /odom is STAMPED when the wheels were read and ARRIVES some
# milliseconds later, and that gap is what the TF buffer, the EKF and the
# controller contend with.

def test_a_link_defaults_to_nothing():
    """Zero delay, zero jitter, publish immediately -- so adding this changed no
    existing result, including the 6/6 host matrix."""
    w = fb.Wire()
    assert w.instant
    seen = []
    w.send(0.0, lambda: seen.append("now"))
    assert seen == ["now"], "a zero-delay link must not queue"


def test_a_delayed_message_is_held_back_and_then_delivered():
    w = fb.Wire(0.020)
    seen = []
    w.send(0.0, lambda: seen.append(1))
    w.pump(0.010)
    assert seen == [], "delivered early"
    w.pump(0.021)
    assert seen == [1]


def test_the_link_never_reorders_even_when_jitter_exceeds_the_gap():
    """A serial stream and an XRCE session deliver in order. Jitter varies the
    gap between arrivals, not their order -- a link that shuffles packets is a
    different fault, and modelling one would invent a failure the bench cannot
    have."""
    import random
    random.seed(7)
    w = fb.Wire(0.020, 0.050)          # jitter far wider than the 20 ms gap
    seen = []
    for i in range(20):
        w.send(i * 0.02, (lambda n: lambda: seen.append(n))(i))
    t = 0.0
    while t < 2.0:
        w.pump(t)
        t += 0.002
    assert len(seen) == 20, seen
    assert seen == sorted(seen), "the link reordered"


def test_jitter_actually_varies_the_arrivals():
    import random
    random.seed(3)
    w = fb.Wire(0.050, 0.020)
    gaps = []
    prev = None
    for i in range(30):
        w.send(i * 0.02, lambda: None)
    for due, _ in w.queue:
        if prev is not None:
            gaps.append(round(due - prev, 6))
        prev = due
    assert len(set(gaps)) > 1, "every gap identical -- jitter is not applied"


def test_the_stamp_stays_the_sample_time():
    """Publishing late with a late stamp hides the very thing this reproduces:
    an extrapolation request into a buffer whose newest entry is older than the
    controller expects, which is the shape of every 102/103 chased here."""
    src = open(os.path.join(REPO_ROOT, "scripts", "fake_base_node.py"),
               encoding="utf-8").read()
    tick = src[src.index("def _tick"):]
    # The stamp is taken once, from the clock, before anything is queued.
    assert "stamp = now.to_msg()" in tick
    assert "self.wire.send(" in tick
    # ...and nothing re-stamps on the way out.
    assert "header.stamp = self.get_clock" not in tick


def test_the_tf_goes_through_the_same_link_as_the_odometry():
    """A TF that arrives instantly while the odometry it describes is late is a
    robot whose transform predicts the future."""
    src = open(os.path.join(REPO_ROOT, "scripts", "fake_base_node.py"),
               encoding="utf-8").read()
    tick = src[src.index("def _tick"):]
    assert "self.wire.send(mono, lambda m=t: self.tf.sendTransform(m))" in tick
    assert "self.tf.sendTransform(t)" not in tick, "the TF bypasses the link"


def test_the_link_uses_a_monotonic_clock():
    """It models wall time on a wire. A simulated or stepped ROS clock would
    stall the link rather than the robot."""
    src = open(os.path.join(REPO_ROOT, "scripts", "fake_base_node.py"),
               encoding="utf-8").read()
    assert "time.monotonic()" in src


def test_the_link_can_be_set_from_the_config_as_well_as_a_parameter():
    src = open(os.path.join(REPO_ROOT, "scripts", "fake_base_node.py"),
               encoding="utf-8").read()
    assert 'declare_parameter("transport_delay_ms"' in src
    assert 'declare_parameter("transport_jitter_ms"' in src
    assert 'sim.get(key, 0.0)' in src, "the config's simulation block is ignored"
