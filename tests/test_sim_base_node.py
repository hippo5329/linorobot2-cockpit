"""The simulated base is the firmware's model, not a second one.

scripts/sim_base_node.py publishes odom/unfiltered and imu/data from this
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
import re
import sys

import pytest
import yaml

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO_ROOT, "scripts"))

import drivetrain_report as dr  # noqa: E402
import sim_base_node as fb  # noqa: E402

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
    src = open(os.path.join(REPO_ROOT, "scripts", "sim_base_node.py"),
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
    src = open(os.path.join(REPO_ROOT, "scripts", "sim_base_node.py"),
               encoding="utf-8").read()
    assert "200_000_000" in src
    firmware = open(os.path.join(REPO_ROOT, "firmware", "src", "main.cpp"),
                    encoding="utf-8").read()
    assert "(millis() - prev_cmd_time) >= 200" in firmware, \
        "the firmware's timeout moved; this node's must follow"


def test_it_does_not_carry_its_own_wheel_model():
    """One copy of the model. A second would drift from sim_wheel.h silently,
    and this node exists to be believed."""
    src = _code_only(os.path.join(REPO_ROOT, "scripts", "sim_base_node.py"))
    assert "import drivetrain_report as dr" in src
    assert "dr._Wheel(" in src and "dr._Pack(" in src
    for token in ("SIM_", "gear_eff", "battery_sag", "math.exp"):
        assert token not in src, f"the node appears to model the wheel itself: {token}"


def test_the_pid_is_the_firmwares_including_anti_windup():
    src = open(os.path.join(REPO_ROOT, "scripts", "sim_base_node.py"),
               encoding="utf-8").read()
    step = src[src.index("    def step(self, target_rpm"):src.index("def target_rpm")]
    assert "i_max = pwm_max / abs(self.ki)" in step
    assert "self.integral[i] += error" in step


def test_bringup_can_run_without_a_board():
    launch = open(os.path.join(REPO_ROOT, "launchers", "bringup.launch.py"),
                  encoding="utf-8").read()
    assert '"sim_base"' in launch
    assert "sim_base_node.py" in launch
    # ...and the agent must NOT start, or it holds a serial port nothing answers.
    agent = launch[launch.index("micro_ros_agent\"] + micro_ros_args") - 900:]
    agent = agent[:agent.index("micro_ros_agent\"] + micro_ros_args")]
    assert "sim_base" in agent, "the agent still starts when the base is simulated"


def test_the_node_is_installed_with_the_package():
    """It runs from share/, like sim_laser_node."""
    cmake = open(os.path.join(REPO_ROOT, "CMakeLists.txt"), encoding="utf-8").read()
    assert "scripts" in cmake
    assert os.path.isfile(os.path.join(REPO_ROOT, "scripts", "sim_base_node.py"))


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
    imu = fb.SimIMU(d)
    for _ in range(20000):                       # far longer than any run
        imu.read(0.0, 0.0, 0.02)
        assert abs(imu.gyro_bias) <= d["gyro_bias"] + 1e-9
    # ...and it actually wandered, rather than sitting at zero.
    walked = any(abs(fb.SimIMU(d).read(0.0, 0.0, 0.02)[0]) > 0 for _ in range(5))
    assert walked


def test_the_random_walk_rate_does_not_depend_on_the_call_rate():
    """The step scales with sqrt(dt), so a node at 50 Hz and one at 200 Hz drift
    at the same rate. Getting this wrong makes the instrument's drift a function
    of its own timer."""
    src = open(os.path.join(REPO_ROOT, "scripts", "sim_base_node.py"),
               encoding="utf-8").read()
    assert "math.sqrt(max(dt, 0.0))" in src
    firmware = open(os.path.join(REPO_ROOT, "firmware", "common", "lib", "encoder",
                                 "sim_wheel.h"), encoding="utf-8").read()
    assert "const float rw = sqrtf(dt);" in firmware


def test_the_gyro_reading_is_the_firmwares_expression():
    """angular_z * k + bias + noise, with k the scale-factor error."""
    d, _ = _rig()
    imu = fb.SimIMU(d)
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
    src = _code_only(os.path.join(REPO_ROOT, "scripts", "sim_base_node.py"))
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
    src = open(os.path.join(REPO_ROOT, "scripts", "sim_base_node.py"),
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
    src = open(os.path.join(REPO_ROOT, "scripts", "sim_base_node.py"),
               encoding="utf-8").read()
    tick = src[src.index("def _tick"):]
    assert "self.wire.send(mono, lambda m=t: self.tf.sendTransform(m))" in tick
    assert "self.tf.sendTransform(t)" not in tick, "the TF bypasses the link"


def test_the_base_does_not_publish_a_transform_by_default():
    """The board does not either. main.cpp has no TransformBroadcaster -- it
    publishes odom/unfiltered as a topic and the EKF owns `odom -> base_link`.
    Two publishers on one transform is the collision bringup already avoids by
    forcing madgwick's publish_tf false, and here it silently invalidated a
    latency sweep: the EKF's fresh transform masked this node's delayed one, so
    legs at 800 and 1200 ms appeared to navigate against 0.2-0.5 s tolerances."""
    src = open(os.path.join(REPO_ROOT, "scripts", "sim_base_node.py"),
               encoding="utf-8").read()
    assert 'declare_parameter("publish_tf", False)' in src

    firmware = open(os.path.join(REPO_ROOT, "firmware", "src", "main.cpp"),
                    encoding="utf-8").read()
    assert "TransformBroadcaster" not in firmware,         "the board publishes a TF now; this node's default should follow it"

    ekf = open(os.path.join(REPO_ROOT, "config", "reference", "gendrv_config.yaml"),
               encoding="utf-8").read()
    assert "publish_tf: true" in ekf, "the EKF no longer owns odom->base_link"


def test_the_link_uses_a_monotonic_clock():
    """It models wall time on a wire. A simulated or stepped ROS clock would
    stall the link rather than the robot."""
    src = open(os.path.join(REPO_ROOT, "scripts", "sim_base_node.py"),
               encoding="utf-8").read()
    assert "time.monotonic()" in src


def test_the_link_can_be_set_from_the_config_as_well_as_a_parameter():
    src = open(os.path.join(REPO_ROOT, "scripts", "sim_base_node.py"),
               encoding="utf-8").read()
    assert 'declare_parameter("transport_delay_ms"' in src
    assert 'declare_parameter("transport_jitter_ms"' in src
    assert 'sim.get(key, 0.0)' in src, "the config's simulation block is ignored"


# ---------------------------------------------------------------------------
# The interval the model is stepped on.
#
# This node stepped the plant, the pose and the IMU on the nominal 1/rate, which
# is deterministic and therefore ideal for CI -- and is also what made it blind
# to the defect it was most needed for. The firmware steps on micros() DELTAS,
# so a late callback on a loaded box is a long interval there and a 20 ms one
# here, and the divergence that put a simulated robot 30 m outside a 6 m room
# could not be reproduced boardless at all. Under-integrating has a second cost:
# the pose then advances slower than wall time while Nav2 plans in wall time, so
# the robot goes sluggish under load for a reason no board has.
def _tick_body():
    code = _code_only(os.path.join(REPO_ROOT, "scripts", "sim_base_node.py"))
    start = code.index("def _tick(self)")
    rest = code[start + 1:]
    end = rest.find("\n    def ")
    return rest[:end if end != -1 else len(rest)]


def test_the_model_is_stepped_on_measured_time_not_the_nominal_rate():
    body = _tick_body()
    assert "self.prev_mono" in body, "the tick keeps no previous reading to subtract"
    assert "* self.dt" not in body, (
        "the tick still integrates on the nominal period; the firmware integrates "
        "on the interval that actually passed")
    assert "/ self.dt" not in body, "the IMU still differentiates on the nominal period"


def test_a_suspended_process_does_not_step_the_model():
    """SimEncoder::integrate()'s own upper guard, in seconds. A step longer than
    this is a stopped world, not a robot, and integrating it invents motion."""
    header = open(os.path.join(REPO_ROOT, "firmware", "common", "lib", "encoder",
                               "sim_wheel.h"), encoding="utf-8").read()
    body = header[header.index("void integrate()"):header.index("public:")]
    m = re.search(r"dt\s*>\s*(\d+)UL", body)
    assert m, "sim_wheel.h no longer guards a long interval"
    secs = int(m.group(1)) / 1e6
    assert f"dt > {secs}" in _tick_body(), (
        f"the firmware skips an interval past {secs} s and this node does not")


def test_the_nominal_rate_is_still_what_the_timer_runs_at():
    """Measured intervals must not turn into a free-running loop: the callback
    rate is still the firmware's CONTROL_TIMER."""
    code = _code_only(os.path.join(REPO_ROOT, "scripts", "sim_base_node.py"))
    assert "self.create_timer(self.dt, self._tick)" in code


# ---------------------------------------------------------------------------
# The induced stall. It exists because a fast host will not stutter to order:
# six 40-goal legs on a 32-core host at load 2.4 produced not one tick past
# 100 ms, so the interval that breaks an unbounded Euler step never occurred and
# the instrument could not have reproduced it however long it ran.
def test_the_stall_lands_before_the_interval_is_measured():
    """A stall measured AFTER dt is computed is a stall the model never sees --
    the whole point is that the plant is handed the long interval."""
    body = _tick_body()
    sleep_at = body.index("time.sleep(self.stall)")
    measure_at = body.index("dt = mono - self.prev_mono")
    assert sleep_at < measure_at, (
        "the stall happens after the interval is measured, so the model is still "
        "told the nominal period")
    assert body.index("mono = time.monotonic()", sleep_at) < measure_at, (
        "the clock is not re-read after the stall, so dt misses it entirely")


def test_a_stall_without_a_measured_interval_is_refused():
    """Stalling a node that integrates on the nominal period changes nothing and
    looks like it changed something -- the worst of both."""
    code = _code_only(os.path.join(REPO_ROOT, "scripts", "sim_base_node.py"))
    assert "control_stall_ms needs measured_dt" in code, \
        "the combination is silently accepted"


def test_the_stall_is_off_by_default():
    """Every other leg, and CI, must be unaffected. The parameter defaults to -1,
    which means "read the config", and the config's own default is zero -- so the
    off state has to be checked at BOTH ends or one of them can drift."""
    code = _code_only(os.path.join(REPO_ROOT, "scripts", "sim_base_node.py"))
    assert "declare_parameter('control_stall_ms', -1.0)" in code, \
        "the stall no longer defers to the config (_code_only re-emits with single quotes)"
    assert "_link('control_stall_ms', 'control_stall_ms')" in code, \
        "the stall is not read through the same -1-means-config helper as the link"
    # _link's own fallback is 0.0, which is what makes an unset config mean off.
    link = code[code.index("def _link("):]
    link = link[:link.index("self.wire = Wire(")]
    assert "sim.get(key, 0.0)" in link, "the config fallback is no longer zero"


def test_a_stall_can_be_aimed_at_the_command_change():
    """WHERE the stall lands decides whether the sweep is an experiment.

    Measured on the unsliced model: a 920 ms interval taken at cruise duty peaks
    at 70.7 rpm against a 140 rpm ceiling -- no divergence, because the driving
    term (no_load - rpm) is nearly zero at equilibrium. The same interval at a
    full-duty step reaches 324, and from rest 574. A fixed cadence therefore
    fires mostly where nothing happens, and a green sweep of those would have
    been read as 'a starved loop is harmless'."""
    body = _tick_body()
    assert "self.stall_on_change" in body, \
        "the stall cannot be aimed at a command change"
    change_at = body.index("self.stall_on_change")
    sleep_at = body.index("time.sleep(self.stall)")
    assert change_at < sleep_at, "the trigger is decided after the stall is taken"
    assert "self.prev_cmd" in body, "nothing remembers the previous command"


def test_the_aimed_stall_is_off_by_default():
    code = _code_only(os.path.join(REPO_ROOT, "scripts", "sim_base_node.py"))
    assert "declare_parameter('control_stall_on_command_change', False)" in code


# ---------------------------------------------------------------------------
# The link's BANDWIDTH, and the best-effort drops that follow from running out.
#
# Latency alone is not what a serial link does to a robot. micro-ROS over 8N1
# carries baudrate/10 bytes per second, the 50 Hz triple is about 1170 bytes a
# cycle, and the firmware publishes best effort -- so what does not fit is not
# late, it is gone. The bench has already shown what that costs asymmetrically:
# /odom at 33 Hz beside /imu/data at 10 Hz on the same leg, because madgwick
# pairs imu/data_raw with imu/mag and an unpaired sample yields nothing.
def _wire(bytes_per_s):
    return fb.Wire(0.0, 0.0, bytes_per_s)


def test_an_unlimited_link_is_the_default_and_drops_nothing():
    w = fb.Wire()
    assert w.bytes_per_s == 0.0
    assert w.instant is True
    for i in range(200):
        assert w.afford(i * 0.02, "odom") is True
    assert w.dropped == {}


def test_a_921600_link_carries_the_fifty_hertz_triple():
    """92 kB/s against about 58 kB/s of traffic: it fits, with room."""
    w = _wire(921600 / 10.0)
    for i in range(500):
        t = i * 0.02
        for kind in ("odom", "imu", "mag"):
            w.afford(t, kind)
    assert w.dropped == {}, f"a 921600 link should carry the triple, dropped {w.dropped}"


def test_a_115200_link_cannot_and_says_which_messages_went():
    """11.5 kB/s against 58 kB/s. The drops are the point, and they must be
    counted per topic -- a total would hide the asymmetry entirely."""
    w = _wire(115200 / 10.0)
    for i in range(500):
        t = i * 0.02
        for kind in ("odom", "imu", "mag"):
            w.afford(t, kind)
    assert w.dropped, "a 115200 link carried 58 kB/s of traffic"
    assert sum(w.dropped.values()) > sum(w.sent.values()), \
        "most of the traffic should not have fitted"
    assert set(w.dropped) & {"odom", "imu", "mag"}, w.dropped


def test_the_budget_does_not_accumulate_across_a_quiet_second():
    """A UART has no backlog to spend later. A deep bucket would let a quiet
    stretch pay for a burst the link could never carry, which turns a hard limit
    into an average and hides exactly the moment that hurts."""
    w = _wire(115200 / 10.0)
    w.afford(0.0, "odom")
    sent_after_quiet = 0
    # one second of silence, then as many odom messages as the bucket allows
    for _ in range(50):
        if w.afford(1.0, "odom"):
            sent_after_quiet += 1
    cap = (115200 / 10.0) * 0.02
    assert sent_after_quiet <= int(cap / fb.Wire.SIZES["odom"]) + 1, (
        f"{sent_after_quiet} messages went at one instant on a "
        f"{cap:.0f} byte bucket -- the quiet second was banked")


def test_a_dropped_message_is_not_published_at_all():
    """Best effort. The alternative -- queueing it -- would model a reliable
    link, which is the configuration the firmware explicitly does not use at
    these rates because it costs an agent round trip per message."""
    w = _wire(1.0)          # one byte a second: nothing fits
    calls = []
    w.send(0.0, lambda: calls.append(1), "odom")
    assert calls == [], "a message that did not fit was published anyway"


def test_a_send_with_no_kind_is_never_rate_limited():
    """The TF and any future publisher must keep working until somebody gives
    them a size, rather than silently vanishing on a rate-limited leg."""
    w = _wire(1.0)
    calls = []
    w.send(0.0, lambda: calls.append(1))
    assert calls == [1]
