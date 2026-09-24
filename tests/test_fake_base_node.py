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
    src = open(os.path.join(REPO_ROOT, "scripts", "fake_base_node.py"),
               encoding="utf-8").read()
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
