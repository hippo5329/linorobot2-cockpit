"""The simulated magnetometer must be fused, or the simulated heading walks away.

SimIMUFromWheels::applyMag rotates a world field into the body frame by the
wheel heading for one purpose: to give madgwick an absolute heading that agrees
with the simulated room. bringup.launch.py then excluded it from fusion
(`... and not use_sim_mag`), so madgwick ran gyro-only, the simulated gyro's bias
walked onto its clamp and stayed there, and the EKF -- which takes madgwick's
yaw as absolute and only the wheels' yaw RATE -- drifted 52 degrees from the
wheel heading in an hour. Measured at rest: wheel 59.4, EKF 7.2. Nav2 steers by
the EKF while the body follows the wheels, so every goal veers.
"""
import os
import re

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LAUNCH = os.path.join(REPO_ROOT, "launchers", "bringup.launch.py")


def use_mag_default(mag_sensor, use_sim_mag):
    """Run the launcher's own default branch and return what it decided.

    This used to lift the `use_mag = ...` line out with a regex that required
    it to follow the comments immediately, and eval the string. The decision
    grew a second statement above it (auto_mag, for `mag: AUTO`) and all four
    tests in this file went red having found nothing wrong -- a check that names
    the SHAPE of the code rather than its behaviour. Execute the block instead.
    """
    import textwrap
    src = open(LAUNCH, encoding="utf-8").read()
    start = src.index("        auto_mag = ")
    end = src.index("\n", src.index("use_mag = (not auto_mag")) + 1
    ns = {"mag_sensor": mag_sensor, "use_sim_mag": use_sim_mag}
    exec(compile(textwrap.dedent(src[start:end]), "<use_mag>", "exec"), {}, ns)
    return ns["use_mag"]


def test_sim_mag_is_not_excluded_from_fusion():
    """The regression this file exists for: `... and not use_sim_mag` put the
    simulated field outside the fusion. Asked as behaviour, so it holds however
    the expression is spelled."""
    assert use_mag_default("NONE", True) is True, (
        "the simulated magnetometer is excluded from fusion again, and madgwick will "
        "integrate the gyro alone and drift off the wheels")


def test_sim_mag_alone_turns_fusion_on():
    """A bare module declares mag: NONE but use_sim_mag: true -- that must fuse."""
    assert use_mag_default("NONE", True) is True, "False for a simulated-mag bare module"


def test_no_mag_at_all_leaves_fusion_off():
    """And this is the real-robot case: simulation mode off, no magnetometer fitted."""
    assert use_mag_default("NONE", False) is False


def test_a_real_mag_still_fuses():
    assert use_mag_default("QMC5883L", False) is True


def test_auto_is_not_a_promise_of_a_magnetometer():
    """AUTO means "ask the bus", which the launch cannot do. Fusing on it starves
    /imu/data on a 6-axis part; see test_auto_mag_does_not_starve_imu.py."""
    assert use_mag_default("AUTO", False) is False
