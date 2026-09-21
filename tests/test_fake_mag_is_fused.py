"""The fake magnetometer must be fused, or the simulated heading walks away.

FakeIMUFromWheels::applyMag rotates a world field into the body frame by the
wheel heading for one purpose: to give madgwick an absolute heading that agrees
with the simulated room. bringup.launch.py then excluded it from fusion
(`... and not use_fake_mag`), so madgwick ran gyro-only, the fake gyro's bias
walked onto its clamp and stayed there, and the EKF -- which takes madgwick's
yaw as absolute and only the wheels' yaw RATE -- drifted 52 degrees from the
wheel heading in an hour. Measured at rest: wheel 59.4, EKF 7.2. Nav2 steers by
the EKF while the body follows the wheels, so every goal veers.
"""
import os
import re

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LAUNCH = os.path.join(REPO_ROOT, "launchers", "bringup.launch.py")


def use_mag_default_expr():
    src = open(LAUNCH, encoding="utf-8").read()
    # the `else:` branch that computes use_mag when no launch arg overrides it
    m = re.search(r"else:\s*\n(?:\s*#.*\n)*\s*use_mag\s*=\s*(.+)", src)
    assert m, "the default use_mag expression is gone"
    return m.group(1).strip()


def test_fake_mag_is_not_excluded_from_fusion():
    expr = use_mag_default_expr()
    assert "not use_fake_mag" not in expr, (
        f"use_mag = {expr}: the fake magnetometer is excluded from fusion again, "
        "and madgwick will integrate the gyro alone and drift off the wheels"
    )


def test_fake_mag_alone_turns_fusion_on():
    """A bare module declares mag: NONE but use_fake_mag: true -- that must fuse."""
    expr = use_mag_default_expr()
    ns = {"mag_sensor": "NONE", "use_fake_mag": True}
    assert eval(expr, {}, ns) is True, f"{expr} is False for a fake-mag bare module"


def test_no_mag_at_all_leaves_fusion_off():
    expr = use_mag_default_expr()
    ns = {"mag_sensor": "NONE", "use_fake_mag": False}
    assert eval(expr, {}, ns) is False


def test_a_real_mag_still_fuses():
    expr = use_mag_default_expr()
    ns = {"mag_sensor": "QMC5883L", "use_fake_mag": False}
    assert eval(expr, {}, ns) is True
