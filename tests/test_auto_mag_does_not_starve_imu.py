"""`mag: AUTO` must not switch on heading fusion, or the IMU topic dies.

imu_filter_madgwick with use_mag=true synchronises /imu/data_raw against
/imu/mag. If no magnetometer ever publishes, the synchroniser never fires and
/imu/data is SILENT -- so a 6-axis part under `mag: AUTO` produces no fused IMU
at all. The topic verifier then reports

    /imu/data : NO DATA (0 msgs received in 10.0s)

and the run aborts before SLAM, on a board whose accelerometer and gyroscope
are both working perfectly.

Measured on a bench Pico 2 with an LSM6DSOX (accel + gyro, no magnetometer) on
2026-09-23, under the same `mag: AUTO` that the auto-detection path wants.

The two failure modes are not symmetric, and that is what decides the default:
a wrong `true` yields NO /imu/data; a wrong `false` yields /imu/data without
heading anchoring -- degraded, and it says so in the launch log. Fail toward
the mode that still produces data.

Since 2026-09-25 the board fuses its own orientation and there is no
synchroniser to starve, but the rule stands for the EKF's sake: `use_mag` is
what makes it fuse the board's yaw as ABSOLUTE, and under AUTO the launch cannot
tell a field-anchored yaw from a 6-axis part's drifting gyro integral.
"""
import os
import re

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LAUNCH = os.path.join(ROOT, "launchers", "bringup.launch.py")


def _src():
    with open(LAUNCH, encoding="utf-8") as fh:
        return fh.read()


def _decide(mag_sensor, use_sim_mag):
    """Run the launcher's own expression against a sensors block."""
    src = _src()
    start = src.index("        auto_mag = ")
    end = src.index("\n", src.index("use_mag = (not auto_mag"))
    snippet = src[start:end]
    ns = {"mag_sensor": mag_sensor, "use_sim_mag": use_sim_mag}
    exec(snippet.replace("        ", "", 1).replace("\n        ", "\n"), {}, ns)
    return ns["use_mag"]


def test_auto_does_not_claim_a_magnetometer():
    assert _decide("AUTO", False) is False
    assert _decide("auto", False) is False
    assert _decide("", False) is False


def test_a_named_magnetometer_is_still_fused():
    """Naming the part is the promise. The ICM-20948 and the GenDrv both do."""
    assert _decide("AK09918", False) is True
    assert _decide("ICM20948", False) is True
    assert _decide("QMC5883L", False) is True


def test_none_is_still_not_fused():
    assert _decide("NONE", False) is False


def test_the_simulated_magnetometer_is_still_fused():
    """Unchanged, and load-bearing: without it madgwick integrates the sim
    gyro alone, its bias walks onto the clamp, and the body ends up 52 degrees
    from where Nav2 thinks it points (measured on a bare Pico 2)."""
    assert _decide("NONE", True) is True
    assert _decide("AUTO", True) is True


def test_the_operator_is_told_that_fusion_is_off():
    """A silent downgrade is how a heading problem becomes a mystery a month
    later. The log names the key and the fix."""
    src = _src()
    assert "will not fuse absolute yaw" in src
    assert re.search(r"mag:\s*AK09918", src), "the message does not say how to turn it on"
