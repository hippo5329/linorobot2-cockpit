"""Two IMU configurations, chosen by whether the robot has a magnetometer.

The user's design, 2026-09-23:

                     no magnetometer          with magnetometer
  madgwick           NOT launched             launched
  base publishes     /imu/data                /imu/data_raw + /imu/mag
  /imu/data from     the base                 madgwick
  EKF takes          angular speed (vyaw)     yaw (mag-anchored) + vyaw

Each row is a complete set and mixing halves is the dangerous case. The one
that bites is the EKF: the shipped configs fuse ABSOLUTE yaw from /imu/data
(imu0_config[5] = True). With no madgwick nothing computes orientation, so that
field is the identity quaternion and the filter would fuse a constant zero
heading -- the estimate pinned to the starting yaw however the robot turns, with
nothing in the log. Shipping the firmware topic rename without the EKF change is
worse than shipping neither, so all three are asserted together here.

The simulation-mode Nav2 matrix is unaffected by design: every sim leg has
sim_wheels true, so publish_mag is true, so the topic names and madgwick stay
exactly as they were and the 30-leg gate does not have to be re-qualified for a
rename.
"""
import os
import re

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MAIN = os.path.join(ROOT, "firmware", "src", "main.cpp")
LAUNCH = os.path.join(ROOT, "launchers", "bringup.launch.py")


def _read(p):
    with open(p, encoding="utf-8") as fh:
        return fh.read()


# --- firmware: the topic name follows the magnetometer ----------------------

def test_the_board_publishes_imu_data_when_there_is_no_magnetometer():
    src = _read(MAIN)
    assert 'const char *imu_topic = publish_mag ? "imu/data_raw" : "imu/data";' in src, \
        "the IMU topic name is fixed again"
    assert "topicName(imu_topic)" in src, "the publisher does not use the chosen name"


def test_the_choice_is_keyed_on_publish_mag_not_a_new_flag():
    """publish_mag is already (a real magnetometer answered) OR (simulated wheels are
    synthesising a field). Keying on it is what keeps every simulation-mode leg on
    imu/data_raw, so the Nav2 matrix does not move."""
    src = _read(MAIN)
    assert re.search(r"publish_mag = envFlag\(\"pub_mag\"", src), "publish_mag is gone"
    topic_line = next(l for l in src.splitlines() if "const char *imu_topic" in l)
    assert "publish_mag" in topic_line


def test_the_board_says_which_configuration_it_is_in():
    """A topic that changes name by configuration must announce which it chose,
    or the next person reads an absent /imu/data_raw as a dead IMU."""
    src = _read(MAIN)
    assert "[imu] publishing %s" in src
    assert "no magnetometer: no madgwick" in src


# --- launcher: madgwick and the EKF ----------------------------------------

def test_madgwick_is_not_launched_without_a_magnetometer():
    src = _read(LAUNCH)
    assert "enable_madgwick = has_imu and use_mag" in src, \
        "madgwick still starts on a magless robot"


def test_the_ekf_stops_fusing_absolute_yaw_without_a_magnetometer():
    src = _read(LAUNCH)
    assert "if not use_mag:" in src
    assert "cfg[5] = False" in src, "imu0_config[5] is left fusing an identity quaternion"
    assert "pin the heading to zero" in src, "the log does not say what it prevented"


def _patch(imu0_config, use_mag):
    """Run the launcher's own EKF patch against a config."""
    src = _read(LAUNCH)
    start = src.index("    if not use_mag:\n")
    end = src.index("    # rcl matches a params section", start)
    import textwrap
    body = textwrap.dedent(src[start:end])
    ekf_data = {"ekf_filter_node": {"ros__parameters": {"imu0_config": list(imu0_config)}}}
    ns = {"use_mag": use_mag, "ekf_data": ekf_data, "print": lambda *a, **k: None}
    exec(compile(body, "<ekf patch>", "exec"), {}, ns)
    return ekf_data["ekf_filter_node"]["ros__parameters"]["imu0_config"]


YAW, VYAW = 5, 11
BASE = [False] * 15
BASE[YAW] = True
BASE[VYAW] = True
BASE[12] = True
BASE[13] = True


def test_the_patch_clears_yaw_and_keeps_angular_speed():
    out = _patch(BASE, use_mag=False)
    assert out[YAW] is False, "absolute yaw is still fused with no magnetometer"
    assert out[VYAW] is True, "angular speed was dropped -- nothing carries rotation"
    assert out[12] is True and out[13] is True, "the accelerometer axes were disturbed"


def test_the_patch_leaves_a_magnetometer_robot_alone():
    assert _patch(BASE, use_mag=True)[YAW] is True


def test_the_patch_is_written_before_the_params_file_is_dumped():
    """It was originally computed after the temp file was written, which made it
    a no-op with no symptom."""
    src = _read(LAUNCH)
    assert src.index("cfg[5] = False") < src.index("yaml.dump(cockpit_paths.namespace_params(ekf_data"), \
        "the yaw patch happens after the EKF params are serialised"
