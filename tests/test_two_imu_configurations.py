"""ONE IMU configuration now: the board fuses, and publishes imu/data.

It used to be two, chosen by whether the robot had a magnetometer:

                     no magnetometer          with magnetometer
  madgwick           NOT launched             launched
  base publishes     /imu/data                /imu/data_raw + /imu/mag
  /imu/data from     the base                 madgwick
  EKF takes          angular speed (vyaw)     yaw (mag-anchored) + vyaw

The right-hand column was retired on 2026-09-25, because that pairing was a
fragility rather than a feature. `imu_filter_madgwick` matches imu/data_raw with
imu/mag through a message_filters ApproximateTime synchroniser five deep, so
imu/data was the rate of MATCHED PAIRS across a best-effort micro-ROS session --
either message can be dropped, and a lost field costs an IMU sample. Two slowed
bench legs measured the asymmetry it produces: /odom, which needs no partner,
held 33 Hz while imu/data fell to 10.

The board holds gyro, accel and field in the same 50 Hz cycle, from the same
trigger, with nothing to synchronise. So it fuses them itself
(firmware/common/lib/imu/ahrs.h, a port of that node's own filter, held to it
numerically by tests/test_ahrs_is_the_filter_it_replaces.py) and publishes the
consumer topic directly. imu/mag is still published, because
magnetometer_calibration needs it; nothing pairs against it.

What survives unchanged is the EKF rule, and it is still the dangerous one: the
shipped configs fuse ABSOLUTE yaw (imu0_config[5] = True), and a heading is only
absolute where a field anchors it. The producer moved; the rule did not.
"""
import os
import re
import textwrap

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MAIN = os.path.join(ROOT, "firmware", "src", "main.cpp")
LAUNCH = os.path.join(ROOT, "launchers", "bringup.launch.py")
AHRS = os.path.join(ROOT, "firmware", "common", "lib", "imu", "ahrs.h")


def _read(p):
    with open(p, encoding="utf-8") as fh:
        return fh.read()


# --- firmware: one topic, and the board computes the orientation -------------

def _fusion_region(src):
    """From the hasFusedOrientation() test to the end of the sensor phase.

    Keyed on the CALL, not on the whole condition: the condition grew a null
    guard (`!imu ||`) and three tests in this file went red having found nothing
    wrong -- a check that names the shape of the code instead of its behaviour,
    which is the mistake this file's neighbour already records."""
    i = src.index("hasFusedOrientation()")
    return src[i:src.index("diagTime(DIAGT_SENSORS", i)]


def test_the_board_always_publishes_the_consumer_topic():
    src = _read(MAIN)
    assert 'const char *imu_topic = "imu/data";' in src, \
        "the IMU topic forked again; imu/data_raw means something must pair it"
    assert "topicName(imu_topic)" in src, "the publisher does not use the chosen name"
    assert "imu/data_raw" not in src.split("const char *imu_topic")[1], \
        "imu/data_raw is still published somewhere after the topic is chosen"


def test_the_board_fuses_unless_the_part_already_did():
    """A BNO085 runs its own AHRS and returns a quaternion, so ahrs.h stands
    aside. Everything else is fused here, and a NEW driver must default to being
    fused rather than trusted to have filled in a quaternion it never touched --
    that failure is an identity orientation published as a heading."""
    src = _read(MAIN)
    assert "hasFusedOrientation()" in src, \
        "the board no longer asks whether the part already fused"
    iface = _read(os.path.join(ROOT, "firmware", "common", "lib", "imu",
                               "imu_interface.h"))
    assert re.search(r"virtual bool hasFusedOrientation\(\)\s*\{\s*return false;",
                     iface), "the default is no longer 'fuse it'"
    bno = _read(os.path.join(ROOT, "firmware", "common", "lib", "imu",
                             "default_imu.h"))
    assert "hasFusedOrientation() override { return true; }" in bno, \
        "the BNO085 no longer claims its own fusion"


def test_the_nine_axis_and_six_axis_paths_are_both_taken():
    src = _read(MAIN)
    block = _fusion_region(src)
    assert "ahrs.update(" in block, "the field is never fused"
    assert "ahrs.updateIMU(" in block, "a magless board has no 6-axis path"
    assert "publish_mag" in block, "the choice is not keyed on whether a field exists"


def test_the_interval_is_measured_not_assumed():
    """The same rule as the wheel model: integrate over the interval that
    actually passed, and skip one longer than a second."""
    src = _read(MAIN)
    block = _fusion_region(src)
    assert "micros()" in block, "the filter is stepped on a nominal period"
    assert "1000000UL" in block, "a stalled loop would be integrated as real motion"


def test_gravity_is_removed_by_the_board_and_not_again_by_the_ekf():
    """Whoever runs the fusion owns the subtraction, because only they have the
    orientation it needs. Doing it twice fabricates 9.81 upward.

    That the subtraction sits OUTSIDE the branch a chip-fused part skips is
    checked by brace matching in test_ahrs_is_the_filter_it_replaces.py -- here is
    the other end of the same invariant."""
    src = _read(MAIN)
    assert "AHRS::gravityFrom(" in src, "the board no longer removes gravity"
    assert "linear_acceleration.x -= " in src, \
        "gravity is computed and not subtracted"
    # The legacy node keeps its own removal, and must: a board built BEFORE
    # 2026-09-25 publishes imu/data_raw with gravity in it, and `madgwick:=true`
    # exists precisely to drive one. So the invariant is that the EKF never
    # removes it, not that the string is absent.
    launch = _read(LAUNCH)
    assert '{"remove_gravity_vector": True}' in launch, \
        "the legacy bisect path would fuse a board's gravity as acceleration"
    assert 'rp["imu0_remove_gravitational_acceleration"] = False' in launch, \
        "the EKF would subtract a gravity that has already been taken out"


def test_the_board_says_which_fusion_it_is_doing():
    src = _read(MAIN)
    assert "[imu] publishing %s" in src
    assert "9-axis" in src and "6-axis" in src, \
        "the banner no longer distinguishes the two, so a log cannot be read"


# --- launcher: madgwick is gone, the EKF rule is not ------------------------

def _madgwick(override, has_imu=True, use_mag=True):
    """Run the launcher's own decision rather than matching its spelling."""
    src = _read(LAUNCH)
    start = src.index('    madgwick_arg = context.launch_configurations.get("madgwick", "")')
    end = src.index("\n", src.index("enable_madgwick = ", start)) + 1
    ns = {"context": type("C", (), {"launch_configurations": {"madgwick": override}})(),
          "has_imu": has_imu, "use_mag": use_mag}
    exec(compile(textwrap.dedent(src[start:end]), "<madgwick>", "exec"), {}, ns)
    return ns["enable_madgwick"]


def test_madgwick_is_not_launched_by_the_hardware_any_more():
    """The whole point: no combination of hardware starts the node. It used to be
    `has_imu and use_mag`, and that is the rule being retired."""
    for has_imu in (True, False):
        for use_mag in (True, False):
            assert _madgwick("", has_imu=has_imu, use_mag=use_mag) is False, \
                f"madgwick started itself with has_imu={has_imu} use_mag={use_mag}"


def test_an_explicit_request_starts_it_only_where_there_is_an_imu():
    """A filter with no input joins the graph and publishes nothing, so anything
    waiting on /imu/data waits for ever."""
    assert _madgwick("true", has_imu=True) is True
    assert _madgwick("true", has_imu=False) is False, \
        "madgwick was started on a board with no IMU at all"
    assert _madgwick("false", has_imu=True) is False


def test_the_override_survives_for_bisecting_against_an_older_image():
    """A board built before 2026-09-25 publishes imu/data_raw and needs the node,
    so `madgwick:=true` has to keep working."""
    src = _read(LAUNCH)
    assert 'context.launch_configurations.get("madgwick", "")' in src
    assert "imu_filter_madgwick" in src, "the node cannot be launched at all now"


def test_the_ekf_stops_fusing_absolute_yaw_without_a_magnetometer():
    src = _read(LAUNCH)
    assert "if not use_mag:" in src
    assert "cfg[5] = False" in src, "imu0_config[5] is left fusing an unanchored yaw"
    assert "pin the heading to zero" in src, "the log does not say what it prevented"


def test_the_ekf_does_not_remove_gravity_because_the_board_did():
    src = _read(LAUNCH)
    assert 'rp["imu0_remove_gravitational_acceleration"] = False' in src, \
        "the EKF would subtract a gravity the board has already taken out"


def _patch(imu0_config, use_mag):
    """Run the launcher's own EKF patch against a config."""
    src = _read(LAUNCH)
    start = src.index("    if not use_mag:\n")
    end = src.index("    # rcl matches a params section", start)
    body = textwrap.dedent(src[start:end])
    ekf_data = {"ekf_filter_node": {"ros__parameters": {"imu0_config": list(imu0_config)}}}
    ns = {"use_mag": use_mag, "ekf_data": ekf_data, "print": lambda *a, **k: None}
    exec(compile(body, "<ekf patch>", "exec"), {}, ns)
    return ekf_data["ekf_filter_node"]["ros__parameters"]


YAW, VYAW = 5, 11
BASE = [False] * 15
BASE[YAW] = True
BASE[VYAW] = True
BASE[12] = True
BASE[13] = True


def test_the_patch_clears_yaw_and_keeps_angular_speed():
    out = _patch(BASE, use_mag=False)["imu0_config"]
    assert out[YAW] is False, "absolute yaw is still fused with no magnetometer"
    assert out[VYAW] is True, "angular speed was dropped -- nothing carries rotation"
    assert out[12] is True and out[13] is True, "the accelerometer axes were disturbed"


def test_the_patch_leaves_a_magnetometer_robot_alone():
    assert _patch(BASE, use_mag=True)["imu0_config"][YAW] is True


def test_the_patch_turns_the_ekfs_own_gravity_removal_off_either_way():
    for use_mag in (True, False):
        rp = _patch(BASE, use_mag=use_mag)
        assert rp["imu0_remove_gravitational_acceleration"] is False, use_mag


def test_the_patch_is_written_before_the_params_file_is_dumped():
    """It was originally computed after the temp file was written, which made it
    a no-op with no symptom."""
    src = _read(LAUNCH)
    assert src.index("cfg[5] = False") < src.index("yaml.dump(cockpit_paths.namespace_params(ekf_data"), \
        "the yaw patch happens after the EKF params are serialised"
