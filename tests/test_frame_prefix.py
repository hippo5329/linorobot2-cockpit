"""Every frame_id the firmware stamps carries the robot's namespace.

The board prefixes its TOPIC names from the env's `topic_prefix`. For a while it
did not prefix the FRAME IDs inside those messages, and the two-robot bench
(2026-09-21) showed what that costs: a robot publishing `/lino1/odom/unfiltered`
stamped `frame_id: odom`, naming a frame that does not exist in its own TF tree
-- robot_state_publisher had published `lino1/odom`. The EKF found no transform
relating the two and silently ignored every message.

A bare literal here is invisible until two robots are on one DDS domain, so it
is worth a test that reads the source.
"""
import os
import re

FW = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "firmware")

# Every file that stamps a frame_id, and what it must stamp.
FRAME_SITES = {
    "common/lib/odometry/odometry.cpp": ["odom", "base_footprint"],
    "common/lib/imu/imu_interface.h": ["imu_link"],
    "common/lib/imu/mag_interface.h": ["imu_link"],
    "common/lib/range/range.cpp": ["sonar_link"],
    "common/lib/encoder/sim_wheel.h": ["imu_link"],
    "src/main.cpp": ["base_link"],
}

# `... .frame_id = micro_ros_string_utilities_set(<x>, <arg>)` -- capture <arg>.
SET_FRAME = re.compile(
    r"frame_id\s*=\s*\n?\s*micro_ros_string_utilities_set\([^,]+,\s*([^)]+)\)",
    re.S)


def _src(rel):
    with open(os.path.join(FW, rel)) as fh:
        return fh.read()


def test_every_bare_literal_is_paired_with_a_prefixed_restamp():
    """A constructor may stamp the plain frame -- it runs during static
    initialisation, where the env reads back empty, and a message with a valid
    unprefixed frame beats one with no frame at all. What it may NOT do is be
    the last word: the same file has to re-stamp that frame through
    envPrefixed() once setup() has the env, or a namespaced robot ships the
    bare name and its own EKF drops the message."""
    offenders = []
    for rel in FRAME_SITES:
        src = _src(rel)
        for arg in SET_FRAME.findall(src):
            arg = arg.strip()
            if not arg.startswith('"'):
                continue
            frame = arg.strip('"')
            if f'envPrefixed("{frame}")' not in src:
                offenders.append(f"{rel}: {arg} is never re-stamped")
    assert not offenders, "; ".join(offenders)


def test_every_frame_site_goes_through_envprefixed():
    for rel, frames in FRAME_SITES.items():
        src = _src(rel)
        for frame in frames:
            assert f'envPrefixed("{frame}")' in src, f"{rel} does not prefix {frame!r}"


def test_the_prefixer_lives_beside_the_env_reader_it_needs():
    # It reads `topic_prefix` from the env, so it cannot run during static
    # initialisation -- which is why it is in mcu_env and not a header that a
    # constructor might reach first.
    hdr = _src("common/lib/mcu_env/mcu_env.h")
    assert "const char *envPrefixed(const char *suffix);" in hdr
    assert "void        envPrefixInit(const char *compiled_fallback);" in hdr


def test_setup_initialises_the_prefixer_before_anything_stamps_a_frame():
    src = _src("src/main.cpp")
    init = src.index("envPrefixInit(")
    for later in ('envPrefixed("base_link")', "applyEnvFrames()"):
        assert src.index(later) > init, f"{later} runs before envPrefixInit()"


def test_the_simulated_sonar_gets_a_frame_too():
    """The simulated sonar branch fills range_msg field by field and never touches
    the header, so the frame has to be set once in setup() -- otherwise every
    simulation-mode board (the default) publishes /sonar with an empty frame_id, which
    no consumer can place."""
    src = _src("src/main.cpp")
    assert 'range_msg->header.frame_id' in src
    assert 'envPrefixed("sonar_link")' in src


def test_the_frames_are_applied_after_the_env_is_readable_not_in_constructors():
    """Static initialisation runs before the flash partition API is usable, so a
    constructor that reads the env gets nothing -- the trap applyEnvCovariance()
    already documents."""
    src = _src("src/main.cpp")
    assert "odometry->applyEnvFrames();" in src
    # The Odometry constructor must not be the thing that reads the env.
    odom = _src("common/lib/odometry/odometry.cpp")
    ctor = odom[odom.index("Odometry::Odometry()"):odom.index("void Odometry::applyEnvFrames")]
    assert "envPrefixed" not in ctor, "the constructor reads the env too early"
