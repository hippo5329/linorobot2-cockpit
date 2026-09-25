"""Every image links both /cmd_vel types, and its default is its own distro's.

The failure has no symptom other than not moving. nav2 on jazzy publishes plain
Twist and from kilted on TwistStamped; a board subscribing the other receives
nothing, while still enumerating, still publishing odometry, still answering
every topic query. It happened on the bench 2026-09-20: a pico2-jazzy image went
out stamped, and the only evidence was the goal test reporting 494 cmd_vel
messages and 0.002 m of travel.

Since 2026-09-25 the type is the run-time env key `stamped_cmd_vel`, defaulting
from the distro the image was built for. So "a jazzy image has no TwistStamped"
is no longer the contract (every image has it); "the image's distro stamp is the
profile's" is. A bare distro name in `strings` cannot show that -- the firmware
compares against "jazzy", so every image contains it -- hence FW_ROS_DISTRO=.
"""
import os
import re
import sys

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO_ROOT, "scripts"))

import build_prebuilt  # noqa: E402

UNSTAMPED = ("humble", "iron", "jazzy")


def test_build_prebuilt_checks_both_types_and_the_distro_stamp():
    src = open(os.path.join(REPO_ROOT, "scripts", "build_prebuilt.py")).read()
    assert '"geometry_msgs/msg/TwistStamped"' in src and '"geometry_msgs/msg/Twist"' in src, \
        "build_prebuilt no longer checks that the image links both /cmd_vel types"
    assert 'f"FW_ROS_DISTRO={distro}"' in src, \
        "build_prebuilt no longer checks the image's distro stamp -- the /cmd_vel default"


def test_the_firmware_carries_the_stamp_and_defaults_from_it():
    main = open(os.path.join(REPO_ROOT, "firmware", "src", "main.cpp")).read()
    assert '"FW_ROS_DISTRO=" FW_ROS_DISTRO' in main
    # The banner prints from the tag, which is what keeps it in the linked image.
    assert re.search(r"toolName\(app_mode\), FW_DISTRO,", main)
    assert "const char *d = FW_DISTRO;" in main


def test_release_yml_check_is_not_the_only_one():
    """CI runs on a tag; a locally cut release must be checked too."""
    ci = open(os.path.join(REPO_ROOT, ".github", "workflows", "release.yml")).read()
    assert "check_cmd_vel_contract" in ci
    src = open(os.path.join(REPO_ROOT, "scripts", "build_prebuilt.py")).read()
    assert "check_cmd_vel_contract(profile, img, distro)" in src, \
        "the script must check the artifact itself, not rely on CI"


def test_every_profile_declares_a_distro_the_check_understands():
    for profile, (_mcu, _env, distro, _desc) in build_prebuilt.PROFILES.items():
        assert distro in UNSTAMPED or distro == "lyrical", (
            f"{profile}: distro {distro!r} is not classified by the /cmd_vel check")


def _uf2(payload, base=0x10000000):
    """`payload` as a UF2 file, 256 bytes per block, the way picotool writes it."""
    import struct
    chunks = [payload[i:i + 256] for i in range(0, len(payload), 256)]
    out = b""
    for n, c in enumerate(chunks):
        hdr = struct.pack("<8I", 0x0A324655, 0x9E5D5157, 0x2000, base + 256 * n,
                          len(c), n, len(chunks), 0xE48BFF59)
        out += hdr + c.ljust(476, b"\0") + struct.pack("<I", 0x0AB16F30)
    return out


def test_a_string_across_a_uf2_block_boundary_is_found(tmp_path):
    """pico-lyrical, 2026-09-25: `strings` on the .uf2 saw the type name in two
    pieces and the check called TwistStamped missing from a good image."""
    body = (b"\0" * 240 + b"geometry_msgs/msg/TwistStamped\0geometry_msgs/msg/Twist\0"
            + b"FW_ROS_DISTRO=lyrical\0").ljust(1024, b"\0")
    img = tmp_path / "firmware.uf2"
    img.write_bytes(_uf2(body))
    assert b"geometry_msgs/msg/TwistStamped" not in img.read_bytes(), "the fixture must straddle"
    build_prebuilt.check_cmd_vel_contract("t", str(img), "lyrical")


def test_the_wrong_stamp_or_a_missing_type_fails(tmp_path):
    import pytest
    img = tmp_path / "firmware.bin"
    img.write_bytes(b"geometry_msgs/msg/Twist\0geometry_msgs/msg/TwistStamped\0FW_ROS_DISTRO=jazzy\0")
    build_prebuilt.check_cmd_vel_contract("t", str(img), "jazzy")
    with pytest.raises(SystemExit):
        build_prebuilt.check_cmd_vel_contract("t", str(img), "lyrical")
    img.write_bytes(b"geometry_msgs/msg/Twist\0FW_ROS_DISTRO=jazzy\0jazzy\0")
    with pytest.raises(SystemExit):
        build_prebuilt.check_cmd_vel_contract("t", str(img), "jazzy")
