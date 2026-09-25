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
    assert "FW_ROS_DISTRO=$distro" in ci
    src = open(os.path.join(REPO_ROOT, "scripts", "build_prebuilt.py")).read()
    assert "strings" in src, "the script must check the artifact itself, not rely on CI"


def test_every_profile_declares_a_distro_the_check_understands():
    for profile, (_mcu, _env, distro, _desc) in build_prebuilt.PROFILES.items():
        assert distro in UNSTAMPED or distro == "lyrical", (
            f"{profile}: distro {distro!r} is not classified by the /cmd_vel check")
