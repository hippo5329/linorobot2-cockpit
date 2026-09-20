"""A jazzy image must not subscribe TwistStamped, and a lyrical one must.

The failure has no symptom other than not moving. nav2 on jazzy publishes plain
Twist; a board subscribing TwistStamped receives nothing, while still
enumerating, still publishing odometry, still answering every topic query. It
happened on the bench 2026-09-20: a pico2-jazzy image went out stamped, and the
only evidence was the goal test reporting 494 cmd_vel messages and 0.002 m of
travel.

release.yml checked this ONE WAY -- a lyrical image missing TwistStamped -- so
the jazzy-image-with-TwistStamped case passed CI. Both directions are failures.
"""
import os
import re
import sys

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO_ROOT, "scripts"))

import build_prebuilt  # noqa: E402

UNSTAMPED = ("humble", "iron", "jazzy")


def test_build_prebuilt_checks_the_artifact_in_both_directions():
    src = open(os.path.join(REPO_ROOT, "scripts", "build_prebuilt.py")).read()
    assert "TwistStamped" in src, "build_prebuilt no longer checks the /cmd_vel contract"
    assert "wants_stamped and found == 0" in src, "the missing-when-required case is gone"
    assert "not wants_stamped and found" in src, (
        "the present-when-forbidden case is gone -- that is the direction that "
        "shipped a stamped jazzy image")


def test_release_yml_check_is_not_the_only_one():
    """CI runs on a tag; a locally cut release must be checked too."""
    ci = open(os.path.join(REPO_ROOT, ".github", "workflows", "release.yml")).read()
    assert "TwistStamped" in ci
    src = open(os.path.join(REPO_ROOT, "scripts", "build_prebuilt.py")).read()
    assert "strings" in src, "the script must check the artifact itself, not rely on CI"


def test_every_profile_declares_a_distro_the_check_understands():
    for profile, (_mcu, _env, distro, _desc) in build_prebuilt.PROFILES.items():
        assert distro in UNSTAMPED or distro == "lyrical", (
            f"{profile}: distro {distro!r} is not classified by the /cmd_vel check")
