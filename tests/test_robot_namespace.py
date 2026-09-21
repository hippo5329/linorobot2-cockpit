"""topic_prefix -> a ROS namespace / frame prefix, normalised like the firmware.

The host has to prefix its topics and TF frames the SAME way the firmware
prefixes the board's, or a prefixed robot's stack talks past its own board. The
normalisation therefore has to match scripts/mcu_env.py: letters, digits,
underscore and / only, no surrounding slashes, and an invalid value is dropped
rather than passed to the middleware (which would reject it and go silent).
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "scripts"))
import cockpit_paths  # noqa: E402


def ns(prefix):
    return cockpit_paths.robot_namespace({"base_controller": {"topic_prefix": prefix}})


def test_plain_prefix():
    assert ns("lino1") == "lino1"


def test_slashes_are_stripped():
    assert ns("/lino1/") == "lino1"
    assert ns("lino1/") == "lino1"


def test_unset_is_empty():
    assert cockpit_paths.robot_namespace({"base_controller": {}}) == ""
    assert cockpit_paths.robot_namespace({}) == ""


def test_an_invalid_prefix_is_dropped_not_passed_through():
    assert ns("bad prefix!") == ""
    assert ns("robot;rm") == ""


def test_a_nested_namespace_is_allowed():
    assert ns("fleet/lino1") == "fleet/lino1"
