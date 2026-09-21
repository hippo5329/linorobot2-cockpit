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


# --- params files are matched by the node's FULLY-QUALIFIED name -------------
#
# Found on the two-robot bench (2026-09-21): with `topic_prefix: lino1` the EKF
# came up healthy, advertised /lino1/odom and published nothing, because its
# params file was keyed `ekf_filter_node:` while the node was
# /lino1/ekf_filter_node. rcl matched no section, robot_localization fell back
# to its own defaults -- frequency 30.0 against a config asking for 50.0, and
# no odom0 at all -- and reported none of it.

def test_params_are_rekeyed_to_the_namespaced_node():
    out = cockpit_paths.namespace_params(
        {"ekf_filter_node": {"ros__parameters": {"frequency": 50.0}}}, "lino1")
    assert list(out) == ["/lino1/ekf_filter_node"]
    assert out["/lino1/ekf_filter_node"]["ros__parameters"]["frequency"] == 50.0


def test_no_namespace_leaves_the_file_untouched():
    data = {"ekf_filter_node": {"ros__parameters": {"frequency": 50.0}}}
    assert cockpit_paths.namespace_params(data, "") is data


def test_every_node_section_is_rekeyed_not_collapsed_to_a_wildcard():
    # `/**` would also match the node, but it would hand every section's
    # parameters to every node -- wrong for a multi-node file like nav2's.
    out = cockpit_paths.namespace_params(
        {"controller_server": {}, "bt_navigator": {}}, "lino2")
    assert sorted(out) == ["/lino2/bt_navigator", "/lino2/controller_server"]


def test_a_nested_namespace_produces_one_path():
    out = cockpit_paths.namespace_params({"slam_toolbox": {}}, "fleet/lino1")
    assert list(out) == ["/fleet/lino1/slam_toolbox"]


def test_a_key_that_is_already_a_path_is_left_alone():
    out = cockpit_paths.namespace_params({"/lino1/ekf_filter_node": {}}, "lino1")
    assert list(out) == ["/lino1/ekf_filter_node"]


def test_all_three_launchers_rekey_their_params_file():
    """A launcher that builds a params file by hand must re-key it, or the
    node it configures silently runs on defaults under a namespace."""
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    for name in ("bringup", "slam", "nav2"):
        src = open(os.path.join(root, "launchers", f"{name}.launch.py")).read()
        assert "namespace_params(" in src, f"{name}.launch.py dumps an un-rekeyed params file"
