"""The board's node name is legal for rcl whatever the robot is called.

A robot name may hold `-` or `.` (the Web UI allows both), but a ROS node name is
letters, digits and `_`, not starting with a digit. `rover-unoq` flashed, read its
env back, and then rcl refused `rover-unoq_base_node`: the board never created its
node, the agent never saw a session, and /odom stayed silent.
"""
import re

import pytest

import cockpit_paths
import mcu_env

RCL_NODE = re.compile(r"^[A-Za-z_][A-Za-z0-9_]*$")


@pytest.mark.parametrize("robot,want", [
    ("rover-unoq", "rover_unoq_base_node"),
    ("tracked_gendrv", "tracked_gendrv_base_node"),
    ("my.robot v2", "my_robot_v2_base_node"),
    ("3pi", "_3pi_base_node"),
    ("", "linorobot2_base_node"),
    (None, "linorobot2_base_node"),
])
def test_base_node_name(robot, want):
    assert cockpit_paths.base_node_name(robot) == want
    assert RCL_NODE.match(want)


def test_the_flash_writes_a_legal_node_name(reference):
    params = reference("unoq")
    params["robot"]["name"] = "rover-unoq"
    assert mcu_env.hardware_env(params)["node"] == "rover_unoq_base_node"
