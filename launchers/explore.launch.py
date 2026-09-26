"""Frontier exploration: drive Nav2 to the edge of the known map until none is left.

explore_lite (m-explore-ros2, vendored into the image) watches SLAM's /map,
finds the frontiers between known free space and unknown space, and sends Nav2
a goal at the best one, again and again, until no frontier remains; then it
drives back to where it started (return_to_init). It is how a robot builds the
map of a space it has never seen -- several rooms, and a sensor that does not
see all round (a depth camera, a mower's or a vacuum's partly blocked LiDAR)
included. Nav2 itself has no exploration: its SLAM tutorial maps by sending
goals by hand.

Needs bringup, SLAM and Nav2 running. Parameters are the package's own
params.yaml as installed, with only what this robot dictates written over it:
use_sim_time false (the vendor launch file defaults it TRUE), the base frame
from the SLAM block, the namespace, and last the robot config's own
`explore: {ros__parameters: {...}}` block for tuning.
"""
import os
import sys

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch_ros.actions import Node

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO_ROOT, "scripts"))
import cockpit_paths  # noqa: E402


def explore_params(params: dict, share_dir=None) -> dict:
    """explore_lite's parameters: the installed params.yaml, then this robot's."""
    rp = {}
    try:
        if share_dir is None:
            from ament_index_python.packages import get_package_share_directory
            share_dir = get_package_share_directory("explore_lite")
        with open(os.path.join(share_dir, "config", "params.yaml")) as fh:
            doc = yaml.safe_load(fh) or {}
        (block,) = doc.values()               # keyed "/**"
        rp = dict(block.get("ros__parameters") or {})
    except Exception:
        rp = {}
    ns = cockpit_paths.robot_namespace(params)
    base = str((params.get("slam") or {}).get("base_frame", "base_link"))
    rp.update({
        "use_sim_time": False,
        "robot_base_frame": f"{ns}/{base}" if ns else base,
    })
    rp.update(((params.get("explore") or {}).get("ros__parameters")) or {})
    return rp


def launch_setup(context, *args, **kwargs):
    config_file = context.launch_configurations.get("config_file", "")
    with open(config_file) as fh:
        params = yaml.safe_load(fh) or {}
    rp = explore_params(params)
    ns = cockpit_paths.robot_namespace(params)
    return [
        LogInfo(msg=f"[explore] frontier exploration on {rp.get('costmap_topic', 'map')}, "
                    f"base {rp['robot_base_frame']}, return_to_init {rp.get('return_to_init')}"),
        Node(package="explore_lite", executable="explore", name="explore_node",
             namespace=ns or None, parameters=[rp], output="screen"),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("config_file", description="Path to the robot's config YAML"),
        OpaqueFunction(function=launch_setup),
    ])
