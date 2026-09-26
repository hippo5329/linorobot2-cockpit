"""Frontier exploration: drive Nav2 to the edge of the known map until none is left.

explore_lite (m-explore-ros2, vendored into the image) watches SLAM's /map,
finds the frontiers between known free space and unknown space, and sends Nav2
a goal at the best one, again and again, until no frontier remains; then it
drives back to where it started (return_to_init). It is how a robot builds the
map of a space it has never seen -- several rooms, and a sensor that does not
see all round (a depth camera, a mower's or a vacuum's partly blocked LiDAR)
included. Nav2 itself has no exploration: its SLAM tutorial maps by sending
goals by hand.

Needs bringup, SLAM and Nav2 running. Frontiers are searched on Nav2's GLOBAL
COSTMAP, the package's own params_costmap.yaml, not on SLAM's raw /map: the
raw map keeps unknown specks inside free space (between beams, a few cells
each), each speck is a frontier, and around the start their centroid is the
robot itself -- explore_lite sent Nav2 a goal at the robot's own position,
reached at once, again and again (measured 2026-09-26, jazzy). The costmap's
obstacle layer raytraces every scan and clears them; it tracks unknown space
(track_unknown_space: true in the template), so real frontiers remain.
Parameters are that file as installed, with only what this robot dictates
written over it:
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
    """explore_lite's parameters: the installed params_costmap.yaml, then this robot's."""
    rp = {}
    try:
        if share_dir is None:
            from ament_index_python.packages import get_package_share_directory
            share_dir = get_package_share_directory("explore_lite")
        with open(os.path.join(share_dir, "config", "params_costmap.yaml")) as fh:
            doc = yaml.safe_load(fh) or {}
        (block,) = doc.values()               # keyed by the vendor's node name
        rp = dict(block.get("ros__parameters") or {})
        # Relative, so a namespaced robot finds /<ns>/global_costmap/costmap.
        for k in ("costmap_topic", "costmap_updates_topic"):
            if isinstance(rp.get(k), str):
                rp[k] = rp[k].lstrip("/")
    except Exception:
        rp = {}
    ns = cockpit_paths.robot_namespace(params)
    base = str((params.get("slam") or {}).get("base_frame", "base_link"))
    rp.update({
        "use_sim_time": False,
        "robot_base_frame": f"{ns}/{base}" if ns else base,
        # Home when done, as the package's other config (params.yaml) has it:
        # params_costmap.yaml leaves it out and the code default is false.
        "return_to_init": True,
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
