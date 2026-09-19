#!/usr/bin/env python3
# ==============================================================================
# slam.launch.py — Linorobot2 Cockpit SLAM Toolbox Launcher
#
# Reads SLAM configuration from <config dir>/<robot>_config.yaml as single source of truth.
# ==============================================================================

import os
import sys
import tempfile
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO_ROOT, "scripts"))
import cockpit_paths  # noqa: E402  (the user's config dir, never the repo's)

DEFAULT_ROBOT = cockpit_paths.DEFAULT_ROBOT
CONFIG_DIR = cockpit_paths.ensure_config_dir(quiet=True)


def default_params_path():
    """
    <config dir>/<robot>_config.yaml for the default robot, or whichever robot config
    exists. One robot per file, one base controller per robot.
    """
    preferred = os.path.join(CONFIG_DIR, f"{DEFAULT_ROBOT}_config.yaml")
    if os.path.isfile(preferred):
        return preferred
    if os.path.isdir(CONFIG_DIR):
        for f in sorted(os.listdir(CONFIG_DIR)):
            if f.endswith("_config.yaml") and not f.startswith("secrets"):
                return os.path.join(CONFIG_DIR, f)
    return preferred


DEFAULT_PARAMS = default_params_path()


def resolve_params_path(context):
    """Pick the robot config this launch is actually for.

    `controller:=` only ever relabelled what DEFAULT_PARAMS already held, so a
    launch for one robot ran with another robot's file: bringup for esp32_wifi
    (transport udp4, /dev/ttyUSB0) came up as transport='serial' on
    /dev/ttyACM0 because rover_pico2_config.yaml is the default. The name has to
    select the file, not decorate it -- §3, one robot per file.

    Order: an explicit config_file, then robot:=, then controller:= by filename,
    then the config whose base_controller.name matches. `robot:=` was being
    passed by one_click_pipeline.py and silently ignored, because an undeclared
    launch argument is still a launch configuration -- it just has to be read.
    """
    explicit = (context.launch_configurations.get("config_file") or "").strip()
    if explicit and os.path.isfile(explicit) and explicit != DEFAULT_PARAMS:
        return explicit

    for key in ("robot", "controller"):
        name = (context.launch_configurations.get(key) or "").strip()
        if not name:
            continue
        cand = os.path.join(CONFIG_DIR, f"{name}_config.yaml")
        if os.path.isfile(cand):
            return cand

    controller_name = (context.launch_configurations.get("controller") or "").strip()
    if controller_name and os.path.isdir(CONFIG_DIR):
        for fname in sorted(f for f in os.listdir(CONFIG_DIR)
                            if f.endswith("_config.yaml") and not f.startswith("secrets")):
            path = os.path.join(CONFIG_DIR, fname)
            cand = load_yaml(path)
            if (cand.get("base_controller") or {}).get("name") == controller_name:
                return path

    return explicit if explicit and os.path.isfile(explicit) else DEFAULT_PARAMS



def load_yaml(path):
    if not os.path.isfile(path):
        return {}
    with open(path, "r") as f:
        try:
            return yaml.safe_load(f) or {}
        except Exception:
            return {}


def launch_setup(context, *args, **kwargs):
    config_file = resolve_params_path(context)
    use_sim_time = context.launch_configurations.get("use_sim_time", "false")
    autostart = context.launch_configurations.get("autostart", "true")
    params = load_yaml(config_file)

    slam_data = params.get("slam", {})

    # A ROS 2 params file is <node>: ros__parameters: <keys>, and the robot
    # config stores the slam block flat because that is the readable shape for a
    # single source of truth. Dumping it straight out produced a file whose very
    # first line is a scalar, and rcl rejected the whole thing with "Cannot have
    # a value before ros__parameters at line 1" -- slam_toolbox then aborted on
    # an uncaught RCLInvalidROSArgsError (exit -6) before it read one scan. Wrap
    # here rather than in the config so the config stays flat and readable.
    # A config that already carries the wrapper is passed through untouched.
    if "ros__parameters" not in slam_data.get("slam_toolbox", {}):
        slam_data = {"slam_toolbox": {"ros__parameters": slam_data}}

    slam_temp = tempfile.NamedTemporaryFile(mode="w", suffix=".yaml", delete=False)
    yaml.dump(slam_data, slam_temp)
    slam_temp.flush()
    slam_params_path = slam_temp.name

    slam_pkg = FindPackageShare("slam_toolbox").find("slam_toolbox")
    slam_launch_path = os.path.join(slam_pkg, "launch", "online_async_launch.py")

    return [
        LogInfo(msg="[Linorobot2 Cockpit] Launching SLAM Toolbox (Lifecycle Online Async)"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_launch_path),
            launch_arguments={
                "slam_params_file": slam_params_path,
                "use_sim_time": use_sim_time,
                "autostart": autostart,
            }.items(),
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "config_file",
            default_value=DEFAULT_PARAMS,
            description="Path to the robot's config YAML (single source of truth)",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation (Gazebo) clock if true",
        ),
        DeclareLaunchArgument(
            "autostart",
            default_value="true",
            description="Automatically startup the slamtoolbox stack",
        ),
        OpaqueFunction(function=launch_setup),
    ])
