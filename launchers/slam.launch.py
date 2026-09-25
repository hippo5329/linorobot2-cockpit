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
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, GroupAction,
                            IncludeLaunchDescription, LogInfo, OpaqueFunction,
                            TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace
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
    /dev/ttyACM0 because pico2_mecanum_config.yaml is the default. The name has to
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

    # Multi-robot: prefix SLAM's frames and scan topic so it maps THIS robot's
    # namespace, matching bringup. Unset -> "" -> untouched.
    ns = cockpit_paths.robot_namespace(params)
    if ns:
        rp = slam_data.setdefault("slam_toolbox", {}).setdefault("ros__parameters", {})
        rp["odom_frame"] = f"{ns}/{str(rp.get('odom_frame', 'odom'))}"
        rp["map_frame"] = f"{ns}/{str(rp.get('map_frame', 'map'))}"
        rp["base_frame"] = f"{ns}/{str(rp.get('base_frame', 'base_link'))}"
        rp["scan_topic"] = f"/{ns}/scan"

    # Fully-qualified section names: under a namespace slam_toolbox is
    # /<ns>/slam_toolbox, and a file keyed by the bare name reaches it not at
    # all (see cockpit_paths.namespace_params).
    slam_temp = tempfile.NamedTemporaryFile(mode="w", suffix=".yaml", delete=False)
    yaml.dump(cockpit_paths.namespace_params(slam_data, ns), slam_temp)
    slam_temp.flush()
    slam_params_path = slam_temp.name

    slam_pkg = FindPackageShare("slam_toolbox").find("slam_toolbox")
    slam_launch_path = os.path.join(slam_pkg, "launch", "online_async_launch.py")

    actions = [
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
    if autostart.lower() in ("true", "1", "yes"):
        actions.append(activation_guard(f"/{ns}/slam_toolbox" if ns else "/slam_toolbox"))
    # Run slam_toolbox under the robot's namespace so its /map and /scan
    # subscription resolve to /<prefix>/…. The guard is a raw shell, so it is
    # NOT namespaced -- it was handed the full node path above -- and stays
    # outside the group.
    if ns:
        return [GroupAction([PushRosNamespace(ns), *[a for a in actions if not isinstance(a, TimerAction)]]),
                *[a for a in actions if isinstance(a, TimerAction)]]
    return actions


# The activation of slam_toolbox hangs on ONE volatile message.
#
# online_async_launch.py emits ChangeState(CONFIGURE) and then activates from a
# RegisterEventHandler(OnStateTransition(start_state="configuring",
# goal_state="inactive")). What feeds that handler is the node's own
# ~/transition_event topic, created by rclcpp_lifecycle with the default QoS --
# reliable, VOLATILE, depth 5 -- and published exactly once as the configure
# transition completes. The launch process subscribes to it, but if that
# subscription has not matched the just-started node's publisher at the instant
# it fires, the message is dropped, the handler never runs, and slam_toolbox
# sits in `inactive` for the rest of the run. The configure itself is a service
# call, discovered separately and retried until it answers, so it always
# succeeds; only the notification is lossy.
#
# The failure does not look like a SLAM failure. on_configure() is where
# slam_toolbox creates its publishers, so /map is listed and has a publisher
# and never carries a message. Nav2 comes up behind it and dies 30 s later on
# `Invalid frame ID "map" ... frame does not exist`, which reads as a Nav2 or a
# TF fault. Measured on a pico2 jazzy bench: "Configuring", the Ceres solver line,
# then 120 s of silence with no "[LifecycleLaunch] Slamtoolbox node is
# activating." It is not reproducible on demand; it is a discovery race.
#
# So ask, once, late, and only if it is actually needed. Reading the state
# first keeps this a no-op on every healthy run -- and makes the recovery
# visible in the log rather than papering over a race that would otherwise be
# diagnosed again from scratch.
#
# `--no-daemon` is not a detail. The ros2 CLI daemon caches the graph, and in
# this container it answered "Node not found" for the full 36 s of the loop
# against a slam_toolbox that was already active -- so the guard would have
# printed its "it did not configure" line on every HEALTHY run, which is worse
# than no guard at all. `--no-daemon` answered "active [3]" immediately from
# the same shell. Twelve short-lived participants over 36 s is a price worth
# paying for an answer about the graph that comes from the graph.
def activation_guard(node="/slam_toolbox"):
    # The node path carries the namespace: under a robot's topic_prefix the node
    # is /<prefix>/slam_toolbox, and this shell (an ExecuteProcess) is not
    # namespaced by PushRosNamespace, so it must be told the full path.
    script = """
NODE="%s"
for i in $(seq 1 12); do
  s=$(ros2 lifecycle get --no-daemon "$NODE" 2>/dev/null)
  case "$s" in
    *inactive*)
      echo "[Linorobot2 Cockpit] slam_toolbox is still inactive -- its launch file missed the transition event. Activating it."
      ros2 lifecycle set --no-daemon "$NODE" activate
      exit 0 ;;
    *active*)
      exit 0 ;;
  esac
  sleep 3
done
if [ -z "$s" ]; then
  echo "[Linorobot2 Cockpit] slam_toolbox answered no lifecycle query: it is still inside its configure step, which serves nothing while it runs."
else
  echo "[Linorobot2 Cockpit] slam_toolbox is '$s', not inactive or active: it did not configure -- check the params file."
fi
""" % node
    return TimerAction(period=15.0, actions=[
        ExecuteProcess(cmd=["bash", "-c", script], output="screen"),
    ])


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
