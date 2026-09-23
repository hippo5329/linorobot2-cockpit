#!/usr/bin/env python3
# ==============================================================================
# nav2.launch.py — Linorobot2 Cockpit Nav2 Launcher
#
# Reads Nav2 configuration from <config dir>/<robot>_config.yaml as single source of truth.
# ==============================================================================

import copy
import os
import sys
import re
import tempfile
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LoadComposableNodes, Node, PushRosNamespace
from launch_ros.descriptions import ComposableNode
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


def _managed_node_names(src: str) -> list:
    """The servers navigation_launch.py starts, read from the file that will start them.

    Spelled two different ways across releases -- a module-level
    `lifecycle_nodes = [...]` in the jazzy era, a `get_lifecycle_nodes()` returning
    a tuple in nav2 1.5.1 -- and the set itself grew (route_server, docking_server,
    following_server). Reading it beats hardcoding either list: a manager handed a
    node that was never started waits for it forever, and one missing a node leaves
    that node unconfigured while the rest go active.
    """
    for pattern in (
        r"lifecycle_nodes\s*=\s*\[(.*?)\]",
        r"def get_lifecycle_nodes\([^)]*\):\s*return\s*[\(\[](.*?)[\)\]]",
    ):
        m = re.search(pattern, src, re.S)
        if m:
            names = re.findall(r"'([a-z0-9_]+)'", m.group(1))
            if names:
                return names
    return []


_NAV2_FRAME_KEYS = {"global_frame", "robot_base_frame", "odom_frame", "base_frame",
                    "base_frame_id", "odom_frame_id", "robot_base_frame_id", "fixed_frame"}
# Topic values nav2 names that are ROOT-absolute in the shipped config and so
# would ignore the namespace; rewrite them under /<prefix>/.
_NAV2_TOPIC_KEYS = {"topic", "scan_topic", "map_topic", "odom_topic",
                    "footprint_topic", "cmd_vel_in_topic", "cmd_vel_out_topic",
                    "local_costmap_topic", "global_costmap_topic",
                    "local_footprint_topic", "global_footprint_topic", "state_topic"}


def _prefix_nav2_namespace(node, ns):
    """Prefix nav2's frame names and its absolute topic references in place, so
    a namespaced nav2 stack refers to this robot's frames and topics.
    - frame values      -> <ns>/<frame>
    - absolute topics    (/scan) -> /<ns>/scan ; relative ones are left to the
      namespace to resolve.
    Booleans and non-strings are left alone (e.g. map_subscribe_transient_local)."""
    if isinstance(node, dict):
        for k, v in node.items():
            if isinstance(v, str) and k in _NAV2_FRAME_KEYS:
                node[k] = f"{ns}/{v}" if v and not v.startswith(f"{ns}/") else v
            elif isinstance(v, str) and k in _NAV2_TOPIC_KEYS and v.startswith("/"):
                node[k] = f"/{ns}{v}" if not v.startswith(f"/{ns}/") else v
            else:
                _prefix_nav2_namespace(v, ns)
    elif isinstance(node, list):
        for item in node:
            _prefix_nav2_namespace(item, ns)


def has_real_sonar(params) -> bool:
    """Does this robot actually have an HC-SR04 wired?

    The pins are the robot's own answer. The firmware reads the same two keys
    and publishes /sonar from the hardware only when both are >= 0; a board
    that reports -1 never drives a trigger line and never echoes one back.

    This matters far more than it looks. A collision_monitor source that does
    not publish does not degrade quietly -- it STOPS THE ROBOT, by design:

        [collision_monitor]: Robot to stop due to invalid source.
        Either due to data not published yet, or to lack of new data

    That is the correct fail-safe for a sensor that died mid-drive. It is also
    a robot that can never move if the sensor was never fitted. So the source
    is declared from the pins, not from taste, and a config that asks for one
    on a board with no pins is overruled below rather than obeyed.
    """
    pins = ((params or {}).get("base_controller") or {}).get("pins") or {}
    sonar = pins.get("sonar") or {}
    try:
        return int(sonar.get("trigger", -1)) >= 0 and int(sonar.get("echo", -1)) >= 0
    except (TypeError, ValueError):
        return False


def launch_setup(context, *args, **kwargs):
    config_file = resolve_params_path(context)
    use_sim_time = context.launch_configurations.get("use_sim_time", "false")
    autostart = context.launch_configurations.get("autostart", "true")
    map_file = context.launch_configurations.get("map", "")
    distro = context.launch_configurations.get("distro", os.environ.get("ROS_DISTRO", "jazzy")).strip().lower()

    params = load_yaml(config_file)
    nav2_data = copy.deepcopy(params.get("nav2", {}))

    stamped_cmd_vel_arg = context.launch_configurations.get("stamped_cmd_vel", "").strip().lower()
    if stamped_cmd_vel_arg in ("true", "1", "yes"):
        stamped_cmd_vel = True
    elif stamped_cmd_vel_arg in ("false", "0", "no"):
        stamped_cmd_vel = False
    else:
        # Follow Nav2 distro defaults: Stamped (TwistStamped) on Lyrical, Unstamped (Twist) on Jazzy
        if distro == "lyrical":
            stamped_cmd_vel = True
        else:
            kine_stamped = params.get("kinematics", {}).get("stamped_cmd_vel", "auto")
            if isinstance(kine_stamped, bool):
                stamped_cmd_vel = kine_stamped
            elif str(kine_stamped).lower() in ("true", "1", "yes"):
                stamped_cmd_vel = True
            else:
                stamped_cmd_vel = False

    # Synchronize stamped_cmd_vel across Nav2 servers
    for server_name in [
        "controller_server",
        "behavior_server",
        "velocity_smoother",
        "collision_monitor",
        "docking_server",
    ]:
        srv_params = nav2_data.setdefault(server_name, {}).setdefault("ros__parameters", {})
        srv_params["enable_stamped_cmd_vel"] = stamped_cmd_vel

    # Cross-distro adaptation: Lyrical (Nav2 >= 1.5.1 / Kilted) vs Jazzy
    if distro == "lyrical":
        # 1. bt_navigator uses error_code_name_prefixes instead of error_code_names
        bt_params = nav2_data.setdefault("bt_navigator", {}).setdefault("ros__parameters", {})
        if "error_code_names" in bt_params:
            del bt_params["error_code_names"]
        bt_params["error_code_name_prefixes"] = [
            "compute_path", "follow_path", "route", "assisted_teleop",
            "backup", "dock_robot", "drive_on_heading", "spin", "undock_robot", "wait"
        ]
        # 2. controller_server nests primary_controller inside FollowPath
        ctrl_params = nav2_data.setdefault("controller_server", {}).setdefault("ros__parameters", {})
        follow_path = ctrl_params.setdefault("FollowPath", {})
        if "primary_controller" in follow_path and isinstance(follow_path["primary_controller"], str):
            primary_plugin = follow_path["primary_controller"]
            follow_path["primary_controller"] = {
                "plugin": primary_plugin,
                "desired_linear_vel": follow_path.pop("desired_linear_vel", 0.4),
                "lookahead_dist": follow_path.pop("lookahead_dist", 0.6),
                "min_lookahead_dist": follow_path.pop("min_lookahead_dist", 0.3),
                "max_lookahead_dist": follow_path.pop("max_lookahead_dist", 0.9),
                "lookahead_time": 1.5,
                "rotate_to_heading_angular_vel": 0.75,
                "transform_tolerance": 0.1,
                "use_velocity_scaled_lookahead_dist": False,
                "min_approach_linear_velocity": 0.05,
                "approach_velocity_scaling_dist": 0.6,
                "use_collision_detection": True,
                "max_allowed_time_to_collision_up_to_carrot": 1.0,
                "use_regulated_linear_velocity_scaling": True,
                "use_cost_regulated_linear_velocity_scaling": True,
                "regulated_linear_scaling_min_radius": 0.9,
                "regulated_linear_scaling_min_speed": 0.25,
                "use_fixed_curvature_lookahead": False,
                "curvature_lookahead_dist": 0.6,
                "use_rotate_to_heading": True,
                "allow_reversing": False,
                "rotate_to_heading_min_angle": 0.785,
                "max_angular_accel": 3.2,
                "max_robot_pose_search_dist": 10.0,
                "stateful": True,
            }
        # 3. bt_navigator has to discover the action servers it calls, and on a
        #    loaded box it loses that race at the 1 s default: it gave up on
        #    compute_path_to_pose while planner_server was alive and logging
        #    costmap work, and the navigation group aborted with "Error loading
        #    BT" -- naming the tree, not the discovery. linorobot2's console
        #    branch measured the same race at 10 s (67db8a3). This is a ceiling,
        #    not a delay: the wait returns as soon as the server appears.
        bt_params.setdefault("wait_for_service_timeout", 30000)
        # 4. Increase bond timeout for Lyrical lifecycle managers
        nav2_data.setdefault("lifecycle_manager_navigation", {})["ros__parameters"] = {"bond_timeout": 60.0}
        nav2_data.setdefault("lifecycle_manager_slam", {})["ros__parameters"] = {"bond_timeout": 60.0}
    else:
        # Jazzy standards
        nav2_data.setdefault("lifecycle_manager_navigation", {})["ros__parameters"] = {"bond_timeout": 20.0}

    # collision_monitor is the one Nav2 node with no usable code default: it
    # reads `observation_sources` during on_configure and errors out if the key
    # was never set. nav2_bringup hands OUR params file to every node it starts,
    # so a node the robot config does not describe gets nothing -- and because
    # lifecycle_manager brings the set up as a unit, that single node failing
    # aborts the whole stack ("Failed to bring up all requested nodes") after
    # planner, controller, behavior and smoother had all configured cleanly.
    #
    # Merge per key rather than testing for the node: the stamped_cmd_vel loop
    # above has already created collision_monitor.ros__parameters, so a
    # `"collision_monitor" not in nav2_data` guard can never fire, and assigning
    # the block wholesale would drop enable_stamped_cmd_vel. Keying off
    # observation_sources tests the thing that actually has to be there.
    #
    # base_frame_id follows the robot config (Nav2 ships base_footprint, but
    # this stack's tree is map -> odom -> base_link per REP-105 and a frame that
    # is not in the description would fail every transform lookup).
    cm_params = nav2_data.setdefault("collision_monitor", {}).setdefault("ros__parameters", {})
    sonar_fitted = has_real_sonar(params)
    if "observation_sources" not in cm_params:
        base_frame = params.get("slam", {}).get("base_frame", "base_link")
        for key, value in {
            "base_frame_id": base_frame,
            "odom_frame_id": "odom",
            "cmd_vel_in_topic": "cmd_vel_smoothed",
            "cmd_vel_out_topic": "cmd_vel",
            "state_topic": "collision_monitor_state",
            "transform_tolerance": 0.2,
            "source_timeout": 1.0,
            "base_shift_correction": True,
            "stop_pub_timeout": 2.0,
            "polygons": ["FootprintApproach"],
            "FootprintApproach": {
                "type": "polygon",
                "action_type": "approach",
                "footprint_topic": "/local_costmap/published_footprint",
                "time_before_collision": 1.2,
                "simulation_time_step": 0.1,
                "min_points": 6,
                "visualize": False,
                "enabled": True,
            },
            # Two ways to hear "something is in front of me", not one -- but
            # only on a robot that has the second one. The LiDAR was the only
            # obstacle input this stack had, so nothing could contradict it
            # when it was wrong; a wired HC-SR04 does, at the range and the
            # height a spinning LiDAR is worst at. On a board with no sonar
            # pins the same entry is not a missing opinion, it is a parking
            # brake -- see has_real_sonar().
            "observation_sources": ["scan", "sonar"] if sonar_fitted else ["scan"],
            "scan": {
                "type": "scan",
                "topic": "scan",
                "min_height": 0.15,
                "max_height": 2.0,
                "enabled": True,
            },
            # The firmware stamps Range with envPrefixed("sonar_link"), and
            # gen_robot_description.py publishes that frame whether or not a
            # sonar is fitted -- a source whose frame is not in the tree is
            # dropped without a word.
            "sonar": {
                "type": "range",
                "topic": "sonar",
                # One point per degree across the beam; field_of_view is 30
                # degrees, so 31 points, well under the monitor's ceiling.
                "obstacles_angle": 0.0175,
                "enabled": True,
            },
        }.items():
            cm_params.setdefault(key, value)

    # And the same rule over a config that declares the source itself. This is
    # not tidiness: on 2026-09-23 a sonar source reached every bench board --
    # the bench variants inherit the reference's whole Nav2 block and replace
    # only base_controller, so the source travelled while the pins did not --
    # and the monitor held all three drivetrains mid-route. The drive suite ran
    # 8/8 on the same board seconds later, which is what a braked robot looks
    # like from outside: the base is fine, something above it is saying stop.
    #
    # A config may not ask for a sensor the robot does not have, so the pins
    # win and the launcher says so out loud rather than leaving a stopped robot
    # to be diagnosed from a costmap.
    if not sonar_fitted and "sonar" in (cm_params.get("observation_sources") or []):
        cm_params["observation_sources"] = [
            s for s in cm_params["observation_sources"] if s != "sonar"
        ]
        cm_params.pop("sonar", None)
        print(
            "[nav2] collision_monitor: dropping the `sonar` source -- this robot "
            "reports no sonar pins (base_controller.pins.sonar), so the topic "
            "would never publish and the monitor would stop the robot."
        )

    # docking_server is the second node of that same class, and it surfaced only
    # once bt_navigator stopped aborting the stack first: it reads `dock_plugins`
    # during on_configure and fails with "Charging dock plugins not given!" when
    # the robot config -- which describes no dock -- leaves the key unset. Same
    # merge discipline as above: per key, keyed off the parameter that actually
    # has to be there, because the stamped_cmd_vel loop already created
    # docking_server.ros__parameters.
    #
    # A dock *instance* is deliberately not supplied. Nav2's own defaults leave
    # `docks` commented out, and inventing a home dock at the map origin would
    # make `dock_robot` drive at a fiction. This block exists to let the node
    # configure, so the lifecycle set comes up; a robot with real docking
    # hardware declares its docks in its own config and these defaults yield.
    dock_params = nav2_data.setdefault("docking_server", {}).setdefault("ros__parameters", {})
    if "dock_plugins" not in dock_params:
        base_frame = params.get("slam", {}).get("base_frame", "base_link")
        for key, value in {
            "controller_frequency": 50.0,
            "initial_perception_timeout": 5.0,
            "wait_charge_timeout": 5.0,
            "dock_approach_timeout": 30.0,
            "undock_linear_tolerance": 0.05,
            "undock_angular_tolerance": 0.1,
            "max_retries": 3,
            "base_frame": base_frame,
            "fixed_frame": "odom",
            "dock_backwards": False,
            "dock_prestaging_tolerance": 0.5,
            "dock_plugins": ["simple_charging_dock"],
            "simple_charging_dock": {
                "plugin": "opennav_docking::SimpleChargingDock",
                "docking_threshold": 0.05,
                "staging_x_offset": -0.7,
                "use_external_detection_pose": True,
                "use_battery_status": False,
                "use_stall_detection": False,
                "external_detection_timeout": 1.0,
                "external_detection_translation_x": -0.18,
                "external_detection_translation_y": 0.0,
                "external_detection_rotation_roll": -1.57,
                "external_detection_rotation_pitch": -1.57,
                "external_detection_rotation_yaw": 0.0,
                "filter_coef": 0.1,
            },
            "controller": {
                "k_phi": 3.0,
                "k_delta": 2.0,
                "v_linear_min": 0.15,
                "v_linear_max": 0.15,
                "use_collision_detection": True,
                "costmap_topic": "local_costmap/costmap_raw",
                "footprint_topic": "local_costmap/published_footprint",
                "transform_tolerance": 0.1,
                "projection_time": 5.0,
                "simulation_step": 0.1,
                "dock_collision_threshold": 0.3,
            },
        }.items():
            dock_params.setdefault(key, value)

    # Multi-robot: run the whole nav2 stack under /<prefix> and prefix its frames
    # and the /scan,/odom,/map topics it names, so it plans for THIS robot in its
    # own namespace. Unset -> "" -> nav2_data and the launch are untouched. Full
    # two-robot validation is still open; this makes a single prefixed robot's
    # nav2 self-consistent.
    ns = cockpit_paths.robot_namespace(params)
    if ns:
        _prefix_nav2_namespace(nav2_data, ns)

    # Fully-qualified section names, the job nav2's own RewrittenYaml(root_key)
    # would do if this launch handed navigation_launch.py a namespace. A COPY is
    # dumped: nav2_data is read again below for the lifecycle manager's params,
    # and those lookups use the bare names.
    nav2_temp = tempfile.NamedTemporaryFile(mode="w", suffix=".yaml", delete=False)
    yaml.dump(cockpit_paths.namespace_params(nav2_data, ns), nav2_temp)
    nav2_temp.flush()
    nav2_params_path = nav2_temp.name

    nav2_bringup_pkg = FindPackageShare("nav2_bringup").find("nav2_bringup")
    nav2_launch_path = os.path.join(nav2_bringup_pkg, "launch", "navigation_launch.py")

    try:
        with open(nav2_launch_path) as f:
            _nav_launch_src = f.read()
    except OSError:
        _nav_launch_src = ""

    # nav2 1.5.1 took the lifecycle manager OUT of navigation_launch.py: that
    # file now starts the servers and nothing else, and bringup_launch.py owns
    # the single manager that drives slam/localization, the zones and navigation
    # together. Jazzy's navigation_launch.py still carries its own. That one
    # difference decides all three things below, so it is asked once, of the file
    # that will actually run, rather than guessed from a distro name.
    needs_own_manager = bool(_nav_launch_src) and "lifecycle_manager" not in _nav_launch_src

    launch_args = {
        "use_sim_time": use_sim_time,
        "autostart": autostart,
        "params_file": nav2_params_path,
    }
    if needs_own_manager:
        # Compose the stack into one container process.
        #
        # Participants are per PROCESS in rmw_fastrtps (__rmw_create_node reuses
        # context->impl->common), so an uncomposed nav2 is ~20 participants all
        # discovering each other at once. linorobot2's console branch measured
        # bt_navigator's client node timing out after 30 s on whichever server it
        # asked for first -- follow_path, is_path_valid and compute_path_to_pose
        # on three different runs -- and composed bringup passing 2 runs of 2
        # where uncomposed passed 1 of 2 (e991673). Composition needs the service
        # QoS profile one_click_pipeline.py exports, because load_node is itself a
        # service and drops the same way.
        #
        # Deliberately NOT applied where the launch file brings its own manager:
        # nav2 defaults use_composition to False on both jazzy and lyrical, so
        # that is the configuration the green jazzy hardware run actually used,
        # and this is a lyrical bring-up problem.
        launch_args["use_composition"] = "True"
    # Costmap filter zones are a kilted+ feature, and IncludeLaunchDescription
    # raises on an argument the included description never declared -- so passing
    # these to jazzy's navigation_launch.py would not be harmlessly ignored, it
    # would kill the nav2 launch outright. Ask the launch file that is actually
    # installed rather than keying off a distro name, so this keeps working on
    # whatever follows lyrical (console 675db40).
    #
    # They go off where they exist: nav2_bringup defaults both to True while the
    # mask paths default to '', which starts map_servers with no mask file. In
    # bringup_launch.py those publish empty maps on /map next to slam_toolbox and
    # the global costmap latches one of them ("Can't update static costmap layer,
    # no map received" forever, with nav2 fully active). This launcher includes
    # navigation_launch.py, where they are only YAML value rewrites, but the
    # default is wrong for this robot either way and we do not configure zones.
    supports_zones = ("'use_keepout_zones'" in _nav_launch_src
                      and "'use_speed_zones'" in _nav_launch_src)
    if supports_zones:
        launch_args["use_keepout_zones"] = "False"
        launch_args["use_speed_zones"] = "False"
    if map_file:
        launch_args["map"] = map_file

    # navigation_launch.py COMPOSES but does not CONTAIN: with use_composition
    # it only issues LoadComposableNodes(target_container='nav2_container') and
    # leaves creating that container to bringup_launch.py, which is what
    # linorobot2's console branch includes. Include navigation_launch.py alone
    # with composition on and the load requests go to a container that does not
    # exist -- silently. The measured result is a nav2.log holding nothing but
    # the launch banner, no node output at all, and lifecycle_manager never
    # reporting either way because lifecycle_manager_navigation is itself one of
    # the composable nodes that never loaded. So start the container here, the
    # way bringup_launch.py does.
    # Under a namespace, TF must stay on the GLOBAL /tf with frame names prefixed
    # (the ROS 2 multi-robot convention, and what bringup does via frame_prefix);
    # the default single-robot layout keeps the historical /tf->tf remap. So the
    # remap is dropped when namespaced, leaving /tf and /tf_static global.
    tf_remaps = [] if ns else [("/tf", "tf"), ("/tf_static", "tf_static")]
    nav2_container = None if not needs_own_manager else Node(
        name="nav2_container",
        namespace=ns or None,
        package="rclcpp_components",
        executable="component_container",
        parameters=[nav2_params_path, {"autostart": autostart}],
        arguments=["--isolated", "--executor-type", "single-threaded"],
        remappings=tf_remaps,
        output="screen",
    )

    # nav2 1.5.1 took the lifecycle manager OUT of navigation_launch.py: that file
    # now starts the eleven servers and nothing else, and bringup_launch.py owns
    # the single `lifecycle_manager_nav2` that drives slam/localization, the zones
    # and navigation together. Including navigation_launch.py alone therefore
    # leaves every server loaded and stuck in state 1 (unconfigured) with nobody
    # to transition it -- measured exactly that: 11 "Loaded node" lines, not one
    # configure, and lifecycle_manager reporting neither success nor failure
    # because there was no lifecycle_manager. On jazzy the file still carries its
    # own manager, so ours would be a second one fighting it for the same nodes.
    #
    # Detect, do not guess by distro: start a manager only where the installed
    # launch file has none, and take the node list from that same file so the set
    # cannot drift from what it actually started (lyrical added route_server,
    # docking_server and following_server).
    manager_actions = []
    if needs_own_manager:
        managed = _managed_node_names(_nav_launch_src)
        if not managed:
            manager_actions = [LogInfo(msg=(
                "[Linorobot2 Cockpit] WARNING: navigation_launch.py has no lifecycle "
                "manager and its managed-node list could not be read, so no manager "
                "was started -- the Nav2 servers will load and stay unconfigured."))]
        if managed:
            mgr_params = dict(nav2_data.get("lifecycle_manager_navigation", {}).get("ros__parameters", {}))
            # The manager drives nodes by name; under a namespace the servers are
            # /<ns>/<server>, so the managed names carry the prefix too.
            managed_names = [f"/{ns}/{n.lstrip('/')}" for n in managed] if ns else managed
            mgr_params.update({
                "autostart": autostart.lower() in ("true", "1", "yes"),
                "node_names": managed_names,
                "use_sim_time": use_sim_time.lower() in ("true", "1", "yes"),
            })
            manager_actions = [LoadComposableNodes(
                target_container=f"/{ns}/nav2_container" if ns else "/nav2_container",
                composable_node_descriptions=[ComposableNode(
                    package="nav2_lifecycle_manager",
                    plugin="nav2_lifecycle_manager::LifecycleManager",
                    name="lifecycle_manager_navigation",
                    parameters=[mgr_params],
                )],
            )]

    actions = [LogInfo(msg=f"[Linorobot2 Cockpit] Launching Nav2 Navigation Stack (distro='{distro}')")]
    if nav2_container is not None:
        # The container already carries namespace=ns; it is not wrapped again.
        actions.append(nav2_container)
    nav2_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch_path),
        launch_arguments=launch_args.items(),
    )
    # Namespace the servers navigation_launch.py starts, so they are
    # /<ns>/<server> for the manager to drive. The manager (LoadComposableNodes)
    # names the container by its absolute path, so it stays outside the group.
    actions.append(GroupAction([PushRosNamespace(ns), nav2_include]) if ns else nav2_include)
    return actions + manager_actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "config_file",
            default_value=DEFAULT_PARAMS,
            description="Path to the robot's config YAML (single source of truth)",
        ),
        DeclareLaunchArgument(
            "distro",
            default_value=os.environ.get("ROS_DISTRO", "jazzy"),
            description="ROS 2 distribution (jazzy, lyrical)",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation (Gazebo) clock if true",
        ),
        DeclareLaunchArgument(
            "autostart",
            default_value="true",
            description="Automatically startup the nav2 stack",
        ),
        DeclareLaunchArgument(
            "map",
            default_value="",
            description="Full path to map yaml file to load",
        ),
        DeclareLaunchArgument(
            "stamped_cmd_vel",
            default_value="",
            description="Explicitly enable or disable stamped cmd_vel across Nav2 (empty to inherit from the robot config)",
        ),
        OpaqueFunction(function=launch_setup),
    ])
