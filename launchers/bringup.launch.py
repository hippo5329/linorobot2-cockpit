#!/usr/bin/env python3
# ==============================================================================
# bringup.launch.py — Linorobot2 Cockpit Bringup Launcher
#
# Reads <config dir>/<robot>_config.yaml as single source of truth.
# Launches:
# 1. imu_filter_madgwick (publish_tf: false to prevent TF graph collision)
# 2. robot_localization (ekf_node)
# 3. micro_ros_agent (optional, defaults to true for native serial/UDP)
# 4. rosbridge_websocket + rosapi on port 9090, which is what the Web Cockpit's
#    browser canvas subscribes to. It is the only way a headless robot shows a
#    map: there is no display for RViz to open on.
# ==============================================================================

import os
import sys
import tempfile
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO_ROOT, "scripts"))
import cockpit_paths  # noqa: E402  (the user's config dir, never the repo's)
import gen_robot_description  # noqa: E402  (the URDF, from the config)

# The LD driver family the LiDAR block's `model` names, in the driver's own
# vocabulary. `bins` is the ray count the fork's node resamples a revolution
# to; each model's angular resolution sets it.
LDLIDAR_MODELS = {
    "ld19":   ("LDLiDAR_LD19", 456),
    "ld06":   ("LDLiDAR_LD06", 456),
    "ld14":   ("LDLiDAR_LD14", 360),
    "ld14p":  ("LDLiDAR_LD14P", 360),
    "stl27l": ("LDLiDAR_STL27L", 2160),
}

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
    params = load_yaml(config_file)

    # The config file names exactly one base controller; the launch argument only
    # relabels it (pins and ports still come from the file).
    controller = params.get("base_controller") or {}
    controller_arg = context.launch_configurations.get("controller", "").strip()
    controller_name = controller_arg or controller.get("name") or "pico2"

    serial_port = (
        context.launch_configurations.get("serial_port")
        or controller.get("serial_port", "/dev/ttyUSB0")
    )
    baudrate = (
        context.launch_configurations.get("baudrate")
        or str(controller.get("baudrate", 921600))
    )
    transport = (
        context.launch_configurations.get("transport")
        or controller.get("transport", "serial")
    )

    lidar_cfg = controller.get("lidar", {})
    # A robot whose config carries no lidar: block has no scan source at all.
    # On a 921600-baud serial ESP32 that is the only supported fake-mode shape:
    # the fake LD19 cannot ride the micro-ROS link and a bare DevKit has no
    # LIDAR_RXD bridge, so the robot runs teleop only. Starting a driver anyway
    # would leave it blocked forever on an empty tty and publish no /scan.
    robot_has_lidar = bool(lidar_cfg)
    lidar_port = (
        context.launch_configurations.get("lidar_port")
        or lidar_cfg.get("serial_port", "/dev/ttyUSB1")
    )
    lidar_baud = (
        context.launch_configurations.get("lidar_baud")
        or str(lidar_cfg.get("baudrate", 230400))
    )
    lidar_comm_mode = (
        context.launch_configurations.get("lidar_comm_mode")
        or lidar_cfg.get("comm_mode", "serial")
    )
    lidar_raw_topic = (
        context.launch_configurations.get("lidar_raw_topic")
        or lidar_cfg.get("raw_scan_topic", "raw_scan")
    )

    udp_port = (
        context.launch_configurations.get("udp_port")
        or str(controller.get("udp_port", 8888))
    )
    lidar_udp_port = (
        context.launch_configurations.get("lidar_udp_port")
        or str(lidar_cfg.get("udp_port", 8889))
    )
    effective_lidar_comm_mode = "udp_server" if lidar_comm_mode in ("udp", "udp_server") else lidar_comm_mode

    if transport in ("udp4", "udp", "wifi"):
        micro_ros_args = ["udp4", "--port", str(udp_port)]
    else:
        micro_ros_args = [transport, "--dev", serial_port, "-b", baudrate]

    base_type = params.get("kinematics", {}).get("base_type", "2wd")
    os.environ["LINOROBOT2_BASE"] = base_type

    # The robot description, generated from THIS config at every launch, so the
    # TF tree carries the user's wheel radius, track, LiDAR and IMU poses --
    # not the stock xacro's. Written next to the config (generated/, gitignored).
    robot_name = (params.get("robot") or {}).get("name") or os.path.basename(config_file).replace("_config.yaml", "")
    urdf_path = gen_robot_description.default_out_path(params, config_file, os.path.dirname(config_file))
    gen_robot_description.write_urdf(params, urdf_path, robot_name)
    with open(urdf_path) as fh:
        robot_description = fh.read()
    geometry = gen_robot_description.effective_geometry(params)
    laser_frame = str(geometry["laser"].get("frame") or gen_robot_description.DEFAULT_LASER_FRAME)
    lidar_model = str(lidar_cfg.get("model", "ld19")).lower()
    lidar_product, lidar_bins = LDLIDAR_MODELS.get(lidar_model, LDLIDAR_MODELS["ld19"])

    # Extract EKF parameters into a temporary YAML file for robot_localization.
    # A ROS 2 params file must be keyed by node name and ros__parameters; the config
    # holds the settings flat, and handing rcl that flat mapping makes ekf_node abort
    # with "Cannot have a value before ros__parameters at line 1".
    ekf_data = params.get("ekf", {}) or {}
    if ekf_data and "ros__parameters" not in ekf_data and "ekf_filter_node" not in ekf_data:
        ekf_data = {"ekf_filter_node": {"ros__parameters": ekf_data}}
    ekf_temp = tempfile.NamedTemporaryFile(mode="w", suffix=".yaml", delete=False)
    yaml.dump(ekf_data, ekf_temp)
    ekf_temp.flush()
    ekf_params_path = ekf_temp.name

    imu_sensor = controller.get("sensors", {}).get("imu", "NONE")
    use_fake_imu = controller.get("sensors", {}).get("use_fake_imu", False)
    has_imu = (imu_sensor != "NONE") or use_fake_imu

    mag_sensor = controller.get("sensors", {}).get("mag", "NONE")
    use_fake_mag = controller.get("sensors", {}).get("use_fake_mag", False)
    use_mag_arg = context.launch_configurations.get("use_mag", "")
    if use_mag_arg != "":
        use_mag = (use_mag_arg.lower() in ("true", "1", "yes"))
    else:
        use_mag = (mag_sensor != "NONE" and not use_fake_mag)

    madgwick_arg = context.launch_configurations.get("madgwick", "")
    if madgwick_arg != "":
        enable_madgwick = (madgwick_arg.lower() in ("true", "1", "yes"))
    else:
        enable_madgwick = has_imu

    nodes = [
        LogInfo(
            msg=f"[Linorobot2 Cockpit] Bringup controller='{controller_name}', transport='{transport}', serial='{serial_port}', baud='{baudrate}', udp_port='{udp_port}', lidar_mode='{effective_lidar_comm_mode}', madgwick={enable_madgwick}, use_mag={use_mag}, rosbridge_port='{context.launch_configurations.get("rosbridge_port", "9090")}', urdf='{urdf_path}'"
        ),
        # 1. Madgwick Filter (fuses imu/data_raw into imu/data with quaternion orientation)
        Node(
            condition=IfCondition("true" if enable_madgwick else "false"),
            package="imu_filter_madgwick",
            executable="imu_filter_madgwick_node",
            name="madgwick_filter_node",
            output="screen",
            parameters=[
                {"publish_tf": False},
                {"use_mag": use_mag},
                {"orientation_stddev": 0.01},
                # dt from the message stamps (0.0 is the package default), not
                # a constant mirroring CONTROL_TIMER (20 ms): the firmware
                # stamps /imu/data_raw with the agent-synced epoch, and the
                # rate that arrives is not always the timer's -- an RP2040 at
                # 921600 delivers ~40 Hz of the 50 -- so a constant 20 ms
                # under-integrated the gyro by every message the link dropped.
                {"constant_dt": 0.0},
            ],
            remappings=[
                ("imu/data_raw", "imu/data_raw"),
                ("imu/data", "imu/data"),
            ],
        ),
        # 2. Robot Localization EKF Node
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node",
            output="screen",
            parameters=[ekf_params_path],
            remappings=[("odometry/filtered", "odom")],
        ),
        # 3. Micro-ROS Agent (Serial or UDP)
        # ExecuteProcess, not Node, on purpose: launch_ros always appends "--ros-args"
        # (plus "-r __node:=..." when the node is named) to a Node's argv, and the
        # agent's own CLI parser overruns its argument buffer on that, dying with
        # "*** stack smashing detected ***" before it ever opens the serial port.
        # `ros2 run` passes the arguments through untouched.
        ExecuteProcess(
            condition=IfCondition(LaunchConfiguration("micro_ros")),
            cmd=["ros2", "run", "micro_ros_agent", "micro_ros_agent"] + micro_ros_args,
            name="micro_ros_agent",
            output="screen",
        ),
        # 4. Robot State Publisher & Description (URDF / TF tree), from the
        # generated file above. joint_state_publisher gives the continuous
        # wheel joints a zero state so every wheel frame exists.
        Node(
            condition=IfCondition(LaunchConfiguration("description")),
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
        ),
        Node(
            condition=IfCondition(LaunchConfiguration("description")),
            package="joint_state_publisher",
            executable="joint_state_publisher",
            name="joint_state_publisher",
            parameters=[{"robot_description": robot_description}],
        ),
        # 5. rosbridge_websocket — the browser's ROS 2 connection (Zero-RViz canvas).
        # ExecuteProcess would work, but rosbridge takes plain node parameters and
        # is well behaved under launch_ros, unlike micro_ros_agent above.
        Node(
            condition=IfCondition(LaunchConfiguration("rosbridge")),
            package="rosbridge_server",
            executable="rosbridge_websocket",
            name="rosbridge_websocket",
            output="screen",
            parameters=[{
                "port": ParameterValue(LaunchConfiguration("rosbridge_port"), value_type=int),
                "address": "0.0.0.0",
                # The viewer runs on a different machine, so it must be reachable
                # off-host; 0.0.0.0 is deliberate, not an oversight.
                "call_services_in_new_thread": True,
                "send_action_goals_in_new_thread": True,
                # /scan and /imu/data are published best-effort (SensorDataQoS).
                # Without this, rosbridge subscribes RELIABLE, the QoS never
                # matches, and the canvas silently receives nothing.
                "default_call_service_timeout": 5.0,
                "topics_glob": "[*]",
                "services_glob": "[*]",
                "params_glob": "[*]",
            }],
        ),
        # 6. rosapi — lets the browser enumerate topics and look up their types
        # before subscribing. rosbridge alone cannot answer that.
        Node(
            condition=IfCondition(LaunchConfiguration("rosbridge")),
            package="rosapi",
            executable="rosapi_node",
            name="rosapi",
            output="screen",
            parameters=[{
                "topics_glob": "[*]",
                "services_glob": "[*]",
                "params_glob": "[*]",
            }],
        ),
        # 7. LiDAR Node (UDP server, virtual room emulator for bare bench, or hardware serial driver)
        (
            None
            if not robot_has_lidar
            else Node(
                condition=IfCondition(LaunchConfiguration("lidar")),
                package="ldlidar_stl_ros2",
                executable="ldlidar_stl_ros2_node",
                name="ld19",
                output="screen",
                parameters=[{
                    "product_name": lidar_product,
                    "topic_name": "scan",
                    "frame_id": laser_frame,
                    "comm_mode": "udp_server",
                    "server_ip": "0.0.0.0",
                    "server_port": int(lidar_udp_port),
                    "laser_scan_dir": True,
                    "bins": lidar_bins,
                    "enable_angle_crop_func": False,
                }],
            )
            if effective_lidar_comm_mode == "udp_server"
            else (
                Node(
                    condition=IfCondition(LaunchConfiguration("lidar")),
                    executable=sys.executable,
                    arguments=[os.path.join(REPO_ROOT, "scripts", "fake_laser_node.py")],
                    name="fake_laser_node",
                    output="screen",
                    parameters=[{"frame_id": laser_frame,
                                 "offset_x": float(geometry["laser"]["x"])}],
                )
                # A `serial` comm_mode means LD19 packets arrive on a real tty, and
                # the driver cannot tell -- nor does it care -- who put them there.
                # On the gendrv bench that producer is the ESP32 itself: fake_ld19
                # emits a synthetic scan out LIDAR_RXD, which is wired to a
                # USB-serial bridge showing up as the lidar port. Letting
                # use_fake_ld19 pick the host-side fake node here would leave those
                # bytes unread and bypass the very driver path under test.
                if (controller.get("sensors", {}).get("use_fake_ld19", False)
                        and effective_lidar_comm_mode != "serial")
                else Node(
                    condition=IfCondition(LaunchConfiguration("lidar")),
                    package="ldlidar_stl_ros2",
                    executable="ldlidar_stl_ros2_node",
                    name="ld19",
                    output="screen",
                    parameters=[{
                        "product_name": lidar_product,
                        "topic_name": "scan",
                        "frame_id": laser_frame,
                        "comm_mode": lidar_comm_mode,
                        "raw_scan_topic": lidar_raw_topic,
                        "port_name": lidar_port,
                        "port_baudrate": int(lidar_baud),
                        "laser_scan_dir": True,
                        "bins": lidar_bins,
                        "enable_angle_crop_func": False,
                    }],
                )
            )
        ),
    ]

    # `None` is the "this robot has no such node" placeholder above; launch
    # rejects it, so it never reaches the description.
    return [n for n in nodes if n is not None]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "config_file",
            default_value=DEFAULT_PARAMS,
            description="Path to the robot's config YAML (single source of truth)",
        ),
        DeclareLaunchArgument(
            "controller",
            default_value="",
            description="Base controller label (e.g. esp32, pico2, gendrv, esp32s3). Defaults to base_controller.name in the robot config",
        ),
        DeclareLaunchArgument(
            "micro_ros",
            default_value="true",
            description="Start micro_ros_agent node",
        ),
        DeclareLaunchArgument(
            "madgwick",
            default_value="",
            description="Start imu_filter_madgwick node (auto-enabled if mag is present)",
        ),
        DeclareLaunchArgument(
            "description",
            default_value="true",
            description="Start robot_state_publisher description node",
        ),
        DeclareLaunchArgument(
            "lidar",
            default_value="true",
            description="Start LD19 LiDAR driver node",
        ),
        DeclareLaunchArgument(
            "serial_port",
            default_value="",
            description="Microcontroller serial port device (defaults to base_controller in the robot config)",
        ),
        DeclareLaunchArgument(
            "baudrate",
            default_value="",
            description="Microcontroller serial baudrate (defaults to base_controller in the robot config)",
        ),
        DeclareLaunchArgument(
            "lidar_port",
            default_value="",
            description="LiDAR serial port device (defaults to base_controller in the robot config)",
        ),
        DeclareLaunchArgument(
            "lidar_baud",
            default_value="",
            description="LiDAR serial baudrate (defaults to base_controller in the robot config)",
        ),
        DeclareLaunchArgument(
            "lidar_comm_mode",
            default_value="",
            description="LiDAR communication mode: serial, topic, udp_server (defaults to the base controller in the robot config)",
        ),
        DeclareLaunchArgument(
            "lidar_raw_topic",
            default_value="",
            description="Raw scan topic name when in topic comm_mode (defaults to the base controller in the robot config)",
        ),
        DeclareLaunchArgument(
            "transport",
            default_value="",
            description="micro-ROS agent transport type (defaults to the base controller in the robot config)",
        ),
        DeclareLaunchArgument(
            "udp_port",
            default_value="",
            description="micro-ROS agent UDP port for udp4 transport (defaults to the base controller in the robot config)",
        ),
        DeclareLaunchArgument(
            "lidar_udp_port",
            default_value="",
            description="LiDAR UDP server port for udp/udp_server mode (defaults to the base controller in the robot config)",
        ),
        DeclareLaunchArgument(
            "rosbridge",
            default_value="true",
            description="Start rosbridge_websocket + rosapi so the Web Cockpit canvas can subscribe",
        ),
        DeclareLaunchArgument(
            "rosbridge_port",
            default_value="9090",
            description="rosbridge WebSocket port (the Web Cockpit visualizer connects here)",
        ),
        DeclareLaunchArgument(
            "use_mag",
            default_value="",
            description="Enable magnetometer fusion in madgwick node (auto-detected if empty)",
        ),
        OpaqueFunction(function=launch_setup),
    ])
