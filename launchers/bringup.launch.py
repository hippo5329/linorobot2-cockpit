#!/usr/bin/env python3
# ==============================================================================
# bringup.launch.py — Linorobot2 Cockpit Bringup Launcher
#
# Reads <config dir>/<robot>_config.yaml as single source of truth.
# Launches:
# 1. (no IMU filter node: the board fuses its own orientation, ahrs.h)
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
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO_ROOT, "scripts"))
import cockpit_paths  # noqa: E402  (the user's config dir, never the repo's)
import gen_robot_description  # noqa: E402  (the URDF, from the config)
import host_firmware  # noqa: E402  (the Sim MCU as the firmware itself)
import depth_camera  # noqa: E402  (who makes /scan: a LiDAR or a depth camera)
import lidar_mask  # noqa: E402  (the robot's own structure, removed from the scan)
import lidar_drivers  # noqa: E402  (which vendor driver reads which LiDAR model)

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
    params = load_yaml(config_file)

    # The config file names exactly one base controller; the launch argument only
    # relabels it (pins and ports still come from the file).
    controller = params.get("base_controller") or {}
    controller_arg = context.launch_configurations.get("controller", "").strip()
    controller_name = controller_arg or controller.get("name") or "pico2"
    # No board at all: `sim_base:=true`, or the base controller named `sim` (the
    # cockpit's "Simulated MCU" choice). ONE decision, used below for the base
    # node, the agent and the LiDAR -- with no board there is no sensor either,
    # so the room is raycast on this computer whatever the config says. Before
    # this, sim_base with a real-LD19 config (pico2_mecanum) started the real
    # driver on an absent /dev/ttyUSB0 and /scan never came.
    #
    # The Sim MCU is the FIRMWARE when this machine has the host build
    # (firmware/host/app, scripts/host_firmware.py): src/main.cpp compiled for
    # this computer, a micro-ROS UDP4 client of the agent below, whose own LD19
    # emulator streams to the udp_server driver -- the path a Wi-Fi board takes,
    # so micro-ROS, the agent and the LiDAR driver are all in the loop. Then
    # there IS a board, as far as the rest of this launch is concerned: it just
    # runs here. The rclpy sim_base_node remains for `sim_base:=true` (the CI
    # instrument that deliberately skips micro-ROS) and for a checkout without
    # the host build.
    sim_base_arg = (str(context.launch_configurations.get("sim_base", "false")).strip().lower()
                    in ("true", "1", "yes"))
    host_fw_bin = host_firmware.binary() if (controller_name == "sim" and not sim_base_arg) else None
    no_board = sim_base_arg or (controller_name == "sim" and host_fw_bin is None)

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
    # On a 921600-baud serial ESP32 that is the only supported simulation-mode shape:
    # the simulated LD19 cannot ride the micro-ROS link and a bare DevKit has no
    # LIDAR_RXD bridge, so the robot runs teleop only. Starting a driver anyway
    # would leave it blocked forever on an empty tty and publish no /scan.
    # Who makes /scan is ONE rule (depth_camera.scan_source), shared with the
    # pipeline. This read `bool(lidar_cfg)`, so `lidar: {model: none}` started a
    # driver on a port nothing was wired to, while the pipeline -- which already
    # honoured "none" -- did not wait for its scan.
    try:
        scan_from = depth_camera.scan_source(controller)
        scan_error = ""
    except ValueError as exc:
        scan_from, scan_error = None, str(exc)
    robot_has_lidar = scan_from == "lidar"
    # A masked LiDAR publishes scan_raw and laser_filters makes /scan of it
    # (lidar_mask.py). A mask that does not parse stops the bringup here, on the
    # robot computer, rather than launching a scan with the mast still in it.
    lidar_masked = robot_has_lidar and lidar_mask.masked(controller)
    lidar_topic = "scan_raw" if lidar_masked else "scan"
    # The simulated LD19's view of the robot's own structure, for the host laser
    # (the firmware's emulator gets the same sectors through the env).
    _occl_sectors, _occl_r = lidar_mask.occlusion(params)
    occl_flat = [v for sec in _occl_sectors for v in sec] or [0.0]
    occl_range = float(_occl_r or 0.12)
    lidar_port = (
        context.launch_configurations.get("lidar_port")
        or lidar_cfg.get("serial_port", "/dev/ttyUSB1")
    )
    # The rate the config (or the launch) names; the LD default is 230400, and
    # every other family takes its model's own rate when none is named.
    lidar_baud_named = (
        context.launch_configurations.get("lidar_baud")
        or lidar_cfg.get("baudrate")
    )
    lidar_baud = str(lidar_baud_named or 230400)
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

    # Should the host-side simulated laser (the "virtual room") stand in for a real
    # driver? Only when the scan is meant to be simd (use_sim_ld19), and:
    #   - the comm mode is not a real serial tty, or
    #   - it *is* serial but that tty is absent.
    # A `serial` comm_mode says LD19 packets arrive on a real port, and on the
    # gendrv bench the ESP32 itself emits sim_ld19 out LIDAR_RXD into a
    # USB-serial bridge -- so when that bridge is present the real driver must
    # read it, and routing to the host node would bypass the very driver path
    # under test. But a *bare* module has only its one micro-ROS USB and no such
    # bridge: the lidar tty never appears, the serial driver dies on a missing
    # port, and /scan never comes -- which would strand SLAM/Nav2 on a bare
    # bench. Falling back to the virtual room only when the port is absent keeps
    # the bench-with-a-bridge case byte-identical while letting a bare module
    # preview SLAM/Nav2 in pure simulation.
    use_host_sim_laser = no_board or (
        controller.get("sensors", {}).get("use_sim_ld19", False)
        and (
            effective_lidar_comm_mode != "serial"
            or not os.path.exists(lidar_port)
        )
    )

    host_fw_env = None
    if host_fw_bin:
        # Whatever the config says: the host has no serial port to be a client
        # on and no UART to send an LD19 stream out of, and the pipeline runs a
        # board's own config on the Sim MCU when no board is plugged in.
        transport = "udp4"
        effective_lidar_comm_mode = "udp_server"
        use_host_sim_laser = False      # the firmware's own emulator is the source
        robot_label = (params.get("robot") or {}).get("name") or \
            os.path.basename(config_file).replace("_config.yaml", "")
        host_fw_env = host_firmware.write_env(
            config_file,
            os.path.join(os.path.dirname(config_file), "generated", f"{robot_label}_host_env.bin"),
            agent_port=int(udp_port), lidar_port=int(lidar_udp_port))

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
    # The wiring chart, beside the URDF, so the sheet a builder reads at the
    # bench exists as a file and not only behind a browser button. Same
    # generated/ directory, regenerated every launch, best-effort: a wiring
    # chart is documentation, and failing to write it must never stop a bringup.
    try:
        import gen_wiring_table
        wiring_path = os.path.join(os.path.dirname(urdf_path),
                                   os.path.basename(urdf_path).replace(".urdf", "_wiring.md"))
        with open(wiring_path + ".tmp", "w") as fh:
            fh.write(gen_wiring_table.render(params))
        os.replace(wiring_path + ".tmp", wiring_path)
    except Exception as exc:  # noqa: BLE001 -- documentation, never fatal
        print(f"[bringup] could not write the wiring chart: {exc}")
    geometry = gen_robot_description.effective_geometry(params)
    laser_frame = str(geometry["laser"].get("frame") or gen_robot_description.DEFAULT_LASER_FRAME)

    # Multi-robot: when the config sets topic_prefix, the board already publishes
    # its topics under /<prefix>/… (firmware topicName()). The host stack joins
    # it by running in the same namespace AND prefixing its TF frames, so two
    # robots on one DDS domain never collide on a topic or a frame. Unset -> "" ->
    # every branch below is a no-op and the single-robot layout is unchanged.
    ns = cockpit_paths.robot_namespace(params)
    frame_prefix = f"{ns}/" if ns else ""
    if ns:
        laser_frame = frame_prefix + laser_frame
    # The simulated sonar's frame, for the host cone below (Sim MCU only).
    sonar_frame = str((geometry.get("sonar") or {}).get("frame") or "sonar_link")
    if frame_prefix:
        sonar_frame = frame_prefix + sonar_frame
    # With no board the host publishes /sonar too: a config whose collision
    # monitor lists `sonar` as a source (pico2_mecanum) was otherwise stopped
    # for good -- "Robot to stop due to invalid source" -- because the board
    # that normally publishes it is the thing that is missing. With a board the
    # firmware's own cone is the publisher; two would disagree.
    host_sonar = no_board and bool(controller.get("sensors", {}).get("use_sim_sonar", True))
    lidar_model = str(lidar_cfg.get("model", "ld19")).lower()
    # A model no driver here reads stops the launch with its name, not an LD19
    # driver respawning on another vendor's port (lidar_drivers.family).
    lidar_family = lidar_drivers.family(lidar_model) if robot_has_lidar else None
    lidar_product, lidar_bins = lidar_drivers.ld_product(lidar_model)

    # Extract EKF parameters into a temporary YAML file for robot_localization.
    # A ROS 2 params file must be keyed by node name and ros__parameters; the config
    # holds the settings flat, and handing rcl that flat mapping makes ekf_node abort
    # with "Cannot have a value before ros__parameters at line 1".
    ekf_data = params.get("ekf", {}) or {}
    if ekf_data and "ros__parameters" not in ekf_data and "ekf_filter_node" not in ekf_data:
        ekf_data = {"ekf_filter_node": {"ros__parameters": ekf_data}}
    if ns:
        # Prefix the EKF's own frame names so its odom->base TF lands in this
        # robot's tree (frame_prefix on robot_state_publisher does the URDF
        # frames; the EKF sets its frames itself, so it has to be told here).
        rp = ekf_data.setdefault("ekf_filter_node", {}).setdefault("ros__parameters", {})
        for key, default in (("odom_frame", "odom"), ("base_link_frame", "base_link"),
                             ("world_frame", "odom"), ("map_frame", "map")):
            rp[key] = frame_prefix + str(rp.get(key, default))

    mag_sensor = controller.get("sensors", {}).get("mag", "NONE")
    use_sim_mag = controller.get("sensors", {}).get("use_sim_mag", False)
    use_mag_arg = context.launch_configurations.get("use_mag", "")
    # Assigned on both branches: the AUTO notice below reads it, and it used to
    # be set only when use_mag was left to the config -- so `use_mag:=true`, a
    # declared argument, raised NameError instead of launching.
    auto_mag = False
    if use_mag_arg != "":
        use_mag = (use_mag_arg.lower() in ("true", "1", "yes"))
    else:
        # The simulated magnetometer is fused, not just published. This used to
        # read `mag_sensor != "NONE" and not use_sim_mag`, which excluded the
        # simulated mag on purpose -- from the era when it pointed along +X and gave
        # madgwick a fixed 90-degree error (see SimIMUFromWheels::applyMag).
        # The mag now points North and is rotated by the wheel heading for
        # exactly one reason: to anchor heading fusion to the simulated room.
        # Left out of the fusion, the heading filter integrates the gyro alone, the sim
        # gyro's bias walks onto its +-0.004 rad/s clamp and stays there
        # (13.7 deg/min), the EKF takes that yaw as absolute, and the
        # body -- which follows the WHEEL yaw -- ends up 52 degrees from where
        # Nav2 thinks it is pointing. Measured at rest on a bare Pico 2 after an
        # hour: wheel yaw 59.4, EKF yaw 7.2. Every goal then veers, and with a
        # wall in the room it eventually parks itself there.
        #
        # AUTO is not a promise of a magnetometer. This launch cannot see the
        # bus, so it cannot know whether the board's heading is anchored to a
        # field or is the gyro's own integral -- and the EKF fuses whatever
        # imu/data says as ABSOLUTE yaw when use_mag is true. A wrong `true`
        # hands it a drifting heading as truth; a wrong `false` leaves the
        # heading to vyaw alone -- degraded, but a working stack that says so.
        # So AUTO resolves to false and names what to do about it.
        auto_mag = str(mag_sensor).strip().upper() in ("AUTO", "")
        use_mag = (not auto_mag and str(mag_sensor).upper() != "NONE") or bool(use_sim_mag)

    if auto_mag and not use_sim_mag:
        print("[bringup] sensors.mag is AUTO: the EKF will not fuse absolute yaw. The "
              "bus decides whether a magnetometer exists and this launch cannot see it. "
              "Name the part (mag: AK09918) to fuse it.")

    # NO IMU FILTER NODE. The board fuses its own orientation (ahrs.h) and
    # publishes imu/data; there is no imu/data_raw for a node to read.
    #
    # imu_filter_madgwick used to pair imu/data_raw with imu/mag through a
    # message_filters ApproximateTime synchroniser five deep, which made imu/data
    # the rate of MATCHED PAIRS across a best-effort micro-ROS session. Two slowed
    # bench legs measured the cost: /odom, which needs no partner, held 33 Hz
    # while imu/data fell to 10. firmware/common/lib/imu/ahrs.h is a port of that
    # node's own ImuFilter, held to it numerically by tests, with nothing to
    # synchronise.

    # THE EKF HALF OF THE RULE: a heading is only absolute where a field anchors it.
    #
    #   magnetometer    the board's imu/data yaw is anchored to the field, and
    #                   the EKF fuses it: imu0_config[5] = True.
    #   no magnetometer the board fuses gyro and accel alone, so its yaw is the
    #                   gyro's own integral with nothing to correct it. Fusing
    #                   index 5 there feeds the filter that drift as absolute
    #                   heading, and not one line of log to say so. Angular speed
    #                   (index 11, vyaw) is what carries rotation instead, and
    #                   it is already true in every shipped config.
    #
    # Patched here rather than kept as two config blocks, for the same reason
    # nav2.launch.py prunes the collision monitor's `sonar` source when no sonar
    # is fitted: one config per robot, and the launch derives what the hardware
    # implies.
    if not use_mag:
        rp = ekf_data.setdefault("ekf_filter_node", {}).setdefault("ros__parameters", {})
        cfg = rp.get("imu0_config")
        if isinstance(cfg, list) and len(cfg) > 5 and cfg[5]:
            cfg = list(cfg)
            cfg[5] = False
            rp["imu0_config"] = cfg
            print("[bringup] no magnetometer: imu0_config[5] (absolute yaw) -> False. "
                  "The board's imu/data yaw is the gyro's own integral with nothing to "
                  "anchor it; vyaw carries rotation instead.")

    # GRAVITY IS REMOVED ONCE, AND NOT HERE.
    #
    # The EKF fuses ax and ay (imu0_config 12, 13). An accelerometer measures
    # SPECIFIC FORCE, so gravity is in that reading, and a part mounted a couple
    # of degrees off leans a constant into the horizontal axes -- 0.85 m/s2 at
    # five degrees of pitch -- which the filter reads as real acceleration and
    # integrates into velocity.
    #
    # Whoever runs the fusion owns the subtraction, because only they have the
    # orientation estimate it needs. Since 2026-09-25 that is the BOARD: ahrs.h
    # subtracts its own gravity estimate before publishing, exactly as the
    # madgwick node's remove_gravity_vector used to. Doing it a second time here would
    # fabricate 9.81 m/s2 upward, which is worse than the fault it was meant to
    # fix -- so this states an invariant rather than a branch, and it holds
    # whether the board has a magnetometer or not.
    #
    # Written only when the key is ABSENT: a value already here is somebody's
    # deliberate choice about their own hardware, and the yaw rule above overrides
    # a template value only because the template always says `true`.
    rp = ekf_data.setdefault("ekf_filter_node", {}).setdefault("ros__parameters", {})
    if rp.get("imu0_remove_gravitational_acceleration") is None:
        rp["imu0_remove_gravitational_acceleration"] = False

    # rcl matches a params section against the node's FULLY-QUALIFIED name, so
    # under a namespace `ekf_filter_node:` matches nothing and the EKF starts on
    # its own defaults -- no odom0, no imu0, nothing published, no complaint.
    ekf_temp = tempfile.NamedTemporaryFile(mode="w", suffix=".yaml", delete=False)
    yaml.dump(cockpit_paths.namespace_params(ekf_data, ns), ekf_temp)
    ekf_temp.flush()
    ekf_params_path = ekf_temp.name

    nodes = [
        LogInfo(
            msg=f"[Linorobot2 Cockpit] Bringup controller='{controller_name}', transport='{transport}', serial='{serial_port}', baud='{baudrate}', udp_port='{udp_port}', lidar_mode='{effective_lidar_comm_mode}', use_mag={use_mag}, rosbridge_port='{context.launch_configurations.get("rosbridge_port", "9090")}', urdf='{urdf_path}'"
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
        # 2b. The base itself, simulated on this computer.
        #
        # `sim_base` replaces the microcontroller entirely: no agent, no serial
        # port, no board. It publishes odom/unfiltered and imu/data from the same
        # wheel model the firmware runs (scripts/sim_base_node.py imports the
        # transcription in drivetrain_report.py), so the rest of the stack -- EKF,
        # SLAM, Nav2, and sim_laser_node raycasting from the odometry -- is
        # unchanged and unaware.
        #
        # It is a diagnostic instrument and a CI leg, NOT a substitute for the
        # board matrix: what it removes is micro-ROS, the transport and the
        # board's timing, which is a large part of what those legs test. The gate
        # stays on hardware.
        Node(
            condition=IfCondition("true" if no_board else "false"),
            executable=sys.executable,
            arguments=[os.path.join(REPO_ROOT, "scripts", "sim_base_node.py")],
            name="sim_base_node",
            output="screen",
            # The same config the rest of this launch was built from, and the
            # same Twist-vs-TwistStamped decision Nav2 is making: Lyrical
            # defaults to stamped, Jazzy to plain, and a base subscribing to the
            # wrong one is silently deaf.
            parameters=[{"params": config_file,
                         "stamped_cmd_vel": (
                             context.launch_configurations.get(
                                 "distro", os.environ.get("ROS_DISTRO", "jazzy")
                             ).strip().lower() == "lyrical")}],
        ),
        # 3. Micro-ROS Agent (Serial or UDP)
        # ExecuteProcess, not Node, on purpose: launch_ros always appends "--ros-args"
        # (plus "-r __node:=..." when the node is named) to a Node's argv, and the
        # agent's own CLI parser overruns its argument buffer on that, dying with
        # "*** stack smashing detected ***" before it ever opens the serial port.
        # `ros2 run` passes the arguments through untouched.
        ExecuteProcess(
            # UnlessCondition on sim_base as well: with the base simulated there
            # is no board for the agent to talk to, and an agent holding a serial
            # port that nothing answers is a 30 s wait and a confusing log.
            condition=IfCondition(PythonExpression([
                "'", LaunchConfiguration("micro_ros"), "'.lower() in ('true','1','yes') and ",
                "True" if not no_board else "False"])),
            cmd=["ros2", "run", "micro_ros_agent", "micro_ros_agent"] + micro_ros_args,
            name="micro_ros_agent",
            output="screen",
            # Respawn, for two reasons. An agent that dies mid-run -- a yanked
            # tty, a crash -- otherwise leaves the whole stack headless with no
            # error beyond one "process has died" line. And a simulated robot
            # resets its pose to the origin on every NEW session (main.cpp,
            # createEntities), so the pipeline can zero the pose between the
            # drive suite and SLAM by ending this process: launch brings it
            # back, the board opens a fresh session, and Nav2 starts from (0,0)
            # instead of wherever six manoeuvres parked it.
            respawn=True,
            respawn_delay=1.0,
        ),
        # 3b. The Sim MCU's firmware, on this computer (see host_fw_bin above):
        # the agent's client, booting from the env block a flash would write.
        # Sourcing the host client workspace for THIS process only: it carries
        # micro-ROS builds of the message packages and rmw_microxrcedds, which
        # the rest of the stack must not see. RMW_IMPLEMENTATION too: the stack
        # runs Fast DDS (the image sets it container-wide) and rcl checks the
        # rmw it is bound to against it -- "Expected RMW implementation
        # identifier of 'rmw_fastrtps_cpp' but instead found 'rmw_microxrcedds',
        # exiting with 102" on every respawn, the first time this ran in the
        # image. Said here, the same check now PROVES the firmware is on
        # micro-ROS. Respawned like a board that the watchdog reboots.
        (
            None if not host_fw_bin else ExecuteProcess(
                cmd=["bash", "-c",
                     f"source {host_firmware.setup_script()} && "
                     f"RMW_IMPLEMENTATION=rmw_microxrcedds LINO_ENV_BIN={host_fw_env} "
                     f"exec {host_fw_bin}"],
                name="sim_mcu_firmware",
                output="screen",
                respawn=True,
                respawn_delay=1.0,
            )
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
            parameters=[{"robot_description": robot_description, "frame_prefix": frame_prefix}],
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
                # Respawn, as the serial driver below does: a UDP server that
                # fails its bind once -- the previous stack's driver still
                # holding the port for a moment -- exits 1 and stays dead, and
                # the robot has no scan for the whole session (Sim MCU,
                # 2026-09-26: "UDP,fail to bind. Address already in use").
                respawn=True,
                respawn_delay=2.0,
                parameters=[{
                    "product_name": lidar_product,
                    "topic_name": lidar_topic,
                    "frame_id": laser_frame,
                    "comm_mode": "udp_server",
                    "server_ip": "0.0.0.0",
                    "server_port": int(lidar_udp_port),
                    "laser_scan_dir": True,
                    "bins": lidar_bins,
                    "enable_angle_crop_func": False,
                }],
            )
            if effective_lidar_comm_mode == "udp_server" and not no_board
            else (
                Node(
                    condition=IfCondition(LaunchConfiguration("lidar")),
                    executable=sys.executable,
                    arguments=[os.path.join(REPO_ROOT, "scripts", "sim_laser_node.py")],
                    name="sim_laser_node",
                    output="screen",
                    parameters=[{"frame_id": laser_frame,
                                 "offset_x": float(geometry["laser"]["x"]),
                                 "sonar": host_sonar,
                                 "sonar_frame_id": sonar_frame,
                                 "topic": lidar_topic,
                                 "occlusion": occl_flat,
                                 "occlusion_range": occl_range,
                                 # The configured room, as the board's LD19 and the
                                 # simulated depth camera get it.
                                 **{k: (bool(v) if k == "wall_obstacle" else float(v))
                                    for k, v in depth_camera.sim_room(params).items()}}],
                )
                # The virtual room stands in for the driver on a bare bench; see
                # use_host_sim_laser above for why a present serial port is not.
                if use_host_sim_laser
                # A real RPLIDAR, YDLIDAR or XV-11: its vendor's node, the
                # vendor's per-model settings (lidar_drivers.py), respawned
                # for the same reason as the LD driver below.
                else Node(
                    condition=IfCondition(LaunchConfiguration("lidar")),
                    output="screen",
                    respawn=True,
                    respawn_delay=2.0,
                    **lidar_drivers.serial_node(lidar_model, lidar_port, lidar_baud_named,
                                                laser_frame, lidar_topic),
                )
                if lidar_family != "ldlidar"
                else Node(
                    condition=IfCondition(LaunchConfiguration("lidar")),
                    package="ldlidar_stl_ros2",
                    executable="ldlidar_stl_ros2_node",
                    name="ld19",
                    output="screen",
                    # The serial driver gives the port about three seconds to
                    # produce a valid frame, then logs "ldlidar communication is
                    # abnormal", exits 1 and stays dead. Nothing restarts it, so
                    # a LiDAR that is merely LATE costs the robot /scan for the
                    # whole session -- and everything downstream, since SLAM and
                    # Nav2 wait on a topic that will never come.
                    #
                    # Late is the normal case, not the exceptional one: the
                    # driver and its producer are launched together. On the
                    # gendrv bench the producer is the ESP32, and a run that
                    # flashes the board reboots it, so the firmware is still
                    # coming up while the driver is already counting. That is
                    # exactly how the esp32-jazzy release test failed with
                    # /scan NO DATA on a board that was, when asked a minute
                    # later, emitting clean 47-byte LD19 packets at 15.3 kB/s --
                    # and the same driver against the same port then reported
                    # "communication is normal". A real LD19 does it too: the
                    # motor has to spin up to speed before any frame is valid.
                    #
                    # So: respawn. The cost of a needless restart is one log
                    # line; the cost of not restarting is a robot with no scan.
                    respawn=True,
                    respawn_delay=2.0,
                    parameters=[{
                        "product_name": lidar_product,
                        "topic_name": lidar_topic,
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

    if lidar_masked:
        chain = lidar_mask.filter_chain_params(controller, float(geometry["laser"].get("yaw", 0.0)),
                                               frame_prefix + "base_link")
        chain_file = tempfile.NamedTemporaryFile(mode="w", suffix="_laser_mask.yaml", delete=False)
        yaml.safe_dump(chain, chain_file)
        chain_file.close()
        nodes.append(LogInfo(msg=f"[bringup] LiDAR mask: {len(chain['scan_to_scan_filter_chain']['ros__parameters'])} "
                                 f"laser_filters stage(s), scan_raw -> scan"))
        nodes.append(Node(
            package="laser_filters",
            executable="scan_to_scan_filter_chain",
            name="scan_to_scan_filter_chain",
            output="screen",
            parameters=[chain_file.name],
            remappings=[("scan", "scan_raw"), ("scan_filtered", "scan")],
        ))
    nodes += depth_scan_actions(context, controller, params, geometry, frame_prefix,
                                no_board or controller_name == "sim", scan_from, scan_error)

    # `None` is the "this robot has no such node" placeholder above; launch
    # rejects it, so it never reaches the description.
    live = [n for n in nodes if n is not None]
    # Multi-robot: run the whole stack under /<prefix> so its relative topic
    # names resolve to the same /<prefix>/… the board publishes. The agent is an
    # ExecuteProcess (raw `ros2 run`), not a Node, so PushRosNamespace does not
    # reach it -- but the board already prefixes its own topic names, so the
    # agent needs no remap. Unset -> no group, byte-identical to before.
    if ns:
        return [GroupAction([PushRosNamespace(ns), *live])]
    return live


def depth_scan_actions(context, controller, params, geometry, frame_prefix, boardless, scan_from, scan_error):
    """A depth camera: its driver (or the simulated one) and depthimage_to_laserscan.

    Its scan is /scan on a robot without a LiDAR, and camera/scan beside a LiDAR,
    where Nav2 takes it as a second obstacle source (depth_camera.camera_role).
    The simulated camera stands in when the pipeline runs in simulation mode
    (sim_depth:=true), when the config asks for it (sensors.use_sim_depth, which
    needs no model), and when there is no board -- the Sim MCU has no camera
    either. Remaps are relative, so a topic_prefix namespace moves them with the
    rest of the stack.
    """
    if scan_error:
        return [LogInfo(msg=f"[bringup] depth camera: {scan_error}")]
    role = depth_camera.camera_role(controller)
    if role is None:
        return []
    scan_topic = "scan" if role == "scan" else depth_camera.CAMERA_SCAN_TOPIC
    model = depth_camera.depth_model(controller)
    cam = geometry["depth_camera"]
    frame = frame_prefix + str(cam["frame"])
    sim_arg = str(context.launch_configurations.get("sim_depth", "false")).strip().lower() in ("true", "1", "yes")
    sim = sim_arg or boardless or depth_camera.use_sim_depth(controller) or model is None
    distro = os.environ.get("ROS_DISTRO", "jazzy")
    label = depth_camera.DEPTH_MODELS[model][1] if model else "depth camera"
    out = []
    if sim:
        room = depth_camera.sim_room(params)
        out.append(LogInfo(msg=f"[bringup] {'/scan' if role == 'scan' else scan_topic + ' (Nav2 obstacles)'} "
                               f"from the SIMULATED depth camera (standing in for the {label})"))
        out.append(Node(
            executable=sys.executable,
            arguments=[os.path.join(REPO_ROOT, "scripts", "sim_depth_node.py")],
            name="sim_depth_node",
            output="screen",
            parameters=[{"frame_id": frame,
                         "offset_x": float(cam["x"]), "offset_y": float(cam["y"]),
                         "offset_yaw": float(cam["yaw"]),
                         "map_width": float(room["map_width"]), "map_height": float(room["map_height"]),
                         "wall_obstacle": bool(room["wall_obstacle"]),
                         "wall_x1": float(room["wall_x1"]), "wall_y1": float(room["wall_y1"]),
                         "wall_x2": float(room["wall_x2"]), "wall_y2": float(room["wall_y2"]),
                         "pose_topic": "odom/unfiltered"}],
        ))
        depth_topic, info_topic, scan_time = depth_camera.SIM_DEPTH_TOPIC, depth_camera.SIM_INFO_TOPIC, 1.0 / 15.0
    else:
        pkg, launch_rel, launch_args = depth_camera.driver_launch(model, distro, frame)
        try:
            from ament_index_python.packages import get_package_share_directory
            share = get_package_share_directory(pkg)
        except Exception:  # PackageNotFoundError, or no ament index at all
            family = depth_camera.DEPTH_MODELS[model][0]
            return [LogInfo(msg=f"[bringup] depth camera {label}: the driver package '{pkg}' is not "
                                f"installed, so there is no /scan. Install {depth_camera.INSTALL_HINT[family]}.")]
        out.append(LogInfo(msg=f"[bringup] {'/scan' if role == 'scan' else scan_topic + ' (Nav2 obstacles)'} "
                               f"from the {label} via depthimage_to_laserscan"))
        out.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, launch_rel)),
            launch_arguments=list(launch_args.items()),
        ))
        _, _, _, depth_topic, info_topic = depth_camera.DEPTH_MODELS[model]
        scan_time = 1.0 / 30.0
    out.append(Node(
        package="depthimage_to_laserscan",
        executable="depthimage_to_laserscan_node",
        name="depthimage_to_laserscan",
        output="screen",
        remappings=[("depth", depth_topic.lstrip("/")),
                    ("depth_camera_info", info_topic.lstrip("/")),
                    ("scan", scan_topic)],
        parameters=[{"scan_time": scan_time,
                     "range_min": depth_camera.SCAN_RANGE_MIN,
                     "range_max": depth_camera.SCAN_RANGE_MAX,
                     "scan_height": depth_camera.SCAN_HEIGHT,
                     "output_frame": frame}],
    ))
    return out


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
            "sim_base",
            default_value="false",
            description="Simulate the base on this computer instead of talking to a "
                        "microcontroller: no agent, no serial port, no board. Diagnostic "
                        "and CI use; the release gate still runs on hardware",
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
            description="Fuse the board's field-anchored yaw in the EKF (imu0_config[5]); from sensors.mag if empty",
        ),
        DeclareLaunchArgument(
            "sim_depth",
            default_value="false",
            description="Use the simulated depth camera instead of the configured one "
                        "(the pipeline sets it in simulation mode)",
        ),
        OpaqueFunction(function=launch_setup),
    ])
