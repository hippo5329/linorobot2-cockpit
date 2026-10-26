"""A depth camera as the robot's scan source, and the one rule for who makes /scan.

A robot with no LiDAR can map and navigate on a depth camera: the camera's
driver publishes a depth image, `depthimage_to_laserscan` takes the rows at the
optical centre and publishes them as /scan, and SLAM and Nav2 run unchanged.
This is upstream linorobot2's pattern (linorobot2_bringup sensors.launch.py),
for every depth camera upstream supports -- written from the pattern rather
than copied, because the upstream file does not run as shipped: depth.launch.py
uses EqualsSubstitution without importing it, maps the OAK-D model by calling
dict.get() on a launch substitution (always None), and sensors.launch.py indexes
its topic table eagerly and skips its first key, `realsense`, in the depth-to-scan
condition.

Config (base_controller):

    depth_camera:
      model: realsense   # none | realsense | zed | zedm | zed2 | zed2i | oakd | oakdlite | oakdpro
                         # | astra_pro | astra_pro_openni2
    sensors:
      use_sim_depth: false   # the simulated camera instead of a real one

and its mount under geometry.depth_camera, like geometry.laser.

There is no depth camera on the bench, so the simulated one (scripts/sim_depth_node.py)
is what the bench runs: a D435 in its 424x240 mode, raycasting the SAME room the
firmware's simulated LD19 raycasts (sim_room() below reads the config keys the
firmware reads), from the same pose.
"""
import math
import os

# How every config here spells "not fitted" (one_click_pipeline.NOT_FITTED).
NOT_FITTED = {"", "none", "null", "off", "false", "no"}

# model -> (family, label, frame, depth image topic, camera_info topic)
#
# frame is the root of the camera's own TF tree, which the robot's URDF parents
# to base_link at geometry.depth_camera. RealSense roots its tree at
# <camera_name>_link; ZED's description roots it at <camera_name>_camera_link;
# depthai takes the parent frame as an argument (camera_link here). Topics are what each driver publishes with the arguments
# driver_launch() gives it.
DEPTH_MODELS = {
    "realsense": ("realsense", "Intel RealSense D400", "camera_link",
                  "/camera/depth/image_rect_raw", "/camera/depth/camera_info"),
    "zed":       ("zed", "Stereolabs ZED", "zed_camera_link",
                  "/zed/zed_node/depth/depth_registered", "/zed/zed_node/depth/camera_info"),
    "zedm":      ("zed", "Stereolabs ZED Mini", "zed_camera_link",
                  "/zed/zed_node/depth/depth_registered", "/zed/zed_node/depth/camera_info"),
    "zed2":      ("zed", "Stereolabs ZED 2", "zed_camera_link",
                  "/zed/zed_node/depth/depth_registered", "/zed/zed_node/depth/camera_info"),
    "zed2i":     ("zed", "Stereolabs ZED 2i", "zed_camera_link",
                  "/zed/zed_node/depth/depth_registered", "/zed/zed_node/depth/camera_info"),
    "oakd":      ("oakd", "Luxonis OAK-D", "camera_link",
                  "/oak/stereo/image_raw", "/oak/stereo/camera_info"),
    "oakdlite":  ("oakd", "Luxonis OAK-D Lite", "camera_link",
                  "/oak/stereo/image_raw", "/oak/stereo/camera_info"),
    "oakdpro":   ("oakd", "Luxonis OAK-D Pro", "camera_link",
                  "/oak/stereo/image_raw", "/oak/stereo/camera_info"),
    # The original Astra Pro: depth over OpenNI2 (USB 2bc5:0403), colour as a separate
    # UVC camera (2bc5:0501). Orbbec's ros2_astra_camera drives both (built into the
    # robot image at ASTRA_WS, launchers/astra_camera_depth.launch.py): depth/image_raw
    # (16UC1, millimetres), color/image_raw (rgb8), both 640x480 at 30 Hz with the
    # camera's intrinsics (measured 2026-10-22). Orbbec's current ROS 2 driver,
    # OrbbecSDK_ROS2, finds the camera but has no colour for it, and its depth is a tenth
    # of the true range with NaN intrinsics.
    "astra_pro": ("astra", "Orbbec Astra Pro", "camera_link",
                  "/camera/depth/image_raw", "/camera/depth/camera_info"),
    # The same camera, depth only, on ros-<distro>-openni2-camera once Orbbec's own
    # OpenNI2 is installed (ORBBEC_SDK_DIR below): the fallback from before the image
    # carried ros2_astra_camera. openni2_camera 2.x publishes depth as depth_raw/image
    # (16UC1, millimetres) with depth_raw/camera_info, and depth/image (32FC1, metres)
    # -- there is no depth/image_raw (2026-10-10).
    "astra_pro_openni2": ("openni2", "Orbbec Astra Pro (OpenNI2, depth only)", "camera_link",
                          "/camera/depth_raw/image", "/camera/depth_raw/camera_info"),
}
# The Astra Pro's two drivers see through one camera: optics and simulation are its.
ASTRA_PRO_MODELS = ("astra_pro", "astra_pro_openni2")
OAKD_MODEL = {"oakd": "OAK-D", "oakdlite": "OAK-D-LITE", "oakdpro": "OAK-D-PRO"}
ZED_MODEL = {"zed": "zed", "zedm": "zedm", "zed2": "zed2", "zed2i": "zed2i"}

# The simulated camera's topics, frame and optics: a D435 in its 424x240 mode.
SIM_DEPTH_TOPIC = "/sim_depth/depth/image_rect_raw"
SIM_INFO_TOPIC = "/sim_depth/depth/camera_info"
SIM_FRAME = "camera_link"

# depthimage_to_laserscan, upstream's fake_laser.yaml values: the rows at the
# optical centre, 0.45 m (a stereo camera's near limit with margin) to 10 m.
SCAN_RANGE_MIN = 0.45
SCAN_RANGE_MAX = 10.0
SCAN_HEIGHT = 1

# A real camera's horizontal view (degrees) and near limit (metres), where it is not
# the simulated D435's (HFOV, SCAN_RANGE_MIN). The Astra Pro: 58.4 x 45.5 deg, 0.6-8 m
# (Orbbec's datasheet) -- inside 0.6 m it returns nothing, not a short range.
DEPTH_OPTICS = {m: (58.4, 0.6) for m in ASTRA_PRO_MODELS}

# Orbbec's OpenNI2, which makes openni2_camera see an Astra. Orbbec's driver
# (liborbbec.so) is built for OpenNI 2.3 and segfaults inside Debian's 2.2.0.33
# (XnOniDriver::deviceOpen, 2026-10-10), so the driver node runs on the libOpenNI2 of
# Orbbec's own SDK instead, with the driver beside it. The image does not carry either:
# scripts/install_orbbec_openni2.py fetches Orbbec's SDK release on the robot and keeps
# them under the config directory; openni2_depth.launch.py puts that directory first on
# the node's LD_LIBRARY_PATH.
ORBBEC_SDK_DIR = "drivers/openni2/sdk"
ORBBEC_SDK_FILES = ("libOpenNI2.so.0", "OpenNI.ini", "OpenNI2/Drivers/liborbbec.so")

# Where the robot image keeps Orbbec's ros2_astra_camera (docker/Dockerfile, stage astra):
# its own colcon install, outside /opt/ros, with Orbbec's OpenNI2 in its lib/.
ASTRA_WS = "/opt/astra_ws/install"


def orbbec_sdk_dir(config_dir: str = None) -> str:
    """Where install_orbbec_openni2.py keeps Orbbec's OpenNI2 for this cockpit."""
    import os
    return os.path.join(config_dir or os.environ.get("COCKPIT_CONFIG_DIR", "/config"), ORBBEC_SDK_DIR)

# What must be installed for each family, and what to tell someone when it is not.
DRIVER_PACKAGE = {"realsense": "realsense2_camera", "zed": "zed_wrapper", "oakd": "depthai_ros_driver",
                  "openni2": "openni2_camera", "astra": "astra_camera"}
INSTALL_HINT = {
    "realsense": "sudo apt install ros-$ROS_DISTRO-realsense2-camera (the robot image carries it)",
    "oakd": "sudo apt install ros-$ROS_DISTRO-depthai-ros-driver (the robot image carries it)",
    "openni2": "sudo apt install ros-$ROS_DISTRO-openni2-camera (the robot image carries it)",
    "astra": ("Orbbec's ros2_astra_camera: the cockpit robot image carries it in " + ASTRA_WS +
              ", so this image is older than the Astra Pro driver. Pull the latest cockpit image, "
              "or choose 'Orbbec Astra Pro, depth only' as the depth camera (depth_camera.model: "
              "astra_pro_openni2)"),
    "zed": ("the ZED SDK (CUDA) and zed-ros2-wrapper, built on the robot: "
            "https://github.com/stereolabs/zed-ros2-wrapper -- no generic image can carry them"),
}


def _fitted(value) -> bool:
    return bool(value) and str(value).strip().lower() not in NOT_FITTED


def _truthy(v) -> bool:
    return str(v).strip().lower() in ("1", "true", "yes", "on")


def depth_model(controller: dict):
    """The configured depth camera model, lower-case, or None."""
    m = ((controller or {}).get("depth_camera") or {}).get("model")
    if not _fitted(m):
        return None
    m = str(m).strip().lower()
    if m not in DEPTH_MODELS:
        raise ValueError(f"depth_camera.model {m!r} is not one of: {', '.join(DEPTH_MODELS)}")
    return m


def lidar_fitted(controller: dict) -> bool:
    """A LiDAR named in the config, or the board's simulated LD19 asked for by name."""
    controller = controller or {}
    lidar = controller.get("lidar") or {}
    sensors = controller.get("sensors") or {}
    if isinstance(lidar, dict) and "use_sim_ld19" in lidar:
        sim = _truthy(lidar["use_sim_ld19"])
    else:
        sim = _truthy(sensors.get("use_sim_ld19", False))
    # The Sim MCU has no real sensor at all: its lidar block describes the
    # SIMULATED LD19, so the LiDAR is fitted exactly when that is on. Without
    # this, turning the simulated LD19 off to run on the simulated camera alone
    # still left a "fitted" LiDAR, named by a model nothing would ever drive.
    if str(controller.get("name") or "").strip().lower() == "sim":
        return sim
    # ...and so has every BARE module (no pin assigned but the LED): it simulates or lacks
    # each device, never reads a real one, so its LiDAR is fitted exactly when simulated. An
    # absent device has no topic (user, 2026-10-06): no LiDAR driver, no /scan to wait for.
    # (A bare module always carries its pins block, every pin -1; a config with no pins block
    # at all says nothing about its pins and is judged by its LiDAR model as before.)
    import pin_catalog
    if isinstance(controller.get("pins"), dict) and not pin_catalog.assigned_pins({"base_controller": controller}):
        return sim
    return (isinstance(lidar, dict) and _fitted(lidar.get("model"))) or sim


def camera_fitted(controller: dict) -> bool:
    """A depth camera: a real model named, or the simulated one asked for (which needs no model)."""
    return bool(depth_model(controller)) or use_sim_depth(controller)


def scan_source(controller: dict):
    """Who publishes /scan: "lidar", "depth", or None.

    A LiDAR wins: it sees 360 degrees where a camera sees ~87, so a robot with
    both maps on the LiDAR, and the camera becomes a second obstacle source for
    Nav2 (camera_role). The camera is the scan source only on a robot without
    one. One function for bringup and the pipeline, so the launcher
    and the run cannot disagree about which topic to wait for -- they did about
    `lidar: {model: none}`, which bringup launched a driver for and the
    pipeline did not wait for.
    """
    if lidar_fitted(controller):
        return "lidar"
    if camera_fitted(controller):
        return "depth"
    return None


# The slowest UART that carries raw_scan beside the 50 Hz control loop: the GenDrv's
# 1.5 Mbaud and the UNO Q's 4 Mbaud do; 921600 does not (every topic drops to 40-45 Hz).
RAW_SCAN_MIN_BAUD = 1_500_000


def raw_scan_native_usb(controller: dict) -> bool:
    """The micro-ROS link is the MCU's own USB (CDC), where the baud rate is nominal: an RP2,
    or an ESP32-S3 on its USB-Serial/JTAG port (`console: usb`)."""
    import gen_firmware_header
    mcu = str((controller or {}).get("mcu", "") or "").strip().lower()
    console = str((controller or {}).get("console", "") or "").strip().lower()
    return mcu in gen_firmware_header.RP2_MCUS or (mcu == "esp32s3" and console == "usb")


def raw_scan_over_uart(controller: dict) -> str:
    """Why this board may not send its scan as raw_scan, or "".

    `lidar.comm_mode: topic` carries the LD19 frames over the micro-ROS serial link, which
    must be fast enough to hold them beside the 50 Hz control loop: native USB (Pico, Pico 2,
    XRP, an ESP32-S3 on CDC) or a UART at RAW_SCAN_MIN_BAUD or more (the GenDrv's 1.5 Mbaud,
    the UNO Q's 4 Mbaud). At 921600 -- an ESP32 DevKit, an ESP32-S3 on its UART port like the
    Yahboom's CP2102 -- the scan exhausts the link (user, 2026-10-06: "We use raw scan topic
    only when the serial link is fast enough (like cdc or 1.5M/4M) as in
    pico/pico2/esp32s3 cdc/unoq"). Most robots need none of this: the MCU is on serial and the
    LiDAR on the robot computer; Wi-Fi with a udp scan is for a robot with no robot computer.
    Never over Wi-Fi: the scan floods the micro-ROS session there (measured below).
    """
    lidar = (controller or {}).get("lidar") or {}
    if str(lidar.get("comm_mode", "") or "").strip().lower() != "topic":
        return ""
    if not lidar_fitted(controller):
        return ""
    return raw_scan_link_problem(controller)


def raw_scan_link_problem(controller: dict) -> str:
    """Why this robot's micro-ROS link cannot carry raw_scan, or "" -- the link alone, whether
    or not a LiDAR is named yet (lidar_link.py asks it to offer the UI's choices)."""
    transport = str(controller.get("transport", "serial") or "serial").strip().lower()
    if transport not in ("serial", ""):
        # micro-ROS over Wi-Fi. Measured 2026-10-08 on a GenDrv with a real LD19: raw_scan
        # through the XRCE session over Wi-Fi came at 3.3 Hz, /scan at 0.77 Hz, and it starved
        # the control topics (/imu 3.4 Hz); the same board on its 1.5 Mbaud cable: raw_scan
        # 37.5 Hz, /scan 9.98 Hz, /imu 40 Hz. Over Wi-Fi the board sends the scan as udp,
        # outside micro-ROS (every packet arrived).
        return ("lidar.comm_mode topic sends the scan as raw_scan over micro-ROS, and this robot's "
                "micro-ROS runs over Wi-Fi: the scan floods the session (measured: /scan 0.8 Hz, the "
                "IMU starved). Over Wi-Fi use udp -- the board sends the LiDAR straight to the robot "
                "computer, outside micro-ROS.")
    if raw_scan_native_usb(controller):
        return ""
    try:
        baud = int(controller.get("baudrate") or 921600)
    except (TypeError, ValueError):
        baud = 921600
    if baud >= RAW_SCAN_MIN_BAUD:
        return ""
    return (f"lidar.comm_mode topic sends the scan as raw_scan over micro-ROS, and this board's "
            f"link is a {baud}-baud UART: the scan exhausts it and every topic drops to 40-45 Hz. "
            f"raw_scan needs native USB or {RAW_SCAN_MIN_BAUD} baud or more. Put the LiDAR on the "
            f"robot computer (serial), or use udp over Wi-Fi on a robot with no robot computer.")


def sim_scan_unreachable(controller: dict, lidar_port: str) -> str:
    """Why a board's SIMULATED LD19 has no way to the LD driver, or "".

    A LiDAR not on the MCU is on the robot computer, at `lidar.serial_port`: the usual robot
    (user, 2026-10-06: "when lidar is not connected to mcu, lidar can be connected to robot
    computer. Only issue is that we cannot raycast in sim mode"). The board's emulator can
    only reach that port through a bridge on its LIDAR_RXD (the GenDrv bench), as raw_scan
    (`topic`) or over Wi-Fi (`udp`). A simulated `serial` scan whose port does not exist
    reaches nothing, so in Sim mode the robot has no /scan -- the robot computer's own laser
    stands in only on the no-board path (sim_base) -- and bringup and the pipeline say so."""
    if scan_source(controller) != "lidar":
        return ""
    if not ((controller.get("sensors") or {}).get("use_sim_ld19", False)):
        return ""
    lidar = controller.get("lidar") or {}
    if str(lidar.get("comm_mode", "serial") or "serial").strip().lower() != "serial":
        return ""
    if os.path.exists(lidar_port):
        return ""
    return (f"the LiDAR is on the robot computer ({lidar_port}, not present), and Sim mode cannot "
            f"raycast into it: the board's simulated LD19 reaches the robot computer only through "
            f"a bridge on LIDAR_RXD, as raw_scan on a fast link, or over Wi-Fi. A real LiDAR there "
            f"is read by the LD driver in Real mode; sim_base:=true raycasts on the robot computer")


def scan_fov_deg(controller: dict):
    """How many degrees /scan sees, or None when nothing publishes it.

    A depth camera's horizontal view (HFOV, ~87 deg); a LiDAR's 360 less what its
    mask blocks (lidar.mask.sectors -- a mower body, a mast). A narrow scan maps
    with a capped turn rate (mapping_limits.py); a depth camera's is also held to
    the odometry (narrow_slam_overrides).
    """
    src = scan_source(controller)
    if src == "depth":
        return depth_optics(controller)[0]
    if src == "lidar":
        import lidar_mask
        blocked = sum(w for _, w in lidar_mask.mask_config(controller)["sectors"])
        return max(0.0, 360.0 - blocked)
    return None


# A scan that sees less than this many degrees is narrow (mapping_limits.py).
# A depth camera (~87) and a masked mower LiDAR (the gate's: 210) are narrow; a
# full LiDAR (360) is not.
NARROW_FOV_DEG = 270.0


def narrow_slam_overrides(params: dict) -> dict:
    """The slam_toolbox parameters a depth camera's scan takes over the template's, or {}.

    slam.narrow_fov_overrides sits beside slam.slam_toolbox in the robot config;
    slam.launch.py applies it when /scan comes from a depth camera. A LiDAR, masked
    or not, keeps slam_toolbox's own values, as every Nav2 robot we compared does
    (TurtleBot 4, Clearpath, Husarion, Neobotix). Held to the odometry, a masked
    LiDAR on a real mecanum base (215 deg, 2026-10-17) followed the odometry's
    heading creep: home within 0.01 m in the map, 0.8 m away on the floor. Without
    the overrides, the same robot's position held; ~10 deg of heading per soak
    chunk remained (the odometry's, not SLAM's).
    """
    narrow = ((params or {}).get("slam") or {}).get("narrow_fov_overrides")
    bc = (params or {}).get("base_controller") or {}
    if isinstance(narrow, dict) and narrow and scan_source(bc) == "depth":
        return dict(narrow)
    return {}


def camera_role(controller: dict):
    """What a fitted depth camera is for: "scan" (it IS /scan), "obstacles", or None.

    "obstacles" is a robot with a LiDAR too -- the Sim MCU with both of its
    simulated sensors on: SLAM keeps the LiDAR's 360 degree /scan, and the
    camera's scan, on CAMERA_SCAN_TOPIC, is added to Nav2's costmap obstacle
    layers as a second observation source.
    """
    if not camera_fitted(controller):
        return None
    return "scan" if scan_source(controller) == "depth" else "obstacles"


# The camera's scan when it is not /scan (camera_role "obstacles"), relative so a
# topic_prefix namespace moves it with the rest of the stack.
CAMERA_SCAN_TOPIC = "camera/scan"


def use_sim_depth(controller: dict) -> bool:
    return _truthy(((controller or {}).get("sensors") or {}).get("use_sim_depth", False))


def depth_optics(controller: dict):
    """(horizontal view in degrees, near limit in metres) of the camera that makes the scan.

    The configured model's own (DEPTH_OPTICS), real or simulated: the simulated camera
    renders that model's optics (SIM_CAMERAS). The D435's for no model, or a model
    with no entry.
    """
    model = depth_model(controller)
    if model in DEPTH_OPTICS:
        return DEPTH_OPTICS[model]
    return math.degrees(HFOV), SCAN_RANGE_MIN


def scan_range_min(controller: dict) -> float:
    """depthimage_to_laserscan's and the costmaps' near limit for this robot's camera."""
    return depth_optics(controller)[1]


def stream_frames(camera_frame: str, stream: str = "depth"):
    """(frame, optical frame) of one of a camera's streams under its root frame.

    camera_link, depth -> camera_depth_frame, camera_depth_optical_frame; a prefix stays
    a prefix (robot1/camera_link -> robot1/camera_depth_frame ...), never a leading "/".
    """
    base = camera_frame[:-len("_link")] if camera_frame.endswith("_link") else camera_frame
    return f"{base}_{stream}_frame", f"{base}_{stream}_optical_frame"


def openni2_frames(camera_frame: str):
    """(depth frame, depth optical frame) under an OpenNI2 camera's root frame."""
    return stream_frames(camera_frame, "depth")


def astra_driver_missing(ws: str = None) -> bool:
    """True when the image lacks ros2_astra_camera's node or the OpenNI2 it runs on."""
    import os
    ws = ws or ASTRA_WS
    return not (os.access(os.path.join(ws, "lib", "astra_camera", "astra_camera_node"), os.X_OK)
                and os.path.isfile(os.path.join(ws, "lib", "libOpenNI2.so")))


def orbbec_driver_missing(model: str, config_dir: str = None) -> bool:
    """True when the model needs Orbbec's OpenNI2 and the config dir lacks any part of it."""
    if model is None or DEPTH_MODELS[model][0] != "openni2":
        return False
    import os
    d = orbbec_sdk_dir(config_dir)
    return not all(os.path.isfile(os.path.join(d, f)) for f in ORBBEC_SDK_FILES)


def driver_launch(model: str, distro: str, frame: str):
    """(package, launch file relative to its share/, launch arguments) for a real camera."""
    family = DEPTH_MODELS[model][0]
    if family == "realsense":
        # An empty namespace and camera_name `camera` give /camera/depth/... and a
        # tree rooted at camera_link. The driver's own defaults (namespace
        # `camera`) double the prefix: /camera/camera/depth/image_rect_raw.
        return "realsense2_camera", "launch/rs_launch.py", {
            "camera_namespace": "", "camera_name": "camera",
            "enable_color": "false", "pointcloud.enable": "false", "initial_reset": "true"}
    if family == "zed":
        # publish_tf / publish_map_tf off: left on, the ZED publishes odom and
        # map from its own visual odometry, a second parent for frames the EKF
        # and SLAM already own.
        return "zed_wrapper", "launch/zed_camera.launch.py", {
            "camera_model": ZED_MODEL[model], "camera_name": "zed",
            "publish_urdf": "true", "publish_tf": "false", "publish_map_tf": "false"}
    if family == "astra":
        # Our own launch file, not the package's astra_pro.launch.xml: that one names its
        # frames after camera_name, turns the IR stream and a point cloud on, and runs on
        # whatever libOpenNI2 the library path finds first.
        return "linorobot2_cockpit", "launchers/astra_camera_depth.launch.py", {
            "namespace": "camera", "camera_frame": frame}
    if family == "openni2":
        # Our own launch file, not the package's camera_only.launch.py: that one turns
        # depth registration on (an Astra Pro has no OpenNI colour stream to register to)
        # and its tfs.launch.py joins frame names as tf_prefix + "/" + namespace, so with no
        # prefix every frame starts with "/" and matches nothing.
        return "linorobot2_cockpit", "launchers/openni2_depth.launch.py", {
            "namespace": "camera", "camera_frame": frame}
    # depthai renamed its launch file between 2.x (jazzy) and 3.x (lyrical).
    launch = "launch/camera.launch.py" if distro == "jazzy" else "launch/driver.launch.py"
    return "depthai_ros_driver", launch, {
        "camera_model": OAKD_MODEL[model], "parent_frame": frame,
        "enable_color": "false", "pointcloud.enable": "false"}


# --------------------------------------------------------------- the simulated room
# The firmware's own defaults (sim_ld19.h), overridden by base_controller.simulation
# exactly as mcu_env.py hands them to the board -- so the camera, the host laser
# and the board's LD19 all see one room, and SLAM draws one map of it.
ROOM_DEFAULTS = {"map_width": 10.0, "map_height": 6.0, "wall_obstacle": True,
                 "wall_x1": 2.0, "wall_y1": -1.5, "wall_x2": 2.0, "wall_y2": 1.5}


def is_real_robot(params: dict) -> bool:
    """Return True if this configuration represents a real physical robot rather than simulation."""
    bc = (params or {}).get("base_controller") or {}
    sim = bc.get("simulation") or {}
    mode = str(sim.get("mode") or bc.get("mode") or (params or {}).get("mode") or "").strip().lower()
    if mode == "real":
        return True
    if mode in ("sim", "simulation"):
        return False
    if sim.get("enabled") is False:
        return True
    if bc.get("name") == "sim":
        return False
    sensors = bc.get("sensors") or {}
    sim_flags = [sensors[k] for k in sensors if k.startswith("use_sim_")]
    if sim_flags and not any(bool(v) for v in sim_flags):
        return True
    return False


def sim_room(params: dict) -> dict:
    if is_real_robot(params):
        return dict(ROOM_DEFAULTS, wall_obstacle=False)
    sim = (((params or {}).get("base_controller") or {}).get("simulation") or {})
    room = dict(ROOM_DEFAULTS)
    for k in ROOM_DEFAULTS:
        if k in sim and sim[k] is not None:
            room[k] = _truthy(sim[k]) if k == "wall_obstacle" else float(sim[k])
    if sim_world(params) == "rooms":
        room["wall_obstacle"] = False     # the rooms replace the single test wall
    return room


# Interior walls: base_controller.simulation.walls, [[x1, y1, x2, y2], ...] in
# metres -- a multi-room world for exploration. The firmware's table holds this
# many (sim_ld19.h SIM_WALLS_MAX); the env key is sim_walls.
SIM_WALLS_MAX = 12


# A multi-room world for exploration tests, inside the default 10 x 6 m box
# (set wall_obstacle: false). Four spaces: west, the middle one the robot starts
# in, and the east room split in two -- joined by 1.2 m doors, wide enough for
# the 0.26 m robot radius plus inflation to pass.
MULTI_ROOM_WALLS = [
    [-1.5, -3.0, -1.5, -0.6], [-1.5, 0.6, -1.5, 3.0],   # west wall of the middle room, door at y 0
    [1.5, -3.0, 1.5, 1.8],                              # east wall of the middle room, door at the north end
    [1.5, 0.0, 3.8, 0.0],                               # splits the east room, gap at the far wall
]


# The simulated worlds a robot can be put in, by name: base_controller.simulation.world.
#   wall   the 10 x 6 m room with the one obstacle wall the Nav2 goal sits behind (default)
#   rooms  the same box as four spaces joined by doors (MULTI_ROOM_WALLS), no obstacle wall:
#          the world for frontier exploration
#   map    a saved map (simulation.world_map), raycast by the host's simulated sensors
#   none   real robot / physical space (simulation disabled)
# Explicit `walls` still add to either, for a layout of one's own.
WORLDS = {"wall": "one room, the obstacle wall the Nav2 goal sits behind",
          "rooms": "four spaces joined by 1.2 m doors, for exploration",
          "map": "a saved map (simulation.world_map), raycast by the host's simulated sensors"}


def sim_world(params: dict) -> str:
    if is_real_robot(params):
        return "none"
    sim = (((params or {}).get("base_controller") or {}).get("simulation") or {})
    w = str(sim.get("world") or "wall").strip().lower()
    if w not in WORLDS:
        raise ValueError(f"simulation.world must be one of {', '.join(WORLDS)}, not {w!r}")
    return w


def world_map(params: dict):
    """(map .yaml path, (x, y, yaw) start in the map) for world "map", else (None, None).

    The start defaults to the map's own (0, 0, 0): a map this pipeline saved is
    anchored where its run started, so the robot starts where that one did.
    A relative path is taken from the maps directory (/ws/maps in the image)."""
    if is_real_robot(params):
        return None, None
    if sim_world(params) != "map":
        return None, None
    sim = (((params or {}).get("base_controller") or {}).get("simulation") or {})
    path = str(sim.get("world_map") or "").strip()
    if not path:
        raise ValueError("simulation.world is map but simulation.world_map names no map .yaml")
    if not os.path.isabs(path):
        path = os.path.join(os.environ.get("COCKPIT_MAPS_DIR", "/ws/maps"), path)
    start = sim.get("world_start") or [0.0, 0.0, 0.0]
    if not isinstance(start, (list, tuple)) or len(start) != 3:
        raise ValueError(f"simulation.world_start is [x, y, yaw], not {start!r}")
    return path, tuple(float(v) for v in start)


class GridWorld:
    """A saved occupancy map as the simulated world: every beam cast at once.

    The pose the simulated sensors get is the robot's odometry pose, which
    starts at (0, 0, 0); `start` places that origin in the map. Cells past the
    map's edge are empty, like the unknown beyond a real room's open door."""

    def __init__(self, yaml_path: str, start=(0.0, 0.0, 0.0)):
        import numpy as np
        import yaml
        with open(yaml_path) as fh:
            meta = yaml.safe_load(fh)
        img = meta["image"]
        if not os.path.isabs(img):
            img = os.path.join(os.path.dirname(yaml_path), img)
        with open(img, "rb") as fh:
            data = fh.read()
        # PGM (P5): magic, optional comments, width height, maxval, bytes.
        tokens, pos = [], 0
        while len(tokens) < 4:
            while data[pos:pos + 1].isspace():
                pos += 1
            if data[pos:pos + 1] == b"#":
                pos = data.index(b"\n", pos) + 1
                continue
            end = pos
            while not data[end:end + 1].isspace():
                end += 1
            tokens.append(data[pos:end]); pos = end
        if tokens[0] != b"P5":
            raise ValueError(f"{img}: only binary PGM (P5) maps are read")
        w, h, maxval = int(tokens[1]), int(tokens[2]), int(tokens[3])
        px = np.frombuffer(data[pos + 1:pos + 1 + w * h], dtype=np.uint8).reshape(h, w).astype(np.float64)
        occ = (maxval - px) / maxval if not meta.get("negate", 0) else px / maxval
        # row 0 of the image is the TOP of the map; index [y][x] from the bottom
        self.occupied = (occ > float(meta.get("occupied_thresh", 0.65)))[::-1, :]
        self.res = float(meta["resolution"])
        self.ox, self.oy = float(meta["origin"][0]), float(meta["origin"][1])
        self.start = tuple(float(v) for v in start)
        self.np = np

    def to_world(self, x: float, y: float, yaw: float):
        sx, sy, syaw = self.start
        c, s = math.cos(syaw), math.sin(syaw)
        return sx + c * x - s * y, sy + s * x + c * y, syaw + yaw

    def ranges(self, ox: float, oy: float, angles, max_range: float = 12.0):
        """Ranges from (ox, oy) in the MAP frame along each absolute angle; inf where nothing is hit."""
        np = self.np
        a = np.asarray(angles, dtype=np.float64)
        steps = np.arange(0.03, max_range, self.res * 0.5)
        xs = ox + np.cos(a)[:, None] * steps[None, :]
        ys = oy + np.sin(a)[:, None] * steps[None, :]
        cx = np.floor((xs - self.ox) / self.res).astype(np.int64)
        cy = np.floor((ys - self.oy) / self.res).astype(np.int64)
        h, w = self.occupied.shape
        inside = (cx >= 0) & (cx < w) & (cy >= 0) & (cy < h)
        hit = np.zeros(cx.shape, dtype=bool)
        hit[inside] = self.occupied[cy[inside], cx[inside]]
        first = np.where(hit.any(axis=1), hit.argmax(axis=1), -1)
        out = np.full(a.shape, np.inf)
        out[first >= 0] = steps[first[first >= 0]]
        return out


def sim_walls(params: dict) -> list:
    """The interior walls as (x1, y1, x2, y2): the world's, then the configured ones; ValueError on a bad one."""
    if is_real_robot(params):
        return []
    sim = (((params or {}).get("base_controller") or {}).get("simulation") or {})
    out = [tuple(float(v) for v in w) for w in MULTI_ROOM_WALLS] if sim_world(params) == "rooms" else []
    for w in sim.get("walls") or []:
        if not isinstance(w, (list, tuple)) or len(w) != 4:
            raise ValueError(f"simulation.walls: each wall is [x1, y1, x2, y2], not {w!r}")
        out.append(tuple(float(v) for v in w))
    if len(out) > SIM_WALLS_MAX:
        raise ValueError(f"simulation.walls: at most {SIM_WALLS_MAX} walls, not {len(out)}")
    return out


def walls_flat(walls) -> list:
    """A ROS double-array parameter: the walls end to end, or [0.0] for none."""
    return [v for w in walls for v in w] or [0.0]


def walls_from_flat(flat) -> list:
    vals = [float(v) for v in (flat or [])]
    return [tuple(vals[i:i + 4]) for i in range(0, len(vals) - len(vals) % 4, 4)] if len(vals) >= 4 else []


def walls_env(walls) -> str:
    """The firmware's sim_walls value: "x1,y1,x2,y2;x1,y1,x2,y2"."""
    return ";".join(",".join(f"{v:g}" for v in w) for w in walls)


def room_segments(room: dict, walls=()) -> list:
    """The room's walls as (x1, y1, x2, y2), centred on the origin, the obstacle, then interior walls."""
    hw, hh = float(room["map_width"]) / 2.0, float(room["map_height"]) / 2.0
    segs = [(-hw, -hh, hw, -hh), (hw, -hh, hw, hh), (hw, hh, -hw, hh), (-hw, hh, -hw, -hh)]
    if room.get("wall_obstacle", True):
        segs.append((float(room["wall_x1"]), float(room["wall_y1"]),
                     float(room["wall_x2"]), float(room["wall_y2"])))
    segs.extend(tuple(float(v) for v in w) for w in walls)
    return segs


def raycast(ox: float, oy: float, angle: float, segments: list, max_range: float) -> float:
    """Distance from (ox, oy) along `angle` to the nearest wall, or inf past max_range."""
    ca, sa = math.cos(angle), math.sin(angle)
    best = max_range
    for (x1, y1, x2, y2) in segments:
        sx, sy = x2 - x1, y2 - y1
        denom = ca * sy - sa * sx
        if abs(denom) < 1e-9:
            continue
        px, py = x1 - ox, y1 - oy
        t = (px * sy - py * sx) / denom
        u = (px * sa - py * ca) / denom
        if t > 0.03 and 0.0 <= u <= 1.0 and t < best:
            best = t
    return best if best < max_range else float("inf")


# --------------------------------------------------------------- the simulated camera
# A RealSense D435 in its 424x240 depth mode (scripts/sim_depth_node.py says why).
WIDTH, HEIGHT = 424, 240
HFOV, VFOV = math.radians(87.0), math.radians(58.0)
FX = (WIDTH / 2.0) / math.tan(HFOV / 2.0)
FY = (HEIGHT / 2.0) / math.tan(VFOV / 2.0)
CX, CY = (WIDTH - 1) / 2.0, (HEIGHT - 1) / 2.0
MIN_Z, MAX_Z = 0.2, 10.0
BASELINE_M, SUBPIXEL = 0.050, 0.08
NOISE_K = SUBPIXEL / (FX * BASELINE_M)        # sigma_z = NOISE_K * z^2
RATE_HZ = 15.0


class SimCamera:
    """The optics the simulated camera renders with: one per model it can stand in for."""

    def __init__(self, name, width, height, hfov_deg, vfov_deg, min_z, max_z, noise_k, rate_hz):
        self.name, self.width, self.height = name, width, height
        self.hfov, self.vfov = math.radians(hfov_deg), math.radians(vfov_deg)
        self.fx = (width / 2.0) / math.tan(self.hfov / 2.0)
        self.fy = (height / 2.0) / math.tan(self.vfov / 2.0)
        self.cx, self.cy = (width - 1) / 2.0, (height - 1) / 2.0
        self.min_z, self.max_z, self.noise_k, self.rate_hz = min_z, max_z, noise_k, rate_hz


# The simulated camera is the configured model's when it has an entry here, so a robot
# simulated with an Astra Pro sees what the Astra will: 58.4 x 45.5 deg, nothing inside
# 0.6 m or past 8 m (Orbbec's datasheet), 640x480 at 30 Hz (openni2_camera's default
# mode). Its noise is a structured-light camera's, sigma_z ~ 1.5 mm x z^2 (a Kinect v1's,
# Khoshelham & Elberink 2012; Orbbec gives +-1-3 mm at 1 m). Any other model, or none,
# is the D435 above.
SIM_CAMERAS = {
    "d435": SimCamera("D435", WIDTH, HEIGHT, 87.0, 58.0, MIN_Z, MAX_Z, NOISE_K, RATE_HZ),
    "astra_pro": SimCamera("Astra Pro", 640, 480, 58.4, 45.5, 0.6, 8.0, 0.0015, 30.0),
}


def sim_camera_key(controller: dict) -> str:
    """Which SIM_CAMERAS entry the simulated camera renders for this robot."""
    model = depth_model(controller)
    if model in ASTRA_PRO_MODELS:
        return "astra_pro"
    return model if model in SIM_CAMERAS else "d435"


def column_angles(cam=None):
    """Each column's horizontal angle from the optical axis, left positive (robot convention)."""
    cam = cam or SIM_CAMERAS["d435"]
    return [math.atan2(cam.cx - u, cam.fx) for u in range(cam.width)]


def depth_row(ox, oy, yaw, segments, angles, rng=None, cam=None):
    """One row of z-depth in metres (0 = no return), with the camera's noise when rng is given."""
    import numpy as np
    cam = cam or SIM_CAMERAS["d435"]
    z = np.zeros(len(angles), dtype=np.float64)
    if isinstance(segments, GridWorld):
        # a saved map as the world: every column in one pass, in the map frame
        wx, wy, wyaw = segments.to_world(ox, oy, yaw)
        ts = segments.ranges(wx, wy, wyaw + np.asarray(angles), 12.0)
    for u, a in enumerate(angles):
        t = float(ts[u]) if isinstance(segments, GridWorld) else raycast(ox, oy, yaw + a, segments, 12.0)
        if math.isinf(t):
            continue
        zz = t * math.cos(a)                  # range along the ray -> depth along the optical axis
        if rng is not None:
            zz += rng.normal(0.0, cam.noise_k * zz * zz)
        if cam.min_z <= zz <= cam.max_z:
            z[u] = zz
    return z
