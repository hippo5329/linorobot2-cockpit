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
    sensors:
      use_sim_depth: false   # the simulated camera instead of a real one

and its mount under geometry.depth_camera, like geometry.laser.

There is no depth camera on the bench, so the simulated one (scripts/sim_depth_node.py)
is what the bench runs: a D435 in its 424x240 mode, raycasting the SAME room the
firmware's simulated LD19 raycasts (sim_room() below reads the config keys the
firmware reads), from the same pose.
"""
import math

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
}
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

# What must be installed for each family, and what to tell someone when it is not.
DRIVER_PACKAGE = {"realsense": "realsense2_camera", "zed": "zed_wrapper", "oakd": "depthai_ros_driver"}
INSTALL_HINT = {
    "realsense": "sudo apt install ros-$ROS_DISTRO-realsense2-camera (the robot image carries it)",
    "oakd": "sudo apt install ros-$ROS_DISTRO-depthai-ros-driver (the robot image carries it)",
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


def sim_room(params: dict) -> dict:
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
# Explicit `walls` still add to either, for a layout of one's own.
WORLDS = {"wall": "one room, the obstacle wall the Nav2 goal sits behind",
          "rooms": "four spaces joined by 1.2 m doors, for exploration"}


def sim_world(params: dict) -> str:
    sim = (((params or {}).get("base_controller") or {}).get("simulation") or {})
    w = str(sim.get("world") or "wall").strip().lower()
    if w not in WORLDS:
        raise ValueError(f"simulation.world must be one of {', '.join(WORLDS)}, not {w!r}")
    return w


def sim_walls(params: dict) -> list:
    """The interior walls as (x1, y1, x2, y2): the world's, then the configured ones; ValueError on a bad one."""
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


def column_angles():
    """Each column's horizontal angle from the optical axis, left positive (robot convention)."""
    return [math.atan2(CX - u, FX) for u in range(WIDTH)]


def depth_row(ox, oy, yaw, segments, angles, rng=None):
    """One row of z-depth in metres (0 = no return), with the camera's noise when rng is given."""
    import numpy as np
    z = np.zeros(WIDTH, dtype=np.float64)
    for u, a in enumerate(angles):
        t = raycast(ox, oy, yaw + a, segments, 12.0)
        if math.isinf(t):
            continue
        zz = t * math.cos(a)                  # range along the ray -> depth along the optical axis
        if rng is not None:
            zz += rng.normal(0.0, NOISE_K * zz * zz)
        if MIN_Z <= zz <= MAX_Z:
            z[u] = zz
    return z
