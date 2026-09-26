"""Which ROS 2 driver reads a real LiDAR, and how, per `lidar.model`.

Every value here is the VENDOR'S: the RPLIDAR baud rates and scan modes are
sllidar_ros2's own per-model launch files, the YDLIDAR settings are read at
launch from ydlidar_ros2_driver's installed params/<Model>.yaml, and the XV-11
takes its driver's defaults. The robot's config names the model and, when it
must, the port and a baud rate; nothing else is kept here to drift.

    family    package               models (lidar.model)
    ldlidar   ldlidar_stl_ros2      ld19 ld06 stl27l
    sllidar   sllidar_ros2          a1 a2 (= a2m8) a2m7 a2m8 a2m12 a3 c1 s1 s2 s3
    ydlidar   ydlidar_ros2_driver   ydlidar (the driver's generic ydlidar.yaml),
                                    ydlidar_<model> for each params/<Model>.yaml
    xv11      xv_11_driver          xv11

A model this table does not know is REFUSED. It used to fall back to the LD19
driver, which then read an RPLIDAR's port at the wrong baud rate, printed
"ldlidar communication is abnormal" and respawned for ever -- the name of the
real fault was nowhere in the log.

Only the serial path is dispatched. The simulated LD19 (the board's emulator
over UDP, or the host's virtual room) is an LD19 whatever model the config
names, so the simulation paths keep the LD driver.
"""
import os

# name -> (product_name, bins): the LD driver's own vocabulary; `bins` is the
# ray count the fork's node resamples a revolution to.
LDLIDAR_MODELS = {
    "ld19":   ("LDLiDAR_LD19", 456),
    "ld06":   ("LDLiDAR_LD06", 456),
    "stl27l": ("LDLiDAR_STL27L", 2160),
}
# The LD14 and LD14P were in bringup's table, and the driver refuses both:
# ldlidar_stl_ros2's node knows LD06, LD19 and STL27L only and exits
# "input <product_name> is illegal". They are the SL family, read by
# ldlidar_sl_ros2, which this image does not carry -- refused here by name.
NOT_CARRIED = {"ld14": "ldlidar_sl_ros2", "ld14p": "ldlidar_sl_ros2"}

# sllidar_ros2/launch/sllidar_<model>_launch.py: serial_baudrate, scan_mode
# ("" = the driver picks the device's typical mode, as sllidar_s1_launch.py does).
SLLIDAR_MODELS = {
    "a1":    (115200, "Sensitivity"),
    "a2m7":  (256000, "Sensitivity"),
    "a2m8":  (115200, "Sensitivity"),
    "a2m12": (256000, "Sensitivity"),
    "a3":    (256000, "Sensitivity"),
    "c1":    (460800, "Standard"),
    "s1":    (256000, ""),
    "s2":    (1000000, "DenseBoost"),
    "s3":    (1000000, "DenseBoost"),
}
SLLIDAR_ALIASES = {"a2": "a2m8"}   # upstream linorobot2's "a2" is the A2M8

# ydlidar_ros2_driver/params/<file>: model code -> file name.
YDLIDAR_FILES = {
    "ydlidar": "ydlidar.yaml",
    "ydlidar_g1": "G1.yaml", "ydlidar_g2": "G2.yaml", "ydlidar_g4": "G4.yaml",
    "ydlidar_g6": "G6.yaml", "ydlidar_gs2": "GS2.yaml", "ydlidar_gs5": "GS5.yaml",
    "ydlidar_tea": "TEA.yaml", "ydlidar_tg": "TG.yaml",
    "ydlidar_tmini": "Tmini.yaml", "ydlidar_tmini_plus_sh": "Tmini-Plus-SH.yaml",
    "ydlidar_x2": "X2.yaml", "ydlidar_x3": "X3.yaml",
    "ydlidar_x4": "X4.yaml", "ydlidar_x4_pro": "X4-Pro.yaml",
    "ydlidar_sdm15": "sdm15.yaml",
}

XV11_BAUD = 115200     # xv_11_driver's XV11_BAUD_RATE_DEFAULT


def family(model: str) -> str:
    """The driver family for a model name; ValueError for one no driver here reads."""
    m = str(model or "").strip().lower()
    if m in LDLIDAR_MODELS:
        return "ldlidar"
    if m in SLLIDAR_MODELS or m in SLLIDAR_ALIASES:
        return "sllidar"
    if m in YDLIDAR_FILES:
        return "ydlidar"
    if m == "xv11":
        return "xv11"
    if m in NOT_CARRIED:
        raise ValueError(f"lidar.model {model!r} is read by {NOT_CARRIED[m]}, which this image "
                         f"does not carry (ldlidar_stl_ros2 knows ld06, ld19 and stl27l only)")
    known = sorted(list(LDLIDAR_MODELS) + list(SLLIDAR_MODELS) + list(SLLIDAR_ALIASES)
                   + list(YDLIDAR_FILES) + ["xv11"])
    raise ValueError(f"lidar.model {model!r} names no driver this image carries; "
                     f"one of: {', '.join(known)}")


def ld_product(model: str):
    """(product_name, bins) for the LD driver. Any non-LD model gets the LD19's:
    only the simulation paths ask for a non-LD model, and they emulate an LD19."""
    return LDLIDAR_MODELS.get(str(model or "").lower(), LDLIDAR_MODELS["ld19"])


def _share(package: str, share_dir=None) -> str:
    if share_dir:
        return share_dir
    from ament_index_python.packages import get_package_share_directory
    return get_package_share_directory(package)


def ydlidar_params(model: str, share_dir=None) -> dict:
    """The vendor's ros__parameters for this YDLIDAR model, read from the installed package."""
    import yaml
    path = os.path.join(_share("ydlidar_ros2_driver", share_dir), "params", YDLIDAR_FILES[model])
    with open(path) as f:
        doc = yaml.safe_load(f) or {}
    # Keyed by the vendor's node name; take the parameters out so the node
    # name this launch gives it cannot leave them unmatched.
    (block,) = doc.values()
    return dict(block["ros__parameters"])


def serial_node(model: str, port: str, baud, frame_id: str, topic: str, share_dir=None) -> dict:
    """Node(**spec) for a serial LiDAR that is not an LD: package, executable, name,
    parameters and a remapping of the driver's `scan` to `topic`.

    `baud` is the config's (or the launch argument's) rate, or None for the model's.
    """
    m = str(model).strip().lower()
    fam = family(m)
    remap = [] if topic == "scan" else [("scan", topic)]
    if fam == "sllidar":
        rate, mode = SLLIDAR_MODELS[SLLIDAR_ALIASES.get(m, m)]
        p = {"channel_type": "serial", "serial_port": port,
             "serial_baudrate": int(baud or rate), "frame_id": frame_id,
             "inverted": False, "angle_compensate": True}
        if mode:
            p["scan_mode"] = mode
        return {"package": "sllidar_ros2", "executable": "sllidar_node",
                "name": "sllidar_node", "parameters": [p], "remappings": remap}
    if fam == "ydlidar":
        p = ydlidar_params(m, share_dir)
        p.update({"port": port, "frame_id": frame_id})
        if baud:
            p["baudrate"] = int(baud)
        return {"package": "ydlidar_ros2_driver", "executable": "ydlidar_ros2_driver_node",
                "name": "ydlidar_ros2_driver_node", "parameters": [p], "remappings": remap}
    if fam == "xv11":
        p = {"port": port, "baud_rate": int(baud or XV11_BAUD), "frame_id": frame_id,
             "firmware_version": 2}
        return {"package": "xv_11_driver", "executable": "xv_11_driver",
                "name": "xv_11_driver", "parameters": [p], "remappings": remap}
    raise ValueError(f"lidar.model {model!r} is an LD model; the LD driver reads it")
