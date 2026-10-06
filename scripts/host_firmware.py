#!/usr/bin/env python3
"""The Sim MCU as the firmware itself: where its build lives, and the env it boots with.

firmware/host/app compiles src/main.cpp and every firmware library, unmodified,
for the robot computer. It is a micro-ROS UDP4 client of micro_ros_agent -- the
path an ESP32 takes over Wi-Fi -- and its own LD19 emulator streams to
ldlidar_stl_ros2 in udp_server mode. So a run with no board still goes through
micro-ROS, the agent, the XRCE session and the LiDAR driver, instead of an rclpy
node publishing straight into DDS (scripts/sim_base_node.py, which remains for
`sim_base:=true` and for a checkout without the host build).

One module so the launcher and the pipeline cannot disagree about when the
firmware stands in or what it is told.
"""
import os

import cockpit_paths
import mcu_env

# Where the image builds the client workspace and the firmware (docker/Dockerfile).
# It must be built at the path it is used from: colcon bakes the prefix into the
# setup scripts. Overridable for a development build elsewhere.
HOST_WS = os.environ.get("LINO_HOST_WS", "/opt/hosturos_ws")


def binary():
    """The host firmware, or None when this machine has no host build."""
    path = os.path.join(HOST_WS, "app", "lib", "lino_host_fw", "lino_host_fw")
    return path if os.access(path, os.X_OK) else None


def setup_script():
    return os.path.join(HOST_WS, "install", "local_setup.bash")


def env(config_file: str, agent_port: int = 8888, lidar_port: int = 8889, world=None) -> dict:
    """The env block a flash of this robot would write, for the host.

    Built by the flasher's own functions (mcu_env.env_from_config, then every
    sensor simulated, as the Sim MCU always is), then the few keys that are
    facts about the HOST rather than about the robot: it reaches the agent and
    the LiDAR driver on this same computer, over udp4 -- it has no USB device
    port to be a serial client on, and no UART to send an LD19 stream out of.
    A robot config written for a board (the pipeline falls back to the Sim MCU
    when none is plugged in) says serial for both, and would otherwise boot a
    firmware that can reach neither.

    `world` is (map .yaml path, (x, y, yaw) start) on a saved-map world, as the launch
    resolved it (a `world_map:=` argument overrides the config). A board cannot hold a
    map, but this one runs here: its LD19 emulator raycasts the map file itself
    (sim_ld19.h, sim_world_map), so the scan still goes through the LD driver and the
    sonar cone sees the map too (user, 2026-10-06: "sim mcu should do the raycasts from
    imported map and let stl driver publish /scan").
    """
    e = mcu_env.env_from_config(config_file, cockpit_paths.secrets_path(), "127.0.0.1")
    mcu_env.apply_sensor_mode(e, "sim", config_file, holds_map=bool(world and world[0]))
    e.update({
        # The stack's own domain: several Sim MCUs run side by side on one
        # machine (host_matrix.sh, a leg per ROS_DOMAIN_ID), and a board's
        # default of 0 would put them all in one graph.
        "domain_id": int(os.environ.get("ROS_DOMAIN_ID", "0") or 0),
        "transport": "udp4",
        "agent_ip": "127.0.0.1",
        "agent_port": int(agent_port),
        "lidar_comm": "udp",
        "lidar_ip": "127.0.0.1",
        "lidar_port": int(lidar_port),
    })
    if world and world[0]:
        path, start = world
        e["sim_world_map"] = os.path.abspath(path)
        e["sim_world_start"] = ",".join(f"{float(v):g}" for v in (start or (0.0, 0.0, 0.0)))
    return e


def write_env(config_file: str, out_path: str, agent_port: int = 8888, lidar_port: int = 8889,
              world=None) -> str:
    """Write the 4096-byte image the host firmware maps (LINO_ENV_BIN)."""
    os.makedirs(os.path.dirname(out_path) or ".", exist_ok=True)
    tmp = out_path + ".tmp"
    with open(tmp, "wb") as fh:
        fh.write(mcu_env.encode(env(config_file, agent_port, lidar_port, world)))
    os.replace(tmp, out_path)
    return out_path
