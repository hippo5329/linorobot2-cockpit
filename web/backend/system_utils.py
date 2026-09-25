#!/usr/bin/env python3
# ==============================================================================
# Linorobot2 Cockpit — System, Containers, Sensors, Autostart & AI Helpers
# ==============================================================================

import json
import math
import os
import platform
import re
import shlex
import shutil
import subprocess
import sys
from typing import Any, Dict, List, Optional

from runners import ros_setup_shell

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
SUPPORTED_DISTROS = ["jazzy", "lyrical", "rolling"]

LASER_SENSORS = {
    "ldlidar": {
        "label": "LDROBOT (LD06 / LD19 / STL27L)",
        "serial": True,
        "symlink": "/dev/ldlidar",
        "default_baud": "230400",
        "docker_key": "ldlidar",
        "driver_pkg": "ldlidar_stl_ros2",
        "models": [
            {"code": "ld06", "label": "LD06", "product": "LDLiDAR_LD06", "bins": 456, "baud": "230400"},
            {"code": "ld19", "label": "LD19", "product": "LDLiDAR_LD19", "bins": 456, "baud": "230400"},
            {"code": "stl27l", "label": "STL27L", "product": "LDLiDAR_STL27L", "bins": 2160, "baud": "921600"},
        ],
        "install": [
            "sudo apt-get update && sudo apt-get install -y ros-$ROS_DISTRO-ldlidar-stl-ros2 || true",
        ],
        "udev": [
            "cd /tmp && wget -q https://raw.githubusercontent.com/linorobot/ldlidar/ros2/ldlidar.rules",
            "sudo cp ldlidar.rules /etc/udev/rules.d",
            "sudo udevadm control --reload-rules && sudo udevadm trigger",
        ],
    },
    "sllidar": {
        "label": "RPLIDAR (A1/A2/A3/C1/S1/S2/S3)",
        "serial": True,
        "symlink": "/dev/rplidar",
        "default_baud": "115200",
        "docker_key": "rplidar",
        "driver_pkg": "sllidar_ros2",
        "models": [
            {"code": "a1", "label": "RPLIDAR A1"},
            {"code": "a2", "label": "RPLIDAR A2"},
            {"code": "a3", "label": "RPLIDAR A3"},
            {"code": "c1", "label": "RPLIDAR C1"},
            {"code": "s1", "label": "RPLIDAR S1"},
            {"code": "s2", "label": "RPLIDAR S2"},
            {"code": "s3", "label": "RPLIDAR S3"},
        ],
        "install": [
            "sudo apt-get update && sudo apt-get install -y ros-$ROS_DISTRO-sllidar-ros2 || true",
        ],
        "udev": [
            "echo 'KERNEL==\"ttyUSB*\", ATTRS{idVendor}==\"10c4\", ATTRS{idProduct}==\"ea60\", MODE:=\"0666\", GROUP:=\"dialout\", SYMLINK+=\"rplidar\"' | sudo tee /etc/udev/rules.d/rplidar.rules",
            "sudo udevadm control --reload-rules && sudo udevadm trigger",
        ],
    },
    "ydlidar": {
        "label": "YDLIDAR (X4 / G4 / TG30)",
        "serial": True,
        "symlink": "/dev/ydlidar",
        "default_baud": "128000",
        "docker_key": "ydlidar",
        "driver_pkg": "ydlidar_ros2_driver",
        "models": [{"code": "ydlidar", "label": "YDLIDAR X4 / G4 / others"}],
        "install": [
            "sudo apt-get update && sudo apt-get install -y ros-$ROS_DISTRO-ydlidar-ros2-driver || true",
        ],
        "udev": [
            'echo \'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0666", GROUP:="dialout", SYMLINK+="ydlidar"\' | sudo tee /etc/udev/rules.d/ydlidar.rules',
            "sudo udevadm control --reload-rules && sudo udevadm trigger",
        ],
    },
    "xv11": {
        "label": "XV11 (Neato)",
        "serial": True,
        "symlink": None,
        "default_baud": "115200",
        "docker_key": "xv11",
        "driver_pkg": "xv_11_driver",
        "models": [{"code": "xv11", "label": "Neato XV11"}],
        "install": [
            "sudo apt-get update && sudo apt-get install -y ros-$ROS_DISTRO-xv-11-driver || true",
        ],
        "udev": None,
    },
}

DEPTH_SENSORS = {
    "realsense": {
        "label": "Intel RealSense (D415 / D435 / D455)",
        "serial": False,
        "symlink": None,
        "docker_key": "realsense",
        "driver_pkg": "realsense2_camera",
        "models": [
            {"code": "realsense", "label": "RealSense D4xx series"},
            {"code": "d435", "label": "RealSense D435 / D435i"},
            {"code": "d455", "label": "RealSense D455"},
        ],
        "install": ["sudo apt-get update && sudo apt-get install -y ros-$ROS_DISTRO-realsense2-camera"],
        "udev": [
            "cd /tmp && wget -q https://raw.githubusercontent.com/IntelRealSense/librealsense/master/config/99-realsense-libusb.rules",
            "sudo cp 99-realsense-libusb.rules /etc/udev/rules.d",
            "sudo udevadm control --reload-rules && sudo udevadm trigger",
        ],
    },
    "oakd": {
        "label": "Luxonis OAK-D / Lite / Pro",
        "serial": False,
        "symlink": None,
        "docker_key": "oakd",
        "driver_pkg": "depthai_ros",
        "models": [
            {"code": "oakd", "label": "OAK-D"},
            {"code": "oakdlite", "label": "OAK-D Lite"},
            {"code": "oakdpro", "label": "OAK-D Pro"},
        ],
        "install": ["sudo apt-get update && sudo apt-get install -y ros-$ROS_DISTRO-depthai-ros"],
        "udev": [
            'echo \'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"\' | sudo tee /etc/udev/rules.d/80-movidius.rules',
            "sudo udevadm control --reload-rules && sudo udevadm trigger",
        ],
    },
    "astra": {
        "label": "Orbbec Astra / Astra Pro",
        "serial": False,
        "symlink": None,
        "docker_key": "astra",
        "driver_pkg": "astra_camera",
        "models": [
            {"code": "astra", "label": "Orbbec Astra"},
            {"code": "astrapro", "label": "Orbbec Astra Pro"},
        ],
        "install": [
            "sudo apt-get update && sudo apt-get install -y ros-$ROS_DISTRO-astra-camera || true",
        ],
        "udev": [
            'echo \'ATTRS{idVendor}=="2bc5", ATTRS{idProduct}=="0401", MODE="0666", GROUP="dialout"\' | sudo tee /etc/udev/rules.d/56-orbbec-usb.rules',
            "sudo udevadm control --reload-rules && sudo udevadm trigger",
        ],
    },
    "zed": {
        "label": "Stereolabs ZED / ZED2 / ZED Mini",
        "serial": False,
        "symlink": None,
        "docker_key": "zed",
        "driver_pkg": "zed_wrapper",
        "models": [
            {"code": "zed", "label": "ZED"},
            {"code": "zedm", "label": "ZED Mini"},
            {"code": "zed2", "label": "ZED 2"},
            {"code": "zed2i", "label": "ZED 2i"},
            {"code": "zedx", "label": "ZED X"},
        ],
        "install": None,
        "udev": None,
    },
}

SENSOR_REGISTRY = {
    "lidar": [
        {"id": "ld19", "name": "LD19 / LD06 (DTOF LiDAR)", "default_port": "/dev/ttyUSB1", "default_baud": 230400, "pkg": "ldlidar_stl_ros2"},
        {"id": "ydlidar", "name": "YDLidar (X4 / G4 / TG30)", "default_port": "/dev/ttyUSB1", "default_baud": 128000, "pkg": "ydlidar_ros2_driver"},
        {"id": "rplidar", "name": "RPLidar (A1 / A2 / A3 / S1)", "default_port": "/dev/ttyUSB1", "default_baud": 115200, "pkg": "sllidar_ros2"},
        {"id": "xv11", "name": "Neato XV-11", "default_port": "/dev/ttyUSB1", "default_baud": 115200, "pkg": "xv_11_driver"},
    ],
    "depth_camera": [
        {"id": "realsense", "name": "Intel RealSense (D435 / D455)", "pkg": "realsense2_camera"},
        {"id": "oakd", "name": "Luxonis OAK-D / Lite / Pro", "pkg": "depthai_ros"},
        {"id": "astra", "name": "Orbbec Astra / Astra Pro", "pkg": "astra_camera"},
        {"id": "zed", "name": "Stereolabs ZED / ZED2", "pkg": "zed_wrapper"},
    ],
    "imu": [
        {"id": "qmi8658", "name": "QMI8658 (Onboard GenDrv)", "i2c_addr": "0x6B", "type": "6-DoF"},
        {"id": "mpu6050", "name": "MPU6050 (Classic)", "i2c_addr": "0x68", "type": "6-DoF"},
        {"id": "mpu9250", "name": "MPU9250", "i2c_addr": "0x68", "type": "9-DoF"},
        {"id": "bno085", "name": "BNO085 (High Precision IMU)", "i2c_addr": "0x4A", "type": "9-DoF"},
    ]
}


def check_container_status() -> Dict[str, Any]:
    has_docker = shutil.which("docker") is not None
    has_podman = shutil.which("podman") is not None
    is_rootless_docker = False

    if has_docker:
        try:
            info_res = subprocess.run(["docker", "info", "-f", "{{.SecurityOptions}}"], capture_output=True, text=True, timeout=3)
            if "rootless" in (info_res.stdout or "").lower():
                is_rootless_docker = True
            elif "docker.sock" in os.environ.get("DOCKER_HOST", "") and "run/user" in os.environ.get("DOCKER_HOST", ""):
                is_rootless_docker = True
            else:
                res_u = subprocess.run(["systemctl", "--user", "is-active", "docker"], capture_output=True, text=True, timeout=2)
                if res_u.stdout.strip() == "active":
                    is_rootless_docker = True
                elif info_res.returncode == 0:
                    # User has direct access to docker (e.g. via docker group or socket)
                    is_rootless_docker = True
        except Exception:
            pass

    return {
        "status": "ok",
        "has_docker": has_docker,
        "is_rootless_docker": is_rootless_docker,
        "has_podman": has_podman,
        "platform_system": platform.system(),
    }


def get_rootless_info() -> Dict[str, Any]:
    c_status = check_container_status()
    uid = os.getuid() if hasattr(os, "getuid") else 1000
    user = os.environ.get("USER", "ubuntu")
    xdg_runtime = os.environ.get("XDG_RUNTIME_DIR", f"/run/user/{uid}")

    return {
        "status": "ok",
        "is_rootless": c_status["is_rootless_docker"],
        "has_docker": c_status["has_docker"],
        "has_podman": c_status["has_podman"],
        "platform_system": c_status["platform_system"],
        "user": user,
        "uid": uid,
        "xdg_runtime_dir": xdg_runtime,
        "socket_path": f"{xdg_runtime}/docker.sock",
    }


def setup_rootless_docker() -> Dict[str, Any]:
    logs = []
    user = os.environ.get("USER", "ubuntu")
    uid = os.getuid() if hasattr(os, "getuid") else 1000
    xdg_runtime = os.environ.get("XDG_RUNTIME_DIR", f"/run/user/{uid}")

    if platform.system() != "Linux":
        return {
            "status": "ok",
            "success": True,
            "message": "Rootless setup is only needed on Linux.",
            "is_rootless": True,
            "logs": "Non-Linux platform."
        }

    status = check_container_status()
    if status["is_rootless_docker"]:
        return {
            "status": "ok",
            "success": True,
            "message": f"Rootless Docker is already active for user '{user}' (UID {uid}).",
            "is_rootless": True,
            "logs": "Rootless daemon is already active."
        }

    setuptool = shutil.which("dockerd-rootless-setuptool.sh")
    if not setuptool:
        for p in ["/usr/bin/dockerd-rootless-setuptool.sh", os.path.expanduser("~/.docker/bin/dockerd-rootless-setuptool.sh")]:
            if os.path.exists(p):
                setuptool = p
                break

    if not setuptool:
        logs.append("Installing rootless Docker prerequisites...")
        if shutil.which("apt-get"):
            env = os.environ.copy()
            env["DEBIAN_FRONTEND"] = "noninteractive"
            subprocess.run(["sudo", "-n", "apt-get", "-o", "APT::Update::Pre-Invoke::=", "update", "-qq"], capture_output=True, text=True, env=env)
            # The prerequisites first: these are plain Ubuntu packages and are
            # wanted whichever Docker is installed.
            r = subprocess.run(["sudo", "-n", "apt-get", "install", "-y", "--no-install-recommends", "-o", "DPkg::Lock::Timeout=60", "uidmap", "dbus-user-session", "slirp4netns"], capture_output=True, text=True, env=env)
            logs.append(r.stdout or r.stderr)
            # Then the setuptool, which comes ONLY from Docker Engine's own
            # repository. There is deliberately no `docker.io` fallback here:
            # Ubuntu's package ships no rootless files at all (verified on
            # 29.1.3-0ubuntu3~24.04.2 -- `dpkg -L docker.io` has nothing
            # matching "rootless"), so installing it cannot produce the
            # setuptool and the attempt would fail a step later anyway. It is
            # also not harmless: docker.io and docker-ce conflict, so a
            # fallback could uninstall a working Docker Engine while trying to
            # add rootless support to it.
            r = subprocess.run(["sudo", "-n", "apt-get", "install", "-y", "--no-install-recommends", "-o", "DPkg::Lock::Timeout=60", "docker-ce-rootless-extras"], capture_output=True, text=True, env=env)
            logs.append(r.stdout or r.stderr)
        setuptool = shutil.which("dockerd-rootless-setuptool.sh")

    if not setuptool:
        return {
            "status": "error",
            "success": False,
            "message": (
                "dockerd-rootless-setuptool.sh not found, so rootless Docker cannot be set "
                "up on this machine. It ships with Docker Engine (docker-ce-rootless-extras) "
                "from docs.docker.com, not with Ubuntu's docker.io package, which contains no "
                "rootless support at all. Install Docker Engine from docker.com, or use "
                "Podman, which is rootless by default. Rootful Docker keeps working either "
                "way -- the container then starts as root and drops to the owner of the "
                "checkout, so your files stay yours either way."
            ),
            "is_rootless": False,
            "logs": "\n".join(logs)
        }

    try:
        r_inst = subprocess.run([setuptool, "install", "-f"], capture_output=True, text=True, timeout=30)
        logs.append(r_inst.stdout)
    except Exception as e:
        logs.append(f"Execution error: {e}")

    subprocess.run(["systemctl", "--user", "daemon-reload"], capture_output=True, text=True)
    subprocess.run(["systemctl", "--user", "enable", "--now", "docker.service"], capture_output=True, text=True)
    subprocess.run(["loginctl", "enable-linger", user], capture_output=True, text=True)

    new_status = check_container_status()
    success = new_status["is_rootless_docker"]
    return {
        "status": "ok" if success else "warning",
        "success": success,
        "is_rootless": success,
        "message": f"Rootless Docker {'configured and active' if success else 'setup completed with warnings'}.",
        "logs": "\n".join(logs)
    }


def install_container_engine(engine: str = "docker") -> Dict[str, Any]:
    engine = engine.lower().strip()
    is_podman = engine in ("podman", "podman_systemd")
    pkg_name = "podman"
    bin_name = "podman" if is_podman else "docker"

    if shutil.which(bin_name):
        return {
            "status": "ok",
            "installed": True,
            "engine": engine,
            "message": f"{bin_name.capitalize()} is already installed.",
        }

    can_sudo = False
    sudo_prefix = []
    if shutil.which("apt-get"):
        if os.geteuid() == 0:
            can_sudo = True
        else:
            try:
                res = subprocess.run(["sudo", "-n", "true"], capture_output=True, timeout=3)
                if res.returncode == 0:
                    can_sudo = True
                    sudo_prefix = ["sudo", "-n"]
            except Exception:
                pass

    if not is_podman:
        # Docker Engine from download.docker.com, set up rootless, through the
        # same script the README runs. This installed Ubuntu's docker.io, which
        # has no rootless support at all, while the page announced "Installing
        # Rootless Docker" -- and on a failure it told the user to install
        # docker.io by hand (every-control browser walk, 2026-09-25).
        script = os.path.join(REPO_ROOT, "scripts", "install_docker.sh")
        command = f"bash {script}"
        if os.geteuid() == 0 or not sudo_prefix:
            why = ("rootless Docker belongs to the robot's user, not root" if os.geteuid() == 0
                   else "it needs passwordless sudo")
            return {
                "status": "error",
                "installed": False,
                "engine": engine,
                "message": f"Cannot install Docker from here: {why}. Run `{command}` in a terminal as the robot's user.",
                "command": command,
            }
        r = subprocess.run(["bash", script], capture_output=True, text=True, timeout=900)
        installed = shutil.which(bin_name) is not None and r.returncode == 0
        return {
            "status": "ok" if installed else "error",
            "installed": installed,
            "engine": engine,
            "message": ("Docker (rootless) installed and configured." if installed
                         else f"Docker install failed; see the log, or run `{command}` in a terminal."),
            "logs": (r.stdout + r.stderr)[-4000:],
        }

    if not can_sudo:
        return {
            "status": "error",
            "installed": False,
            "engine": engine,
            "message": f"Cannot install {pkg_name}: passwordless sudo or root is required.",
            "command": f"sudo apt update && sudo apt install -y {pkg_name}",
        }

    proxy_args = []
    proxy = os.environ.get("APT_PROXY", "").strip()
    if proxy:
        proxy_args = ["-o", f"Acquire::http::Proxy={proxy}"]

    env = os.environ.copy()
    env["DEBIAN_FRONTEND"] = "noninteractive"
    update_cmd = (
        sudo_prefix
        + ["apt-get", "-o", "APT::Update::Pre-Invoke::="]
        + proxy_args
        + ["update", "-qq"]
    )
    subprocess.run(update_cmd, check=False, env=env)

    install_cmd = (
        sudo_prefix
        + ["apt-get", "install", "-y", "-o", "DPkg::Lock::Timeout=60", "-o", "APT::Get::Assume-Yes=true"]
        + proxy_args
        + [pkg_name]
    )
    r = subprocess.run(install_cmd, capture_output=True, text=True, env=env, timeout=300)
    installed = shutil.which(bin_name) is not None

    return {
        "status": "ok" if installed else "error",
        "installed": installed,
        "engine": engine,
        "message": (
            f"{bin_name.capitalize()} installed successfully!"
            if installed
            else f"Failed to install {pkg_name}."
        ),
        "logs": r.stdout or r.stderr,
    }


def get_autostart_status() -> Dict[str, Any]:
    service_file = os.path.expanduser("~/.config/systemd/user/linorobot2-cockpit.service")
    has_service = os.path.exists(service_file)
    enabled = False
    active = False
    user = os.environ.get("USER", "ubuntu")
    lingering = False

    try:
        r_l = subprocess.run(["loginctl", "show-user", user, "--property=Linger"], capture_output=True, text=True, timeout=2)
        lingering = "yes" in r_l.stdout.lower()
    except Exception:
        pass

    if has_service:
        try:
            r_e = subprocess.run(["systemctl", "--user", "is-enabled", "linorobot2-cockpit.service"], capture_output=True, text=True, timeout=2)
            enabled = r_e.stdout.strip() == "enabled"
        except Exception:
            pass
        try:
            r_a = subprocess.run(["systemctl", "--user", "is-active", "linorobot2-cockpit.service"], capture_output=True, text=True, timeout=2)
            active = r_a.stdout.strip() == "active"
        except Exception:
            pass

    return {
        "status": "ok",
        "has_service": has_service,
        "enabled": enabled,
        "active": active,
        "lingering": lingering,
        "service_name": "linorobot2-cockpit.service",
    }


def enable_autostart(data: Dict[str, Any]) -> Dict[str, Any]:
    user = os.environ.get("USER", "ubuntu")
    systemd_dir = os.path.expanduser("~/.config/systemd/user")
    os.makedirs(systemd_dir, exist_ok=True)
    service_path = os.path.join(systemd_dir, "linorobot2-cockpit.service")

    service_content = f"""[Unit]
Description=Linorobot2 Cockpit Autostart Daemon
After=network.target

[Service]
Type=simple
WorkingDirectory={REPO_ROOT}
ExecStart=/usr/bin/python3 {REPO_ROOT}/web/backend/main.py
Restart=on-failure
RestartSec=5

[Install]
WantedBy=default.target
"""
    with open(service_path, "w") as f:
        f.write(service_content)

    subprocess.run(["systemctl", "--user", "daemon-reload"], capture_output=True)
    subprocess.run(["systemctl", "--user", "enable", "--now", "linorobot2-cockpit.service"], capture_output=True)
    subprocess.run(["loginctl", "enable-linger", user], capture_output=True)

    return {"status": "ok", "enabled": True, "service_path": service_path}


def disable_autostart() -> Dict[str, Any]:
    subprocess.run(["systemctl", "--user", "disable", "--now", "linorobot2-cockpit.service"], capture_output=True)
    service_file = os.path.expanduser("~/.config/systemd/user/linorobot2-cockpit.service")
    if os.path.exists(service_file):
        try:
            os.remove(service_file)
        except Exception:
            pass
    subprocess.run(["systemctl", "--user", "daemon-reload"], capture_output=True)
    return {"status": "ok", "enabled": False}


def get_autostart_logs() -> Dict[str, Any]:
    try:
        r = subprocess.run(["journalctl", "--user-unit=linorobot2-cockpit.service", "-n", "100", "--no-pager"], capture_output=True, text=True, timeout=5)
        return {"status": "ok", "logs": r.stdout}
    except Exception as e:
        return {"status": "error", "logs": str(e)}


def sensor_registry() -> Dict[str, Any]:
    def entries(table):
        out = {}
        for key, e in table.items():
            out[key] = {
                "label": e["label"],
                "serial": e.get("serial", False),
                "symlink": e.get("symlink"),
                "default_baud": e.get("default_baud"),
                "docker_key": e.get("docker_key"),
                "driver_pkg": e.get("driver_pkg", ""),
                "models": e.get("models", []),
                "has_install": bool(e.get("install")),
                "has_udev": bool(e.get("udev")),
            }
        return out

    return {
        "laser": entries(LASER_SENSORS),
        "depth": entries(DEPTH_SENSORS),
        "lidar": SENSOR_REGISTRY["lidar"],
        "depth_camera": SENSOR_REGISTRY["depth_camera"],
        "imu": SENSOR_REGISTRY["imu"],
    }


def get_sensor_driver_status(sensor: str, ws: str = "") -> Dict[str, Any]:
    pkg = ""
    # Check LASER_SENSORS
    for k, v in LASER_SENSORS.items():
        if k == sensor or any(m.get("code") == sensor for m in v.get("models", [])):
            pkg = v.get("driver_pkg", "")
            break
    # Check DEPTH_SENSORS
    if not pkg:
        for k, v in DEPTH_SENSORS.items():
            if k == sensor or any(m.get("code") == sensor for m in v.get("models", [])):
                pkg = v.get("driver_pkg", "")
                break
    # Check SENSOR_REGISTRY
    if not pkg:
        for cat in SENSOR_REGISTRY.values():
            for s in cat:
                if s.get("id") == sensor:
                    pkg = s.get("pkg", "")
                    break
    installed = ros_pkg_installed(pkg) if pkg else False
    return {"sensor": sensor, "pkg": pkg, "installed": installed}


def build_sensor_install_cmd(sensor: str, distro: str = "jazzy", ws: str = "") -> Dict[str, Any]:
    pkg = ""
    for k, v in {**LASER_SENSORS, **DEPTH_SENSORS}.items():
        if k == sensor or any(m.get("code") == sensor for m in v.get("models", [])):
            pkg = v.get("driver_pkg", "")
            break
    if not pkg:
        for cat in SENSOR_REGISTRY.values():
            for s in cat:
                if s.get("id") == sensor:
                    pkg = s.get("pkg", "")
                    break
    clean_pkg = pkg.replace("_", "-") if pkg else sensor.replace("_", "-")
    cmd = f"sudo apt-get update && sudo apt-get install -y ros-{distro}-{clean_pkg}"
    return {"sensor": sensor, "command": cmd, "pkg": pkg}


def build_ros2_install_cmd(distro: str = "jazzy") -> str:
    return f"sudo apt-get update && sudo apt-get install -y ros-{distro}-ros-base python3-colcon-common-extensions"


def build_base_install_cmd(ws: str, distro: str = "jazzy") -> str:
    return f"cd {ws} && colcon build --symlink-install --packages-select linorobot2_cockpit"


def ros_pkg_installed(pkg: str, distro: str = "auto") -> bool:
    """Whether ROS can find `pkg`, asked from a shell that has sourced ROS.

    The supervisor itself runs unsourced -- `ros2` is not on its PATH -- so a
    bare `ros2 pkg prefix` raised FileNotFoundError and every package read as
    missing: Start SLAM and Start Navigation refused on the robot image, whose
    /opt/ros/<distro> has both (every-control browser walk, 2026-09-25).
    """
    try:
        res = subprocess.run(["bash", "-c", f"{ros_setup_shell(distro)}; ros2 pkg prefix {shlex.quote(pkg)}"],
                             capture_output=True, text=True, timeout=15)
        return res.returncode == 0
    except Exception:
        return False


def get_package_install_info(pkg: str, distro: str = "jazzy", ws: str = "") -> Dict[str, Any]:
    return {"pkg": pkg, "installed": ros_pkg_installed(pkg, distro), "distro": distro}


def nav2_stack_status(distro: Optional[str] = "jazzy", ws: Optional[str] = None) -> Dict[str, Any]:
    dist = distro or "jazzy"
    nav2_installed = ros_pkg_installed("nav2_bringup", dist)
    slam_installed = ros_pkg_installed("slam_toolbox", dist)
    return {
        "status": "ok",
        "distro": dist,
        "nav2_installed": nav2_installed,
        "slam_installed": slam_installed,
    }


def list_dir(dir_path: str, only: str = "any", exts: str = "") -> Dict[str, Any]:
    p = os.path.abspath(os.path.expanduser(dir_path or REPO_ROOT))
    if not os.path.isdir(p):
        return {"error": f"Not a directory: {p}", "entries": []}
    ext_list = [e.strip().lower() for e in exts.split(",") if e.strip()]
    items = []
    try:
        for entry in sorted(os.listdir(p)):
            ep = os.path.join(p, entry)
            is_dir = os.path.isdir(ep)
            if only == "dir" and not is_dir:
                continue
            if only == "file" and is_dir:
                continue
            if ext_list and not is_dir and not any(entry.lower().endswith(e) for e in ext_list):
                continue
            items.append({
                "name": entry,
                "path": ep,
                "is_dir": is_dir,
                "size": os.path.getsize(ep) if not is_dir else 0,
            })
    except Exception as e:
        return {"error": str(e), "entries": []}
    return {"path": p, "entries": items}


def analyze_robotics_ai(prompt: str, base: str = "2wd", distro: str = "jazzy", model: Any = None) -> Dict[str, Any]:
    p = prompt.lower()
    diagnosis = []
    recommendations = []
    nav2_patch = {}
    ekf_patch = {}
    slam_patch = {}
    target_base = base

    if any(k in p for k in ["mecanum", "omni", "holonomic"]):
        target_base = "mecanum"
        diagnosis.append("Holonomic kinematics requested. Enabling lateral velocity in velocity_smoother, AMCL OmniMotionModel, and EKF odom0 vy fusion.")
        recommendations.append("Enable lateral velocity (v_y = 0.5 m/s) in velocity_smoother.")
        recommendations.append("Set AMCL robot_model_type to nav2_amcl::OmniMotionModel.")
        recommendations.append("Set EKF odom0_config to fuse lateral velocity.")
        nav2_patch.update({"base": "mecanum", "max_vel_x": 0.5, "max_vel_y": 0.5, "max_accel_x": 2.5, "max_accel_y": 2.5})
        ekf_patch["fuse_vy"] = True
    elif any(k in p for k in ["diff", "2wd", "4wd", "skid"]):
        target_base = "2wd"
        nav2_patch.update({"base": "2wd", "max_vel_y": 0.0, "max_accel_y": 0.0})
        ekf_patch["fuse_vy"] = False

    if any(k in p for k in ["door", "narrow", "tight", "hesitat"]):
        diagnosis.append("Doorway hesitation caused by default inflation radius overlapping across narrow passages.")
        recommendations.append("Reduce costmap inflation_radius to 0.52m and steepen cost_scaling_factor to 5.5.")
        nav2_patch.update({"inflation_radius": 0.52, "cost_scaling_factor": 5.5, "desired_linear_vel": 0.35})

    if any(k in p for k in ["oscillat", "wobble", "spin at goal", "overshoot"]):
        diagnosis.append("End-goal oscillation caused by loose rotation acceleration authority.")
        recommendations.append("Smooth max_angular_accel to 2.2 rad/s² and max_vel_theta to 2.0 rad/s.")
        nav2_patch.update({"max_vel_theta": 2.0, "max_accel_theta": 2.2})

    if any(k in p for k in ["drift", "slip", "spin drift"]):
        diagnosis.append("State estimation drift detected during in-place turns.")
        recommendations.append("Standardize EKF frequency to 50 Hz and lock two_d_mode = true.")
        if target_base != "mecanum":
            ekf_patch["fuse_vy"] = False
        ekf_patch["two_d_mode"] = True
        ekf_patch["frequency"] = 50.0

    if not diagnosis:
        diagnosis.append("Custom robotic parameter optimization for smooth mobile navigation.")
        recommendations.append("Applied balanced velocity limits (0.5 m/s) and 50 Hz EKF state estimation.")

    return {
        "target_base": target_base,
        "diagnosis": " ".join(diagnosis),
        "recommendations": recommendations,
        "nav2_patch": nav2_patch,
        "ekf_patch": ekf_patch,
        "slam_patch": slam_patch,
    }


def generate_custom_robot_specs(description: str) -> Dict[str, Any]:
    d = description.lower()
    base = "mecanum" if any(k in d for k in ["mecanum", "omni"]) else ("4wd" if "4wd" in d else "2wd")
    wheel_diam = 0.097 if base == "mecanum" else (0.130 if base == "4wd" else 0.066)
    track_width = 0.30 if base == "mecanum" else (0.35 if base == "4wd" else 0.16)
    wheelbase = 0.25 if base == "mecanum" else (0.28 if base == "4wd" else 0.0)

    # The config's own key names, so /api/ai/deploy_robot can write them
    # straight into `kinematics` and the browser can show them without a
    # second vocabulary (it once read wheel_diameter_m / track_width_m, which
    # this never produced, and showed "-" for every number).
    return {
        "design": {
            "base_type": base,
            "title": f"{base.upper()} chassis",
            "wheel_diameter": wheel_diam,
            "lr_wheels_distance": track_width,
            "fr_wheels_distance": wheelbase,
            "laser_sensor": "ld19",
        },
        "tuning": {
            "nav2": {"base": base, "max_vel_x": 0.5, "max_vel_theta": 2.5},
            "ekf": {"base": base, "frequency": 50.0, "fuse_vy": (base == "mecanum")},
            "slam": {"resolution": 0.05, "max_laser_range": 10.0},
        }
    }
