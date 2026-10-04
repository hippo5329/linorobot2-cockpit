"""Every colcon build in the robot image is a Release build.

colcon passes no build type unless told, and CMake then compiles at -O0. The
/opt/lino_ws build -- explore_lite and the LiDAR drivers -- had no build type in
rc-20261004.1: explore_lite shipped at 12.4 MB with no optimised sections, and
lyrical exploration on a busy bench starved Nav2 until the robot could not get
home. Each `colcon build` in the Dockerfile must carry CMAKE_BUILD_TYPE=Release,
or say why not.
"""
import os
import re

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _colcon_builds():
    src = open(os.path.join(ROOT, "docker", "Dockerfile")).read().replace("\\\n", " ")
    return [m.group(0) for m in re.finditer(r"colcon build[^;&|]*", src)]


def test_the_lino_ws_build_is_release():
    lino = [b for b in _colcon_builds() if "/opt/lino_ws" in b]
    assert lino, "the /opt/lino_ws colcon build is gone from docker/Dockerfile"
    assert all("CMAKE_BUILD_TYPE=Release" in b for b in lino), lino


def test_every_colcon_build_is_release():
    """Not just lino_ws: the micro-ROS agent every board talks through, the Sim MCU
    firmware built for the host, and Nav2's source packages were all -O0 too."""
    builds = [b for b in _colcon_builds() if "FAILED" not in b]
    assert len(builds) >= 4, builds
    missing = [b[:90] for b in builds if "CMAKE_BUILD_TYPE=Release" not in b]
    assert not missing, missing


def test_the_host_micro_ros_client_libraries_are_release():
    src = open(os.path.join(ROOT, "firmware", "host", "build_client_ws.sh")).read()
    assert re.search(r"^ARGS=\(.*-DCMAKE_BUILD_TYPE=Release.*\)$", src, re.M)
