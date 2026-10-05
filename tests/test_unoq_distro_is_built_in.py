"""The UNO Q's Zephyr image names the micro-ROS distro it was built with, never a fixed one.

FW_ROS_DISTRO picks the firmware's default cmd_vel type (main.cpp: Twist through jazzy,
TwistStamped after). The Zephyr CMakeLists had it as a literal "jazzy", so the first
lyrical image listened for Twist while lyrical's Nav2 sent TwistStamped: every topic
at rate, and the robot never moved (2026-10-05).
"""
import os
import re

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
UNOQ = os.path.join(REPO_ROOT, "firmware", "zephyr", "unoq")


def _read(*p):
    return open(os.path.join(UNOQ, *p)).read()


def test_cmake_takes_the_distro_and_refuses_without_one():
    c = _read("fw", "CMakeLists.txt")
    assert 'FW_ROS_DISTRO="${FW_ROS_DISTRO}"' in c
    assert not re.search(r'FW_ROS_DISTRO="(humble|iron|jazzy|kilted|lyrical|rolling)"', c)
    assert re.search(r"if\(NOT DEFINED FW_ROS_DISTRO\)\s*\n\s*message\(FATAL_ERROR", c)


def test_build_sh_passes_one_distro_to_cmake_and_the_header():
    b = _read("build.sh")
    assert "-DFW_ROS_DISTRO=$DISTRO" in b
    assert '--distro "$DISTRO"' in b
    assert re.search(r"^DISTRO=\$\{MICROROS_DISTRO:-jazzy\}", b, re.M)
