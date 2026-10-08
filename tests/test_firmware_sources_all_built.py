"""Every firmware/src/*.cpp is compiled by the builds that list their sources.

PlatformIO compiles the whole src/ directory, so a new source there links on every board.
The host firmware (the Sim MCU, firmware/host/app) and the UNO Q's Zephyr build name
their sources in CMake instead, and both named main.cpp alone: base_ident.cpp linked on
the boards and broke both of them on `baseIdentTick` (rc-20261009.1's release run). They
glob src/*.cpp now; this keeps it so.
"""
import os
import re

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LISTS = ("firmware/host/app/CMakeLists.txt", "firmware/zephyr/unoq/fw/CMakeLists.txt")


def test_cmake_builds_glob_the_whole_src_directory():
    for rel in LISTS:
        text = open(os.path.join(REPO, rel), encoding="utf-8").read()
        assert re.search(r"file\(GLOB APP_SOURCES \$\{FIRMWARE_ROOT\}/src/\*\.cpp\)", text), rel
        assert "${APP_SOURCES}" in text, rel
        assert "${FIRMWARE_ROOT}/src/main.cpp " not in text and "${FIRMWARE_ROOT}/src/main.cpp\n" not in text, rel
