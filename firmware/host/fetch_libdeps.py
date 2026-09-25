#!/usr/bin/env python3
"""Fetch the third-party Arduino libraries the host target compiles, from the boards' own list.

A board takes them from platformio's `lib_deps` (common/platformio_base.ini). The
host target compiles the same sources, so it takes the same list from the same
file -- a second list would drift from the boards' on the first edit. Three
entries are not for the host: the micro-ROS library (the host links its own
colcon client), and SPI and Wire (the core's, which the shim provides).

    fetch_libdeps.py <dest dir>       (needs `pio` on PATH; build stage only)

The result has the layout of a platformio .pio/libdeps/<env> directory, which is
what firmware/host/app/CMakeLists.txt takes as LINO_LIBDEPS.
"""
import configparser
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
BASE_INI = os.path.join(HERE, "..", "common", "platformio_base.ini")
NOT_FOR_HOST = ("micro_ros_platformio", "SPI", "Wire")


def lib_deps(ini_path: str = BASE_INI) -> list:
    cfg = configparser.ConfigParser(interpolation=None, strict=False)
    cfg.read(ini_path)
    raw = cfg.get("env", "lib_deps", fallback="")
    deps = [line.strip() for line in raw.splitlines() if line.strip() and not line.strip().startswith(";")]
    return [d for d in deps if not any(d.rstrip("/").endswith(n) or d == n for n in NOT_FOR_HOST)]


def main(argv):
    if len(argv) != 2:
        print(__doc__, file=sys.stderr)
        return 2
    dest = os.path.abspath(argv[1])
    os.makedirs(dest, exist_ok=True)
    for dep in lib_deps():
        print(f"[libdeps] {dep}", flush=True)
        subprocess.run(["pio", "pkg", "install", "--storage-dir", dest, "--library", dep], check=True)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
