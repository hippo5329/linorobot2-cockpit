#!/usr/bin/env python3
"""The colcon.meta for the host micro-ROS client: the BOARD's meta, plus what a host needs.

The host target is only worth running if its micro-ROS client is configured like
a board's. The first whole-firmware run on the host proved why: the workspace
carried the client's default custom-transport MTU (512), a best-effort XRCE stream
cannot fragment, and nav_msgs/Odometry is ~720 bytes -- so /odom/unfiltered failed
on every cycle, exactly as it once did on the ESP32 before firmware/esp32.meta
raised the MTU to 1024. The board's settings live in that file; this reads them
from there instead of keeping a second copy that would drift.

Merged on top of the host platform's own meta (micro_ros_setup's `host`, which
builds shared libraries): the board's entity pools, history and MTU, the custom
transport, and BUILD_TESTING=OFF on the rmw -- its upstream test calls agent
autodiscovery, which does not exist under the custom transport and fails to
compile (see README.md).

    client_meta.py <host colcon.meta> <firmware/esp32.meta>  > merged colcon.meta
"""
import json
import sys


def read_meta(path):
    """A .meta file is JSON with leading '#' comment lines (firmware/*.meta)."""
    with open(path) as fh:
        text = "".join(line for line in fh if not line.lstrip().startswith("#"))
    return json.loads(text)


def _merge_args(base, extra):
    """Later -DNAME=... wins over an earlier one of the same NAME."""
    def key(a):
        return a.split("=", 1)[0]
    out = [a for a in base if key(a) not in {key(e) for e in extra}]
    return out + list(extra)


def merged(host_meta: dict, board_meta: dict) -> dict:
    names = json.loads(json.dumps(host_meta.get("names", {})))
    for pkg, cfg in board_meta.get("names", {}).items():
        cur = names.setdefault(pkg, {})
        cur["cmake-args"] = _merge_args(cur.get("cmake-args", []), cfg.get("cmake-args", []))
    rmw = names.setdefault("rmw_microxrcedds", {})
    # No compiled-in UDP address: under the custom transport every byte goes
    # through uros_transport.cpp, and the address comes from the env image.
    rmw["cmake-args"] = [a for a in _merge_args(rmw.get("cmake-args", []),
                                                ["-DRMW_UXRCE_TRANSPORT=custom",
                                                 "-DBUILD_TESTING=OFF"])
                         if not a.startswith("-DRMW_UXRCE_DEFAULT_UDP_")
                         # the host platform's 32; a board takes the rmw default
                         and not a.startswith("-DRMW_UXRCE_STREAM_HISTORY=")]
    return {"names": names}


def main(argv):
    if len(argv) != 3:
        print(__doc__, file=sys.stderr)
        return 2
    json.dump(merged(read_meta(argv[1]), read_meta(argv[2])), sys.stdout, indent=4)
    print()
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
