#!/usr/bin/env python3
"""
check_ros_abi.py — can the ROS packages in this image actually be loaded together?

"The package is installed" and "the package works" are different questions, and
on a rolling-snapshot distro they come apart. Every rosidl typesupport library
is dlopened lazily -- the first time something publishes that type -- so a base
image that lags the archive by a few weeks produces an image where every
structural check passes and nodes die the moment they start.

rc-20260919's lyrical image shipped std_msgs from 2026-08-11, whose typesupport
library defines no `has_buffer_fields_*` symbols, under geographic_msgs,
rosbridge_msgs and robot_localization from 2026-09-15, which need them. It cost
ekf_node (so no /odom), rosbridge_websocket and rosapi (so nothing served the
browser canvas on :9090) -- discovered on a bench, not in the build.

Creating one publisher per message package is what forces the dlopen. Run it in
the image, at build time and against the published copy.

A symbol lookup error kills the interpreter outright rather than raising, so the
exit status is the contract here, not the report: the loader prints
`symbol lookup error: ...` and the process dies with 127. The try/except below
catches the milder failures (a missing package, a bad import) and names them.
"""
import sys

# One representative type per message package that bringup depends on: the
# three that broke, plus what the stack itself publishes.
TYPES = [
    ("geographic_msgs.msg", "GeoPose"),      # pulled in by robot_localization
    ("rosbridge_msgs.msg", "ConnectedClients"),
    ("rosapi_msgs.srv", "TopicsForType"),
    ("sensor_msgs.msg", "LaserScan"),
    ("sensor_msgs.msg", "Imu"),
    ("nav_msgs.msg", "Odometry"),
    ("geometry_msgs.msg", "Twist"),
    ("tf2_msgs.msg", "TFMessage"),
]


def main() -> int:
    import importlib

    import rclpy

    rclpy.init()
    node = rclpy.create_node("ros_abi_check")
    failed = []
    for i, (module, name) in enumerate(TYPES):
        try:
            mod = importlib.import_module(module)
            typ = getattr(mod, name)
            # A service type has no publisher; importing it is the whole check.
            if module.endswith(".srv"):
                continue
            node.create_publisher(typ, f"/ros_abi_check_{i}", 1)
        except Exception as exc:                      # noqa: BLE001 - report anything
            failed.append(f"{module}.{name}: {exc}")
    for line in failed:
        print(f"[abi] FAILED {line}", file=sys.stderr)
    if failed:
        print(f"[abi] {len(failed)} of {len(TYPES)} message packages do not load",
              file=sys.stderr)
        return 1
    print(f"[abi] all {len(TYPES)} message packages load and publish")
    return 0


if __name__ == "__main__":
    sys.exit(main())
