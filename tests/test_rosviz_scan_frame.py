"""The map viewer draws a scan from where its LiDAR is mounted.

The tracked robot's LD19 sits at -5 deg (geometry.laser.yaw -0.0873) and 2 cm ahead of
base_link. The viewer drew every scan from the robot's centre with the robot's
heading, so the live scan stood turned against the walls the map had just built
(2026-10-16, after a Map Viewer goal) -- and an operator reads that as SLAM
having lost the robot.
"""
import json
import math
import pathlib
import re
import shutil
import subprocess

import pytest

FRONTEND = pathlib.Path(__file__).resolve().parents[1] / "web" / "frontend"


def _rosviz():
    return (FRONTEND / "rosviz.js").read_text()


def _function(js, name):
    start = js.index(f"function {name}(")
    end = js.index("\n  }\n", start) + len("\n  }\n")
    return js[start:end]


def _run_node(expr):
    node = shutil.which("node")
    if not node:
        pytest.skip("node is not installed")
    js = _rosviz()
    src = "\n".join(_function(js, n) for n in ("compose2d", "frameInBase"))
    out = subprocess.run([node, "-e", src + f"\nconsole.log(JSON.stringify({expr}));"],
                         capture_output=True, text=True, check=True)
    return json.loads(out.stdout)


TRACKED_STATICS = ("new Map([['laser', {parent: 'base_link', x: 0.02, y: 0, yaw: -0.0873}],"
                 " ['base_link', {parent: 'base_footprint', x: 0, y: 0, yaw: 0}]])")


def test_the_scan_frame_is_found_through_the_chain():
    off = _run_node(f"frameInBase({TRACKED_STATICS}, 'laser', 'base_footprint')")
    assert off["x"] == pytest.approx(0.02)
    assert off["yaw"] == pytest.approx(-0.0873)
    off = _run_node(f"frameInBase({TRACKED_STATICS}, 'laser', 'base_link')")
    assert off["yaw"] == pytest.approx(-0.0873)


def test_an_unknown_chain_is_null_not_a_guess():
    assert _run_node(f"frameInBase({TRACKED_STATICS}, 'laser', 'chassis')") is None
    assert _run_node(f"frameInBase({TRACKED_STATICS}, 'camera', 'base_link')") is None


def test_the_laser_pose_in_map_carries_the_mount():
    # robot at (1, 2) facing +y: the LiDAR is 2 cm further along +y, turned -5 deg
    p = _run_node("compose2d({x: 1, y: 2, yaw: Math.PI / 2}, {x: 0.02, y: 0, yaw: -0.0873})")
    assert p["x"] == pytest.approx(1.0, abs=1e-9)
    assert p["y"] == pytest.approx(2.02)
    assert p["yaw"] == pytest.approx(math.pi / 2 - 0.0873)


def test_the_scan_is_drawn_from_the_sensor_pose():
    js = _rosviz()
    body = js[js.index("const ranges = scan.ranges"):]
    body = body[: body.index("ctx.fillRect")]
    assert "frameInBase(" in body and "scan.header" in body


def test_the_mounts_are_subscribed_unthrottled():
    """/tf_static is latched and sent once: a throttle would drop the very message."""
    js = _rosviz()
    m = re.search(r'bridge\.subscribe\("/tf_static",.*?\n      \}\)(, *\d+)?;', js, re.S)
    assert m, "the viewer does not subscribe to /tf_static"
    assert not m.group(1), "/tf_static must not be throttled"


def test_the_pose_knows_which_frame_it_is():
    js = _rosviz()
    line = [l for l in js.splitlines() if "counts.odom++" in l][0]
    assert "child_frame_id" in line
