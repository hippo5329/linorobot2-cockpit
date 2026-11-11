#!/usr/bin/env python3
"""The 8x8 multizone time-of-flight sensor (DFRobot SEN0628, ST VL53L7CX): frames to points.

Pure functions, no ROS: tof_matrix_node.py wraps them, tests/test_tof_matrix.py checks them.

A frame is 64 distances in millimetres, row-major (``frame[row * 8 + col]``), from either
connection:

- USB: the board streams text at power-on, one frame = eight lines ``y0:d,d,...,d,`` to
  ``y7:``, 15 Hz (``FrameParser``).
- I2C on the MCU: the firmware asks the board for all 64 zones (the board's ``CMD_ALLData``)
  and publishes them as 128 bytes, little-endian uint16 (``decode_raw``).

4000 mm is the board's "no valid reading" (nothing in range, or a result the sensor
rejected); 0 never comes from a real target. Both are dropped.

The VL53L5CX/L7CX report the PERPENDICULAR distance to the sensor's face, not the distance
along the zone's ray: a flat wall square-on reads the same in every zone. So a zone at
(row, col) is the point ``(d, d * tan(az), d * tan(el))`` in the sensor frame, x out of the
lens, y left, z up (REP-103), with the zone centres spread evenly over the 60 x 60 degree field.

Which corner the board's row 0 / column 0 is depends on how it is mounted: ``rotation`` (0,
90, 180, 270 degrees about the viewing axis) and ``mirror`` turn the grid so that, after
them, row 0 is the TOP of the view and column 0 its LEFT, as seen from behind the sensor.
"""
import math
import struct

ZONES = 8
FOV_DEG = 60.0             # horizontal and vertical field of view (DFRobot: 60 x 60, 90 diagonal)
INVALID_MM = 4000          # the board's "no valid reading"
MAX_RANGE_M = 3.5          # DFRobot rates the range to 3.5 m (4 m on a white target)
MIN_RANGE_M = 0.02
FRAME_BYTES = ZONES * ZONES * 2


def zone_angle_deg(index, zones=ZONES, fov_deg=FOV_DEG):
    """Centre angle of zone ``index`` across the field: -26.25 .. +26.25 degrees for 8 x 60."""
    return (index - (zones - 1) / 2.0) * (fov_deg / zones)


def _grid(frame):
    if len(frame) != ZONES * ZONES:
        raise ValueError(f"a frame is {ZONES * ZONES} distances, got {len(frame)}")
    return [list(frame[r * ZONES:(r + 1) * ZONES]) for r in range(ZONES)]


def orient(frame, rotation=0, mirror=False):
    """Turn the board's grid so row 0 is the top of the view and column 0 its left.

    ``mirror`` flips left/right first, then the grid is rotated by ``rotation`` degrees
    clockwise as seen from behind the sensor. Returns a new flat row-major list.
    """
    if rotation not in (0, 90, 180, 270):
        raise ValueError(f"rotation must be 0, 90, 180 or 270, got {rotation!r}")
    g = _grid(frame)
    if mirror:
        g = [row[::-1] for row in g]
    for _ in range(rotation // 90):
        g = [list(r) for r in zip(*g[::-1])]   # 90 degrees clockwise
    return [d for row in g for d in row]


def valid_mm(d, min_range_m=MIN_RANGE_M, max_range_m=MAX_RANGE_M):
    return 0 < d < INVALID_MM and min_range_m * 1000.0 <= d <= max_range_m * 1000.0


def to_points(frame, rotation=0, mirror=False, min_range_m=MIN_RANGE_M, max_range_m=MAX_RANGE_M,
              fov_deg=FOV_DEG):
    """64 distances (mm) -> list of (x, y, z) in metres in the sensor frame; invalid zones dropped."""
    g = orient(frame, rotation, mirror)
    tans = [math.tan(math.radians(zone_angle_deg(i, ZONES, fov_deg))) for i in range(ZONES)]
    pts = []
    for row in range(ZONES):
        for col in range(ZONES):
            d = g[row * ZONES + col]
            if not valid_mm(d, min_range_m, max_range_m):
                continue
            x = d / 1000.0
            # column 0 is the left of the view: +y; row 0 is the top: +z
            pts.append((x, -x * tans[col], -x * tans[row]))
    return pts


SCAN_ROWS = (3, 4)          # the two rows either side of the horizon (+-3.75 deg) on a level mount


def scan_angles():
    """(angle_min, angle_increment) of the ToF's LaserScan, radians: right to left, CCW positive."""
    return (math.radians(zone_angle_deg(0)), math.radians(FOV_DEG / ZONES))


def to_scan_ranges(frame, rotation=0, mirror=False, min_range_m=MIN_RANGE_M, max_range_m=MAX_RANGE_M,
                   rows=SCAN_ROWS):
    """64 distances (mm) -> 8 LaserScan ranges (m), right to left; inf where no row sees anything.

    For a robot whose /scan is the ToF (no LiDAR, no depth camera: depth_camera.scan_source).
    Each column's nearest valid zone in the middle rows, turned from the perpendicular
    distance the sensor reports into the distance along that column's beam (d / cos az).
    """
    g = orient(frame, rotation, mirror)
    out = []
    for col in reversed(range(ZONES)):            # angle_min is the RIGHT edge: column 7
        az = math.radians(zone_angle_deg(col))
        best = math.inf
        for row in rows:
            d = g[row * ZONES + col]
            if valid_mm(d, min_range_m, max_range_m):
                best = min(best, d / 1000.0 / math.cos(az))
        out.append(best)
    return out


def decode_raw(data):
    """The MCU's 128-byte frame (64 x little-endian uint16) -> 64 distances in mm."""
    data = bytes(data)
    if len(data) != FRAME_BYTES:
        raise ValueError(f"a raw frame is {FRAME_BYTES} bytes, got {len(data)}")
    return list(struct.unpack("<64H", data))


def encode_raw(frame):
    """64 distances (mm) -> the 128 bytes the firmware publishes (for tests and the simulator)."""
    return struct.pack("<64H", *[max(0, min(0xFFFF, int(d))) for d in frame])


class FrameParser:
    """Assembles the USB text stream (``y0:...`` .. ``y7:...``) into whole frames.

    ``feed(bytes)`` returns the frames completed by that chunk. A frame is only emitted when
    all eight rows arrived in order with eight values each, so the partial frame the stream
    starts in (or one cut by a dropped line) is skipped, never half-filled.
    """

    def __init__(self):
        self._buf = b""
        self._rows = {}
        self.bad_lines = 0

    def feed(self, chunk):
        self._buf += chunk
        *lines, self._buf = self._buf.split(b"\n")
        if len(self._buf) > 4096:            # no newline in 4 KiB: not this sensor's stream
            self._buf = b""
        frames = []
        for raw in lines:
            line = raw.decode("ascii", errors="replace").strip()
            if not line:
                continue
            name, sep, values = line.partition(":")
            if not sep or len(name) != 2 or name[0] not in "yY" or not name[1].isdigit():
                self.bad_lines += 1
                continue
            row = int(name[1])
            try:
                vals = [int(v) for v in values.split(",") if v.strip()]
            except ValueError:
                self.bad_lines += 1
                self._rows = {}
                continue
            if len(vals) != ZONES or row >= ZONES:
                self.bad_lines += 1
                self._rows = {}
                continue
            if row == 0:
                self._rows = {}
            elif row != len(self._rows) or row - 1 not in self._rows:
                self._rows = {}               # a row out of order: drop the frame in progress
                continue
            self._rows[row] = vals
            if row == ZONES - 1 and len(self._rows) == ZONES:
                frames.append([d for r in range(ZONES) for d in self._rows[r]])
                self._rows = {}
        return frames


# -- the robot config: base_controller.tof -------------------------------------
#
#   tof:
#     model: sen0628          # none (default) | sen0628
#     connection: usb         # usb: the robot computer reads its USB port | i2c: the MCU reads it
#     usb_port: /dev/serial/by-id/usb-Raspberry_Pi_Pico_<serial>-if00
#     i2c_address: 0x33       # 0x30-0x33 (DFRobot's default 0x33)
#     rotation: 0             # 0/90/180/270, how the board is turned on its mount
#     mirror: false
#     floor_clearance: 0.05   # m: points lower than this above the floor are the floor, not obstacles
#     max_height: 0.5         # m: points higher than this cannot touch the robot
#     max_range: 3.5          # m
#
# geometry.tof (x, y, z, roll, pitch, yaw, frame) places tof_link; gen_robot_description.py.

MODELS = ("sen0628",)
CONNECTIONS = ("usb", "i2c")
I2C_ADDRESSES = (0x30, 0x31, 0x32, 0x33)
DEFAULT_I2C_ADDRESS = 0x33
DEFAULT_FRAME = "tof_link"
RAW_TOPIC = "tof/raw"
POINTS_TOPIC = "tof/points"
# The board shows up as a Raspberry Pi Pico (2e8a:000a, product "Pico"): name it by serial.
USB_ID = "2e8a:000a"
NOT_FITTED = {"", "none", "null", "off", "false", "no"}


def _int(v):
    return int(v, 0) if isinstance(v, str) else int(v)


def tof_configured(controller):
    """Does the config name a ToF model at all (without validating the rest)?"""
    t = (controller or {}).get("tof") or {}
    return str(t.get("model") or "none").strip().lower() not in NOT_FITTED


def tof_settings(controller):
    """base_controller -> the ToF's settings, or None when no ToF is fitted.

    Raises ValueError for a config that names a ToF it cannot drive: a silent source
    stops the collision monitor, so a half-configured sensor is refused, not guessed at.
    """
    t = (controller or {}).get("tof") or {}
    model = str(t.get("model") or "none").strip().lower()
    if model in NOT_FITTED:
        return None
    if model not in MODELS:
        raise ValueError(f"base_controller.tof.model {model!r}: one of {', '.join(MODELS)} or none")
    import depth_camera
    if depth_camera.lidar_fitted(controller):
        raise ValueError("base_controller.tof: an 8x8 ToF cannot be used beside a LiDAR (both are "
                         "infrared time-of-flight and blind each other) -- set lidar.model none "
                         "(and use_sim_ld19 false), or tof.model none")
    conn = str(t.get("connection") or "usb").strip().lower()
    if conn not in CONNECTIONS:
        raise ValueError(f"base_controller.tof.connection {conn!r}: usb or i2c")
    port = str(t.get("usb_port") or "").strip()
    if conn == "usb" and not port:
        raise ValueError("base_controller.tof.connection is usb but tof.usb_port is empty "
                         "(its /dev/serial/by-id/... path: the board looks like a Raspberry Pi Pico)")
    try:
        addr = _int(t.get("i2c_address", DEFAULT_I2C_ADDRESS))
    except (TypeError, ValueError):
        raise ValueError(f"base_controller.tof.i2c_address {t.get('i2c_address')!r} is not a number")
    if conn == "i2c" and addr not in I2C_ADDRESSES:
        raise ValueError(f"base_controller.tof.i2c_address 0x{addr:02x}: the board answers on 0x30-0x33")
    rotation = int(t.get("rotation", 0) or 0)
    if rotation not in (0, 90, 180, 270):
        raise ValueError(f"base_controller.tof.rotation {rotation}: 0, 90, 180 or 270")
    return {
        "model": model,
        "connection": conn,
        "usb_port": port,
        "i2c_address": addr,
        "rotation": rotation,
        "mirror": bool(t.get("mirror", False)),
        "floor_clearance": float(t.get("floor_clearance", 0.05)),
        "max_height": float(t.get("max_height", 0.5)),
        "max_range": float(t.get("max_range", MAX_RANGE_M)),
    }


def tof_fitted(controller):
    """True when the config names a ToF (a bad config raises, see tof_settings)."""
    return tof_settings(controller) is not None


def usb_serial(port):
    """The USB serial number in a /dev/serial/by-id path (usb-<maker>_<product>_<serial>-if00)."""
    import os
    name = os.path.basename(str(port or ""))
    m = name.rsplit("-if", 1)[0] if "-if" in name else name
    return m.rsplit("_", 1)[-1] if "_" in m else ""


def claimed_serials(directory=None):
    """USB serials the robot configs name as their 8x8 ToF (tof.usb_port), lower-case.

    Over USB the SEN0628 is a Raspberry Pi Pico to every descriptor (2e8a:000a, "Pico",
    the same Arduino-Pico CDC our own firmware has), so only the config can tell them
    apart: a port named as a ToF is never offered as a board, never flashed.
    """
    import yaml
    import cockpit_paths
    out = set()
    for path in cockpit_paths.robot_config_files(directory):
        try:
            with open(path) as fh:
                ctrl = ((yaml.safe_load(fh) or {}).get("base_controller") or {})
        except Exception:
            continue
        t = ctrl.get("tof") or {}
        if tof_configured(ctrl) and str(t.get("connection") or "usb").lower() == "usb":
            s = usb_serial(t.get("usb_port"))
            if s:
                out.add(s.lower())
    return out


def tty_usb_serial(port):
    """The USB serial number of the device behind a tty (/dev/ttyACM3 or a by-id link), or ""."""
    import os
    real = os.path.realpath(str(port or ""))
    path = os.path.realpath(f"/sys/class/tty/{os.path.basename(real)}/device")
    for _ in range(6):
        f = os.path.join(path, "serial")
        if os.path.isfile(f) and os.path.isfile(os.path.join(path, "idVendor")):
            try:
                with open(f) as fh:
                    return fh.read().strip()
            except OSError:
                return ""
        parent = os.path.dirname(path)
        if parent == path:
            break
        path = parent
    return ""


def is_claimed_tof(port, directory=None):
    """Is the device behind this tty a robot config's 8x8 ToF (see claimed_serials)?"""
    s = tty_usb_serial(port)
    return bool(s) and s.lower() in claimed_serials(directory)


# Simulation: the ToF rendered like the simulated depth camera (depth_camera.depth_row), at
# 8 x 8 over 60 x 60 degrees. The simulated world is the room's walls, so a column sees the
# same wall in every row; the floor and anything under the LiDAR's plane are not in it.
# Noise: sigma ~1 % of the distance at 1 m (z^2 model, like the camera's); DFRobot rates the
# accuracy +-5 % from 0.2 to 4 m, a bound, not the typical.
SIM_NOISE_K = 0.01
SIM_RATE_HZ = 15.0


def sim_optics():
    import depth_camera
    return depth_camera.SimCamera("SEN0628", ZONES, ZONES, FOV_DEG, FOV_DEG, MIN_RANGE_M,
                                  MAX_RANGE_M, SIM_NOISE_K, SIM_RATE_HZ)


def sim_frame(depth_row_m):
    """One rendered row of 8 depths in metres (0 = no return) -> a 64-zone frame in mm.

    The row is the view from left to right; every row of the frame sees it (2D world).
    Column 0 of the board's grid is the LEFT of the view, as after orient(rotation=0).
    """
    row = [int(round(z * 1000.0)) if z > 0 else INVALID_MM for z in depth_row_m]
    if len(row) != ZONES:
        raise ValueError(f"a rendered row is {ZONES} depths, got {len(row)}")
    return row * ZONES


def pack_pointcloud2_xyz(points):
    """(x, y, z) tuples -> (fields, point_step, data) for a dense float32 x/y/z PointCloud2.

    ``fields`` is a list of (name, offset, datatype FLOAT32=7, count) for sensor_msgs/PointField.
    """
    fields = [("x", 0, 7, 1), ("y", 4, 7, 1), ("z", 8, 7, 1)]
    data = b"".join(struct.pack("<fff", *p) for p in points)
    return fields, 12, data
