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


def pack_pointcloud2_xyz(points):
    """(x, y, z) tuples -> (fields, point_step, data) for a dense float32 x/y/z PointCloud2.

    ``fields`` is a list of (name, offset, datatype FLOAT32=7, count) for sensor_msgs/PointField.
    """
    fields = [("x", 0, 7, 1), ("y", 4, 7, 1), ("z", 8, 7, 1)]
    data = b"".join(struct.pack("<fff", *p) for p in points)
    return fields, 12, data
