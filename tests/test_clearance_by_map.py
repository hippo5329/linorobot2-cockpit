"""The room check gets out of a corner with the last run's map (clearance.py, map_grid.py).

The MC200's LiDAR is masked 118-263 deg (its own body), and the laser sits 9.5 cm ahead of
the centre, so the ground beside its flanks is never seen. In a corner -- a shelf 0.28 m to
the left, a door edge 0.63 m ahead, open floor to the right -- every way out sweeps some of
it, and the check refused "move the robot by hand" (2026-10-16). The last run's map holds
that ground: used when the live scan fits the map at exactly one pose, and only at the
blind bearings.
"""
import argparse
import math
import os
import sys

import numpy as np
import pytest

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO, "scripts"))

import clearance  # noqa: E402
import map_grid  # noqa: E402

LASER = (0.095, 0.0)
BLIND = clearance.parse_blind(["117:147"])     # the MC200's mask 118-263 with its margin
CIRCLE = 0.31 + clearance.SPIN_MARGIN_M         # its Nav2 radius, as the 1-Click passes it
RES = 0.05


def wall(x0, y0, x1, y1, step=0.01):
    n = max(2, int(math.hypot(x1 - x0, y1 - y0) / step))
    return np.c_[np.linspace(x0, x1, n), np.linspace(y0, y1, n)]


# The garage corner, world frame: a 4 x 3 m room, the robot backed into its top-left corner
# facing +x; a door edge just ahead of it, and a box out on the floor (the room is not
# symmetric, as no real one is).
ROBOT = (0.0, 0.60, 2.72)                         # (yaw, x, y)
CORNER = [wall(0, 0, 4, 0), wall(4, 0, 4, 3), wall(4, 3, 0, 3), wall(0, 3, 0, 0),
          wall(1.23, 2.52, 1.23, 3.0),              # the door edge, 0.63 m ahead
          wall(2.5, 0.8, 3.0, 0.8), wall(3.0, 0.8, 3.0, 1.0),
          wall(3.0, 1.0, 2.5, 1.0), wall(2.5, 1.0, 2.5, 0.8)]


def to_base(W, pose):
    th, x, y = pose
    c, s = math.cos(th), math.sin(th)
    D = np.asarray(W, float) - [x, y]
    return D @ np.array([[c, -s], [s, c]])


def scan(walls, pose, blind=BLIND):
    """What the LiDAR returns at `pose`: the nearest wall point per degree, none where blind."""
    P = to_base(np.vstack(walls), pose)
    Q = P - LASER
    b = np.floor(np.degrees(np.arctan2(Q[:, 1], Q[:, 0]))).astype(int) % 360
    d = np.hypot(Q[:, 0], Q[:, 1])
    keep = []
    for deg in range(360):
        m = b == deg
        if m.any() and not clearance.in_blind(np.array([deg + 0.5]), blind)[0]:
            keep.append(P[m][np.argmin(d[m])])
    return np.array(keep)


def grid(walls, w=4.0, h=3.0):
    """The map SLAM would have saved of the room: walls occupied, the inside free, a margin
    of unknown round it."""
    pad = 0.5
    ox, oy = -pad, -pad
    nx, ny = int(round((w + 2 * pad) / RES)), int(round((h + 2 * pad) / RES))
    occ = np.zeros((ny, nx), bool)
    free = np.zeros((ny, nx), bool)
    cx = ox + (np.arange(nx) + 0.5) * RES
    cy = oy + (ny - 1 - np.arange(ny) + 0.5) * RES
    X, Y = np.meshgrid(cx, cy)
    free[(X > 0) & (X < w) & (Y > 0) & (Y < h)] = True
    m = map_grid.MapGrid(occ, free, RES, (ox, oy))
    r, c, inside = m.cells(np.vstack(walls))
    occ[r[inside], c[inside]] = True
    free &= ~occ
    return map_grid.MapGrid(occ, free, RES, (ox, oy))


def test_the_corner_refuses_without_the_map():
    P = scan(CORNER, ROBOT)
    for travel in (0.8, 2.5):
        assert clearance.open_spot(P, BLIND, CIRCLE, True, travel, LASER) is None


def test_the_scan_finds_where_it_is_on_the_map():
    m = grid(CORNER)
    pose, why = map_grid.locate(scan(CORNER, ROBOT), m, LASER)
    assert pose is not None, why
    th, x, y, rms, hit = pose
    assert abs(math.degrees(th - ROBOT[0])) < 3.0
    assert math.hypot(x - ROBOT[1], y - ROBOT[2]) < 0.06
    assert rms < map_grid.MAX_RMS


def test_the_map_gets_the_robot_out_of_the_corner():
    P = scan(CORNER, ROBOT)
    m = grid(CORNER)
    pose, _ = map_grid.locate(P, m, LASER)
    also = map_grid.seen_by_map(m, pose[:3], BLIND, LASER)
    spot = clearance.open_spot(P, BLIND, CIRCLE, True, 0.8, LASER, also_free=also)
    assert spot is not None
    h, d = spot
    assert math.sin(h) < 0 and d <= 0.8 + 1e-9        # to the right, away from the shelf
    # where it ends, the whole turning circle is on ground the room really has free
    end = np.array([ROBOT[1] + d * math.cos(h), ROBOT[2] + d * math.sin(h)])
    assert end[1] + CIRCLE < 3.0 and end[0] - CIRCLE > 0.0


def test_the_map_never_overrules_what_the_lidar_sees():
    """A box set down since the map was saved: map-free ground at a VISIBLE bearing stays
    the live scan's call."""
    m = grid(CORNER)
    pose, _ = map_grid.locate(scan(CORNER, ROBOT), m, LASER)
    also = map_grid.seen_by_map(m, pose[:3], BLIND, LASER)
    ahead_right = np.array([[0.5, -0.5]])               # bearing about -50 deg: visible
    behind = np.array([[-0.25, -0.30]])                 # bearing about -130 deg: blind
    assert m.is_free(map_grid.to_map(ahead_right, pose)).all()
    assert not also(ahead_right).any()
    assert also(behind).all()


def test_a_symmetric_room_is_ambiguous():
    room = [wall(0, 0, 4, 0), wall(4, 0, 4, 3), wall(4, 3, 0, 3), wall(0, 3, 0, 0)]
    pose, why = map_grid.locate(scan(room, (0.0, 2.0, 1.5)), grid(room), LASER)
    assert pose is None and "ambiguous" in why


def test_a_scan_of_another_room_does_not_fit():
    other = [wall(0, 0, 1.6, 0), wall(1.6, 0, 1.6, 1.2), wall(1.6, 1.2, 0, 1.2), wall(0, 1.2, 0, 0),
             wall(0.7, 0.2, 0.9, 0.2)]
    pose, why = map_grid.locate(scan(other, (0.0, 0.5, 0.6)), grid(CORNER), LASER)
    assert pose is None


def test_a_map_round_trips_through_its_files(tmp_path):
    m = grid(CORNER)
    px = np.full(m.occ.shape, 205, np.uint8)
    px[m.free] = 254
    px[m.occ] = 0
    h, w = px.shape
    (tmp_path / "one_click_map.pgm").write_bytes(f"P5\n# CREATOR: test\n{w} {h}\n255\n".encode() + px.tobytes())
    (tmp_path / "one_click_map.yaml").write_text(
        "image: one_click_map.pgm\nmode: trinary\nresolution: 0.050\n"
        f"origin: [{m.ox}, {m.oy}, 0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n")
    back = map_grid.MapGrid.load(str(tmp_path / "one_click_map.yaml"))
    assert (back.occ == m.occ).all() and (back.free == m.free).all()
    assert (back.ox, back.oy, back.res) == (m.ox, m.oy, m.res)


def test_the_1click_passes_the_last_map_only_when_it_helps(tmp_path):
    import one_click_pipeline as o
    out = str(tmp_path / "one_click_map")
    args = argparse.Namespace(map_output=out)
    masked = {"lidar": {"mask": {"sectors": [[118, 263]]}}}
    assert o.last_map_flag(args, masked, moves=True) == ""            # no map yet
    open(out + ".yaml", "w").write("image: x.pgm\n")
    assert o.last_map_flag(args, masked, moves=True) == f" --map {out}.yaml"
    assert o.last_map_flag(args, masked, moves=False) == ""           # nothing moves
    assert o.last_map_flag(args, {"lidar": {}}, moves=True) == ""     # nothing masked


def test_a_shelf_at_ten_centimetres_does_not_drown_the_room():
    """Jammed against a shelf, most beams hit its face; thinned by beam those points were
    the scan and a straight face fits anywhere along a wall ("ambiguous", 2026-10-17)."""
    jam = (math.radians(90), 1.55, 2.75)     # facing the top wall, 0.10 m past a 0.15 m body
    room = CORNER + [wall(2.6, 3.0, 2.6, 2.6), wall(2.6, 2.6, 3.1, 2.6), wall(3.1, 2.6, 3.1, 3.0)]  # a cabinet
    pose, why = map_grid.locate(scan(room, jam), grid(room), LASER)
    assert pose is not None, why
    assert math.hypot(pose[1] - jam[1], pose[2] - jam[2]) < 0.06
    assert abs(math.degrees(pose[0] - jam[0])) < 3.0


def test_a_corner_of_a_bare_room_stays_ambiguous():
    """Four bare walls: the opposite corner, turned round, fits as well and no beam crosses a
    wall at either -- refused, not guessed."""
    bare = CORNER[:4]
    pose, why = map_grid.locate(scan(bare, (math.radians(90), 3.5, 2.75)), grid(bare), LASER)
    assert pose is None and "ambiguous" in why


class _StallBot(clearance.Robot):
    """Robot without ROS: /odom stops at t=0 and comes back after `gap` s (never if None)."""

    def __init__(self, gap):
        self.gap, self.t0, self.stops = gap, None, 0

    def odom_age(self):
        import time
        now = time.monotonic()
        if self.t0 is None:
            self.t0 = now
        resumed = self.gap is not None and now - self.t0 >= self.gap
        return 0.02 if resumed else 0.31 + (now - self.t0)

    def scan_age(self):
        return 0.05

    def pump(self, timeout=0.05):
        import time
        time.sleep(0.01)

    def stop(self):
        self.stops += 1


def test_a_short_odom_pause_holds_the_robot_and_goes_on():
    """The UNO Q's EKF stops /odom for 0.2-0.4 s whenever a process joins its DDS; the
    look-around's fifth turn was abandoned on one (2026-10-17)."""
    bot = _StallBot(gap=0.4)
    assert bot.stale(guarded=True) == ""
    assert bot.stops == 1                          # held still while blind


def test_a_long_odom_pause_still_stops_the_move():
    bot = _StallBot(gap=None)
    why = bot.stale(guarded=False)
    assert why.startswith("odometry stopped") and bot.stops == 1


def test_a_tracked_robot_in_a_corner_turns_where_it_stands():
    """A tracked robot: a narrow blind sliver (139:22 deg), not holonomic, a wall 0.32 m ahead and
    0.30 m to its left. It may only look for room ahead, which the wall fills, and turning in
    place sweeps the sliver -- refused. With the map covering the sliver it turns where it is."""
    blind = clearance.parse_blind(["139:22"])
    laser, circle = (0.02, 0.0), 0.21 + clearance.SPIN_MARGIN_M
    pose = (0.0, 0.70, 2.70)                       # facing +x
    room = [wall(0, 0, 4, 0), wall(4, 0, 4, 3), wall(4, 3, 0, 3), wall(0, 3, 0, 0),
            wall(1.02, 1.5, 1.02, 3.0),             # the wall ahead, 0.32 m
            wall(2.5, 0.8, 3.0, 0.8), wall(3.0, 0.8, 3.0, 1.0), wall(3.0, 1.0, 2.5, 1.0), wall(2.5, 1.0, 2.5, 0.8)]
    P = scan(room, pose, blind)
    assert clearance.open_spot(P, blind, circle, False, 0.8, laser) is None
    m = grid(room)
    fix, why = map_grid.locate(P, m, laser)
    assert fix is not None, why
    also = map_grid.seen_by_map(m, fix[:3], blind, laser)
    assert clearance.open_spot(P, blind, circle, False, 0.8, laser, also_free=also) == (0.0, 0.0)
