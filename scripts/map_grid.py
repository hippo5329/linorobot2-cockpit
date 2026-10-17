"""A saved map (map_saver's .yaml + .pgm) as ground a robot may count as seen.

The room check (clearance.py inch) moves a real robot only over ground its LiDAR has
seen free. A LiDAR masked behind the robot (the MC200's 118-263 deg) never sees the
ground beside its own flanks, so in a corner every way out sweeps some of it and the
check refused -- "move the robot by hand" (2026-10-16). The last run's map often holds
that ground: if the live scan matches the map at one pose, and only one, the map's free
cells stand in for the blind bearings. Where the LiDAR can see, its own view wins: a box
put down since the map was saved is never overruled.

numpy only (no scipy, no imaging package): the robot computers have neither.
"""
import math
import os

import numpy as np
import yaml

import scan_match

SEARCH_STEP_M = 0.10     # locate(): candidate positions on this grid
SEARCH_STEP_DEG = 5.0    # ... and headings this far apart
HIT_CELLS = 1            # a point within this many cells of a wall counts as a hit
MAX_POINTS = 200         # the scan is thinned to at most this many points for the search
MIN_HIT_FRAC = 0.60      # a fix needs this share of the scan on the map's walls
MAX_RMS = 0.05           # ... and the refined fit this tight
RIVAL_FRAC = 0.80        # another pose with this share of the best's hits: ambiguous
RIVAL_MIN_M = 0.25       # "another pose": farther than this ...
RIVAL_MIN_DEG = 25.0     # ... or turned more than this
FREE_MARGIN_M = 0.10     # map-free ground counts only this far from a wall or unknown cell
REFINE_CANDIDATES = 8    # locate(): distinct coarse poses refined and compared
RAY_SAMPLES = 40         # through_walls(): points sampled along each beam
RAY_START_M = 0.15       # ... from this far out (the robot's own body) ...
RAY_END_M = 0.15         # ... to this short of where it ended (its wall, blurred by SLAM)
RAY_MIN_SIN = 0.5        # ... only beams meeting their surface at 30 deg or more
CONTRADICTION_WEIGHT = 2 # locate(): a beam through a wall or a return on clear floor costs
                         # two points on walls (a hit is cheap near any wall; this is not)


class MapError(ValueError):
    pass


def read_pgm(path: str) -> np.ndarray:
    """A P5 (binary, 8-bit) PGM as a uint8 array, row 0 the top row."""
    with open(path, "rb") as f:
        data = f.read()
    if data[:2] != b"P5":
        raise MapError(f"{path}: not a binary PGM")
    tokens, i = [], 2
    while len(tokens) < 3:
        while i < len(data) and data[i:i + 1].isspace():
            i += 1
        if data[i:i + 1] == b"#":
            while i < len(data) and data[i:i + 1] not in (b"\n", b"\r"):
                i += 1
            continue
        j = i
        while j < len(data) and not data[j:j + 1].isspace():
            j += 1
        tokens.append(int(data[i:j]))
        i = j
    w, h, maxval = tokens
    if maxval > 255:
        raise MapError(f"{path}: 16-bit PGM")
    px = np.frombuffer(data[i + 1:i + 1 + w * h], np.uint8)
    if px.size != w * h:
        raise MapError(f"{path}: truncated")
    return px.reshape(h, w)


def _dilate(mask: np.ndarray, cells: int) -> np.ndarray:
    out = mask.copy()
    for dy in range(-cells, cells + 1):
        for dx in range(-cells, cells + 1):
            if dx * dx + dy * dy > cells * cells:
                continue
            out |= np.roll(np.roll(mask, dy, 0), dx, 1)
    return out


class MapGrid:
    """free / occupied / unknown cells, indexed [row, col] with row 0 at the TOP (as the PGM)."""

    def __init__(self, occ: np.ndarray, free: np.ndarray, res: float, origin=(0.0, 0.0)):
        self.occ, self.free, self.res = occ.astype(bool), free.astype(bool), float(res)
        self.ox, self.oy = float(origin[0]), float(origin[1])
        self.h, self.w = occ.shape
        m = max(1, int(math.ceil(FREE_MARGIN_M / self.res)))
        self.free_ok = self.free & ~_dilate(~self.free, m)          # far from walls AND unknown
        self.hit = _dilate(self.occ, HIT_CELLS)

    @classmethod
    def load(cls, yaml_path: str) -> "MapGrid":
        with open(yaml_path) as f:
            meta = yaml.safe_load(f) or {}
        img = meta.get("image")
        if not img:
            raise MapError(f"{yaml_path}: no image")
        if not os.path.isabs(img):
            img = os.path.join(os.path.dirname(yaml_path), img)
        px = read_pgm(img).astype(float) / 255.0
        p = px if meta.get("negate", 0) else 1.0 - px                 # occupancy probability
        occ = p > float(meta.get("occupied_thresh", 0.65))
        free = p < float(meta.get("free_thresh", 0.196))
        origin = meta.get("origin", [0.0, 0.0, 0.0])
        if abs(float(origin[2] if len(origin) > 2 else 0.0)) > 1e-6:
            raise MapError(f"{yaml_path}: a rotated map origin is not supported")
        return cls(occ, free, float(meta["resolution"]), origin[:2])

    def cells(self, Q):
        """(row, col, inside) of map-frame points Q (N x 2)."""
        Q = np.asarray(Q, float).reshape(-1, 2)
        col = np.floor((Q[:, 0] - self.ox) / self.res).astype(int)
        row = self.h - 1 - np.floor((Q[:, 1] - self.oy) / self.res).astype(int)
        inside = (row >= 0) & (row < self.h) & (col >= 0) & (col < self.w)
        return np.clip(row, 0, self.h - 1), np.clip(col, 0, self.w - 1), inside

    def centres(self, mask: np.ndarray) -> np.ndarray:
        r, c = np.nonzero(mask)
        return np.c_[self.ox + (c + 0.5) * self.res, self.oy + (self.h - 1 - r + 0.5) * self.res]

    def is_free(self, Q) -> np.ndarray:
        r, c, inside = self.cells(Q)
        return inside & self.free_ok[r, c]


def to_map(Q, pose) -> np.ndarray:
    th, tx, ty = pose[:3]
    c, s = math.cos(th), math.sin(th)
    Q = np.asarray(Q, float).reshape(-1, 2)
    return Q @ np.array([[c, s], [-s, c]]) + np.array([tx, ty])


THIN_CELL_M = 0.05       # locate(): one scan point per cell this size


def _thin(S: np.ndarray, n: int) -> np.ndarray:
    """One point per THIN_CELL_M cell, then at most n. A LiDAR 0.10 m from a shelf puts dozens
    of points on every centimetre of it; thinned by beam they were most of the scan, a
    straight face fit anywhere along a wall, and the corner test was "ambiguous" between
    poses 4 m apart (2026-10-17). By space, the far walls that pin the pose count again."""
    if len(S) == 0:
        return S
    _, first = np.unique(np.floor(S / THIN_CELL_M).astype(int), axis=0, return_index=True)
    S = S[np.sort(first)]
    return S if len(S) <= n else S[np.linspace(0, len(S) - 1, n).astype(int)]


def through_walls(S, m: MapGrid, pose, laser=(0.0, 0.0)) -> int:
    """How many of the scan's beams pass through a wall of the map on their way out: at the
    true pose none should (bar a beam grazing a corner); at a mirror pose the beams that saw
    open floor cut straight through the map's walls."""
    S = np.asarray(S, float).reshape(-1, 2)
    if len(S) < 3:
        return 0
    # A beam that meets its surface at a shallow angle runs inside that wall's cells for longer
    # than any end margin and would count against the true pose (19 of 59 beams did, along a
    # wall 0.25 m away). The surface's direction comes from the scan itself, the neighbours.
    seg = np.roll(S, -1, 0) - np.roll(S, 1, 0)
    beam = S - np.asarray(laser[:2], float)
    near = np.hypot(seg[:, 0], seg[:, 1]) < 3 * THIN_CELL_M + 0.05
    cross = np.abs(beam[:, 0] * seg[:, 1] - beam[:, 1] * seg[:, 0])
    sin_inc = cross / np.maximum(np.hypot(beam[:, 0], beam[:, 1]) * np.hypot(seg[:, 0], seg[:, 1]), 1e-9)
    S = S[near & (sin_inc >= RAY_MIN_SIN)]
    if len(S) == 0:
        return 0
    L = to_map(np.asarray(laser[:2], float).reshape(1, 2), pose)[0]
    E = to_map(S, pose)
    D = E - L
    dist = np.hypot(D[:, 0], D[:, 1])
    f = np.linspace(0.0, 1.0, RAY_SAMPLES)[None, :]
    stop = np.clip((dist - RAY_END_M) / np.maximum(dist, 1e-9), 0.0, 1.0)[:, None]
    start = np.clip(RAY_START_M / np.maximum(dist, 1e-9), 0.0, 1.0)[:, None]
    frac = start + (stop - start) * f                           # N x K, the beam short of its end
    Q = L + frac[:, :, None] * D[:, None, :]
    r, c, inside = m.cells(Q.reshape(-1, 2))
    blocked = (inside & m.occ[r, c]).reshape(frac.shape) & (stop > start)
    return int(blocked.any(1).sum())


def locate(S, m: MapGrid, laser=(0.0, 0.0)):
    """(pose, why): pose = (th, tx, ty, rms, hit_frac) of the scan S (base_link points) in the
    map, or None and the reason. A coarse search over every free cell and heading; the best
    few distinct poses refined by ICP and scored by points on walls less beams through walls;
    refused when the fit is loose or a second, different pose scores nearly as well."""
    S = _thin(np.asarray(S, float).reshape(-1, 2), MAX_POINTS)
    if len(S) < 20:
        return None, f"only {len(S)} scan points once thinned"
    step = max(1, int(round(SEARCH_STEP_M / m.res)))
    cand = m.centres(m.free_ok)
    keep = (np.floor((cand[:, 0] - m.ox) / m.res).astype(int) % step == 0) & \
           (np.floor((cand[:, 1] - m.oy) / m.res).astype(int) % step == 0)
    cand = cand[keep]
    if len(cand) == 0:
        return None, "the map has no free ground"
    scores = []                                    # (hits, th, x, y)
    for deg in np.arange(0.0, 360.0, SEARCH_STEP_DEG):
        th = math.radians(deg)
        c, s = math.cos(th), math.sin(th)
        R = S @ np.array([[c, s], [-s, c]])        # rotated scan, N x 2
        for lo in range(0, len(cand), 512):
            C = cand[lo:lo + 512]
            P = C[:, None, :] + R[None, :, :]      # M x N x 2
            r, cc, inside = m.cells(P.reshape(-1, 2))
            hits = (inside & m.hit[r, cc]).reshape(len(C), len(S)).sum(1)
            for k in np.argsort(-hits)[:3]:
                scores.append((int(hits[k]), th, float(C[k, 0]), float(C[k, 1])))
    scores.sort(key=lambda t: -t[0])
    if scores[0][0] < MIN_HIT_FRAC * len(S):
        return None, f"the scan does not fit the map ({scores[0][0]}/{len(S)} points on its walls)"
    distinct = []
    for t in scores:
        if all(math.hypot(t[2] - d[2], t[3] - d[3]) > RIVAL_MIN_M
               or abs(math.degrees(scan_match.wrap(t[1] - d[1]))) > RIVAL_MIN_DEG for d in distinct):
            distinct.append(t)
        if len(distinct) == REFINE_CANDIDATES:
            break
    walls = m.centres(m.occ)
    refined = []                                   # (score, hits, through, th, tx, ty, rms, pairs)
    for t in distinct:
        if t[0] < RIVAL_FRAC * MIN_HIT_FRAC * len(S):
            continue
        near = walls[np.hypot(walls[:, 0] - t[2], walls[:, 1] - t[3]) < 8.0]
        th, tx, ty, rms, pairs = scan_match.icp(S, near, t[1], (t[2], t[3]))
        if not (rms == rms):
            continue
        hits = scan_match.inliers(S, near, th, tx, ty, tol=1.5 * m.res)
        # contradictions: a beam through a wall, or a return where the map has clear floor
        bad = through_walls(S, m, (th, tx, ty), laser) + int(m.is_free(to_map(S, (th, tx, ty))).sum())
        refined.append((hits - CONTRADICTION_WEIGHT * bad, hits, bad, scan_match.wrap(th), tx, ty, rms, pairs))
    if not refined:
        return None, "no candidate pose survived refinement"
    refined.sort(key=lambda r: -r[0])
    b = refined[0]
    for r in refined[1:]:
        far = math.hypot(r[4] - b[4], r[5] - b[5]) > RIVAL_MIN_M or \
              abs(math.degrees(scan_match.wrap(r[3] - b[3]))) > RIVAL_MIN_DEG
        if far and r[0] >= RIVAL_FRAC * b[0]:
            return None, (f"ambiguous: ({b[4]:+.2f}, {b[5]:+.2f}) and ({r[4]:+.2f}, {r[5]:+.2f}) "
                          f"fit nearly as well (score {b[0]} vs {r[0]}: on walls less contradictions)")
        if far:
            break
    if b[6] > MAX_RMS or b[1] < MIN_HIT_FRAC * len(S):
        return None, (f"the refined fit is loose (rms {b[6]:.3f} m, {b[1]}/{len(S)} points on walls, "
                      f"{b[2]} contradictions)")
    return (b[3], b[4], b[5], b[6], b[1] / len(S)), ""


def seen_by_map(m: MapGrid, pose, blind, laser=(0.0, 0.0)):
    """A predicate for clearance.open_spot(): True for base_link points on map-free ground
    at a BLIND bearing (about the laser). Visible bearings are the live scan's alone."""
    import clearance

    lx, ly = laser[:2]

    def pred(Q):
        Q = np.asarray(Q, float).reshape(-1, 2)
        b = np.degrees(np.arctan2(Q[:, 1] - ly, Q[:, 0] - lx)) % 360.0
        return clearance.in_blind(b, blind) & m.is_free(to_map(Q, pose))

    return pred
