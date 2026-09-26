"""Masking the LiDAR where the robot sees itself: a mast, posts, a bumper.

A LiDAR mounted under a deck or beside a mast sees those parts at a few
centimetres in every scan. Unmasked, SLAM maps them as a wall that travels with
the robot and the costmap marks the robot as its own obstacle. The mask removes
those beams before anything reads /scan.

Config (base_controller.lidar):

    mask:
      sectors: [[150, 210]]     # degrees in the ROBOT frame: 0 ahead, counter-clockwise
      boxes:                    # metres in base_link; any beam ending inside is removed
        - {min_x: -0.10, max_x: 0.10, min_y: -0.05, max_y: 0.05, min_z: -1.0, max_z: 1.0}

and, so the bench can prove it without a robot, the simulated LD19 can be
given the posts to see (base_controller.simulation):

    lidar_occlusion: [[150, 210]]   # same frame and units as mask.sectors
    lidar_occlusion_range: 0.12     # metres: where the post is

The filtering is laser_filters' scan_to_scan_filter_chain between the driver
(which then publishes scan_raw) and /scan -- the package upstream linorobot2
documents for exactly this. Two details decide whether it works:

  * replace_with_nan: true. Without it LaserScanAngularBoundsFilterInPlace
    writes range_max + 1, a VALID "nothing out to here" reading, and the
    costmap clears straight through the mast every scan. NaN is neither a
    mark nor a clear (Nav2's obstacle layer skips it; inf_is_valid is false).
  * The sector is compared against the scan's own raw angles. lyrical's
    laser_filters (2.3) has no wrap handling at all, jazzy's (2.0) has an
    opt-in wrap_angle, and the scans differ too: the LD driver publishes
    0..2 pi, the host's simulated LD19 -pi..pi. So each sector is emitted as
    plain intervals, split at 2 pi and repeated shifted by -2 pi; a copy
    outside a scan's range matches nothing. Correct on both distros, both
    conventions, without wrap_angle.

The LD driver's own angle_crop was not used: one interval, in the LD19's raw
clockwise degrees before its direction flip, and only for that driver.
"""
import math

MAX_SECTORS = 4          # the firmware's occlusion table (sim_ld19.h) holds this many
BOX_KEYS = ("min_x", "max_x", "min_y", "max_y", "min_z", "max_z")


def _sectors(raw, what):
    out = []
    for s in raw or []:
        if not isinstance(s, (list, tuple)) or len(s) != 2:
            raise ValueError(f"{what}: each sector is [from_deg, to_deg], not {s!r}")
        a, b = float(s[0]), float(s[1])
        width = (b - a) % 360.0
        if width == 0.0:
            raise ValueError(f"{what}: sector {s!r} is empty (or the whole circle)")
        out.append((a % 360.0, width))
    return out


def mask_config(controller: dict) -> dict:
    """{'sectors': [(start_deg, width_deg)], 'boxes': [dict]} in the robot frame; empty when unmasked."""
    mask = (((controller or {}).get("lidar") or {}).get("mask") or {})
    sectors = _sectors(mask.get("sectors"), "lidar.mask.sectors")
    boxes = []
    for bx in mask.get("boxes") or []:
        if not isinstance(bx, dict) or any(k not in bx for k in BOX_KEYS):
            raise ValueError(f"lidar.mask.boxes: each box needs {', '.join(BOX_KEYS)}, not {bx!r}")
        box = {k: float(bx[k]) for k in BOX_KEYS}
        for lo, hi in (("min_x", "max_x"), ("min_y", "max_y"), ("min_z", "max_z")):
            if box[lo] >= box[hi]:
                raise ValueError(f"lidar.mask.boxes: {lo} must be below {hi} in {bx!r}")
        boxes.append(box)
    return {"sectors": sectors, "boxes": boxes}


def masked(controller: dict) -> bool:
    m = mask_config(controller)
    return bool(m["sectors"] or m["boxes"])


def scan_intervals(start_deg: float, width_deg: float, laser_yaw_rad: float = 0.0) -> list:
    """One robot-frame sector as raw-angle intervals (radians) that cover a scan in any convention.

    The scan's angles are the LiDAR's own, so the mount's yaw is taken off.
    """
    two_pi = 2.0 * math.pi
    lo = (math.radians(start_deg) - laser_yaw_rad) % two_pi
    hi = lo + math.radians(width_deg)
    parts = [(lo, hi)] if hi <= two_pi else [(lo, two_pi), (0.0, hi - two_pi)]
    return [(a + shift, b + shift) for (a, b) in parts for shift in (0.0, -two_pi)]


def filter_chain_params(controller: dict, laser_yaw_rad: float, base_frame: str = "base_link") -> dict:
    """The scan_to_scan_filter_chain parameters for this robot's mask, or {} when unmasked."""
    m = mask_config(controller)
    filters = []
    for i, (start, width) in enumerate(m["sectors"]):
        for j, (lo, hi) in enumerate(scan_intervals(start, width, laser_yaw_rad)):
            filters.append({"name": f"mask_sector_{i}_{j}",
                            "type": "laser_filters/LaserScanAngularBoundsFilterInPlace",
                            "params": {"lower_angle": round(lo, 6), "upper_angle": round(hi, 6),
                                       "replace_with_nan": True}})
    for i, box in enumerate(m["boxes"]):
        filters.append({"name": f"mask_box_{i}",
                        "type": "laser_filters/LaserScanBoxFilter",
                        # invert false: remove what is INSIDE the box.
                        "params": dict(box, box_frame=base_frame, invert=False)})
    if not filters:
        return {}
    return {"scan_to_scan_filter_chain": {"ros__parameters": {
        f"filter{k + 1}": f for k, f in enumerate(filters)}}}


def occlusion(params: dict):
    """(sectors [(start_deg, width_deg)], range_m) for the simulated LD19, or ([], None)."""
    sim = (((params or {}).get("base_controller") or {}).get("simulation") or {})
    sectors = _sectors(sim.get("lidar_occlusion"), "simulation.lidar_occlusion")
    if len(sectors) > MAX_SECTORS:
        raise ValueError(f"simulation.lidar_occlusion: at most {MAX_SECTORS} sectors, not {len(sectors)}")
    if not sectors:
        return [], None
    return sectors, float(sim.get("lidar_occlusion_range", 0.12))


def occlusion_env(params: dict) -> dict:
    """The env keys the firmware's simulated LD19 reads: sim_occl = "start,width,...", sim_occl_r."""
    sectors, rng = occlusion(params)
    if not sectors:
        return {}
    return {"sim_occl": ",".join(f"{a:g},{w:g}" for a, w in sectors), "sim_occl_r": f"{rng:g}"}


def occluded(ccw_deg: float, sectors) -> bool:
    """Is a robot-frame angle (degrees, counter-clockwise) inside one of the sectors?"""
    x = ccw_deg % 360.0
    return any(((x - a) % 360.0) < w for a, w in sectors)
