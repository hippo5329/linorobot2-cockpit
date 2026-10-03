"""Motion limits while SLAM builds the map (nav2.launch.py applies them)."""

import depth_camera

# While SLAM builds the map from a narrow scan, the robot turns no faster than
# this. At 1.2 rad/s a 10 Hz scan sweeps ~7 deg while it is taken; a masked
# LiDAR (210 deg) matched those skewed scans and SLAM's heading stepped 13 deg
# then 28 deg in two turns at 1.0-1.25 rad/s, with the odometry right to 0.3 deg,
# and the robot never found its way home (gate, 2026-10-03). Half the rate halves
# the skew. A 360 deg LiDAR, and any run on a saved map, keeps its limits.
NARROW_MAPPING_WZ = 0.6


def cap_mapping_turn_rate(nav2_data, params, map_file):
    """Cap every source of yaw rate while mapping from a narrow scan; the cap, or None."""
    if map_file:
        return None
    fov = depth_camera.scan_fov_deg((params or {}).get("base_controller") or {})
    if fov is None or fov >= depth_camera.NARROW_FOV_DEG:
        return None
    cap = NARROW_MAPPING_WZ
    vs = nav2_data.setdefault("velocity_smoother", {}).setdefault("ros__parameters", {})
    for key, sign in (("max_velocity", 1.0), ("min_velocity", -1.0)):
        v = list(vs.get(key) or [])
        if len(v) == 3:
            v[2] = sign * min(abs(float(v[2])), cap)
            vs[key] = v
    fp = nav2_data.setdefault("controller_server", {}).setdefault("ros__parameters", {}).setdefault("FollowPath", {})
    if "rotate_to_heading_angular_vel" in fp:
        fp["rotate_to_heading_angular_vel"] = min(float(fp["rotate_to_heading_angular_vel"]), cap)
    bs = nav2_data.setdefault("behavior_server", {}).setdefault("ros__parameters", {})
    bs["max_rotational_vel"] = min(float(bs.get("max_rotational_vel", cap)), cap)
    bs["min_rotational_vel"] = min(float(bs.get("min_rotational_vel", 0.4)), cap)
    return cap
