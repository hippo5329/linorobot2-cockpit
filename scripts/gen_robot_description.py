#!/usr/bin/env python3
# ==============================================================================
# gen_robot_description.py — the robot's URDF, generated from its config
#
# robot_state_publisher used to be handed a stock xacro picked by
# kinematics.base_type alone (2wd / 4wd / mecanum from linorobot2_description),
# whose wheel radius, track and LiDAR pose were fixed numbers that nothing read
# the config to update. The odometry was computed from the user's numbers and
# the TF tree from somebody else's: base_link -> wheel and base_link -> laser
# described a machine the user did not own.
#
# Now every dimension comes from <robot>_config.yaml:
#
#   kinematics.wheel_diameter        wheel radius            (also drives odometry)
#   kinematics.lr_wheels_distance    left/right axle length  (also drives odometry)
#   kinematics.fr_wheels_distance    front/rear axle spacing (4wd / mecanum)
#   kinematics.base_type             2 wheels + casters, or 4 wheels
#   geometry.body                    the base_link box: length width height mass
#   geometry.wheel                   width, mass, z (axle height below base_link)
#   geometry.casters                 front / rear (2wd only)
#   geometry.laser                   x y z roll pitch yaw frame  (base_link -> laser)
#   geometry.imu                     x y z roll pitch yaw        (base_link -> imu_link)
#   geometry.mesh                    optional base / wheel mesh URIs
#
# A config that predates `geometry:` gets one derived from its kinematics --
# see effective_geometry(); migrate_config_schema.py writes that derivation into
# the file so the numbers are the user's to see and change, not a code default.
#
# The output is plain URDF, not xacro: nothing at run time has to find a
# package, and the file is small enough to read. It is written to
# <config dir>/generated/<robot>.urdf (gitignored there) by bringup.launch.py
# at every launch and by the supervisor whenever the config is saved.
#
#   python3 scripts/gen_robot_description.py --params <cfg> [--out <file>]
# ==============================================================================
import argparse
import math
import os
import sys
import xml.etree.ElementTree as ET
from typing import Any, Dict, List, Tuple

try:
    import yaml
except ImportError:
    print("PyYAML is required: sudo apt install -y python3-yaml", file=sys.stderr)
    sys.exit(1)

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import cockpit_paths  # noqa: E402

# Frames the rest of the stack already agrees on. `imu_link` is what the firmware
# stamps on /imu/data_raw (imu_interface.h) and `base_footprint` is the child
# frame of its /odom/unfiltered; the EKF publishes odom -> base_link, which is
# why base_footprint is a CHILD of base_link here and not its parent. They are
# protocol, not robot facts, which is why they are not config keys.
FOOTPRINT_FRAME = "base_footprint"
BASE_FRAME = "base_link"
IMU_FRAME = "imu_link"
DEFAULT_LASER_FRAME = "laser"
# The firmware stamps its Range messages with envPrefixed("sonar_link"), so the
# description has to publish a frame by exactly that name or every consumer
# drops the message when it fails to transform it -- silently, which is how a
# sonar can publish at 10 Hz and reach nothing.
DEFAULT_SONAR_FRAME = "sonar_link"

WHEEL_JOINTS_2 = ("left", "right")
WHEEL_JOINTS_4 = ("front_left", "front_right", "rear_left", "rear_right")


def _f(v, default=0.0) -> float:
    try:
        return float(v)
    except (TypeError, ValueError):
        return float(default)


def effective_geometry(params: Dict[str, Any]) -> Dict[str, Any]:
    """The geometry block with every gap filled from the kinematics.

    Only proportions are assumed, never a length in metres: a config that
    says nothing about its body gets a chassis sized from its own wheel and
    track. Explicit keys always win. Returned as a fresh dict, so callers can
    write it back into a config (migration) or hand it to a form.
    """
    kine = params.get("kinematics") or {}
    geo = params.get("geometry") or {}
    wheel_d = _f(kine.get("wheel_diameter"), 0.0)
    lr = _f(kine.get("lr_wheels_distance"), 0.0)
    fr = _f(kine.get("fr_wheels_distance"), 0.0)
    base = str(kine.get("base_type", "2wd")).lower()

    wheel = dict(geo.get("wheel") or {})
    wheel.setdefault("width", round(wheel_d * 0.25, 4))   # a tyre a quarter as wide as it is tall
    wheel.setdefault("mass", round(2.0 * wheel_d, 3))      # 2 kg per metre of diameter (152 mm -> 0.3 kg)
    wheel.setdefault("z", 0.0)                             # axle through the base_link origin

    body = dict(geo.get("body") or {})
    body.setdefault("length", round((fr if fr > 0 else 0.0) + 1.5 * wheel_d, 4))
    body.setdefault("width", round(max(lr - 1.5 * _f(wheel["width"]), wheel_d), 4))
    body.setdefault("height", round(0.3 * wheel_d, 4))
    body.setdefault("mass", 1.0)

    casters = dict(geo.get("casters") or {})
    casters.setdefault("front", base == "2wd")
    casters.setdefault("rear", base == "2wd")

    laser = dict(geo.get("laser") or {})
    laser.setdefault("x", 0.0)
    laser.setdefault("y", 0.0)
    # On a deck one wheel radius above the chassis top: clear of the body box.
    laser.setdefault("z", round(_f(body["height"]) / 2.0 + wheel_d / 2.0, 4))
    for k in ("roll", "pitch", "yaw"):
        laser.setdefault(k, 0.0)
    laser.setdefault("frame", DEFAULT_LASER_FRAME)

    # An ultrasonic looks forward from the front face, at axle height. A range
    # sensor sunk into the body would read the chassis at its minimum range for
    # ever, which the collision monitor would believe.
    sonar = dict(geo.get("sonar") or {})
    sonar.setdefault("x", round(_f(body["length"]) / 2.0, 4))
    sonar.setdefault("y", 0.0)
    sonar.setdefault("z", 0.0)
    for k in ("roll", "pitch", "yaw"):
        sonar.setdefault(k, 0.0)
    sonar.setdefault("frame", DEFAULT_SONAR_FRAME)

    imu = dict(geo.get("imu") or {})
    for k in ("x", "y", "z", "roll", "pitch", "yaw"):
        imu.setdefault(k, 0.0)

    mesh = dict(geo.get("mesh") or {})
    mesh.setdefault("base", "")
    mesh.setdefault("wheel", "")

    return {"body": body, "wheel": wheel, "casters": casters, "laser": laser,
            "sonar": sonar, "imu": imu, "mesh": mesh}


def geometry_warnings(params: Dict[str, Any]) -> List[str]:
    """Things a description can get wrong that ROS will not report."""
    out = []
    kine = params.get("kinematics") or {}
    base = str(kine.get("base_type", "2wd")).lower()
    wheel_d = _f(kine.get("wheel_diameter"), 0.0)
    lr = _f(kine.get("lr_wheels_distance"), 0.0)
    fr = _f(kine.get("fr_wheels_distance"), 0.0)
    if wheel_d <= 0 or lr <= 0:
        out.append("kinematics.wheel_diameter and lr_wheels_distance must be positive; the description is empty without them")
        return out
    geo = effective_geometry(params)
    if base in ("4wd", "mecanum") and fr <= 0:
        out.append(f"{base} base but kinematics.fr_wheels_distance is 0: all four wheels sit on one axle in the description")
    if _f(geo["wheel"]["width"]) >= lr:
        out.append("geometry.wheel.width is not smaller than lr_wheels_distance; the wheels overlap")
    if _f(geo["laser"]["z"]) <= _f(geo["body"]["height"]) / 2.0 and _f(geo["laser"]["z"]) >= -_f(geo["body"]["height"]) / 2.0:
        out.append("geometry.laser.z is inside the body box; a scan from there sees the chassis")
    # Nav2 plans with a circle; if the body's corners stick out of it the robot
    # clips obstacles the costmap said it would clear.
    half_diag = math.hypot(_f(geo["body"]["length"]) / 2.0, max(_f(geo["body"]["width"]) / 2.0, lr / 2.0 + _f(geo["wheel"]["width"]) / 2.0))
    nav2 = params.get("nav2") or {}
    for cm in ("local_costmap", "global_costmap"):
        node = nav2.get(cm) or {}
        node = node.get(cm) or node          # nav2 keys are <name>: <name>: ros__parameters
        rp = node.get("ros__parameters", node) or {}
        r = rp.get("robot_radius")
        if r is not None and _f(r) < half_diag:
            out.append(f"nav2 {cm}.robot_radius {r} is smaller than the body's half diagonal {half_diag:.3f} m")
    return out


# --- URDF construction --------------------------------------------------------

def _origin(parent, xyz=(0, 0, 0), rpy=(0, 0, 0)):
    ET.SubElement(parent, "origin", xyz=" ".join(f"{_f(v):.5g}" for v in xyz),
                  rpy=" ".join(f"{_f(v):.5g}" for v in rpy))


def _inertial(parent, mass, ixx, iyy, izz):
    inertial = ET.SubElement(parent, "inertial")
    _origin(inertial)
    ET.SubElement(inertial, "mass", value=f"{mass:.5g}")
    ET.SubElement(inertial, "inertia", ixx=f"{ixx:.5g}", ixy="0", ixz="0",
                  iyy=f"{iyy:.5g}", iyz="0", izz=f"{izz:.5g}")


def _material(parent, name, rgba):
    m = ET.SubElement(parent, "material", name=name)
    ET.SubElement(m, "color", rgba=rgba)


def _box_link(robot, name, length, width, height, mass, mesh, rgba):
    link = ET.SubElement(robot, "link", name=name)
    visual = ET.SubElement(link, "visual")
    _origin(visual)
    geom = ET.SubElement(visual, "geometry")
    if mesh:
        ET.SubElement(geom, "mesh", filename=mesh, scale="1 1 1")
    else:
        ET.SubElement(geom, "box", size=f"{length:.5g} {width:.5g} {height:.5g}")
    _material(visual, "body", rgba)
    collision = ET.SubElement(link, "collision")
    _origin(collision)
    ET.SubElement(ET.SubElement(collision, "geometry"), "box", size=f"{length:.5g} {width:.5g} {height:.5g}")
    _inertial(link, mass,
              mass / 12.0 * (width ** 2 + height ** 2),
              mass / 12.0 * (length ** 2 + height ** 2),
              mass / 12.0 * (length ** 2 + width ** 2))
    return link


def _wheel_link(robot, name, radius, width, mass, mesh, rgba):
    link = ET.SubElement(robot, "link", name=name)
    visual = ET.SubElement(link, "visual")
    _origin(visual, rpy=(math.pi / 2, 0, 0))
    geom = ET.SubElement(visual, "geometry")
    if mesh:
        ET.SubElement(geom, "mesh", filename=mesh, scale="1 1 1")
    else:
        ET.SubElement(geom, "cylinder", radius=f"{radius:.5g}", length=f"{width:.5g}")
    _material(visual, "wheel", rgba)
    collision = ET.SubElement(link, "collision")
    _origin(collision, rpy=(math.pi / 2, 0, 0))
    ET.SubElement(ET.SubElement(collision, "geometry"), "cylinder", radius=f"{radius:.5g}", length=f"{width:.5g}")
    i = 0.25 * mass * radius ** 2 + mass * width ** 2 / 12.0
    _inertial(link, mass, i, 0.5 * mass * radius ** 2, i)
    return link


def _sphere_link(robot, name, radius, mass, rgba):
    link = ET.SubElement(robot, "link", name=name)
    visual = ET.SubElement(link, "visual")
    _origin(visual)
    ET.SubElement(ET.SubElement(visual, "geometry"), "sphere", radius=f"{radius:.5g}")
    _material(visual, "caster", rgba)
    collision = ET.SubElement(link, "collision")
    _origin(collision)
    ET.SubElement(ET.SubElement(collision, "geometry"), "sphere", radius=f"{radius:.5g}")
    i = 0.4 * mass * radius ** 2
    _inertial(link, mass, i, i, i)
    return link


def _joint(robot, name, jtype, parent, child, xyz, rpy=(0, 0, 0), axis=None):
    j = ET.SubElement(robot, "joint", name=name, type=jtype)
    ET.SubElement(j, "parent", link=parent)
    ET.SubElement(j, "child", link=child)
    _origin(j, xyz, rpy)
    if axis:
        ET.SubElement(j, "axis", xyz=axis)
    return j


def wheel_positions(params: Dict[str, Any]) -> List[Tuple[str, float, float]]:
    """(joint prefix, x, y) of every driven wheel, from the kinematics."""
    kine = params.get("kinematics") or {}
    base = str(kine.get("base_type", "2wd")).lower()
    half_y = _f(kine.get("lr_wheels_distance")) / 2.0
    half_x = _f(kine.get("fr_wheels_distance")) / 2.0
    if base == "2wd":
        return [("left", 0.0, half_y), ("right", 0.0, -half_y)]
    return [("front_left", half_x, half_y), ("front_right", half_x, -half_y),
            ("rear_left", -half_x, half_y), ("rear_right", -half_x, -half_y)]


def build_urdf(params: Dict[str, Any], robot_name: str = None) -> str:
    kine = params.get("kinematics") or {}
    geo = effective_geometry(params)
    body, wheel, casters, laser, sonar, imu, mesh = (
        geo[k] for k in ("body", "wheel", "casters", "laser", "sonar", "imu", "mesh"))
    name = robot_name or (params.get("robot") or {}).get("name") or "linorobot2"

    radius = _f(kine.get("wheel_diameter")) / 2.0
    wheel_z = _f(wheel["z"])
    ground_clearance = radius - wheel_z        # base_link origin above the floor

    robot = ET.Element("robot", name=str(name))
    robot.append(ET.Comment(
        f" Generated by scripts/gen_robot_description.py from {name}_config.yaml. "
        "Edit the config (kinematics, geometry), not this file. "))

    ET.SubElement(robot, "link", name=FOOTPRINT_FRAME)
    _box_link(robot, BASE_FRAME, _f(body["length"]), _f(body["width"]), _f(body["height"]),
              _f(body["mass"]), mesh.get("base") or "", "0.88 0.66 0.66 1.0")
    # base_footprint hangs BELOW base_link, never above it. The EKF publishes
    # odom -> base_link, so base_link's one and only parent is odom; a URDF that
    # made base_footprint the parent gave base_link two parents, tf2 kept the
    # dynamic one, base_footprint became an orphan root, and robot_localization
    # could not transform the firmware's odom twist (child frame base_footprint)
    # into base_link -- 7447 times in one run, silently, on the GenDrv: the EKF
    # never moved while the base drove 4 m, and every Nav2 goal "failed to make
    # progress". The costmaps and SLAM use base_link too, so the ground contact
    # frame is a leaf: base_link -> base_footprint, ground_clearance straight down.
    _joint(robot, "base_to_footprint", "fixed", BASE_FRAME, FOOTPRINT_FRAME, (0, 0, -ground_clearance))

    for prefix, x, y in wheel_positions(params):
        link = f"{prefix}_wheel_link"
        _wheel_link(robot, link, radius, _f(wheel["width"]), _f(wheel["mass"]),
                    mesh.get("wheel") or "", "0.18 0.46 0.85 1.0")
        _joint(robot, f"{prefix}_wheel_joint", "continuous", BASE_FRAME, link, (x, y, wheel_z), axis="0 1 0")

    if str(kine.get("base_type", "2wd")).lower() == "2wd":
        caster_r = ground_clearance / 2.0
        for side, on in (("front", casters.get("front")), ("rear", casters.get("rear"))):
            if not on:
                continue
            x = _f(body["length"]) / 2.0 - caster_r
            if side == "rear":
                x = -x
            link = f"{side}_caster_wheel_link"
            _sphere_link(robot, link, caster_r, 0.05 * _f(wheel["mass"]) + 1e-3, "0 0 0 1.0")
            _joint(robot, f"{side}_caster_wheel_joint", "fixed", BASE_FRAME, link, (x, 0, -caster_r))

    ET.SubElement(robot, "link", name=IMU_FRAME)
    _joint(robot, "imu_to_base_link", "fixed", BASE_FRAME, IMU_FRAME,
           (imu["x"], imu["y"], imu["z"]), (imu["roll"], imu["pitch"], imu["yaw"]))

    laser_frame = str(laser.get("frame") or DEFAULT_LASER_FRAME)
    ET.SubElement(robot, "link", name=laser_frame)
    _joint(robot, f"{laser_frame}_to_base_link", "fixed", BASE_FRAME, laser_frame,
           (laser["x"], laser["y"], laser["z"]), (laser["roll"], laser["pitch"], laser["yaw"]))

    # The sonar frame, whether or not a sonar is fitted. It costs one static
    # transform and it means a board that IS publishing Range is never dropped
    # for want of a frame -- and the fake sonar is on by default
    # (mcu_env.py: use_fake_sonar defaults true), so that is most boards.
    sonar_frame = str(sonar.get("frame") or DEFAULT_SONAR_FRAME)
    ET.SubElement(robot, "link", name=sonar_frame)
    _joint(robot, f"{sonar_frame}_to_base_link", "fixed", BASE_FRAME, sonar_frame,
           (sonar["x"], sonar["y"], sonar["z"]), (sonar["roll"], sonar["pitch"], sonar["yaw"]))

    ET.indent(robot, space="  ")
    return '<?xml version="1.0"?>\n' + ET.tostring(robot, encoding="unicode") + "\n"


def default_out_path(params: Dict[str, Any], params_path: str = None, directory: str = None) -> str:
    name = (params.get("robot") or {}).get("name")
    if not name and params_path:
        name = os.path.basename(params_path).replace("_config.yaml", "").replace(".yaml", "")
    return os.path.join(cockpit_paths.generated_dir(directory), f"{name or 'linorobot2'}.urdf")


def write_urdf(params: Dict[str, Any], out_path: str, robot_name: str = None) -> str:
    text = build_urdf(params, robot_name)
    os.makedirs(os.path.dirname(os.path.abspath(out_path)), exist_ok=True)
    tmp = out_path + ".tmp"
    with open(tmp, "w") as fh:
        fh.write(text)
    os.replace(tmp, out_path)
    return out_path


def load_yaml(path: str) -> Dict[str, Any]:
    with open(path) as fh:
        return yaml.safe_load(fh) or {}


def main() -> int:
    ap = argparse.ArgumentParser(description="Generate the robot's URDF from its config")
    ap.add_argument("--params", default=None, help="<config dir>/<robot>_config.yaml (default: the default robot)")
    ap.add_argument("--out", default=None, help="output file (default: <config dir>/generated/<robot>.urdf)")
    ap.add_argument("--print", action="store_true", help="write to stdout instead of a file")
    a = ap.parse_args()

    params_path = a.params or cockpit_paths.robot_config_path()
    if not os.path.isfile(params_path):
        print(f"Error: no such config: {params_path}", file=sys.stderr)
        return 2
    params = load_yaml(params_path)
    for w in geometry_warnings(params):
        print(f"[geometry] warn: {w}", file=sys.stderr)
    if a.print:
        sys.stdout.write(build_urdf(params))
        return 0
    out = write_urdf(params, a.out or default_out_path(params, params_path))
    print(f"✅ Generated robot description: {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
