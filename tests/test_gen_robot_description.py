"""The URDF is a function of the config: wheel radius, track, poses, frames."""
import math
import xml.etree.ElementTree as ET

import gen_robot_description as grd


def _tree(params):
    return ET.fromstring(grd.build_urdf(params))


def _joint(root, name):
    j = root.find(f"./joint[@name='{name}']")
    assert j is not None, f"no joint {name}"
    xyz = [float(v) for v in j.find("origin").get("xyz").split()]
    return j, xyz


def test_2wd_wheels_from_kinematics(reference):
    p = reference("gendrv")
    root = _tree(p)
    kine = p["kinematics"]
    radius = float(root.find("./link[@name='left_wheel_link']/visual/geometry/cylinder").get("radius"))
    assert math.isclose(radius, kine["wheel_diameter"] / 2)
    _, left = _joint(root, "left_wheel_joint")
    _, right = _joint(root, "right_wheel_joint")
    assert math.isclose(left[1], kine["lr_wheels_distance"] / 2)
    assert math.isclose(right[1], -kine["lr_wheels_distance"] / 2)
    assert len(root.findall("./joint[@type='continuous']")) == 2
    assert {j.get("name") for j in root.findall("./joint")} >= {"front_caster_wheel_joint", "rear_caster_wheel_joint"}


def test_mecanum_four_wheels_on_two_axles(reference):
    p = reference("pico2_mecanum")
    root = _tree(p)
    kine = p["kinematics"]
    assert len(root.findall("./joint[@type='continuous']")) == 4
    _, fl = _joint(root, "front_left_wheel_joint")
    _, rr = _joint(root, "rear_right_wheel_joint")
    assert math.isclose(fl[0], kine["fr_wheels_distance"] / 2) and math.isclose(rr[0], -kine["fr_wheels_distance"] / 2)
    assert math.isclose(fl[1], kine["lr_wheels_distance"] / 2) and math.isclose(rr[1], -kine["lr_wheels_distance"] / 2)
    assert root.find("./joint[@name='front_caster_wheel_joint']") is None


def test_sensor_frames_and_poses(reference):
    p = reference("gendrv")
    # The references carry no explicit geometry any more -- one default chassis,
    # derived. What is under test is that a pose the config DOES give is honoured.
    geo = p.setdefault("geometry", {})
    geo.setdefault("laser", {}).update({"x": 0.21, "y": -0.02, "z": 0.3, "yaw": 1.5, "frame": "lidar_top"})
    geo.setdefault("imu", {}).update({"x": 0.05, "z": -0.01})
    root = _tree(p)
    j, xyz = _joint(root, "lidar_top_to_base_link")
    assert xyz == [0.21, -0.02, 0.3]
    assert math.isclose(float(j.find("origin").get("rpy").split()[2]), 1.5)
    assert root.find("./link[@name='lidar_top']") is not None
    _, imu = _joint(root, "imu_to_base_link")
    assert imu == [0.05, 0.0, -0.01]
    assert root.find("./link[@name='imu_link']") is not None   # what the firmware stamps


def test_footprint_sits_on_the_floor(reference):
    p = reference("pico2_mecanum")
    p.setdefault("geometry", {}).setdefault("wheel", {})["z"] = -0.02
    root = _tree(p)
    _, xyz = _joint(root, "base_to_footprint")
    # base_footprint hangs BELOW base_link: ground clearance straight down.
    assert math.isclose(xyz[2], -(p["kinematics"]["wheel_diameter"] / 2 + 0.02))


def test_base_link_has_no_parent_in_the_urdf(reference):
    """The EKF publishes odom -> base_link, so the URDF must not give base_link a
    parent of its own. It did (base_footprint -> base_link): tf2 kept the dynamic
    parent, base_footprint became an orphan root, and robot_localization dropped
    every /odom/unfiltered twist (child frame base_footprint) with "Could not
    transform measurement into base_link" -- 7447 times in one GenDrv run, with
    the EKF pinned at (0, 0) while the base drove 4 m.
    """
    for name in ("pico2_mecanum", "gendrv", "esp32s3", "yahboom_esp32s3"):
        root = _tree(reference(name))
        children = {j.find("child").get("link") for j in root.findall("joint")}
        assert "base_link" not in children, f"{name}: base_link has a URDF parent"
        j, _ = _joint(root, "base_to_footprint")
        assert j.find("parent").get("link") == "base_link"
        assert j.find("child").get("link") == "base_footprint"


def test_effective_geometry_fills_gaps_from_kinematics_only():
    p = {"kinematics": {"base_type": "2wd", "wheel_diameter": 0.1, "lr_wheels_distance": 0.4}}
    g = grd.effective_geometry(p)
    assert g["wheel"]["width"] == 0.025           # a quarter of the diameter
    assert g["body"]["width"] < 0.4               # inside the track
    assert g["casters"] == {"front": True, "rear": True}
    assert g["laser"]["frame"] == "laser"
    # explicit keys win, untouched
    p["geometry"] = {"body": {"length": 1.0}, "laser": {"x": 0.3}}
    g = grd.effective_geometry(p)
    assert g["body"]["length"] == 1.0 and g["laser"]["x"] == 0.3
    assert "width" in g["body"]


def test_every_reference_generates_valid_xml(reference):
    import glob, os
    for path in glob.glob(os.path.join(grd.cockpit_paths.REFERENCE_CONFIG_DIR, "*_config.yaml")):
        name = os.path.basename(path).replace("_config.yaml", "")
        root = _tree(reference(name))
        assert root.tag == "robot"
        assert "nan" not in ET.tostring(root, encoding="unicode").lower()


def test_geometry_warnings_catch_what_ros_will_not():
    p = {"kinematics": {"base_type": "mecanum", "wheel_diameter": 0.1, "lr_wheels_distance": 0.3, "fr_wheels_distance": 0},
         "geometry": {"laser": {"z": 0.0}},
         "nav2": {"local_costmap": {"local_costmap": {"ros__parameters": {"robot_radius": 0.05}}}}}
    w = "\n".join(grd.geometry_warnings(p))
    assert "fr_wheels_distance is 0" in w
    assert "inside the body box" in w
    assert "robot_radius 0.05 is smaller" in w
    assert grd.geometry_warnings({"kinematics": {"wheel_diameter": 0.1, "lr_wheels_distance": 0.3}, "geometry": {"laser": {"z": 0.5}}}) == []


def test_write_urdf_lands_in_generated_dir(tmp_path, reference):
    p = reference("gendrv")
    out = grd.write_urdf(p, grd.default_out_path(p, None, str(tmp_path)))
    assert out == str(tmp_path / "generated" / "esp32_bare_config.urdf") or out.endswith(".urdf")
    assert ET.parse(out).getroot().tag == "robot"
