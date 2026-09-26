"""Where Nav2 itself has an answer, the stack takes Nav2's.

Upstream Nav2 (nav2_bringup, as installed per distro) is the reference, not the
older linorobot2 configs this template grew from. Three places differed:

  * composition: nav2_bringup's bringup_launch.py composes on jazzy and
    lyrical; this stack composed on lyrical only.
  * inflation: nav2_params.yaml inflates 0.70 m; the template had 0.55.
  * a mecanum base: Nav2's controller is MPPI, which has an omni model; the
    template's pure pursuit cannot command vy.
"""
import copy
import os

import yaml

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def read(*p):
    return open(os.path.join(REPO_ROOT, *p)).read()


def _launcher_fn(name):
    src = read("launchers", "nav2.launch.py")
    a = src.index(f"def {name}(")
    b = src.index("\ndef ", a + 10)
    ns = {"os": os, "yaml": yaml, "copy": copy, "FindPackageShare": None}
    exec(src[a:b], ns)
    return ns[name]


JAZZY_MPPI = {"plugin": "nav2_mppi_controller::MPPIController", "vx_max": 0.5, "vy_max": 0.5,
              "wz_max": 1.9, "transform_tolerance": 0.1, "motion_model": "DiffDrive",
              "critics": ["ConstraintCritic", "CostCritic"], "CostCritic": {"enabled": True}}
LYRICAL_MPPI = {"plugin": "nav2_mppi_controller::MPPIController", "vx_max": 0.5, "vy_max": 0.5,
                "wz_max": 1.9, "motion_model": "diff_drive",
                "diff_drive": {"plugin": "mppi::DiffDriveMotionModel"},
                "critics": ["ConstraintCritic", "CostCritic"]}
TEMPLATE_FP = {"plugin": "nav2_rotation_shim_controller::RotationShimController",
               "desired_linear_vel": 0.24, "transform_tolerance": 0.3}


def _share(tmp_path, follow_path):
    share = tmp_path / "nav2_bringup"
    (share / "params").mkdir(parents=True)
    (share / "params" / "nav2_params.yaml").write_text(yaml.safe_dump(
        {"controller_server": {"ros__parameters": {"FollowPath": follow_path}}}))
    return str(share)


def _nav2(fp=None):
    return {"controller_server": {"ros__parameters": {"FollowPath": dict(fp or TEMPLATE_FP)}},
            "velocity_smoother": {"ros__parameters": {"max_velocity": [0.29, 0.29, 0.69],
                                                      "max_accel": [0.8, 0.8, 0.87]}}}


def test_jazzy_mecanum_gets_mppi_omni(tmp_path):
    fn = _launcher_fn("mecanum_controller")
    nav2 = _nav2()
    assert fn(nav2, {"kinematics": {"base_type": "mecanum"}}, _share(tmp_path, JAZZY_MPPI))
    fp = nav2["controller_server"]["ros__parameters"]["FollowPath"]
    assert fp["plugin"] == "nav2_mppi_controller::MPPIController" and fp["motion_model"] == "Omni"
    assert fp["vx_max"] == 0.24 and fp["vy_max"] == 0.24 and fp["wz_max"] == 0.69
    assert fp["ax_max"] == 0.8 and fp["az_max"] == 0.87
    assert fp["transform_tolerance"] == 0.3
    assert fp["CostCritic"] == {"enabled": True}, "Nav2's critics stand"


def test_lyrical_mecanum_loads_the_omni_plugin(tmp_path):
    fn = _launcher_fn("mecanum_controller")
    nav2 = _nav2()
    assert fn(nav2, {"kinematics": {"base_type": "mecanum"}}, _share(tmp_path, LYRICAL_MPPI))
    fp = nav2["controller_server"]["ros__parameters"]["FollowPath"]
    assert fp["motion_model"] == "omni" and fp["omni"] == {"plugin": "mppi::OmniMotionModel"}
    assert "diff_drive" not in fp
    assert "transform_tolerance" not in fp, "lyrical's MPPI does not read it"


def test_a_differential_base_keeps_the_template(tmp_path):
    fn = _launcher_fn("mecanum_controller")
    for base in ("2wd", "4wd"):
        nav2 = _nav2()
        assert not fn(nav2, {"kinematics": {"base_type": base}}, _share(tmp_path / base, JAZZY_MPPI))
        assert nav2["controller_server"]["ros__parameters"]["FollowPath"] == TEMPLATE_FP


def test_a_robots_own_controller_is_left_alone(tmp_path):
    fn = _launcher_fn("mecanum_controller")
    own = {"plugin": "dwb_core::DWBLocalPlanner"}
    nav2 = _nav2(own)
    assert not fn(nav2, {"kinematics": {"base_type": "mecanum"}}, _share(tmp_path, JAZZY_MPPI))
    assert nav2["controller_server"]["ros__parameters"]["FollowPath"] == own


def test_composed_on_every_distro():
    src = read("launchers", "nav2.launch.py")
    assert '    if _nav_launch_src:\n        # Compose the stack into one container process, on every distro.' in src
    assert "nav2_container = None if not _nav_launch_src else Node(" in src
    assert "uses_mppi = mecanum_controller(nav2_data, params)" in src
