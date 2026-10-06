"""Applying a reference design to the user's robot (user, 2026-10-06).

The user names their robot; a design is a template applied to it. The design brings the
controller side; the robot keeps its own chassis and tuning ("user may have chassis, just want
to try different controller"). A robot still on the untouched default chassis has none to
keep and takes the design's. A kit (robot.kit) brings its own motors, wheels and chassis --
"but user can override with his motor/wheel/chassis" afterwards.
"""
import copy
import glob
import os
import sys

import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "scripts"))

import gen_bare_config  # noqa: E402
import robot_templates as rt  # noqa: E402


def design(name):
    return yaml.safe_load(open(os.path.join(ROOT, "config", "reference", f"{name}_config.yaml")))


def designs():
    return sorted(os.path.basename(p)[:-len("_config.yaml")]
                  for p in glob.glob(os.path.join(ROOT, "config", "reference", "*_config.yaml")))


def fresh(name="my_rover", mcu="pico2"):
    return rt.apply_bare(name, mcu)


def my_chassis(robot):
    r = copy.deepcopy(robot)
    r["kinematics"]["wheel_diameter"] = 0.0815
    r["kinematics"]["lr_wheels_distance"] = 0.33
    r["kinematics"]["motor_max_rpm"] = 210
    return r


def test_a_fresh_robot_is_a_bare_module_on_the_default_chassis():
    r = fresh()
    assert r["robot"]["name"] == "my_rover" and "reference" not in r["robot"]
    assert not rt.custom_chassis(r)
    assert all(v for k, v in r["base_controller"]["sensors"].items() if k.startswith("use_sim_"))


def test_every_design_applies_under_the_robots_own_name_and_records_itself():
    for d in designs():
        out, kept, kit = rt.apply_design(fresh(), design(d), d)
        assert out["robot"]["name"] == "my_rover" and out["robot"]["reference"] == d
        assert out["base_controller"] == design(d)["base_controller"], d   # the controller side
        assert not any(v for k, v in (out["base_controller"].get("sensors") or {}).items()
                       if k.startswith("use_sim_")), f"{d}: a real robot simulates nothing"


def test_the_users_chassis_is_kept_when_trying_another_controller():
    mine = my_chassis(fresh())
    for d in designs():
        if design(d)["robot"].get("kit"):
            continue
        out, kept, kit = rt.apply_design(mine, design(d), d)
        assert kept and not kit, d
        for block in rt.CHASSIS_BLOCKS:
            assert out.get(block) == mine.get(block), f"{d}: {block} not kept"


def test_a_fresh_robot_takes_the_designs_chassis():
    for d in designs():
        out, kept, kit = rt.apply_design(fresh(), design(d), d)
        assert not kept, d
        assert out["kinematics"] == design(d)["kinematics"], d


def test_a_kit_brings_its_own_chassis_over_the_users():
    for d in designs():
        if not design(d)["robot"].get("kit"):
            continue
        out, kept, kit = rt.apply_design(my_chassis(fresh()), design(d), d)
        assert kit and not kept, d
        assert out["kinematics"] == design(d)["kinematics"], d


def test_switching_design_keeps_the_chassis_the_last_one_brought():
    """XRP kit, then the user tries a Pico 2 board on that chassis: the XRP's wheels stay."""
    xrp, _, _ = rt.apply_design(fresh(), design("xrp"), "xrp") if "xrp" in designs() else (None, 0, 0)
    if xrp is None:
        return
    nxt = [d for d in designs() if not design(d)["robot"].get("kit")][0]
    out, kept, _ = rt.apply_design(xrp, design(nxt), nxt)
    assert kept and out["kinematics"] == xrp["kinematics"]


def test_back_to_no_design_is_a_bare_module_again():
    out, _, _ = rt.apply_design(fresh(), design(designs()[0]), designs()[0])
    bare = rt.apply_bare(out["robot"]["name"], "esp32")
    assert "reference" not in bare["robot"] and bare["base_controller"]["mcu"] == "esp32"
    assert bare["kinematics"] == gen_bare_config.bare_config("esp32")["kinematics"]
