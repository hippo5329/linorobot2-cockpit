"""A bare module has no pin assigned but the LED; once one is, it is real (user, 2026-10-06).

"a bare module has no pin assigned, except led pins. a bare module can run sim. once a pin is
assigned, it is no longer a bare module, it can not run sim." The one other exception is the
pin that carries a SIMULATED LiDAR out (the GenDrv's LIDAR_RXD with sim_ld19 on): it is the
simulation's own output, not wiring to a real part.
"""
import copy
import os
import sys

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "scripts"))

import gen_bare_config  # noqa: E402
import one_click_pipeline as ocp  # noqa: E402
import pin_catalog  # noqa: E402


@pytest.mark.parametrize("mcu", sorted(gen_bare_config.KNOWN))
def test_every_bare_module_assigns_no_pin_but_the_led(mcu):
    cfg = gen_bare_config.bare_config(mcu)
    assert pin_catalog.assigned_pins(cfg) == [], mcu
    assert ocp.is_bare_config(cfg)


@pytest.mark.parametrize("path, value", [
    (("motor1", "in_a"), 9), (("encoder2", "pin_a"), 34), (("i2c", "sda"), 21),
    (("battery", "pin"), 36), (("sonar", "trigger"), 5),
])
def test_one_pin_and_it_is_no_longer_bare(path, value):
    cfg = gen_bare_config.bare_config("esp32")
    cfg["robot"]["name"] = "my_rover"
    cfg["base_controller"]["pins"][path[0]][path[1]] = value
    assert pin_catalog.assigned_pins(cfg)
    assert not ocp.is_bare_config(cfg)
    assert ocp.robot_kind("my_rover", None, ocp.is_bare_config(cfg)) == "real"
    assert ocp.is_real_robot(cfg)          # Sim refused, its board needed


def test_the_led_is_not_a_pin_that_counts():
    cfg = gen_bare_config.bare_config("esp32")
    cfg["base_controller"]["pins"]["led"] = 13
    assert ocp.is_bare_config(cfg)


def test_a_simulated_lidars_output_pin_is_allowed_a_real_lidars_is_not():
    cfg = gen_bare_config.bare_config("esp32")
    bc = cfg["base_controller"]
    bc["lidar"].update(comm_mode="serial", rx_pin=4)
    bc["sensors"]["use_sim_ld19"] = True
    assert ocp.is_bare_config(cfg)          # the GenDrv emits its simulated LD19 out GPIO 4
    real = copy.deepcopy(cfg)
    real["base_controller"]["sensors"]["use_sim_ld19"] = False
    assert not ocp.is_bare_config(real)


def test_a_named_robot_with_no_pins_is_a_simulated_one():
    cfg = gen_bare_config.bare_config("pico2")
    cfg["robot"]["name"] = "my_rover"
    assert ocp.robot_kind("my_rover", None, True) == "sim"
    assert not ocp.is_real_robot(cfg)


def test_the_robots_list_reports_the_pins():
    core = open(os.path.join(ROOT, "web", "backend", "core.py")).read()
    assert "one_click_pipeline.is_bare_config(yd)" in core
    assert "one_click_pipeline.robot_kind(r_name, reference, bare)" in core


def test_a_bare_module_may_have_any_chassis_and_still_run_sim():
    """User, 2026-10-06: with no pin assigned, any motor/wheel/drivetrain/battery parameter can be
    changed and the robot still runs sim. Only pins decide; the chassis never does."""
    cfg = gen_bare_config.bare_config("esp32")
    cfg["robot"]["name"] = "my_rover"
    k = cfg["kinematics"]
    k.update(base_type="mecanum", wheel_diameter=0.0815, lr_wheels_distance=0.33,
             fr_wheels_distance=0.25, motor_max_rpm=210, counts_per_rev=1440,
             motor_operating_voltage=7.4, motor_power_max_voltage=8.4)
    bat = cfg["base_controller"]["pins"]["battery"]
    bat.update(r1=47000, r2=10000)                      # the divider is not a pin
    cfg["base_controller"].setdefault("battery", {}).update(cells=2, chemistry="lipo")
    assert ocp.is_bare_config(cfg)
    assert ocp.robot_kind("my_rover", None, ocp.is_bare_config(cfg)) == "sim"
    assert not ocp.is_real_robot(cfg)
    bat["pin"] = 36                                     # the battery's SENSE pin is a pin
    assert not ocp.is_bare_config(cfg)
