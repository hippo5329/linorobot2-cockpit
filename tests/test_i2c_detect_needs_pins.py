"""i2c_detect runs on a real robot by default, unless its config assigns no I2C pins
(user, 2026-10-07). The host refuses before the write; the firmware's toolSelect
refuses an env written any other way. A bus fixed in hardware is named (pins.i2c.bus:
the UNO Q's Qwiic). A bare module has none: it runs in simulation mode first, and the
robot's reference design brings the I2C and motor pins."""
import os

import mcu_env

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def test_a_robot_with_i2c_pins_may_scan_its_bus():
    assert mcu_env.tool_refusal({"app": "i2c_detect", "i2c_sda": 21, "i2c_scl": 22}) is None


def test_no_i2c_pins_is_refused():
    for env in ({"app": "i2c_detect"},
                {"app": "i2c_detect", "i2c_sda": -1, "i2c_scl": -1},
                {"app": "i2c_detect", "i2c_sda": 21, "i2c_scl": -1}):
        why = mcu_env.tool_refusal(env)
        assert why and "assigns no I2C pins" in why, env


def test_a_named_bus_counts_and_the_uno_q_design_names_its_qwiic():
    assert mcu_env.tool_refusal({"app": "i2c_detect", "i2c_sda": -1, "i2c_scl": -1,
                                 "i2c_bus": "qwiic"}) is None
    design = mcu_env.load_yaml(os.path.join(ROOT, "config", "reference", "unoq_config.yaml"))
    i2c = design["base_controller"]["pins"]["i2c"]
    assert i2c.get("bus") == "qwiic"
    assert mcu_env.hardware_env(design).get("i2c_bus") == "qwiic"


def test_the_firmware_applies_the_same_rule():
    src = open(os.path.join(ROOT, "firmware", "src", "tools", "tools.cpp")).read()
    assert "i2cUnassigned(TOOLS[i].mode)" in src
    assert 'envGet("i2c_bus", "")' in src
    assert 'envInt("i2c_sda", -1) < 0 || envInt("i2c_scl", -1) < 0' in src


def test_applying_a_reference_design_brings_its_i2c():
    """A bare module runs in simulation mode first; the reference design is what brings
    the I2C bus. Every design assigns it -- pins or a named bus -- except the makerspet
    mini, which breaks no I2C out (user, 2026-10-07)."""
    import glob
    no_bus = {"makerspet_mini"}
    for path in glob.glob(os.path.join(ROOT, "config", "reference", "*_config.yaml")):
        name = os.path.basename(path)[:-len("_config.yaml")]
        env = dict(mcu_env.hardware_env(mcu_env.load_yaml(path)), app="i2c_detect")
        refused = mcu_env.tool_refusal(env)
        assert bool(refused) == (name in no_bus), (name, refused)
