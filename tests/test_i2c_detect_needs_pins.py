"""The tool's `i2c` scan runs on a real robot by default, unless its config assigns no I2C
pins (user, 2026-10-07). The command refuses it then (tools.cpp i2cUnassigned), whatever
else is simulated. A bus fixed in hardware is named (pins.i2c.bus: the UNO Q's Qwiic). A
bare module has none: it runs in simulation mode first, and the robot's reference design
brings the I2C and motor pins."""
import os

import mcu_env

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC = open(os.path.join(ROOT, "firmware", "src", "tools", "tools.cpp")).read()


def _unassigned(env: dict) -> bool:
    """The firmware's rule (toolI2cUnassigned), on the env the host writes."""
    if env.get("i2c_bus"):
        return False

    def pin(k):
        try:
            return int(env.get(k, -1)) >= 0
        except (TypeError, ValueError):
            return False
    return not (pin("i2c_sda") and pin("i2c_scl"))


def test_the_firmware_rule_is_the_one_mirrored_here():
    fn = SRC[SRC.index("bool toolI2cUnassigned(void)"):SRC.index("static bool i2cUnassigned")]
    assert 'envGet("i2c_bus", "")' in fn
    assert 'envInt("i2c_sda", -1) < 0 || envInt("i2c_scl", -1) < 0' in fn
    i2c = SRC[SRC.index('if (!strcmp(cmd, "i2c")) {'):]
    assert i2c.index("if (i2cUnassigned())") < i2c.index("runBlocking(\"i2c\"")
    assert "assigns no" in i2c[:400]


def test_a_robot_with_i2c_pins_may_scan_its_bus():
    assert not _unassigned({"i2c_sda": 21, "i2c_scl": 22})


def test_no_i2c_pins_is_refused():
    for env in ({}, {"i2c_sda": -1, "i2c_scl": -1}, {"i2c_sda": 21, "i2c_scl": -1}):
        assert _unassigned(env), env


def test_a_named_bus_counts_and_the_uno_q_design_names_its_qwiic():
    assert not _unassigned({"i2c_sda": -1, "i2c_scl": -1, "i2c_bus": "qwiic"})
    design = mcu_env.load_yaml(os.path.join(ROOT, "config", "reference", "unoq_config.yaml"))
    i2c = design["base_controller"]["pins"]["i2c"]
    assert i2c.get("bus") == "qwiic"
    assert mcu_env.hardware_env(design).get("i2c_bus") == "qwiic"


def test_applying_a_reference_design_brings_its_i2c():
    """A bare module runs in simulation mode first; the reference design is what brings
    the I2C bus. Every design assigns it -- pins or a named bus -- except the makerspet
    mini, which breaks no I2C out (user, 2026-10-07)."""
    import glob
    no_bus = {"makerspet_mini"}
    for path in glob.glob(os.path.join(ROOT, "config", "reference", "*_config.yaml")):
        name = os.path.basename(path)[:-len("_config.yaml")]
        env = mcu_env.hardware_env(mcu_env.load_yaml(path))
        assert _unassigned(env) == (name in no_bus), name
