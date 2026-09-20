"""The wiring chart says what the config says, in the form a bench wants.

Two tables, because two questions get asked at the bench: "where does this
wire go?" (function -> GPIO) and "what is already on this pin?" (GPIO ->
functions). The second is what makes a SHARED pin readable: the two-PWM bridge
scheme puts one enable on four motors, which is correct and looks like a
collision until it is shown as one row.
"""
import copy
import os
import sys

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO_ROOT, "scripts"))

import gen_wiring_table  # noqa: E402
import yaml  # noqa: E402
from gen_bare_config import bare_config  # noqa: E402

MECANUM = os.path.join(REPO_ROOT, "config", "reference", "pico2_mecanum_config.yaml")


def test_a_bare_module_says_so_instead_of_printing_an_empty_table():
    md = gen_wiring_table.render(bare_config("pico2"))
    assert "bare module" in md
    assert "| Function |" not in md, "an empty table is not a wiring chart"


def test_every_wired_pin_appears_once_by_function():
    params = yaml.safe_load(open(MECANUM))
    md = gen_wiring_table.render(params)
    by_function = md[md.index("## By function"):md.index("## By GPIO")]
    for fn in ("Motor 1 IN A", "Encoder 4 B", "I2C SDA", "Battery sense",
               "Sonar echo", "Status LED"):
        assert by_function.count(f"| {fn} |") == 1, fn
    assert "divider 30000 Ω / 7500 Ω" in md


def test_a_shared_pin_is_one_row_and_flagged():
    """Four motors on GPIO 22 -- one bridge enable -- must not read as four
    separate wires, and must not be silently normal either."""
    params = yaml.safe_load(open(MECANUM))
    md = gen_wiring_table.render(params)
    gpio_section = md[md.index("## By GPIO"):]
    row = [l for l in gpio_section.splitlines() if l.startswith("| 22 |")]
    assert len(row) == 1
    assert row[0].count("Motor") == 4 and "shared" in row[0]


def test_inversion_is_noted_on_the_direction_pins_not_the_pwm():
    params = yaml.safe_load(open(MECANUM))
    md = gen_wiring_table.render(params)
    assert "| Motor 2 IN A | 6 | output | direction inverted |" in md
    assert "| Motor 2 PWM | 22 | output |  |" in md


def test_pin_catalogue_findings_are_on_the_same_sheet():
    """A chart describing an impossible pinout must say so where it is read."""
    params = copy.deepcopy(yaml.safe_load(open(MECANUM)))
    params["base_controller"]["pins"]["led"] = 23   # internal to the Pico W radio
    params["base_controller"]["mcu"] = "pico2w"
    md = gen_wiring_table.render(params)
    assert "## Pin catalogue findings" in md
