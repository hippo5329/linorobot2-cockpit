"""The per-MCU pin catalogue and the config warnings that ride with it."""
import copy
import glob
import os

import gen_firmware_header as gh
import pin_catalog as pc


def _levels(findings):
    return sorted(l for l, _ in findings)


def test_shipped_references_have_no_pin_errors(reference):
    """Whatever is in config/reference, not a list that goes stale when one is
    added or removed -- this one still named the DevKit pair a day after they
    were deleted."""
    ref_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                           "config", "reference")
    names = sorted(os.path.basename(p)[: -len("_config.yaml")]
                   for p in glob.glob(os.path.join(ref_dir, "*_config.yaml")))
    assert names, "no reference configs found at all"
    for name in names:
        errors = [m for l, m in pc.check_config(reference(name)) if l == "error"]
        assert not errors, (name, errors)


def test_esp32_flash_bus_and_missing_gpio_are_errors(reference):
    params = copy.deepcopy(reference("gendrv"))
    pins = params["base_controller"]["pins"]
    pins["motor1"]["in_a"] = 6      # SPI flash
    pins["motor2"]["in_a"] = 20     # does not exist on the WROOM
    msgs = [m for l, m in pc.check_config(params) if l == "error"]
    assert any("GPIO 6" in m and "flash" in m for m in msgs)
    assert any("GPIO 20" in m and "does not exist" in m for m in msgs)


def test_esp32_input_only_pin_cannot_drive_a_motor(reference):
    params = copy.deepcopy(reference("gendrv"))
    params["base_controller"]["pins"]["motor1"]["in_b"] = 34
    assert any("input-only" in m for l, m in pc.check_config(params) if l == "error")


def test_esp32_adc2_battery_warns_only_with_the_radio(reference):
    params = copy.deepcopy(reference("gendrv"))
    params["base_controller"]["pins"].setdefault("battery", {})["pin"] = 25
    params["base_controller"]["wifi"] = {"enabled": False}
    assert not [m for l, m in pc.check_config(params) if "ADC2" in m]
    params["base_controller"]["wifi"] = {"enabled": True}
    assert [m for l, m in pc.check_config(params) if "ADC2" in m]


def test_rp2_i2c_pair_must_be_one_block(reference):
    params = copy.deepcopy(reference("pico2_mecanum"))
    params["base_controller"]["pins"]["i2c"] = {"sda": 20, "scl": 22}
    assert any("SDA/SCL pair" in m for l, m in pc.check_config(params) if l == "error")
    params["base_controller"]["pins"]["i2c"] = {"sda": 4, "scl": 5}
    assert not [m for l, m in pc.check_config(params) if l == "error"]


def test_rp2_battery_must_be_an_adc_pin(reference):
    params = copy.deepcopy(reference("pico2_mecanum"))
    params["base_controller"]["pins"]["battery"]["pin"] = 22
    assert any("not an ADC" in m for l, m in pc.check_config(params) if l == "error")


def test_duplicate_pins_are_errors_but_a_shared_bts7960_enable_is_not(reference):
    params = copy.deepcopy(reference("pico2_mecanum"))
    assert all(params["base_controller"]["pins"][f"motor{n}"]["pwm"] == 22 for n in range(1, 5))
    assert not [m for l, m in pc.check_config(params) if l == "error"]
    params["base_controller"]["pins"]["encoder1"]["pin_a"] = 20   # collides with i2c.sda
    assert any("GPIO 20 is used by" in m for l, m in pc.check_config(params) if l == "error")


def test_wireless_pico_led_on_gp25_warns(reference):
    """The W boards lost their reference configs; the rule did not.

    picow/pico2w_config.yaml were removed when the reference designs were
    consolidated onto the two mecanum files. The fact they encoded -- GP23/24/
    25/29 belong to the CYW43, so the LED is not on a GPIO -- lives in
    pin_catalog's wireless_rp2 branch, keyed on the mcu name. Build the config
    here rather than load one, so a user who writes `mcu: pico2w` by hand is
    still warned.
    """
    for mcu in ("picow", "pico2w"):
        params = copy.deepcopy(reference("pico2_mecanum"))
        params["base_controller"]["mcu"] = mcu
        params["base_controller"]["pins"]["led"] = 25
        assert any("CYW43" in m for l, m in pc.check_config(params)), mcu


def test_unknown_mcu_is_a_single_warning():
    assert _levels(pc.check_config({"base_controller": {"mcu": "stm32"}})) == ["warn"]


def test_counts_per_rev_from_parts():
    assert gh.counts_per_rev({"encoder_ppr": 11, "quadrature": 4, "gear_ratio": 30}) == 1320
    assert gh.counts_per_rev({"encoder_ppr": 11, "gear_ratio": 30}) == 1320      # quadrature defaults to 4
    assert gh.counts_per_rev({"counts_per_rev": 4000, "encoder_ppr": 11}) == 4000  # the product wins


def test_mecanum_config_warnings(reference):
    assert gh.config_warnings(reference("pico2_mecanum")) == []
    params = copy.deepcopy(reference("pico2_mecanum"))
    # mecanum-derived configs mix the shapes: a flat ekf, a wrapped nav2.
    params["ekf"]["odom0_config"][7] = False
    params["nav2"]["controller_server"]["ros__parameters"]["min_y_velocity_threshold"] = 0.5
    w = gh.config_warnings(params)
    assert any("vy" in m for m in w) and any("min_y_velocity_threshold" in m for m in w)
    assert gh.config_warnings(reference("pico2_mecanum")) == []
    # ...and the wrapped shape is read too.
    wrapped = {"kinematics": {"base_type": "mecanum"},
               "ekf": {"ekf_filter_node": {"ros__parameters": {"odom0_config": [False] * 15}}},
               "nav2": {"controller_server": {"ros__parameters": {"min_y_velocity_threshold": 0.5}}}}
    assert len([m for m in gh.config_warnings(wrapped) if m.startswith("mecanum base but")]) == 2


def _led(reference, name):
    pins = (reference(name).get("base_controller") or {}).get("pins") or {}
    return pins.get("led", -1)


def test_boards_with_an_onboard_led_default_to_driving_it(reference):
    """The blink pattern is the only thing a board says before micro-ROS is up.

    A bench board in fake mode needs it as much as a real one: a simulated
    robot fails in the same ways, and with led: -1 it fails silently.
    """
    assert _led(reference, "pico2_mecanum") == 25
    assert _led(reference, "gendrv") == 2
    assert _led(reference, "esp32s3") == 48


def test_every_esp32_board_shares_one_led_pin(reference):
    """GPIO 2 on the GenDrv is not connected, so the DevKit default is safe there.

    One LED pin across every ESP32 board beats a per-board exception: driving
    an unconnected pin costs nothing, and a config that differs only where it
    has to is easier to keep right. The DevKit pair (esp32/esp32_wifi) that
    used to be checked here was deleted on 2026-09-20 -- same silicon as
    gendrv, differing only in env keys -- so gendrv carries the rule now.
    """
    assert _led(reference, "gendrv") == 2


# The W boards' `led: -1` used to be asserted from picow/pico2w_config.yaml.
# Those files are gone (consolidated onto the mecanum references); the rule they
# encoded is covered by test_wireless_pico_led_on_gp25_warns above, which builds
# a pico2w config in memory and checks the catalogue still objects.


def test_the_esp32_led_is_flagged_as_a_strapping_pin(reference):
    """GPIO 2 is both the DevKit's LED and a strapping pin.

    The warning is correct and must keep firing -- it is safe here only
    because an LED to ground pulls the pin the way the bootloader wants, and a
    reader deserves to be told that rather than have the warning suppressed.
    """
    warnings = [m for l, m in pc.check_config(reference("gendrv")) if l == "warn"]
    assert any("strapping" in m and "led" in m for m in warnings), warnings
