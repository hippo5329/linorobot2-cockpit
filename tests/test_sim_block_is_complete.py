"""The `simulation:` block is written out in full, and its values are the firmware's.

Two rules, one from the user on 2026-09-24: *"if a key is not entered in env we
will use default value; config yaml will show them in full."*

The firmware is deliberately tolerant -- `envFloat("sim_sag", SIM_BATT_SAG)`
keeps the compiled default when the key is absent, which is what lets a blank env
boot a freshly flashed board. That tolerance is exactly why the config has to be
explicit: sim mode is this project's default, so on every bench run the room,
the mass and the drivetrain losses ARE the robot, and a config listing only the
overrides describes none of it. Someone sweeping `battery_sag` should be able to
see what they are sweeping from.

The failure this guards against is the boring one: a term is added to
sim_wheel.h, plumbed through mcu_env.py, and never appears in a config, so it is
configurable in principle and invisible in practice. Three keys
(battery_sag_tau_ms, driver_drop, driver_resistance) reached exactly that state
before this test existed.
"""
import glob
import os
import sys

import yaml

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO_ROOT, "scripts"))

import gen_bare_config as gbc  # noqa: E402
import mcu_env  # noqa: E402

REF = os.path.join(REPO_ROOT, "config", "reference")

# robot_radius is the one simulation key deliberately absent from a config: it
# follows the largest robot_radius the costmaps plan with, and a literal would
# break that agreement -- the mismatch that parked a soak in a lethal cell for
# 154 consecutive goals.
DERIVED = {"robot_radius"}


def _keys_the_writer_maps():
    """Every `simulation.<key>` mcu_env.py knows how to write."""
    src = open(os.path.join(REPO_ROOT, "scripts", "mcu_env.py"), encoding="utf-8").read()
    start = src.index('src = {"sim_map_w"')
    end = src.index("}[key]", start)
    table = src[start:end]
    return {v for v in yaml.safe_load("{" + table.split("{", 1)[1] + "}").values()}


def test_the_generated_bare_config_carries_every_simulation_key():
    written = _keys_the_writer_maps() - DERIVED
    got = set(gbc.bare_config("pico2")["base_controller"]["simulation"])
    assert got == written, f"missing {sorted(written - got)}, extra {sorted(got - written)}"


def test_every_reference_config_carries_every_simulation_key():
    written = _keys_the_writer_maps() - DERIVED
    files = sorted(glob.glob(os.path.join(REF, "*_config.yaml")))
    assert files, "no reference configs found"
    for f in files:
        sim = (yaml.safe_load(open(f))["base_controller"] or {}).get("simulation")
        assert sim, f"{os.path.basename(f)} has no simulation block"
        assert set(sim) >= written, \
            f"{os.path.basename(f)} is missing {sorted(written - set(sim))}"


def test_the_shown_values_are_the_firmware_defaults_not_a_second_opinion():
    """A config that shows a value the firmware does not default to is worse than
    showing nothing: it reads like documentation and behaves like an override."""
    defaults = gbc.bare_simulation()
    for f in sorted(glob.glob(os.path.join(REF, "*_config.yaml"))):
        sim = yaml.safe_load(open(f))["base_controller"]["simulation"]
        for key, want in defaults.items():
            got = sim[key]
            if isinstance(want, bool):
                assert got is want, (f, key, got, want)
            else:
                assert abs(float(got) - want) < 1e-9, (os.path.basename(f), key, got, want)


def test_the_generator_reads_the_headers_rather_than_restating_them():
    """A macro it cannot find is left OUT, not guessed at -- and not fatal.

    Both halves matter, and the second was learned the hard way. Writing a
    number the firmware does not use would make the config lie; but raising
    took down a whole Nav2 matrix, because this runs during bringup inside a
    container whose /ws/firmware is the image's older copy while the bench
    stages the repo's scripts over it. An absent key is the pre-existing,
    correct behaviour: nothing is written and the firmware's #ifndef default
    stands.
    """
    import pytest
    original = gbc.SIM_DEFAULTS
    gbc.SIM_DEFAULTS = (("battery_sag", "encoder/sim_wheel.h", "SIM_BATT_SAG", float),
                        ("bogus", "encoder/sim_wheel.h", "FAKE_NO_SUCH_MACRO", float))
    try:
        got = gbc.bare_simulation()
        assert "battery_sag" in got, "a readable macro must still be read"
        assert "bogus" not in got, "an unreadable one must be omitted, not invented"
        with pytest.raises(SystemExit):
            gbc.bare_simulation(strict=True)
    finally:
        gbc.SIM_DEFAULTS = original


def test_a_missing_firmware_tree_does_not_stop_a_bringup():
    """The failure mode that cost a matrix: gen_bare_config runs during bringup,
    so it must degrade rather than exit however broken its inputs are."""
    original = gbc.SIM_DEFAULTS
    gbc.SIM_DEFAULTS = (("battery_sag", "no/such/header.h", "SIM_BATT_SAG", float),)
    try:
        assert gbc.bare_simulation() == {}
    finally:
        gbc.SIM_DEFAULTS = original
    # ...and the config it generates is still a valid config.
    cfg = gbc.bare_config("pico2")
    assert cfg["kinematics"]["base_type"] == "2wd"


def test_an_absent_key_is_not_written_so_the_firmware_default_stands():
    """The other half of the rule. A config with no simulation block must not
    cause mcu_env to emit zeros -- that would turn 'unset' into 'no gearbox'."""
    params = {"robot": {"name": "x"}, "base_controller": {"name": "esp32", "mcu": "esp32"},
              "kinematics": {"base_type": "2wd", "wheel_diameter": 0.1,
                             "lr_wheels_distance": 0.3, "max_rpm": 100,
                             "counts_per_rev": 100}}
    env = mcu_env.hardware_env(params)
    for key in ("sim_gear_eff", "sim_coulomb", "sim_sag", "sim_sag_tau",
                "sim_drv_drop", "sim_drv_r", "sim_mass"):
        assert key not in env, f"{key} written from a config that never mentioned it"


def test_the_headers_the_generator_reads_are_installed_with_the_package():
    """gen_bare_config runs from the INSTALLED copy under share/, at bringup.

    It used to run from a share tree with no firmware directory at all, so it
    read nothing and generated a config with the whole simulation block missing
    -- not a crash, which is worse: the firmware falls back to its compiled
    defaults silently and a config somebody reads stops describing the robot.
    Whatever SIM_DEFAULTS names has to be installed alongside the scripts.
    """
    cmake = open(os.path.join(REPO_ROOT, "CMakeLists.txt"), encoding="utf-8").read()
    install = cmake[cmake.index("share/${PROJECT_NAME}/firmware"):]
    install = cmake[cmake.rindex("install(", 0, cmake.index("share/${PROJECT_NAME}/firmware")):]
    install = install[:install.index(")") + 1]
    for _, rel, _, _ in gbc.SIM_DEFAULTS:
        directory = "firmware/common/lib/" + os.path.dirname(rel)
        assert directory in install, \
            f"{rel} is read by the generator but {directory} is not installed"


def test_the_generator_finds_its_headers_from_this_checkout():
    """The repo layout itself: REPO_ROOT/firmware/common/lib/<rel> must resolve,
    or every run falls back to the degraded path and nobody notices."""
    for _, rel, macro, _ in gbc.SIM_DEFAULTS:
        path = os.path.join(REPO_ROOT, "firmware", "common", "lib", rel)
        assert os.path.isfile(path), path
        assert f"#define {macro}" in open(path, encoding="utf-8").read(), (rel, macro)
