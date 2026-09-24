"""The `simulation:` block is written out in full, and its values are the firmware's.

Two rules, one from the user on 2026-09-24: *"if a key is not entered in env we
will use default value; config yaml will show them in full."*

The firmware is deliberately tolerant -- `envFloat("fake_sag", FAKE_BATT_SAG)`
keeps the compiled default when the key is absent, which is what lets a blank env
boot a freshly flashed board. That tolerance is exactly why the config has to be
explicit: fake mode is this project's default, so on every bench run the room,
the mass and the drivetrain losses ARE the robot, and a config listing only the
overrides describes none of it. Someone sweeping `battery_sag` should be able to
see what they are sweeping from.

The failure this guards against is the boring one: a term is added to
fake_wheel.h, plumbed through mcu_env.py, and never appears in a config, so it is
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
    start = src.index('src = {"fake_map_w"')
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
    """Rename a macro and the generator must stop, not write a stale number."""
    import pytest
    bad = os.path.join(REPO_ROOT, "firmware", "common", "lib", "encoder", "fake_wheel.h")
    original = gbc.SIM_DEFAULTS
    gbc.SIM_DEFAULTS = (("battery_sag", "encoder/fake_wheel.h", "FAKE_NO_SUCH_MACRO", float),)
    try:
        with pytest.raises(SystemExit):
            gbc.bare_simulation()
    finally:
        gbc.SIM_DEFAULTS = original
    assert os.path.exists(bad)


def test_an_absent_key_is_not_written_so_the_firmware_default_stands():
    """The other half of the rule. A config with no simulation block must not
    cause mcu_env to emit zeros -- that would turn 'unset' into 'no gearbox'."""
    params = {"robot": {"name": "x"}, "base_controller": {"name": "esp32", "mcu": "esp32"},
              "kinematics": {"base_type": "2wd", "wheel_diameter": 0.1,
                             "lr_wheels_distance": 0.3, "max_rpm": 100,
                             "counts_per_rev": 100}}
    env = mcu_env.hardware_env(params)
    for key in ("fake_gear_eff", "fake_coulomb", "fake_sag", "fake_sag_tau",
                "fake_drv_drop", "fake_drv_r", "fake_mass"):
        assert key not in env, f"{key} written from a config that never mentioned it"
