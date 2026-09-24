"""A real-sensor run must have NO simulated sensor in it. Not one.

The rule is the user's and it is not a preference: a bench run whose purpose is
to say "this chip works" cannot contain a chip that does not exist, because
every reading afterwards has to be attributed by hand.

It was broken by a list. The real-sensor config writer turned off the five
`use_sim_*` keys someone remembered and the schema has six -- so `sim_sonar`
reached the board as 1 on 2026-09-23, simulating an HC-SR04 that is physically
wired to GP27/28 of the very board under test.

This test enumerates the flags from mcu_env.py, which is the code that writes
them, so adding a seventh sim sensor fails here rather than passing quietly.

It spans two repos, and during the `fake_` -> `sim_` rename the two halves cannot
move together: the lab's bench scripts write env keys into a RELEASED image, whose
firmware understands whichever spelling it was built with. Renaming them before the
next cut would hand a `sim_*` key to a board that only reads `fake_*` -- and an env
key a board does not recognise is not an error, it is a default silently taken.
So the comparison below is on the flag's SENSOR, not on its prefix: whichever side
renames first, "the writer disables every simulation flag the schema declares"
stays exactly as true, which is the property this test was always about.
"""
import os
import re

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MCU_ENV = os.path.join(ROOT, "scripts", "mcu_env.py")
LAB = os.path.join(os.path.dirname(ROOT), "linorobot2-cockpit-lab")
WRITER = os.path.join(LAB, "skills", "bench-rig", "scripts",
                      "icm20948_config_inside.py")

import pytest


# `use_fake_imu` and `use_sim_imu` name the same flag; only the era differs.
_PREFIX = re.compile(r"^use_(?:sim|fake)_")


def _sensors(keys):
    """The sensor each flag is about, with the era's prefix removed."""
    return {_PREFIX.sub("", k) for k in keys}


def _schema_sim_keys():
    """Every use_sim_*/use_fake_* key the env writer knows how to translate."""
    src = open(MCU_ENV, encoding="utf-8").read()
    return set(re.findall(r"use_(?:sim|fake)_[a-z0-9_]+", src))


def test_the_schema_has_the_keys_this_test_thinks_it_has():
    """A guard on the guard: if mcu_env stops spelling them this way, the
    enumeration below silently becomes empty and every assertion passes."""
    keys = _schema_sim_keys()
    assert len(keys) >= 5, f"only found {keys} -- the enumeration has stopped working"
    assert "use_sim_sonar" in keys, "the key this test exists for is gone"


@pytest.mark.skipif(not os.path.exists(WRITER), reason="the lab repo is not checked out here")
def test_the_real_sensor_writer_disables_every_sim_flag():
    src = open(WRITER, encoding="utf-8").read()
    declared = set(re.findall(r'"(use_(?:sim|fake)_[a-z0-9_]+)"', src))
    missing = _sensors(_schema_sim_keys()) - _sensors(declared)
    assert not missing, (
        f"the real-sensor config leaves {sorted(missing)} at the config's value; "
        "a real-sensor run must simulate nothing")


@pytest.mark.skipif(not os.path.exists(WRITER), reason="the lab repo is not checked out here")
def test_the_writer_sets_them_from_the_set_not_one_by_one():
    """Written out line by line, the next flag is added to the schema and not
    here, and nothing says so -- which is exactly how sim_sonar survived."""
    src = open(WRITER, encoding="utf-8").read()
    assert re.search(r"for key in sorted\((?:SIM|FAKE)_KEYS\)", src), \
        "the flags are assigned individually again"
