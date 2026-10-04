"""The reference designs cover the three drivetrains between them.

gendrv is 2wd, yb_eet01 is a 4WD skid steer, pico2_mecanum is mecanum (user,
2026-10-04: the Yahboom board drives four channels, and a user changes the
drivetrain in one field). A skid steer with only two channels wired, or with no
axle distance, would be a 2wd under a 4wd label.
"""
import os

import yaml

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _cfg(name):
    with open(os.path.join(REPO_ROOT, "config", "reference", f"{name}_config.yaml"), encoding="utf-8") as fh:
        return yaml.safe_load(fh)


def test_each_drivetrain_has_a_reference():
    got = {name: _cfg(name)["kinematics"]["base_type"] for name in ("gendrv", "yb_eet01", "pico2_mecanum")}
    assert got == {"gendrv": "2wd", "yb_eet01": "skid_steer", "pico2_mecanum": "mecanum"}


def test_the_yahboom_skid_steer_wires_all_four_channels():
    cfg = _cfg("yb_eet01")
    pins = cfg["base_controller"]["pins"]
    assert [(pins[f"motor{n}"]["in_a"], pins[f"motor{n}"]["in_b"]) for n in (1, 2, 3, 4)] == \
        [(4, 5), (15, 16), (9, 10), (13, 14)]
    assert [(pins[f"encoder{n}"]["pin_a"], pins[f"encoder{n}"]["pin_b"]) for n in (1, 2, 3, 4)] == \
        [(6, 7), (47, 48), (11, 12), (1, 2)]
    assert cfg["kinematics"]["fr_wheels_distance"] > 0


def test_the_yahboom_preset_matches_its_config():
    js = open(os.path.join(REPO_ROOT, "web", "frontend", "app-presets.js"), encoding="utf-8").read()
    blk = js[js.index('id: "yb_eet01"'):]
    blk = blk[:blk.index("\n    },")]
    assert 'kinematics: "4wd"' in blk
    for frag in ("motor3: { in_a: 9, in_b: 10", "motor4: { in_a: 13, in_b: 14",
                 "encoder3: { a: 11, b: 12 }", "encoder4: { a: 1, b: 2 }", "fr_wheels_distance: 0.18"):
        assert frag in blk, frag
