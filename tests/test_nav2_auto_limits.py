"""Nav2's limits are derived from the motors, by default.

The shipped limits asked 103% of a differential base's motors, 171% at the
velocity smoother's ceiling and 129% on mecanum -- and nothing checked, so Nav2
commanded what it could not get, Kinematics scaled the whole request down to
fit, tracking degraded, and three mecanum legs left the room on 2026-09-23. They
were then lowered by hand, which fixes exactly one chassis: change the wheels,
the load or the pack and the hand-tuned numbers are stale again with nothing to
notice.

So they are computed, and it is ON by default. `kinematics.auto_nav2_limits:
false` hands control back to a person tuning by hand, and then nothing is
rewritten -- that is the other half of the contract and the half that is easy to
break silently.

The fractions are calibrated to reproduce the values this project settled on for
its default chassis, so turning the feature on does not quietly re-tune a robot
that was already right; it makes the same judgement scale to the next one.
"""
import copy
import os
import sys

import pytest
import yaml

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO_ROOT, "scripts"))

import drivetrain_report as dr  # noqa: E402

REF = os.path.join(REPO_ROOT, "config", "reference")
SMOOTHER = "nav2.velocity_smoother.ros__parameters"
FOLLOW = "nav2.controller_server.ros__parameters.FollowPath"


def _cfg(name="gendrv"):
    with open(os.path.join(REF, f"{name}_config.yaml"), encoding="utf-8") as fh:
        return yaml.safe_load(fh)


def _dig(params, dotted):
    node = params
    for part in dotted.split("."):
        node = node[part]
    return node


def test_the_default_chassis_keeps_the_limits_it_was_given():
    """Calibration check. The derived values must land on the hand-tuned ones for
    the robot they were hand-tuned for, or switching this on re-tunes a robot
    that was already correct."""
    params = _cfg("gendrv")
    got = dr.suggest_nav2_limits(dr.drivetrain(params))
    v = got[f"{SMOOTHER}.max_velocity"]
    a = got[f"{SMOOTHER}.max_accel"]
    assert v[0] == pytest.approx(0.30, abs=0.02), v
    assert v[2] == pytest.approx(1.20, abs=0.05), v
    assert a[0] == pytest.approx(0.80, abs=0.05), a
    assert a[2] == pytest.approx(1.50, abs=0.05), a
    assert got[f"{FOLLOW}.desired_linear_vel"] == pytest.approx(0.25, abs=0.02)
    assert got[f"{FOLLOW}.rotate_to_heading_angular_vel"] == pytest.approx(1.0, abs=0.05)


def test_mecanum_gets_a_lower_yaw_rate_than_a_differential_of_the_same_size():
    """The failure this whole exercise came from. A mecanum turns on (lr + fr)/2,
    so the same rad/s costs it 66% more wheel speed -- and it was being given the
    differential base's angular limits."""
    diff = dr.suggest_nav2_limits(dr.drivetrain(_cfg("gendrv")))
    mec = dr.suggest_nav2_limits(dr.drivetrain(_cfg("pico2_mecanum")))
    assert mec[f"{SMOOTHER}.max_velocity"][2] < diff[f"{SMOOTHER}.max_velocity"][2]
    assert mec[f"{FOLLOW}.rotate_to_heading_angular_vel"] < \
        diff[f"{FOLLOW}.rotate_to_heading_angular_vel"]


def test_only_mecanum_is_given_a_y_limit():
    """A non-holonomic base cannot execute a y velocity, and a smoother that
    passes one through is handing the kinematics a command it will silently
    reinterpret."""
    for name in ("gendrv", "yb_eet01"):
        got = dr.suggest_nav2_limits(dr.drivetrain(_cfg(name)))
        assert got[f"{SMOOTHER}.max_velocity"][1] == 0.0, name
        assert got[f"{SMOOTHER}.max_accel"][1] == 0.0, name
    mec = dr.suggest_nav2_limits(dr.drivetrain(_cfg("pico2_mecanum")))
    assert mec[f"{SMOOTHER}.max_velocity"][1] > 0.0


def test_a_heavier_robot_is_given_lower_limits():
    light = copy.deepcopy(_cfg("gendrv"))
    light["base_controller"]["simulation"]["robot_mass"] = 1.5
    heavy = copy.deepcopy(_cfg("gendrv"))
    heavy["base_controller"]["simulation"]["robot_mass"] = 15.0
    l = dr.suggest_nav2_limits(dr.drivetrain(light))
    h = dr.suggest_nav2_limits(dr.drivetrain(heavy))
    assert h[f"{SMOOTHER}.max_velocity"][0] < l[f"{SMOOTHER}.max_velocity"][0]
    assert h[f"{SMOOTHER}.max_accel"][0] < l[f"{SMOOTHER}.max_accel"][0]


def test_the_derived_pair_always_fits_the_wheel_speed_budget():
    """Translation and rotation ADD at the outer wheel. Each limit looked
    survivable alone, which is exactly how the shipped set got to 171%."""
    for name in ("gendrv", "yb_eet01", "pico2_mecanum"):
        params = _cfg(name)
        d = dr.drivetrain(params)
        got = dr.suggest_nav2_limits(d)
        v = got[f"{SMOOTHER}.max_velocity"]
        need = dr.demand_rpm(d, v[0], v[2])
        assert need <= d["command_rpm"] + 1e-6, \
            f"{name}: smoother envelope needs {need:.1f} of {d['command_rpm']:.1f} rpm"


def test_a_pack_below_the_motor_rating_lowers_everything():
    params = copy.deepcopy(_cfg("gendrv"))
    params["kinematics"]["motor_operating_voltage"] = 24.0
    params["kinematics"]["motor_power_max_voltage"] = 12.0
    got = dr.suggest_nav2_limits(dr.drivetrain(params))
    base = dr.suggest_nav2_limits(dr.drivetrain(_cfg("gendrv")))
    assert got[f"{SMOOTHER}.max_velocity"][0] < base[f"{SMOOTHER}.max_velocity"][0]


def test_it_is_on_by_default_and_the_flag_turns_it_off():
    params = _cfg("gendrv")
    assert dr.auto_limits_enabled(params) is True, "absent key must mean ON"
    params.setdefault("kinematics", {})["auto_nav2_limits"] = False
    assert dr.auto_limits_enabled(params) is False
    params["kinematics"]["auto_nav2_limits"] = "false"
    assert dr.auto_limits_enabled(params) is False, "a YAML string must work too"


def test_apply_writes_nothing_when_the_flag_is_off():
    params = _cfg("gendrv")
    params["kinematics"]["auto_nav2_limits"] = False
    before = copy.deepcopy(params["nav2"])
    assert dr.apply_nav2_limits(params) == {}
    assert params["nav2"] == before


def test_apply_writes_the_values_and_reports_what_it_changed():
    params = _cfg("gendrv")
    _dig(params, SMOOTHER)["max_velocity"] = [9.99, 0.0, 9.99]
    changed = dr.apply_nav2_limits(params)
    assert f"{SMOOTHER}.max_velocity" in changed
    assert changed[f"{SMOOTHER}.max_velocity"]["from"] == [9.99, 0.0, 9.99]
    assert _dig(params, SMOOTHER)["max_velocity"] == changed[f"{SMOOTHER}.max_velocity"]["to"]


def test_apply_is_idempotent():
    """Saving twice must not drift the numbers, or every save is a diff."""
    params = _cfg("gendrv")
    dr.apply_nav2_limits(params)
    assert dr.apply_nav2_limits(params) == {}


def test_a_config_without_motors_is_left_alone_rather_than_crashing():
    """The save path calls this; a half-entered chassis must not block a save."""
    params = {"kinematics": {"base_type": "2wd", "max_rpm": 0, "wheel_diameter": 0}}
    assert dr.apply_nav2_limits(params) == {}


def test_the_shipped_references_are_within_their_own_budget():
    """Whatever the auto-tuner would do, what is committed has to be legal now."""
    for name in ("gendrv", "yb_eet01", "pico2_mecanum"):
        params = _cfg(name)
        d = dr.drivetrain(params)
        v = _dig(params, SMOOTHER)["max_velocity"]
        need = dr.demand_rpm(d, float(v[0]), float(v[2]))
        assert need <= d["command_rpm"], \
            f"{name} ships a smoother envelope needing {need:.1f} of {d['command_rpm']:.1f} rpm"


# --- the driver margin is headroom, not a target ------------------------------
#
# Set to 80% on 2026-09-24, straight after the 2wd cliff: with the smoother at
# [0.3, 0, 1.23] the 2wd slice came back 7/10 twice, and at [0.3, 0, 1.2] it came
# back 10/10. A 2.5% rise was the whole difference. The derivation measures what
# the motors CAN deliver (0.87-0.91 on the default chassis); spending all of it
# because the measurement says you could is the same mistake one level up.

def test_the_default_margin_is_eighty_percent():
    d = dr.drivetrain(_cfg("gendrv"))
    assert dr.DEFAULT_MARGIN == 0.80
    assert dr.suggest_max_rpm_ratio(d) == 0.80


def test_the_measurement_can_only_make_the_margin_smaller():
    """A heavy robot that cannot reach 80% gets what it can reach; a light one
    that could reach 93% still gets 80%. The measurement exists to catch the
    robot that falls SHORT of the margin, not to spend it."""
    light = copy.deepcopy(_cfg("gendrv"))
    light["base_controller"]["simulation"]["robot_mass"] = 1.4
    heavy = copy.deepcopy(_cfg("gendrv"))
    heavy["base_controller"]["simulation"]["robot_mass"] = 15.0
    assert dr.suggest_max_rpm_ratio(dr.drivetrain(light)) == dr.DEFAULT_MARGIN
    heavy_ratio = dr.suggest_max_rpm_ratio(dr.drivetrain(heavy))
    assert heavy_ratio < dr.DEFAULT_MARGIN, heavy_ratio


def test_every_shipped_config_carries_the_margin():
    for name in ("gendrv", "yb_eet01", "pico2_mecanum"):
        got = _cfg(name)["kinematics"]["max_rpm_ratio"]
        assert got <= dr.DEFAULT_MARGIN + 1e-9, f"{name} asks for more than the margin"


def test_the_generated_bare_config_carries_it_too():
    import gen_bare_config
    assert gen_bare_config.bare_config("pico2")["kinematics"]["max_rpm_ratio"] \
        <= dr.DEFAULT_MARGIN + 1e-9


def test_the_firmware_header_default_agrees_with_the_margin():
    """A config that omits the key must not get a more generous default than a
    config that sets it -- that is how a robot ends up asking for 85% because
    nobody wrote a number down."""
    src = open(os.path.join(REPO_ROOT, "scripts", "gen_firmware_header.py"),
               encoding="utf-8").read()
    assert 'kine.get("max_rpm_ratio", 0.80)' in src
    assert '"max_rpm_ratio": 0.80,' in src


def test_the_limits_still_fit_the_smaller_budget():
    """Lowering the margin lowers the wheel-speed budget the limits are checked
    against, so the check has to be re-run -- not assumed to still hold."""
    for name in ("gendrv", "yb_eet01", "pico2_mecanum"):
        params = _cfg(name)
        d = dr.drivetrain(params)
        v = _dig(params, SMOOTHER)["max_velocity"]
        need = dr.demand_rpm(d, float(v[0]), float(v[2]))
        assert need <= d["command_rpm"], \
            f"{name}: {need:.1f} rpm of a {d['command_rpm']:.1f} rpm budget"
