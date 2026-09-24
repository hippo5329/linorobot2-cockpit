"""The forward hazard stop arms on a real sensor and never on a simulated one.

main.cpp runs it every control cycle, below ROS:

    if (safety_stop_on) {
        const float range = rangeAheadOrNegative();
        const bool blocked = (range >= 0.0f) && (range < safety_stop_range);
        if (blocked && twist_msg.linear.x > 0.0) { linear.x = 0; linear.y = 0; }
    }

which is the one layer that still works when the ROS side is wedged, the link
has dropped, or the planner is confident and wrong. nav2_collision_monitor
cannot cover that case because it IS the ROS side.

It had never been enabled anywhere: mcu_env defaulted it to "0" and no
reference config, doc or test asked for it. Now the one config with a real
HC-SR04 wired turns it on -- and simulation mode turns it back off, because
main.cpp's range_sim raycasts the simulated room and a hazard stop must not
be exercised against an imaginary obstacle.
"""
import os
import sys

import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "scripts"))

import mcu_env  # noqa: E402

MAIN = os.path.join(ROOT, "firmware", "src", "main.cpp")


def _cfg(name):
    with open(os.path.join(ROOT, "config", "reference", f"{name}_config.yaml"), encoding="utf-8") as fh:
        return yaml.safe_load(fh)


def _reference_names():
    """Discovered, never listed -- the set of shipped designs changes."""
    import glob
    return sorted(
        os.path.basename(f)[: -len("_config.yaml")]
        for f in glob.glob(os.path.join(ROOT, "config", "reference", "*_config.yaml"))
    )


def test_only_the_config_with_a_real_sonar_asks_for_it():
    on = _cfg("pico2_mecanum")["base_controller"]
    assert on["safety_stop"]["enabled"] is True
    pins = on["pins"]["sonar"]
    assert pins["trigger"] >= 0 and pins["echo"] >= 0, "armed without a sensor is theatre"

    for name in _reference_names():
        bc = _cfg(name)["base_controller"]
        sonar = (bc.get("pins") or {}).get("sonar") or {}
        if int(sonar.get("trigger", -1)) >= 0 and int(sonar.get("echo", -1)) >= 0:
            continue                      # the one with a sensor, checked above
        assert not bc.get("safety_stop"), name


def test_a_simd_range_disarms_it_however_the_config_asks():
    """Simulation mode overriding an explicit setting is the established rule for
    every other sim_* key."""
    env = {"sim_ld19": "1", "sim_sonar": "1", "safety_stop": "1"}
    assert mcu_env._truthy(env["sim_ld19"]) and mcu_env._truthy(env["sim_sonar"])
    src = open(os.path.join(ROOT, "scripts", "mcu_env.py"), encoding="utf-8").read()
    assert 'if _truthy(env.get("sim_ld19")) and _truthy(env.get("sim_sonar")):' in src
    assert 'env["safety_stop"] = "0"' in src


def test_the_rule_matches_the_firmwares_own():
    """mcu_env mirrors range_sim; if one changes the other must."""
    m = open(MAIN, encoding="utf-8").read()
    assert 'range_sim = sim_lidar_on && envFlag("sim_sonar", true);' in m


def test_only_forward_motion_is_blocked():
    """A robot stopped against something must still be able to back off."""
    m = open(MAIN, encoding="utf-8").read()
    # the guard is on forward motion, and only x and y are zeroed
    assert "if (blocked && twist_msg.linear.x > 0.0)" in m
    blk = m[m.index("if (blocked && twist_msg.linear.x > 0.0)"):]
    blk = blk[:blk.index("}")]
    assert "twist_msg.linear.x = 0.0;" in blk and "twist_msg.linear.y = 0.0;" in blk
    assert "angular" not in blk, "rotation must stay available to back off"


def test_a_missing_sensor_never_brakes():
    m = open(MAIN, encoding="utf-8").read()
    blk = m[m.index("static inline float rangeAheadOrNegative"):]
    blk = blk[:blk.index("void moveBase")]
    assert "if (!publish_range)\n        return -1.0f;" in blk
    # and -1 fails the >= 0 test at the call site
    assert "(range >= 0.0f)" in m
