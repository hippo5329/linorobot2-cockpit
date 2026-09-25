"""The simulated battery reaches the board, and /battery is 1 Hz carrying the sag.

Two things fixed on 2026-09-25 (user: "add fake battery voltage sensor option";
"keep it in 1 Hz, but show the sag instead of averaged voltage"; "compile in
battery sag detector, let user adjust the dip percentage"):

  * `use_sim_battery` is a simulated battery voltage sensor (battery.cpp). Like
    the magnetometer and barometer before it, `current: NONE` (no chip) must
    not silence the simulated one.
  * the dip detector sat behind #ifdef BATTERY_DIP, which nothing defined, so no
    image had it; and when on, it published an EXTRA /battery message per dip,
    so the topic's rate followed the load. It is always compiled in now, its
    threshold is the `bat_dip` env key, and /battery has one publish site.
"""
import os
import re
import sys

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO_ROOT, "scripts"))
import mcu_env  # noqa: E402
import gen_bare_config  # noqa: E402

MAIN = os.path.join(REPO_ROOT, "firmware", "src", "main.cpp")
BATTERY = os.path.join(REPO_ROOT, "firmware", "common", "lib", "battery", "battery.cpp")


def env_for(sensors, battery=None):
    bc = {"name": "pico2", "sensors": sensors}
    if battery is not None:
        bc["pins"] = {"battery": battery}
    return mcu_env.hardware_env({"robot": {"name": "t"}, "base_controller": bc})


def test_sim_battery_is_written_and_published():
    env = env_for({"current": "NONE", "use_sim_battery": True})
    assert str(env["sim_battery"]) == "1"
    assert int(env["pub_battery"]) == 1, "current: NONE must not silence the simulated battery"


def test_no_sim_battery_by_default():
    env = env_for({"current": "NONE"})
    assert str(env.get("sim_battery", "0")) == "0"


def test_every_bare_robot_has_one_and_so_does_the_sim_mcu():
    for mcu in gen_bare_config.KNOWN:
        sensors = gen_bare_config.bare_config(mcu)["base_controller"]["sensors"]
        assert sensors.get("use_sim_battery") is True, mcu


def test_the_dip_threshold_is_an_env_key():
    env = env_for({}, battery={"pin": -1, "dip_pct": 5.0})
    assert float(env["bat_dip"]) == 5.0


def test_simulation_mode_forces_it_on_and_real_forces_it_off():
    env = {"sim_battery": "0"}
    mcu_env.apply_sensor_mode(env, "sim")
    assert env["sim_battery"] == "1"


def test_the_firmware_reads_both_keys():
    assert 'envFlag("sim_battery"' in open(BATTERY).read()
    assert 'envFloat("bat_dip"' in open(MAIN).read()


def test_battery_has_one_publish_site_and_no_ifdef():
    src = open(MAIN).read()
    assert len(re.findall(r"rcl_publish\(&battery_publisher", src)) == 1, \
        "a second /battery publish (the old per-dip message) makes the rate follow the load"
    assert not re.search(r"^\s*#\s*if(def)?\b.*BATTERY_DIP", src, re.M), \
        "the sag detector must be compiled into every image"


def test_battery_is_published_at_one_hz():
    """BATTERY_TIMER was 2000 from the first commit while everything around it
    said 1 Hz; the host target measured /battery at 0.495 Hz on its first run."""
    src = open(os.path.join(REPO_ROOT, "firmware", "src", "main.cpp")).read()
    m = re.search(r"#define BATTERY_TIMER (\d+)", src)
    assert m and int(m.group(1)) == 1000
