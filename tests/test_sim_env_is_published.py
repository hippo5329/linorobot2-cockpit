"""The simulated barometer must be published, and a real one is found, not declared.

Every bare config says `env: NONE` (no chip) with `use_sim_env: true`. mcu_env
read the NONE as "never publish" and wrote pub_env=0, so the synthetic barometer
the firmware built and read at 1 Hz was never sent -- the same mistake the
simulated magnetometer had (test_sim_mag_is_published.py). And the barometer is
not a build option: initEnv() is in every image and finds a BMP280/BME280 at
0x76/0x77 at boot, so `env: AUTO` (the UI's default) must leave pub_env unset.
"""
import os
import sys

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO_ROOT, "scripts"))
import mcu_env  # noqa: E402
import gen_bare_config  # noqa: E402


def env_for(sensors):
    params = {"robot": {"name": "t"}, "base_controller": {"name": "pico2", "sensors": sensors}}
    return mcu_env.hardware_env(params)


def test_sim_env_on_a_bare_module_is_published():
    env = env_for({"env": "NONE", "use_sim_env": True})
    assert int(env.get("pub_env", 0)) == 1, "env: NONE must not silence the simulated barometer"


def test_generated_bare_config_publishes_its_barometer():
    for mcu in ("pico", "pico2", "esp32", "esp32s3"):
        params = gen_bare_config.bare_config(mcu)
        assert int(mcu_env.hardware_env(params).get("pub_env", 0)) == 1, mcu


def test_auto_leaves_the_decision_to_the_probe():
    env = env_for({"env": "AUTO", "use_sim_env": False})
    assert "pub_env" not in env


def test_none_without_sim_stays_off():
    env = env_for({"env": "NONE", "use_sim_env": False})
    assert int(env.get("pub_env", 1)) == 0
