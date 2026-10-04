"""The diagnostic tools run on a real robot only -- per tool, by what each one tests.

test_sensors needs a real IMU, test_motors and test_acc real wheels, adc_calibrate a real
battery; i2c_detect runs anywhere (user, 2026-10-04). A simulated subsystem has nothing to
measure: test_sensors on a simulated-IMU ESP32-S3 printed uninitialised floats (ACC Y /
GYR Z of 90-180 digits) and "[+] IMU initialized successfully" on a board with no IMU.
The host refuses before the env write (flash_mcu.refuse_simulated_tool, through
mcu_env.tool_refusal); the firmware's toolSelect() applies the same table.
"""
import os
import re
import sys
import tempfile

import pytest
import yaml

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO_ROOT, "scripts"))
import gen_bare_config  # noqa: E402
import mcu_env  # noqa: E402

REF = os.path.join(REPO_ROOT, "config", "reference")
TOOLS = ("test_sensors", "test_motors", "test_acc", "adc_calibrate")


def _env(params, app):
    env = mcu_env.hardware_env(params)
    env["app"] = app
    return env


@pytest.mark.parametrize("tool", TOOLS)
def test_a_bare_module_in_simulation_mode_runs_no_tool(tool):
    assert mcu_env.tool_refusal(_env(gen_bare_config.bare_config("esp32"), tool))


@pytest.mark.parametrize("ref,tool", [
    ("gendrv", "test_sensors"), ("gendrv", "test_motors"), ("gendrv", "test_acc"),
    ("yb_eet01", "test_sensors"), ("yb_eet01", "test_motors"), ("yb_eet01", "test_acc"),
])
def test_real_robots_keep_their_tools_despite_simulating_what_they_lack(ref, tool):
    # The GenDrv simulates a sonar it does not have, the Yahboom a magnetometer, env
    # and sonar; neither is what these tools test, so neither is refused.
    params = yaml.safe_load(open(os.path.join(REF, f"{ref}_config.yaml")))
    assert mcu_env.tool_refusal(_env(params, tool)) is None


def test_each_tool_is_judged_on_its_own_subsystem():
    base = {"imu": "qmi8658", "sim_wheel": "0", "sim_battery": "0"}
    assert mcu_env.tool_refusal({**base, "app": "test_sensors", "imu": "sim"})
    assert mcu_env.tool_refusal({**base, "app": "test_motors", "imu": "sim"}) is None
    assert mcu_env.tool_refusal({**base, "app": "test_acc", "sim_wheel": "1"})
    assert mcu_env.tool_refusal({**base, "app": "test_sensors", "sim_wheel": "1"}) is None
    assert mcu_env.tool_refusal({**base, "app": "adc_calibrate", "sim_battery": "1"})
    assert mcu_env.tool_refusal({**base, "app": "i2c_detect", "imu": "sim", "sim_wheel": "1"}) is None
    assert mcu_env.tool_refusal({**base, "app": "base", "imu": "sim", "sim_wheel": "1"}) is None


def test_the_flasher_stops_before_writing(capsys):
    import flash_mcu
    env = _env(gen_bare_config.bare_config("esp32s3"), "test_sensors")
    with tempfile.NamedTemporaryFile(suffix=".bin", delete=False) as fh:
        fh.write(mcu_env.encode(env))
    try:
        with pytest.raises(SystemExit) as exc:
            flash_mcu.refuse_simulated_tool(fh.name)
        assert exc.value.code == 2
        assert "TOOL REFUSED" in capsys.readouterr().out
    finally:
        os.unlink(fh.name)


def test_firmware_and_host_refuse_the_same_tools():
    src = open(os.path.join(REPO_ROOT, "firmware", "src", "tools", "tools.cpp")).read()
    fn = src[src.index("static const char *simulatedFor"):src.index("AppMode toolSelect")]
    for tool, (key, value, _) in mcu_env.REAL_ONLY_TOOLS.items():
        case = "APP_" + tool.upper()
        assert case in fn, f"{tool} is refused on the host but not in toolSelect()"
        assert f'"{key}"' in fn, f"toolSelect() does not read {key} for {tool}"
    assert "APP_I2C_DETECT" not in fn
