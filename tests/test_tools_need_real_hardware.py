"""The tool's commands run on a real robot only -- per command, by what each one tests.

`motor`, `motors` and `ramp` need real wheels, `acc` too (in its own words, the ones
test_acc_run.py waits for), `adc` a real battery; `i2c` needs only the robot's I2C bus --
pins in its config -- whatever is simulated (user, 2026-10-07; test_i2c_detect_needs_pins.py).
A simulated subsystem has nothing to measure: the sensor test on a simulated-IMU ESP32-S3
printed uninitialised floats (ACC Y / GYR Z of 90-180 digits) and "[+] IMU initialized
successfully" on a board with no IMU. `sensors` creates nothing simulated at all: the tool
runs on real parts only (envSetRealOnly).

The board boots base only and the tool is opened by its boot escape (user, 2026-10-16), so
the refusals live in the commands (tools.cpp simulatedFor, i2cUnassigned, test_acc's
wheelsAreSim) and nowhere on the host: no env write selects a tool to refuse.
"""
import os
import sys

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO_ROOT, "scripts"))
import mcu_env  # noqa: E402

TOOLS = os.path.join(REPO_ROOT, "firmware", "src", "tools")
SRC = open(os.path.join(TOOLS, "tools.cpp")).read()


def _simulated_for() -> str:
    return SRC[SRC.index("static const char *simulatedFor"):SRC.index("bool toolI2cUnassigned")]


def test_the_drive_commands_need_real_wheels():
    fn = _simulated_for()
    for cmd in ("motor", "motors", "ramp"):
        assert f'"{cmd}"' in fn, f"`{cmd}` is not refused on simulated wheels"
    assert '"sim_wheel"' in fn


def test_adc_needs_a_real_battery():
    fn = _simulated_for()
    assert '"adc"' in fn and '"sim_battery"' in fn


def test_acc_refuses_simulated_wheels_in_the_words_test_acc_run_waits_for():
    acc = open(os.path.join(TOOLS, "test_acc.cpp")).read()
    assert "wheelsAreSim()" in acc and "test_acc refused" in acc
    import test_acc_run
    assert test_acc_run.ended(["test_acc refused: sim_wheel=1, nothing to measure"])


def test_the_tool_runs_on_real_parts_only_unless_the_sim_mcu_walks():
    # every way into the tool: the restart flag and the escape
    assert SRC.count("envSetRealOnly(!toolWalk());") == 2


def test_i2c_is_judged_by_its_bus_not_by_a_simulation():
    assert '"i2c"' not in _simulated_for()
    handle = SRC[SRC.index("static void handleLine"):]
    assert "if (i2cUnassigned())" in handle[handle.index('!strcmp(cmd, "i2c")'):]


def test_the_host_keeps_no_table_of_tools_to_refuse():
    # it judged an env's `app` before the write; no env boots a tool now
    for name in ("REAL_ONLY_TOOLS", "tool_refusal", "_sensors_refusal"):
        assert not hasattr(mcu_env, name), name
    import flash_mcu
    assert not hasattr(flash_mcu, "refuse_simulated_tool")
    assert not hasattr(flash_mcu, "warn_if_app_unsupported")
