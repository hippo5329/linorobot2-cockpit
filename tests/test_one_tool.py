"""One firmware mode with a tool behind a boot window (user, 2026-10-12: "merge tools into one,
use terminal command to drive i2c, sensors, motors, acc"; "only one mode, base boot wait for
tool escape for 3 or 5 seconds then proceed to micro-ros"; "in wifi mode, host can use a reset
command port with ota like logic").

Behaviour is proven on the Sim MCU (the firmware built for the host, stdin as its console):
escape in the window, every command, a stop mid-run, `base` back, no escape -> micro-ROS.
These pin the parts a source edit could quietly undo.
"""
import hashlib
import os
import re
import sys

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FW = os.path.join(REPO_ROOT, "firmware")
sys.path.insert(0, os.path.join(REPO_ROOT, "scripts"))


def _read(*parts):
    with open(os.path.join(FW, *parts), encoding="utf-8") as f:
        return f.read()


def test_the_host_answers_the_nonce_the_way_the_board_checks_it():
    import lino_tool
    ota = _read("common", "lib", "wifi", "ota.cpp")
    assert 'md5.add(String(pwd) + ":" + tool_nonce);' in ota
    assert lino_tool.tool_answer("s3cret", "0a1b2c3d") == hashlib.md5(b"s3cret:0a1b2c3d").hexdigest()
    # One use, right or wrong, and only for 10 s.
    assert "tool_nonce[0] = '\\0';              // one use" in ota
    assert "millis() - tool_nonce_ms < 10000" in ota


def test_lino_stop_stays_unauthenticated():
    """It only takes motion away; a password in front of it would slow the one command
    that must never wait."""
    ota = _read("common", "lib", "wifi", "ota.cpp")
    stop = ota[ota.index("if (stop) {"):ota.index("} else if (tool) {")]
    assert "toolRequest" not in stop and "nonce" not in stop


def test_a_set_value_shadows_the_stored_one_and_base_pays_nothing():
    env = _read("common", "lib", "mcu_env", "mcu_env.cpp")
    get = env[env.index("const char *envGet(const char *key, const char *fallback)"):]
    assert get.index("envOverride(key)") < get.index("if (!env_valid")
    # The table is allocated on the first `set`: base never sets one.
    assert "static EnvOverride *ov = NULL;" in env
    assert "calloc(ENV_OVERRIDES, sizeof(EnvOverride))" in env


def test_every_former_tool_name_still_boots_its_command():
    src = _read("src", "tools", "tools.cpp")
    for app, run in (("test_sensors", "sensors"), ("test_motors", "motors"), ("test_acc", "acc"),
                     ("i2c_detect", "i2c 5"), ("adc_calibrate", "adc")):
        assert f'{{"{app}",' in src and f'"{run}"}}' in src, app
    for cmd in ("i2c", "sensors", "motor", "motors", "acc", "set", "get", "status", "base"):
        assert f'!strcmp(cmd, "{cmd}")' in src, cmd


def test_rp2_micro_ros_keeps_the_bare_usb_port():
    """The console wrapper exists on RP2 for the terminal only; base's bytes take the
    path they always took."""
    uros = _read("common", "lib", "uros_transport", "uros_transport.cpp")
    assert re.search(r"#if defined\(ARDUINO_ARCH_RP2040\)\s*rmw_uros_set_custom_transport\(true, "
                     r"\(void \*\)&linoConsoleStream\(\)", uros)


def test_the_window_is_three_seconds_on_a_board_and_none_on_the_sim_mcu():
    src = _read("src", "tools", "tools.cpp")
    assert re.search(r"#ifdef LINO_HOST\s*#define TOOL_WAIT_DEFAULT 0", src)
    assert "#define TOOL_WAIT_DEFAULT 3000" in src
    main = _read("src", "main.cpp")
    # After the radio (the ping port answers lino-tool), before micro-ROS.
    assert main.index("initPing(formatBanner);") < main.index("if (micro_ros && toolEscape())")
    assert main.index("if (micro_ros && toolEscape())") < main.index("initUrosTransport(")


def test_a_stopped_command_always_turns_the_motors_off_first():
    src = _read("src", "tools", "tools.cpp")
    abort = src[src.index("void toolAbort(const char *why)"):]
    assert abort.index("toolMotorsOff();") < abort.index("longjmp(to_prompt, 1);")


def test_the_sim_mcu_reads_its_console_from_stdin_without_blocking():
    shim = _read("host", "shim", "Arduino.h")
    assert "poll(&pfd, 1, 0)" in shim
    assert "eof_ = true" in shim
