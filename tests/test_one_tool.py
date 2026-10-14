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
        assert re.search(r'\{"%s",\s*"%s"\}' % (app, re.escape(run)), src), app
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
    assert main.index("if (micro_ros && toolEscape())") < main.index("    initUrosTransport();")


def test_a_stopped_command_always_turns_the_motors_off_first():
    src = _read("src", "tools", "tools.cpp")
    jump = src[src.index("static void abortJump(const char *why)"):src.index("void toolAbort(const char *why)")]
    assert jump.index("toolMotorsOff();") < jump.index("longjmp(to_prompt, 1);")
    for fn in ("void toolAbort(const char *why)", "void toolAbortQuiet(const char *why)"):
        body = src[src.index(fn):]
        assert body.index("abortJump(why);") < body.index("\n}")


def test_the_sim_mcu_reads_its_console_from_stdin_without_blocking():
    shim = _read("host", "shim", "Arduino.h")
    assert "poll(&pfd, 1, 0)" in shim
    assert "eof_ = true" in shim


def _toolselect():
    src = _read("src", "tools", "tools.cpp")
    return src[src.index("AppMode toolSelect(void)"):src.index("const char *toolName(")]


def test_a_refused_former_name_boots_base_as_it_did():
    """Before the merge a refused tool fell back to the robot firmware; an env written
    without the host's check must not leave a robot out of micro-ROS."""
    sel = _toolselect()
    refusal = sel[sel.index("const char *why = legacyRefusal(want);"):]
    assert refusal.index("return APP_BASE;") < refusal.index("envSetRealOnly(!toolWalk());")
    src = _read("src", "tools", "tools.cpp")
    fn = src[src.index("static const char *legacyRefusal"):src.index("AppMode toolSelect(void)")]
    for app, key in (("test_sensors", '"imu"'), ("test_motors", '"sim_wheel"'), ("test_acc", '"sim_wheel"'),
                     ("adc_calibrate", '"sim_battery"'), ("i2c_detect", "toolI2cUnassigned()")):
        assert app in fn and key in fn, app
    assert "#if !ADC_LUT_SUPPORTED" in fn


def test_test_motors_with_one_wheel_in_the_env_runs_that_wheel():
    sel = _toolselect()
    assert 'envInt("tm_only", 0)' in sel and 'boot_run = "motor";' in sel
    src = _read("src", "tools", "tools.cpp")
    motor = src[src.index("} else if (!strcmp(cmd, \"motor\")) {"):src.index("} else if (!strcmp(cmd, \"motors\")) {")]
    assert '`motor` alone runs the env' in motor and 'envInt("tm_only", 0)' in motor


def test_base_leaves_the_tool_for_one_boot():
    """`base` in a tool the env boots (app=tool, a former name) would come straight back."""
    sel = _toolselect()
    assert sel.index("if (next == NEXT_BOOT_BASE)") < sel.index('envGet("app"') or \
        sel.index("if (next == NEXT_BOOT_BASE)") < sel.index("if (!want")
    src = _read("src", "tools", "tools.cpp")
    restart = src[src.index("void toolRestart(bool tool)"):src.index("static const char *legacyRefusal")]
    assert "nextBootSet(tool ? NEXT_BOOT_TOOL : NEXT_BOOT_BASE);" in restart
    assert "sys_reboot(SYS_REBOOT_COLD);" in restart        # the UNO Q restarts too
    assert "watchdog_hw->scratch[0]" in src                 # survives an RP2 reboot


def test_set_takes_the_test_settings_only():
    src = _read("src", "tools", "tools.cpp")
    fn = src[src.index("static bool settable"):src.index("static void status")]
    allowed = set(re.findall(r'"(\w+)"', fn))
    # Not sim_wheel: the encoders are built simulated or real at boot, and real motors
    # against simulated encoders run away (the 2026-10-13 robot-builder review).
    assert allowed == {"tm_pct", "tm_only", "tm_secs", "ta_ms", "ta_pct"}
    assert "pwm_bits" not in allowed and not any("password" in k or "psk" in k for k in allowed)


def test_a_bare_cr_ends_a_line():
    """minicom, screen and PuTTY send CR alone; `stop` must work from them."""
    for path in (("src", "tools", "tools.cpp"), ("src", "tools", "tool_term.cpp")):
        src = _read(*path)
        assert re.search(r"if \(c == '\\r' \|\| c == '\\n'\)", src), path


def test_the_terminal_login_is_a_challenge_and_needs_a_password():
    term = _read("src", "tools", "tool_term.cpp")
    assert 'client.print("nonce ");' in term
    assert 'snprintf(want, sizeof(want), "%s:%s", password(), nonce);' in term
    begin = term[term.index("void termBegin(void)"):term.index("const char *termAddress")]
    assert begin.index("if (!password())") < begin.index("server = new WiFiServer(port);")
    assert "#define LOGIN_MS" in term and "#define LOCKOUT_MS" in term
    import lino_tool
    assert lino_tool.tool_answer("pw", "abcd") == hashlib.md5(b"pw:abcd").hexdigest()


def test_a_drive_from_the_terminal_stops_when_it_goes_quiet():
    src = _read("src", "tools", "tools.cpp")
    assert "#define TERM_DEADMAN_MS 3000" in src
    poll = src[src.index("static void pollInput"):src.index("bool toolStopRequested")]
    assert "millis() - termLastInput() > deadman" in poll and "job_from_term" in poll
    # 2 s under acc (full power), 3 s otherwise.
    assert '!strcmp(job_name, "acc")) ? ACC_DEADMAN_MS : TERM_DEADMAN_MS' in poll
    assert "#define ACC_DEADMAN_MS 2000" in src
    import lino_tool
    assert 'sendall(b"\\0")' in open(lino_tool.__file__).read()


def test_lino_tool_never_leaves_a_wheel_turning():
    import lino_tool
    text = open(lino_tool.__file__).read()
    main = text[text.index("def main()"):]
    fin = main[main.index("finally:"):]
    assert 'link.write(b"stop\\r\\n")' in fin
