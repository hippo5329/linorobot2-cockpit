"""One firmware mode with a tool behind a boot window (user, 2026-10-12: "merge tools into one,
use terminal command to drive i2c, sensors, motors, acc"; "only one mode, base boot wait for
tool escape for 3 or 5 seconds then proceed to micro-ros"; "in wifi mode, host can use a reset
command port with ota like logic"; 2026-10-16: "keep only base with tool escape, remove all
tools mode including test_sensors, test_motors, test_acc, adc_calibrate. we will use tool
escape to do all these tasks").

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


def test_every_former_tool_is_a_command_and_no_env_boots_one():
    src = _read("src", "tools", "tools.cpp")
    for cmd in ("i2c", "sensors", "motor", "motors", "acc", "set", "get", "status", "base"):
        assert f'!strcmp(cmd, "{cmd}")' in src, cmd
    # the env's back doors into the tool are gone: no name table, no refusal by name, no
    # boot command
    for gone in ("LEGACY", "legacyRefusal", "tool_run", "boot_run", '"test_sensors"',
                 '"test_motors"', '"test_acc"', '"i2c_detect"', '"adc_calibrate"'):
        assert gone not in src, gone


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


def test_tool_select_returns_base_for_any_app_value():
    """Only the restart flag (toolRestart(true): lino-tool, the tool's own restart; the Sim
    MCU's LINO_NEXT_BOOT=tool) opens the tool at boot. The env's `app` is said once when it
    names anything but base, and base boots."""
    sel = _toolselect()
    assert sel.count("return APP_TOOL;") == 1
    tool = sel[:sel.index("return APP_TOOL;")]
    assert "if (next == NEXT_BOOT_TOOL) {" in tool and 'envGet("app"' not in tool
    after = sel[sel.index("return APP_TOOL;"):]
    assert 'envGet("app", "base")' in after
    assert "is not an application: this firmware boots base; open the " in after
    assert "tool with the escape (type tool at boot)" in after
    assert after.rstrip().endswith("return APP_BASE;\n}") or after.strip().endswith("return APP_BASE;\n}")


def test_motor_alone_runs_the_wheel_the_env_names():
    src = _read("src", "tools", "tools.cpp")
    motor = src[src.index("} else if (!strcmp(cmd, \"motor\")) {"):src.index("} else if (!strcmp(cmd, \"ramp\")) {")]
    assert '`motor` alone runs the env' in motor and 'envInt("tm_only", 0)' in motor


def test_base_leaves_the_tool_for_one_boot():
    """`base` restarts the board into base: the restart flag is set to base, and the next
    boot after that is the env's (base) again."""
    src = _read("src", "tools", "tools.cpp")
    restart = src[src.index("void toolRestart(bool tool)"):src.index("AppMode toolSelect(void)")]
    assert "nextBootSet(tool ? NEXT_BOOT_TOOL : NEXT_BOOT_BASE);" in restart
    assert "sys_reboot(SYS_REBOOT_COLD);" in restart        # the UNO Q restarts too
    assert "watchdog_hw->scratch[0]" in src                 # survives an RP2 reboot
    sel = _toolselect()
    assert sel.index("nextBootSet(0);") < sel.index("if (next == NEXT_BOOT_TOOL)")


def test_status_says_what_runs_not_an_app():
    src = _read("src", "tools", "tools.cpp")
    fn = src[src.index("static void status(void)"):src.index("// Numbers and keys are typed")]
    assert "app=" not in fn
    assert '"[tool] running: %s\\n"' in fn and '"[tool] idle"' in fn


def test_the_tool_banner_still_says_it_is_in_the_tool():
    # lino_tool.py, tool_session.py and routes_tool.py wait for these
    src = _read("src", "tools", "tools.cpp")
    setup = src[src.index("void toolSetup(void)"):]
    assert 'Serial.printf("[app] %s\\n", app_name);' in setup
    assert "[tool] ready -- 'help' lists the commands" in setup
    esc = src[src.index("bool toolEscape(void)"):src.index("void toolSetup(void)")]
    assert 'app_name = "tool";' in esc


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


def test_the_sim_mcus_tool_is_entered_by_its_restart_flag_not_the_env(monkeypatch):
    """lino_tool.SimLink: the env boots base only, so the Sim MCU enters its tool the way a
    board's restart into the tool does (LINO_NEXT_BOOT=tool, tools.cpp nextBootGet)."""
    import subprocess
    import host_firmware
    import lino_tool
    import mcu_env
    seen = {}

    class P:
        def __init__(self, argv, **kw):
            seen.update(kw)
            r, w = os.pipe()
            os.close(w)
            self.stdout = os.fdopen(r, "rb")
            self.stdin = None

        def poll(self):
            return 0
    monkeypatch.setattr(host_firmware, "binary", lambda: "/nonexistent/lino_host_fw")
    monkeypatch.setattr(host_firmware, "shell_command", lambda b: "true")
    monkeypatch.setattr(subprocess, "Popen", P)
    cfg = os.path.join(REPO_ROOT, "config", "reference", "gendrv_config.yaml")
    link = lino_tool.SimLink(cfg)
    link.closed = True
    assert seen["env"]["LINO_NEXT_BOOT"] == "tool"
    with open(seen["env"]["LINO_ENV_BIN"], "rb") as fh:
        env = mcu_env.decode(fh.read())
    assert env.get("app", "base") == "base"
    assert env["tool_wait"] == "0" and env["term_port"] == "0"


def test_an_answer_on_the_prompts_line_is_an_answer():
    # the UNO Q printed "tool> ok: status (done)" and lino_tool said the firmware answers nothing
    import tool_answer
    a = tool_answer.parse("tool> ok: status (done)")
    assert a == {"kind": "ok", "line": "status", "ok": True, "msg": "done"}
    assert tool_answer.parse("tool> tool> fail: motor 3 40 -- this robot has 2 wheels")["kind"] == "fail"
    assert tool_answer.parse("tool> ") is None
    assert tool_answer.parse("[tool] ok: not an answer") is None
