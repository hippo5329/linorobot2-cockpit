#include <Arduino.h>
#include <setjmp.h>
#include <stdlib.h>
#include "config.h"
#include "tools.h"
#include "tool_hw.h"
#include "tool_term.h"
#include "mcu_env.h"
#include "adc_lut.h"
#include "wifis.h"
#include "ota.h"
#include "syslog.h"

// The commands' entry points, one translation unit each. A command is either a
// blocking run (it returns when it is done, or a stop longjmps out of it) or a pair:
// begin once, then step from every loop() until step returns false.
extern "C" {
void i2c_detect_scan(void);
void test_sensors_begin(void);   bool test_sensors_step(void);
void test_motors_begin(void);    bool test_motors_step(void);
void test_motors_one_begin(void); bool test_motors_one_step(void);
void test_acc_run(void);
#if ADC_LUT_SUPPORTED
void adc_calibrate_run(void);
#endif
}

// ---------------------------------------------------------------------------
// Which application boots. Two modes (user, 2026-10-12): `base`, the robot, and `tool`,
// this shell. The former tools' names still boot the shell and run their command, so a
// host that writes `app=test_acc` gets what it always got.
struct Legacy { const char *app; const char *run; };
static const Legacy LEGACY[] = {
    {"test_sensors",  "sensors"},
    {"test_motors",   "motors"},
    {"test_acc",      "acc"},
    {"i2c_detect",    "i2c 5"},     // it scanned every 5 s
    {"adc_calibrate", "adc"},
};

static const char *app_name = "base";
static const char *boot_run = "";

// "Come up in the tool": survives a soft reset (ESP.restart / rp2040.reboot), not a power
// cycle, so a robot never stays out of micro-ROS by accident.
#define TOOL_NEXT_BOOT 0x544F4F4Cu      // "TOOL"
#if defined(ARDUINO_ARCH_ESP32)
#include <esp_attr.h>
RTC_NOINIT_ATTR static uint32_t tool_next_boot;
#elif defined(ARDUINO_ARCH_RP2040)
static uint32_t tool_next_boot __attribute__((section(".uninitialized_data.lino_tool")));
#elif defined(LINO_HOST)
// The Sim MCU restarts by exec'ing itself; the flag rides in its environment.
#include <unistd.h>
static uint32_t tool_next_boot = 0;
static void hostLoadFlag(void)
{
    const char *f = getenv("LINO_TOOL_NEXT_BOOT");
    if (f && strcmp(f, "1") == 0) tool_next_boot = TOOL_NEXT_BOOT;
    unsetenv("LINO_TOOL_NEXT_BOOT");
}
#else
static uint32_t tool_next_boot = 0;
#endif

#ifndef TOOL_WAIT_DEFAULT
#ifdef LINO_HOST
#define TOOL_WAIT_DEFAULT 0       // the Sim MCU: no console to type into
#else
#define TOOL_WAIT_DEFAULT 3000
#endif
#endif

void toolRestart(bool tool)
{
    toolMotorsOff();
    tool_next_boot = tool ? TOOL_NEXT_BOOT : 0;
    Serial.printf("[tool] restarting into %s\n", tool ? "the tool" : "base");
    Serial.flush();
    delay(100);
#if defined(ARDUINO_ARCH_ESP32)
    ESP.restart();
#elif defined(ARDUINO_ARCH_RP2040)
    rp2040.reboot();
#elif defined(LINO_HOST)
    setenv("LINO_TOOL_NEXT_BOOT", tool ? "1" : "0", 1);
    execl("/proc/self/exe", "lino_host_fw", (char *)NULL);
#endif
    for (;;) delay(1000);      // a board that cannot restart itself: power-cycle it
}

AppMode toolSelect(void)
{
    initMcuEnv();
    const char *want = envGet("app", "base");
    // `base` is the robot, not a command: it returns here, before anything that
    // belongs to the tool (rc-20261012.2: base turned real-only and faulted).
#if defined(LINO_HOST)
    hostLoadFlag();
#endif
    const bool asked = (tool_next_boot == TOOL_NEXT_BOOT);
    tool_next_boot = 0;              // once: the next reset is base again
    if (asked) {
        app_name = "tool";
        envSetRealOnly(true);
        return APP_TOOL;
    }
    if (!want || !*want || strcmp(want, "base") == 0)
        return APP_BASE;
    if (strcmp(want, "tool") == 0) {
        app_name = "tool";
        boot_run = envGet("tool_run", "");
    } else {
        size_t i = 0;
        for (; i < sizeof(LEGACY) / sizeof(LEGACY[0]); i++)
            if (strcmp(LEGACY[i].app, want) == 0) break;
        if (i == sizeof(LEGACY) / sizeof(LEGACY[0])) {
            Serial.printf("[app] '%s' is not an application this image carries - "
                          "starting the robot firmware instead.\n", want);
            return APP_BASE;
        }
        app_name = LEGACY[i].app;
        boot_run = LEGACY[i].run;
    }
    // A real robot only: no simulated IMU, magnetometer, battery or barometer stands
    // in for a part the tool cannot find (mcu_env.h, envSetRealOnly).
    envSetRealOnly(true);
    return APP_TOOL;
}

const char *toolName(AppMode mode) { return mode == APP_TOOL ? app_name : "base"; }

// ---------------------------------------------------------------------------
// Refusals, per command, by what it tests (user, 2026-10-04): a simulated subsystem
// has nothing to measure. Same rules as scripts/mcu_env.py tool_refusal.
static const char *simulatedFor(const char *cmd)
{
    if (!strcmp(cmd, "motor") || !strcmp(cmd, "motors"))
        return envInt("sim_wheel", 0) == 1 ? "the wheels" : NULL;
    if (!strcmp(cmd, "adc"))
        return envInt("sim_battery", 0) == 1 ? "the battery" : NULL;
    return NULL;     // acc says so itself, in the words test_acc_run.py waits for
}

// `i2c` needs the robot's I2C bus (user, 2026-10-07): both pins in the env, or a bus
// fixed in hardware named by `i2c_bus` (the UNO Q's Qwiic i2c4).
static bool i2cUnassigned(void)
{
    const char *bus = envGet("i2c_bus", "");
    if (bus && bus[0])
        return false;
    return envInt("i2c_sda", -1) < 0 || envInt("i2c_scl", -1) < 0;
}

// ---------------------------------------------------------------------------
// Stop, and the jump back to the prompt.
static jmp_buf to_prompt;
static bool stop_flag = false;
static bool (*job_step)(void) = nullptr;   // the running stepped command
static const char *job_name = nullptr;     // stepped or blocking, while it runs
static bool job_moves = false;             // it drives the wheels
static unsigned i2c_every_ms = 0;          // `i2c <secs>`
static uint32_t i2c_next = 0;

void toolMotorsOff(void)
{
    for (int i = 0; i < 4; i++)
        if (motor_controllers[i]) motor_controllers[i]->spin(0);
}

static void prompt(void) { Serial.print("tool> "); }

void toolAbort(const char *why)
{
    toolMotorsOff();
    Serial.printf("\n[tool] STOPPED: %s -- motors off\n", why);
    syslog(LOG_INFO, "tool stopped: %s", why);
    longjmp(to_prompt, 1);
}

static void handleLine(char *line, bool busy);

// The input, from the console and the terminal, a line at a time.
static char con_buf[96];
static size_t con_len = 0;

static bool consoleLine(char *out, size_t n)
{
    while (Serial.available() > 0) {
        const int c = Serial.read();
        if (c < 0) break;
        if (c == '\r') continue;
        if (c == '\n') {
            Serial.print("\r\n");      // a serial terminal does not echo what is typed
            con_buf[con_len] = '\0';
            strncpy(out, con_buf, n - 1);
            out[n - 1] = '\0';
            con_len = 0;
            return true;
        }
        if (c == 0x08 || c == 0x7f) {                                      // backspace
            if (con_len) { con_len--; Serial.print("\b \b"); }
            continue;
        }
        if (c >= 0x20 && c < 0x7f && con_len < sizeof(con_buf) - 1) {
            con_buf[con_len++] = (char)c;
            Serial.write((uint8_t)c);
        }
    }
    return false;
}

static void pollInput(bool busy)
{
    char line[96];
    while (consoleLine(line, sizeof(line)))
        handleLine(line, busy);
    while (termLine(line, sizeof(line)))
        handleLine(line, busy);
    if (hostStopRequested()) {
        hostStopClear();
        if (busy) stop_flag = true;
    }
    // The terminal that started a drive went away: nobody can type `stop` now.
    if (termDropped() && busy && job_moves)
        stop_flag = true;
}

bool toolStopRequested(void)
{
    pollInput(job_name != nullptr);
    return stop_flag;
}

void toolWait(uint32_t ms)
{
    const uint32_t end = millis() + ms;
    for (;;) {
        runWifis();
        runOta();
        feedWatchdogFromTool();
        if (toolStopRequested())
            toolAbort("stop");
        const int32_t left = (int32_t)(end - millis());
        if (left <= 0)
            return;
        delay(left < 5 ? (uint32_t)left : 5u);
    }
}

// ---------------------------------------------------------------------------
// The boot's tool window.
bool toolEscape(void)
{
    const uint32_t wait = envU32("tool_wait", TOOL_WAIT_DEFAULT);
    if (!wait)
        return false;
    termBegin();
    Serial.printf("[boot] type 'tool' and Enter within %lu s for the tool%s%s; "
                  "micro-ROS after that\n", (unsigned long)((wait + 999) / 1000),
                  termAddress()[0] ? ", here or on tcp " : "", termAddress());
    char tail[8] = "";              // the last few characters typed
    size_t n = 0;
    char line[16];
    bool enter = false;
    const uint32_t t0 = millis();
    while (!enter && millis() - t0 < wait) {
        while (Serial.available() > 0) {
            const int c = Serial.read();
            if (c == '\r' || c == '\n') {
                tail[n] = '\0';
                // Trailing spaces forgiven; anything else typed before it is not.
                while (n && tail[n - 1] == ' ') tail[--n] = '\0';
                if (n >= 4 && strcasecmp(tail + n - 4, "tool") == 0 && (n == 4 || tail[n - 5] == ' '))
                    enter = true;
                n = 0;
            } else if (c >= 0x20 && c < 0x7f) {
                if (n == sizeof(tail) - 1) { memmove(tail, tail + 1, n - 1); n--; }
                tail[n++] = (char)c;
            } else {
                n = 0;                  // micro-ROS's binary frames never spell it
            }
        }
        if (termLine(line, sizeof(line)) && strcasecmp(line, "tool") == 0)
            enter = true;
        runOta();
        if (hostToolRequested())
            enter = true;
        delay(5);
    }
    if (!enter) {
        termEnd();
        return false;
    }
    app_name = "tool";
    envSetRealOnly(true);
    return true;
}

// ---------------------------------------------------------------------------
// The commands.
static void help(void)
{
    Serial.println(
        "commands (a word, then numbers; 'stop', 's' or an empty line stops):\n"
        "  i2c [secs]                scan the I2C bus (again every secs)\n"
        "  sensors                   IMU, magnetometer, battery, range, barometer, 1 Hz\n"
        "  motor <1-4|all> <pct> [secs]\n"
        "                            run a wheel at pct % power (minus = reverse) until stop\n"
        "  motors [pct]              each wheel forward then reverse, 8 s each\n"
        "  acc [ms]                  drivetrain test on the floor (run phase 300-1000 ms)\n"
#if ADC_LUT_SUPPORTED
        "  adc                       battery ADC calibration (DAC pin wired to the ADC)\n"
#endif
        "  set <key> <value>         change a setting until reboot (set <key> to clear)\n"
        "  get [key]                 show one setting, or all\n"
        "  status                    what this board has and what is running\n"
        "  base                      restart into the robot firmware (also: exit, reboot)");
}

static void printKey(const char *k, const char *v, bool overridden)
{
    Serial.printf("  %-16s %s%s\n", k, (strstr(k, "password") || strstr(k, "psk")) ? "********" : v,
                  overridden ? "   (set)" : "");
}

static void status(void)
{
    Serial.printf("[tool] app=%s, %s\n", app_name, job_name ? job_name : "idle");
    tool_hw::report();
    Serial.printf("[tool] wheels: %u, %s\n", tool_hw::total_motors,
                  envInt("sim_wheel", 0) == 1 ? "simulated" : "real");
    if (termAddress()[0])
        Serial.printf("[tool] terminal on tcp %s\n", termAddress());
}

// Numbers and keys are typed by a person: a word that is not a number is an error.
static bool toInt(const char *s, long *out)
{
    if (!s || !*s) return false;
    char *end;
    *out = strtol(s, &end, 10);
    return *end == '\0';
}

static bool setKey(const char *k, const char *v)
{
    if (!envSetOverride(k, v)) {
        Serial.printf("[tool] could not set %s\n", k);
        return false;
    }
    return true;
}

static void startStepped(const char *name, void (*begin)(void), bool (*step)(void), bool moves)
{
    stop_flag = false;
    hostStopClear();
    job_name = name;
    job_moves = moves;
    job_step = step;
    begin();
}

static void runBlocking(const char *name, void (*fn)(void), bool moves)
{
    stop_flag = false;
    hostStopClear();
    job_name = name;
    job_moves = moves;
    fn();                       // a stop longjmps to toolLoop's to_prompt
    toolMotorsOff();
    job_name = nullptr;
    job_moves = false;
}

static void handleLine(char *line, bool busy)
{
    // Words: the command and up to three arguments.
    char *argv[4] = {nullptr, nullptr, nullptr, nullptr};
    int argc = 0;
    for (char *t = strtok(line, " \t"); t && argc < 4; t = strtok(nullptr, " \t"))
        argv[argc++] = t;
    const char *cmd = argc ? argv[0] : "";
    for (char *p = (char *)cmd; *p; p++) *p = (char)tolower(*p);

    if (!*cmd || !strcmp(cmd, "stop") || !strcmp(cmd, "s")) {
        if (busy) stop_flag = true;
        else { toolMotorsOff(); prompt(); }
        return;
    }
    // While a command runs, only these: the rest waits for its prompt.
    if (!strcmp(cmd, "set")) {
        if (argc < 2) { Serial.println("[tool] set <key> <value>"); }
        else if (setKey(argv[1], argc >= 3 ? argv[2] : nullptr))
            Serial.printf("[tool] %s = %s\n", argv[1], argc >= 3 ? argv[2] : "(stored value)");
        if (!busy) prompt();
        return;
    }
    if (!strcmp(cmd, "get")) {
        if (argc >= 2) printKey(argv[1], envGet(argv[1], "(not set)"), false);
        else envForEach(printKey);
        if (!busy) prompt();
        return;
    }
    if (!strcmp(cmd, "status")) { status(); if (!busy) prompt(); return; }
    if (!strcmp(cmd, "help") || !strcmp(cmd, "?")) { help(); if (!busy) prompt(); return; }
    if (busy) {
        Serial.printf("[tool] busy: %s is running -- 'stop' first\n", job_name);
        return;
    }

    if (!strcmp(cmd, "base") || !strcmp(cmd, "exit") || !strcmp(cmd, "reboot"))
        toolRestart(false);
    const char *why = simulatedFor(cmd);
    if (why) {
        Serial.printf("[tool] '%s' tests %s, and this board's env simulates it - "
                      "the tool runs on a real robot only.\n", cmd, why);
        prompt();
        return;
    }
    long n;
    if (!strcmp(cmd, "i2c")) {
        if (i2cUnassigned()) {
            Serial.println("[tool] 'i2c' scans the I2C bus, and this board's env assigns no "
                           "I2C pins and names no fixed bus.");
            prompt();
            return;
        }
        i2c_every_ms = (argc >= 2 && toInt(argv[1], &n) && n > 0) ? (unsigned)n * 1000u : 0;
        runBlocking("i2c", i2c_detect_scan, false);
        if (i2c_every_ms) {
            job_name = "i2c";
            i2c_next = millis() + i2c_every_ms;
            stop_flag = false;
            return;              // toolLoop repeats it until stop
        }
    } else if (!strcmp(cmd, "sensors")) {
        startStepped("sensors", test_sensors_begin, test_sensors_step, false);
        return;
    } else if (!strcmp(cmd, "motor")) {
        // motor <1-4|all> <pct> [secs]: sugar for tm_only / tm_pct / tm_secs, which the
        // run reads every second -- so `set tm_pct 40` while it runs changes its power.
        long wheel = 0, pct = 0;
        const bool all = argc >= 2 && !strcmp(argv[1], "all");
        if (argc < 3 || (!all && (!toInt(argv[1], &wheel) || wheel < 1 || wheel > 4))
            || !toInt(argv[2], &pct) || pct < -100 || pct > 100 || pct == 0) {
            Serial.println("[tool] motor <1-4|all> <pct -100..100, not 0> [secs]");
            prompt();
            return;
        }
        char v[12];
        snprintf(v, sizeof(v), "%ld", all ? 5L : wheel); setKey("tm_only", v);
        snprintf(v, sizeof(v), "%ld", pct);             setKey("tm_pct", v);
        if (argc >= 4 && toInt(argv[3], &n) && n >= 0) {
            snprintf(v, sizeof(v), "%ld", n);           setKey("tm_secs", v);
        }
        startStepped("motor", test_motors_one_begin, test_motors_one_step, true);
        return;
    } else if (!strcmp(cmd, "motors")) {
        if (argc >= 2) {
            if (!toInt(argv[1], &n) || n < 1 || n > 100) {
                Serial.println("[tool] motors [pct 1..100]");
                prompt();
                return;
            }
            setKey("tm_pct", argv[1]);
        }
        startStepped("motors", test_motors_begin, test_motors_step, true);
        return;
    } else if (!strcmp(cmd, "acc")) {
        if (argc >= 2) {
            if (!toInt(argv[1], &n) || n < 300 || n > 1000) {
                Serial.println("[tool] acc [ms 300..1000]");
                prompt();
                return;
            }
            setKey("ta_ms", argv[1]);
        }
        runBlocking("acc", test_acc_run, true);
#if ADC_LUT_SUPPORTED
    } else if (!strcmp(cmd, "adc")) {
        runBlocking("adc", adc_calibrate_run, false);
#endif
    } else {
        Serial.printf("[tool] unknown command '%s' -- 'help' lists them\n", cmd);
    }
    prompt();
}

// ---------------------------------------------------------------------------
void toolSetup(void)
{
    // A Wi-Fi robot may be off the cable: every console line also goes to syslog, where
    // the cockpit's Monitor streams it, and to the terminal client once one connects.
#if LINO_CONSOLE_WRAPPED
    if (wifiWanted())
        lino_console.teeToSyslog(true);
#endif
    Serial.printf("[app] %s\n", app_name);
    tool_hw::init();
    termBegin();
    Serial.println("[tool] ready -- 'help' lists the commands");
    if (!boot_run || !*boot_run)
        prompt();
}

// The boot command (tool_run, or a former tool's name) runs from the first loop(), as
// the old tools' work did: announceBanner() and the radio are serviced there.
static bool boot_done = false;

void toolLoop(void)
{
    if (setjmp(to_prompt) != 0) {
        // A stop, from anywhere inside a command.
        job_step = nullptr;
        job_name = nullptr;
        job_moves = false;
        i2c_every_ms = 0;
        stop_flag = false;
        prompt();
        return;
    }
    if (!boot_done) {
        boot_done = true;
        if (boot_run && *boot_run) {
            Serial.printf("[tool] boot command: %s\n", boot_run);
            char line[48];
            strncpy(line, boot_run, sizeof(line) - 1);
            line[sizeof(line) - 1] = '\0';
            handleLine(line, false);
            return;
        }
    }
    pollInput(job_name != nullptr);
    if (stop_flag && job_name)
        toolAbort("stop");
    if (job_step) {
        if (!job_step()) {
            toolMotorsOff();
            Serial.printf("[tool] %s done\n", job_name);
            job_step = nullptr;
            job_name = nullptr;
            job_moves = false;
            prompt();
        }
    } else if (i2c_every_ms && (int32_t)(millis() - i2c_next) >= 0) {
        i2c_next = millis() + i2c_every_ms;
        i2c_detect_scan();
    }
}
