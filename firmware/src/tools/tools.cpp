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
// Which application boots. One mode (user, 2026-10-12): `base`, which waits a moment for
// the tool's escape before micro-ROS (toolEscape). `app=tool` boots the tool directly,
// for a host, and so do the former tools' names, running their command, so a host that
// writes `app=test_acc` gets what it always got -- including the old refusal, which
// boots the robot firmware instead.
struct Legacy { const char *app; const char *run; };
static const Legacy LEGACY[] = {
    {"test_sensors",  "sensors"},
    {"test_motors",   "motors"},     // or `motor`, the env's one wheel (tm_only 1-5)
    {"test_acc",      "acc"},
    {"i2c_detect",    "i2c 5"},     // it scanned every 5 s
    {"adc_calibrate", "adc"},
};

static const char *app_name = "base";
static const char *boot_run = "";

// The next boot's mode, set before a restart: "come up in the tool" (lino-tool, or the
// tool's own restart) or "come up in base" (`base` typed in a tool the env boots, which
// would otherwise come straight back). Survives a soft reset, not a power cycle, so a
// robot never stays out of micro-ROS by accident.
#define NEXT_BOOT_TOOL 0x544F4F4Cu      // "TOOL"
#define NEXT_BOOT_BASE 0x42415345u      // "BASE"
#if defined(ARDUINO_ARCH_ESP32)
#include <esp_attr.h>
RTC_NOINIT_ATTR static uint32_t next_boot_ram;
static uint32_t nextBootGet(void) { return next_boot_ram; }
static void nextBootSet(uint32_t v) { next_boot_ram = v; }
#elif defined(ARDUINO_ARCH_RP2040)
// A watchdog scratch register: the SDK keeps 0-3 for the application and the reboot
// path (watchdog_reboot) leaves them alone, on the RP2040 and the RP2350 alike.
#include <hardware/structs/watchdog.h>
static uint32_t nextBootGet(void) { return watchdog_hw->scratch[0]; }
static void nextBootSet(uint32_t v) { watchdog_hw->scratch[0] = v; }
#elif defined(LINO_ZEPHYR)
#include <zephyr/sys/reboot.h>
static __noinit uint32_t next_boot_ram;
static uint32_t nextBootGet(void) { return next_boot_ram; }
static void nextBootSet(uint32_t v) { next_boot_ram = v; }
#elif defined(LINO_HOST)
// The Sim MCU restarts by exec'ing itself; the flag rides in its environment.
#include <unistd.h>
static uint32_t next_boot_host = 0;
static uint32_t nextBootGet(void)
{
    const char *f = getenv("LINO_NEXT_BOOT");
    if (f && strcmp(f, "tool") == 0) next_boot_host = NEXT_BOOT_TOOL;
    if (f && strcmp(f, "base") == 0) next_boot_host = NEXT_BOOT_BASE;
    unsetenv("LINO_NEXT_BOOT");
    return next_boot_host;
}
static void nextBootSet(uint32_t v)
{
    next_boot_host = v;
    setenv("LINO_NEXT_BOOT", v == NEXT_BOOT_TOOL ? "tool" : v == NEXT_BOOT_BASE ? "base" : "", 1);
}
#else
static uint32_t next_boot_plain = 0;
static uint32_t nextBootGet(void) { return next_boot_plain; }
static void nextBootSet(uint32_t v) { next_boot_plain = v; }
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
    nextBootSet(tool ? NEXT_BOOT_TOOL : NEXT_BOOT_BASE);
    Serial.printf("[tool] restarting into %s\n", tool ? "the tool" : "base");
    Serial.flush();
    delay(100);
#if defined(ARDUINO_ARCH_ESP32)
    ESP.restart();
#elif defined(ARDUINO_ARCH_RP2040)
    rp2040.reboot();
#elif defined(LINO_ZEPHYR)
    sys_reboot(SYS_REBOOT_COLD);
#elif defined(LINO_HOST)
    execl("/proc/self/exe", "lino_host_fw", (char *)NULL);
#endif
    for (;;) delay(1000);      // a board that cannot restart itself: power-cycle it
}

// The former tools' refusals, unchanged: a refused app name boots the robot firmware,
// as it always did, so an env written without the host's check never leaves a robot
// out of micro-ROS. Same rules as scripts/mcu_env.py tool_refusal.
static const char *legacyRefusal(const char *app)
{
    if (!strcmp(app, "test_sensors"))
        return strcmp(envGet("imu", ""), "sim") == 0 ? "tests the IMU, and this board's env simulates it" : NULL;
    if (!strcmp(app, "test_motors") || !strcmp(app, "test_acc"))
        return envInt("sim_wheel", 0) == 1 ? "tests the wheels, and this board's env simulates them" : NULL;
    if (!strcmp(app, "adc_calibrate")) {
#if !ADC_LUT_SUPPORTED
        return "needs a hardware DAC, which this MCU does not have";
#else
        return envInt("sim_battery", 0) == 1 ? "tests the battery, and this board's env simulates it" : NULL;
#endif
    }
    if (!strcmp(app, "i2c_detect"))
        return toolI2cUnassigned() ? "scans the I2C bus, and this board's env assigns no I2C pins" : NULL;
    return NULL;
}

AppMode toolSelect(void)
{
    initMcuEnv();
    const char *want = envGet("app", "base");
    // `base` is the robot, not a command: it returns before anything that belongs to the
    // tool (rc-20261012.2: base turned real-only and faulted).
    const uint32_t next = nextBootGet();
    nextBootSet(0);                  // once: the reset after is the env's again
    if (next == NEXT_BOOT_BASE)
        return APP_BASE;
    if (next == NEXT_BOOT_TOOL) {
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
        const char *why = legacyRefusal(want);
        if (why) {
            Serial.printf("[app] '%s' %s - the tools run on a real robot only. Starting the "
                          "robot firmware instead.\n", want, why);
            return APP_BASE;
        }
        app_name = LEGACY[i].app;
        boot_run = LEGACY[i].run;
        // test_motors with tm_only 1-5 in the env ran that one wheel (or all four).
        if (!strcmp(want, "test_motors")) {
            const int only = envInt("tm_only", 0);
            if (only >= 1 && only <= 5) boot_run = "motor";
        }
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
bool toolI2cUnassigned(void)
{
    const char *bus = envGet("i2c_bus", "");
    if (bus && bus[0])
        return false;
    return envInt("i2c_sda", -1) < 0 || envInt("i2c_scl", -1) < 0;
}
static bool i2cUnassigned(void) { return toolI2cUnassigned(); }

// `adc` drives the DAC pin and reads the battery pin: never one a motor driver owns
// (the GenDrv's default DAC pin, 25, is its motor PWM).
static const char *adcPinConflict(void)
{
    static const char *suffix[] = {"pwm", "in_a", "in_b"};
    const int dac = envInt("dac_pin", 25), bat = envInt("battery_pin", -1);
    char key[12];
    for (int m = 1; m <= 4; m++)
        for (const char *s : suffix) {
            snprintf(key, sizeof(key), "m%d_%s", m, s);
            const int pin = envInt(key, -1);
            if (pin >= 0 && (pin == dac || pin == bat))
                return pin == dac ? "the DAC pin" : "the battery pin";
        }
    return NULL;
}

// ---------------------------------------------------------------------------
// Stop, and the jump back to the prompt.
static jmp_buf to_prompt;
static bool stop_flag = false;
static const char *stop_why = "stop";      // what the STOPPED line says
static bool (*job_step)(void) = nullptr;   // the running stepped command
static const char *job_name = nullptr;     // stepped or blocking, while it runs
static bool job_moves = false;             // it drives the wheels
static bool job_from_term = false;         // typed on the Wi-Fi terminal
static bool line_from_term = false;        // the line being handled came from it
static unsigned i2c_every_ms = 0;          // `i2c <secs>`
static uint32_t i2c_next = 0;

// A drive started from the network terminal stops when the terminal goes quiet: a link
// that dies without a FIN still reads as connected for minutes, and nobody could type
// `stop` into it. lino_tool.py sends a keep-alive (NUL) every second.
#define TERM_DEADMAN_MS 3000

void toolMotorsOff(void)
{
    for (int i = 0; i < 4; i++)
        if (motor_controllers[i]) motor_controllers[i]->spin(0);
}

static void prompt(void) { Serial.print("tool> "); }

const char *toolStopReason(void) { return stop_why; }

void toolAbort(const char *why)
{
    toolMotorsOff();
    Serial.printf("\n[tool] STOPPED: %s -- motors off\n", why);
    syslog(LOG_INFO, "tool stopped: %s", why);
    longjmp(to_prompt, 1);
}

static void handleLine(char *line, bool busy);

// The console, a line at a time. A line ends at CR or LF -- minicom, screen and PuTTY
// send CR alone -- and the LF of a CR LF pair is swallowed.
static char con_buf[96];
static size_t con_len = 0;
static bool con_cr = false;

static bool consoleLine(char *out, size_t n)
{
    while (Serial.available() > 0) {
        const int c = Serial.read();
        if (c < 0) break;
        if (c == '\n' && con_cr) { con_cr = false; continue; }
        con_cr = (c == '\r');
        if (c == '\r' || c == '\n') {
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
    while (consoleLine(line, sizeof(line))) {
        line_from_term = false;
        handleLine(line, busy);
    }
    while (termLine(line, sizeof(line))) {
        line_from_term = true;
        handleLine(line, busy);
        line_from_term = false;
    }
    if (hostStopRequested()) {
        hostStopClear();
        if (busy) { stop_flag = true; stop_why = "lino-stop"; }
    }
    if (busy && job_moves && job_from_term) {
        // The terminal that started a drive went away, or went quiet: nobody can type
        // `stop` into it now.
        if (termDropped()) {
            stop_flag = true;
            stop_why = "the terminal went away";
        } else if (millis() - termLastInput() > TERM_DEADMAN_MS) {
            stop_flag = true;
            stop_why = "the terminal went quiet";
        }
    } else {
        termDropped();          // read once: a drop between commands is no stop
    }
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
        linoWdtFeed();
        if (toolStopRequested())
            toolAbort(stop_why);
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
    // Exactly `tool` (any case), trailing spaces forgiven. A line is printable bytes
    // between line ends; micro-ROS's binary frames reset it, so they never spell it.
    char typed[8];
    size_t n = 0;
    bool spoiled = false;           // longer than `tool`, or a byte that is not text
    char line[16];
    bool enter = false;
    const uint32_t t0 = millis();
    while (!enter && millis() - t0 < wait) {
        while (Serial.available() > 0) {
            const int c = Serial.read();
            if (c == '\r' || c == '\n') {
                while (n && typed[n - 1] == ' ') n--;
                typed[n] = '\0';
                if (!spoiled && n == 4 && strcasecmp(typed, "tool") == 0)
                    enter = true;
                n = 0;
                spoiled = false;
            } else if (c >= 0x20 && c < 0x7f) {
                if (n < sizeof(typed) - 1) typed[n++] = (char)c;
                else spoiled = true;
            } else {
                n = 0;
                spoiled = true;
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
        "                            run a wheel at pct % power (minus = reverse); 120 s,\n"
        "                            or secs (0: until stop). `motor` alone: the env's.\n"
        "  motors [pct]              each wheel forward then reverse, 8 s each\n"
        "  acc [ms]                  drivetrain test on the floor (run phase 300-1000 ms)\n"
#if ADC_LUT_SUPPORTED
        "  adc                       battery ADC calibration (DAC pin wired to the ADC)\n"
#endif
        "  set <key> <value>         a test setting until reboot (set <key> clears it):\n"
        "                            tm_pct tm_only tm_secs ta_ms sim_wheel sim_battery\n"
        "  get [key]                 show one setting, or all\n"
        "  status                    what this board has and what is running\n"
        "  base                      restart into the robot firmware (also: exit, reboot)");
}

static bool secretKey(const char *k)
{
    return strstr(k, "password") || strstr(k, "psk") || strstr(k, "ssid");
}

static void printKey(const char *k, const char *v, bool overridden)
{
    Serial.printf("  %-16s %s%s\n", k, secretKey(k) ? "********" : v, overridden ? "   (set)" : "");
}

// What `set` may change: the tests' own settings, read while a command runs. Everything
// else is read once at boot (pins, PWM width, drivers -- a new value would describe a
// drivetrain that was not built) or is a secret: flash the env to change those.
static bool settable(const char *k)
{
    static const char *const OK[] = {"tm_pct", "tm_only", "tm_secs", "ta_ms", "sim_wheel", "sim_battery"};
    for (const char *o : OK)
        if (!strcmp(k, o)) return true;
    return false;
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
        Serial.printf("[tool] could not set %s (16 settings at most)\n", k);
        return false;
    }
    return true;
}

static void startJob(const char *name, bool moves)
{
    stop_flag = false;
    stop_why = "stop";
    hostStopClear();
    termDropped();
    termTouch();
    job_name = name;
    job_moves = moves;
    job_from_term = line_from_term;
}

static void startStepped(const char *name, void (*begin)(void), bool (*step)(void), bool moves)
{
    startJob(name, moves);
    job_step = step;
    begin();
}

static void runBlocking(const char *name, void (*fn)(void), bool moves)
{
    startJob(name, moves);
    fn();                       // a stop longjmps to toolLoop's to_prompt
    toolMotorsOff();
    job_name = nullptr;
    job_moves = false;
}

static void usage(const char *what)
{
    Serial.printf("[tool] %s\n", what);
    prompt();
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
        if (busy) { stop_flag = true; stop_why = "stop"; }
        else { toolMotorsOff(); prompt(); }
        return;
    }
    // While a command runs, only these: the rest waits for its prompt.
    if (!strcmp(cmd, "set")) {
        if (argc < 2)
            Serial.println("[tool] set <key> <value>");
        else if (!settable(argv[1]))
            Serial.printf("[tool] %s is not a test setting -- write the env to change it\n", argv[1]);
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
        if (i2cUnassigned())
            return usage("'i2c' scans the I2C bus, and this board's env assigns no "
                         "I2C pins and names no fixed bus.");
        if (argc >= 2 && (!toInt(argv[1], &n) || n < 1))
            return usage("i2c [secs >= 1]");
        i2c_every_ms = argc >= 2 ? (unsigned)n * 1000u : 0;
        runBlocking("i2c", i2c_detect_scan, false);
        if (i2c_every_ms) {
            startJob("i2c", false);
            i2c_next = millis() + i2c_every_ms;
            return;              // toolLoop repeats it until stop
        }
    } else if (!strcmp(cmd, "sensors")) {
        startStepped("sensors", test_sensors_begin, test_sensors_step, false);
        return;
    } else if (!strcmp(cmd, "motor")) {
        // motor <1-4|all> <pct> [secs]: sugar for tm_only / tm_pct / tm_secs, which the run
        // reads every second -- so `set tm_pct 40` while it runs changes its power. Each
        // `motor` sets all three (secs absent: the stored value, else 120), so nothing
        // carries over from the last one. `motor` alone runs the env's (app=test_motors).
        if (argc > 1) {
            long wheel = 0, pct = 0, secs = -1;
            const bool all = !strcmp(argv[1], "all");
            if (argc < 3 || (!all && (!toInt(argv[1], &wheel) || wheel < 1 || wheel > 4))
                || !toInt(argv[2], &pct) || pct < -100 || pct > 100 || pct == 0
                || (argc >= 4 && (!toInt(argv[3], &secs) || secs < 0)))
                return usage("motor <1-4|all> <pct -100..100, not 0> [secs, 0 = until stop]");
            char a[12], b[12], c[12];
            snprintf(a, sizeof(a), "%ld", all ? 5L : wheel);
            snprintf(b, sizeof(b), "%ld", pct);
            snprintf(c, sizeof(c), "%ld", secs);
            if (!setKey("tm_only", a) || !setKey("tm_pct", b) || !setKey("tm_secs", secs >= 0 ? c : nullptr)) {
                prompt();
                return;
            }
        } else {
            const int only = envInt("tm_only", 0);
            if (only < 1 || only > 5)
                return usage("motor <1-4|all> <pct> [secs] (or tm_only 1-5 in the env)");
        }
        startStepped("motor", test_motors_one_begin, test_motors_one_step, true);
        return;
    } else if (!strcmp(cmd, "motors")) {
        if (argc >= 2 && (!toInt(argv[1], &n) || n < 1 || n > 100))
            return usage("motors [pct 1..100]");
        // Its own power, or the stored one: never the last `motor`'s.
        if (!setKey("tm_pct", argc >= 2 ? argv[1] : nullptr)) { prompt(); return; }
        startStepped("motors", test_motors_begin, test_motors_step, true);
        return;
    } else if (!strcmp(cmd, "acc")) {
        if (argc >= 2 && (!toInt(argv[1], &n) || n < 300 || n > 1000))
            return usage("acc [ms 300..1000]");
        if (!setKey("ta_ms", argc >= 2 ? argv[1] : nullptr)) { prompt(); return; }
        runBlocking("acc", test_acc_run, true);
#if ADC_LUT_SUPPORTED
    } else if (!strcmp(cmd, "adc")) {
        const char *pin = adcPinConflict();
        if (pin) {
            Serial.printf("[tool] 'adc' would drive %s, which a motor driver uses -- set "
                          "dac_pin / battery_pin in the env to free pins\n", pin);
            prompt();
            return;
        }
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
    // The same watchdog as base: a command stuck on a bus or a network write must not
    // leave a motor driven. Every wait in a command feeds it (toolWait, waitMs, polls).
    linoWdtBegin();
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
        job_from_term = false;
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
        toolAbort(stop_why);
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
