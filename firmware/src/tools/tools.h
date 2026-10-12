// Tool mode: one image, several applications, selected at boot.
//
// The diagnostics used to be six separate PlatformIO projects, each producing
// its own binary. Swapping between them meant a reflash, which on a robot that
// is already assembled is the expensive step -- and the reason a user reaches
// for a diagnostic at all is usually that the robot is misbehaving in place.
//
// They are one image now, chosen by the `app` key in the env partition, so
// moving between the robot firmware and a sensor scan is a 4 KB env rewrite
// (or, on ESP32, no reflash at all) rather than a rebuild.
//
// And one application (2026-10-12, user: "merge tools into one, use terminal command
// to drive i2c, sensors, motors, acc ... so that we have only two modes, base and
// tool"): `app=tool` boots a command shell on the console -- USB serial, or on a
// Wi-Fi robot a TCP terminal -- and each former tool is one of its commands. The old
// app names still boot the tool, running their command at once (`app=test_acc` is
// `app=tool` + `tool_run=acc`), so a host that writes one keeps working.
//
// Two rules keep this from costing the robot firmware anything:
//
//   * Each tool is its own translation unit inside its own namespace, exposing
//     nothing but a setup/loop pair. Several tools and `base` independently
//     defined `imu_msg`, `battery_msg`, `setLed` and motor objects at file
//     scope; linked together those are duplicate symbols.
//   * No tool owns hardware. The drivetrain comes from hw_factory, which builds
//     it in setup() from the env partition -- so a tool cannot run a Motor
//     constructor at boot for a mode the user did not select, and a tool sees
//     the same wiring the robot firmware does.
//
// `blink` is gone entirely. It existed as a standalone image so it could be
// flashed when the main one would not run -- but with a single image there is
// nothing smaller to fall back TO, and flashing the image that just failed is
// not a recovery. Folding it in instead would have kept the maintenance and lost
// the only thing that justified it: as an app it answers nothing this image does
// not already answer, because `[app] <name>` goes out over serial at boot and
// ledInit()/setLed drive the LED regardless of mode. A board that will not run
// the firmware is recovered through BOOTSEL / the 1200-baud touch and a reflash,
// which is what actually recovered one before.
#ifndef TOOLS_H
#define TOOLS_H

#include <stdint.h>

enum AppMode {
    APP_BASE = 0,       // the robot firmware; the default when `app` is unset
    APP_TOOL,           // the command shell (tool_shell.cpp)
};

// Reads the `app` env key: `tool` (or a former tool's name) boots the shell, anything
// else the robot firmware, so a typo boots the robot rather than nothing.
AppMode toolSelect(void);

// Name of a mode, for the banner's `app=`.
const char *toolName(AppMode mode);

// The boot's tool window (user, 2026-10-12: "only one mode, base boot wait for tool
// escape for 3 or 5 seconds then proceed to micro-ros"). Called by base after the radio
// is up and before micro-ROS: for `tool_wait` ms (3000; 0 = none) it listens for the
// word `tool` and Enter on the console or the Wi-Fi terminal, or "lino-tool" on the ping
// port. True: the tool takes over (no micro-ROS this boot).
bool toolEscape(void);
// "lino-tool" while base runs: restart, and come up in the tool (a RAM flag that
// survives a soft reset, not an env write). With tool=false, back to base.
[[noreturn]] void toolRestart(bool tool);

// The shell. Not called for APP_BASE.
void toolSetup(void);
void toolLoop(void);

// ---- for the commands (tool_shell.cpp) ----
// True once `stop` (or `s`, or an empty line) arrived on the console or the terminal,
// or "lino-stop" on the ping port. Reads the input as it goes, so a long command that
// calls it is stoppable mid-run -- and `set` works while a command runs.
bool toolStopRequested(void);
// Waits `ms`, servicing the radio and the input; a stop ends the command (toolAbort).
void toolWait(uint32_t ms);
// Motors off, a line saying why, and back to the prompt from wherever the command was.
// toolStopReason() is what asked: "stop", "lino-stop" or "the terminal went away".
const char *toolStopReason(void);
[[noreturn]] void toolAbort(const char *why);
// The same, printing nothing: for a command that has said it in its own words.
[[noreturn]] void toolAbortQuiet(const char *why);
// Motors off, all four.
void toolMotorsOff(void);
// The env names no I2C bus: neither both pins nor a fixed bus (`i2c_bus`). Nothing scans
// a bus nobody wired -- its default pins may be a motor's.
bool toolI2cUnassigned(void);

// main.cpp's watchdog, for the tool: armed in toolSetup(), fed from every wait.
void linoWdtBegin(void);
void linoWdtFeed(void);

#endif // TOOLS_H
