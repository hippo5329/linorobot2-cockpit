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

enum AppMode {
    APP_BASE = 0,       // the robot firmware; the default when `app` is unset
    APP_TEST_SENSORS,
    APP_TEST_MOTORS,
    APP_TEST_ACC,
    APP_I2C_DETECT,
    APP_BNO085_CAL,
    APP_ADC_CALIBRATE,
};

// Reads the `app` env key. Falls back to APP_BASE for an unset, unknown or
// unsupported value, so a typo boots the robot firmware rather than nothing.
AppMode toolSelect(void);

// Name of a mode, for logging and for the "unknown app" message.
const char *toolName(AppMode mode);

// Dispatch. Not called for APP_BASE.
void toolSetup(AppMode mode);
void toolLoop(AppMode mode);

#endif // TOOLS_H
