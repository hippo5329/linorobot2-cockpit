#include <Arduino.h>
#include <Wire.h>
#include <stdlib.h>
#include <string.h>
#include "config.h"
#include "board_init.h"
#include "mcu_env.h"

// Fallbacks, so this compiles against any generated header. A config without
// an i2c: block leaves the pins at -1, which means "whatever the core defaults
// to" -- the same behaviour as the bare Wire.begin() this replaced.
#ifndef SDA_PIN
#define SDA_PIN -1
#endif
#ifndef SCL_PIN
#define SCL_PIN -1
#endif
#ifndef I2C_CLOCK
#define I2C_CLOCK 400000
#endif
#ifndef BOARD_OUT_PINS
#define BOARD_OUT_PINS ""
#endif
#ifndef BOARD_OUT_PINS_LATE
#define BOARD_OUT_PINS_LATE ""
#endif
#ifndef BOOT_DELAY_MS
#define BOOT_DELAY_MS 0
#endif

static int envInt(const char *key, int fallback)
{
    const char *value = envGet(key, NULL);
    if (!value || !*value)
        return fallback;
    return (int)strtol(value, NULL, 10);
}


// "5=1,13=0,4=1" -- drive each pin to each level. A bare "5" means high, which
// is what a board enable line almost always wants and saves the user spelling
// out the common case. Anything unparseable is skipped rather than fatal: a
// typo in an env key should not brick a board into a boot loop.
static void driveOutputs(const char *spec)
{
    if (!spec)
        return;
    while (*spec)
    {
        while (*spec == ',' || *spec == ' ')
            spec++;
        if (!*spec)
            break;

        char *end = NULL;
        long pin = strtol(spec, &end, 10);
        if (end == spec)
        {
            // Not a number -- skip to the next comma rather than spin here.
            while (*spec && *spec != ',')
                spec++;
            continue;
        }
        spec = end;

        int level = HIGH;
        if (*spec == '=')
            level = (strtol(spec + 1, &end, 10) != 0) ? HIGH : LOW;
        while (*spec && *spec != ',')
            spec++;

        if (pin >= 0)
        {
            pinMode((uint8_t)pin, OUTPUT);
            digitalWrite((uint8_t)pin, level);
        }
    }
}

void initBoard(void)
{
    // The env is read here and not in a constructor: the flash partition API is
    // not usable during static initialisation. initMcuEnv() is idempotent, so
    // whichever of the boot-time callers runs first pays for it.
    initMcuEnv();

    const int sda = envInt("i2c_sda", SDA_PIN);
    const int scl = envInt("i2c_scl", SCL_PIN);
    const uint32_t clock = envU32("i2c_clock", I2C_CLOCK);

#if defined(ESP32) || defined(ARDUINO_ARCH_ESP32)
    if (sda >= 0 && scl >= 0)
        Wire.begin(sda, scl);
    else
        Wire.begin();
#elif defined(ARDUINO_ARCH_RP2040) || defined(ARDUINO_ARCH_RP2350)
    // arduino-pico takes the pins before begin(), not as arguments to it.
    if (sda >= 0)
        Wire.setSDA(sda);
    if (scl >= 0)
        Wire.setSCL(scl);
    Wire.begin();
#else
    (void)sda;
    (void)scl;
    Wire.begin();
#endif
    if (clock)
        Wire.setClock(clock);

    driveOutputs(envGet("gpio_out", BOARD_OUT_PINS));

    const uint32_t settle = envU32("boot_delay", BOOT_DELAY_MS);
    if (settle)
        delay(settle);
}

void initBoardLate(void)
{
    driveOutputs(envGet("gpio_out_late", BOARD_OUT_PINS_LATE));
}
