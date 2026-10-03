#include <Arduino.h>
#include <Wire.h>
#include <stdlib.h>
#include <string.h>
#include "config.h"
#include "board_init.h"
#include "mcu_env.h"

#if defined(ARDUINO_ARCH_RP2040) || defined(ARDUINO_ARCH_RP2350)
#include <new>
#include <hardware/i2c.h>
#endif
#if defined(PICO2W) && defined(CYW43_PIN_WL_DYNAMIC) && CYW43_PIN_WL_DYNAMIC
#include <cyw43_wrappers.h>
#endif

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

#if defined(PICO2W) && defined(CYW43_PIN_WL_DYNAMIC) && CYW43_PIN_WL_DYNAMIC
// The radio's pins, from the env, before arduino-pico starts the CYW43 driver
// (variants/rpipico2w_lino/init.cpp calls this from initVariant()). One Pico 2 W
// image runs a Pico 2 W -- whose wiring is the driver's default -- and the
// SparkFun XRP Controller, whose RM2 is on GP26/29/28/27:
//     cyw43_pins = REG_ON,DATA,CLOCK,CS          e.g. 26,29,28,27 on the XRP
// DATA is the one bidirectional SPI line and also the host-wake IRQ, as on every
// CYW43 board (pico-sdk boards/sparkfun_xrp_controller.h). Absent or malformed,
// the defaults stand. Runs before setup(), so it uses envPeek(): nothing is loaded
// or printed here.
extern "C" void lino_cyw43_pins(void)
{
    const char *spec = envPeek("cyw43_pins", "");
    int v[4];
    int n = 0;
    while (*spec && n < 4)
    {
        char *end = NULL;
        long x = strtol(spec, &end, 10);
        if (end == spec || x < 0 || x >= NUM_BANK0_GPIOS)
            return;
        v[n++] = (int)x;
        spec = end;
        while (*spec == ',' || *spec == ' ')
            spec++;
    }
    if (n != 4)
        return;
    static uint pins[CYW43_PIN_INDEX_WL_COUNT];
    pins[CYW43_PIN_INDEX_WL_REG_ON] = (uint)v[0];
    pins[CYW43_PIN_INDEX_WL_DATA_OUT] = (uint)v[1];
    pins[CYW43_PIN_INDEX_WL_DATA_IN] = (uint)v[1];
    pins[CYW43_PIN_INDEX_WL_HOST_WAKE] = (uint)v[1];
    pins[CYW43_PIN_INDEX_WL_CLOCK] = (uint)v[2];
    pins[CYW43_PIN_INDEX_WL_CS] = (uint)v[3];
    cyw43_set_pins_wl(pins);
}
#endif

#if defined(ARDUINO_ARCH_RP2040) || defined(ARDUINO_ARCH_RP2350)
// An RP2 pin's I2C controller: GP0/1 I2C0, GP2/3 I2C1, GP4/5 I2C0, ... with SDA
// on the even pin and SCL on the odd one (RP2040/RP2350 GPIO function tables).
static int rp2I2cOf(int pin) { return (pin >> 1) & 1; }
#endif

#if defined(ARDUINO_ARCH_RP2040) || defined(ARDUINO_ARCH_RP2350)
void boardI2cPins(int sda, int scl)
{
    // arduino-pico takes the pins before begin(), not as arguments to it -- and
    // only pins of the controller `Wire` was built on: setSDA() halts the core on
    // any other. `Wire` is I2C0, so a bus on I2C1 pins (the XRP's IMU on GP38/39,
    // or GP2/3, GP26/27 on a Pico) rebuilds it on I2C1 first. TwoWire has no
    // destructor and its constructor only fills fields and allocates the buffer,
    // so rebuilding it before its first begin() is safe; the first buffer
    // (WIRE_BUFFER_SIZE) is lost once, at boot. A pair that is not one controller's
    // SDA and SCL is reported and the core defaults kept, rather than halting.
    static bool wire_placed = false;
    if (sda >= 0 && scl >= 0)
    {
        const bool pair = (sda % 2 == 0) && (scl % 2 == 1) && rp2I2cOf(sda) == rp2I2cOf(scl)
                          && sda < NUM_BANK0_GPIOS && scl < NUM_BANK0_GPIOS;
        if (!pair)
        {
            Serial.printf("[i2c] SDA %d / SCL %d is not one RP2 I2C controller's pair -- "
                          "keeping the core's default pins\n", sda, scl);
            return;
        }
        if (rp2I2cOf(sda) == 1 && !wire_placed)
            new (&Wire) TwoWire(i2c1, sda, scl);
        wire_placed = true;
        Wire.setSDA(sda);
        Wire.setSCL(scl);
        return;
    }
    if (sda >= 0)
        Wire.setSDA(sda);
    if (scl >= 0)
        Wire.setSCL(scl);
}
#endif

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
    boardI2cPins(sda, scl);
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
