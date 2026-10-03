#pragma once

// linorobot2-cockpit: arduino-pico's rpipico2w variant, built for the RP2350B's
// 48 GPIO (PICO_RP2350A 0) so the one Pico 2 image also drives an RP2350B board --
// the SparkFun XRP Controller's motors (GP32-35), left encoder (GP30/31), IMU bus
// (GP38/39) and battery/current ADC (GP40-47). On an RP2350A those pins simply do
// not exist and no Pico 2 config names them. The two things that differ by package
// at run time are handled in the firmware, not here: the ADC channel of a pin
// (adc_pin.h, from SYSINFO package_sel) and the CYW43 pins (init.cpp, from
// the env's cyw43_pins), the I2C controller of the env's
// SDA/SCL (board_init.cpp). Everything else is the stock file.

#include <cyw43_wrappers.h>

// Pin definitions taken from:
//    https://datasheets.raspberrypi.org/pico/pico-datasheet.pdf

#define PICO_RP2350A 0 // RP2350B pin numbering: see the note at the top

// LEDs
#define PIN_LED        (64u)

// Serial
#define PIN_SERIAL1_TX (0u)
#define PIN_SERIAL1_RX (1u)

#define PIN_SERIAL2_TX (8u)
#define PIN_SERIAL2_RX (9u)

// SPI
#define PIN_SPI0_MISO  (16u)
#define PIN_SPI0_MOSI  (19u)
#define PIN_SPI0_SCK   (18u)
#define PIN_SPI0_SS    (17u)

#define PIN_SPI1_MISO  (12u)
#define PIN_SPI1_MOSI  (15u)
#define PIN_SPI1_SCK   (14u)
#define PIN_SPI1_SS    (13u)

// Wire
#define PIN_WIRE0_SDA  (4u)
#define PIN_WIRE0_SCL  (5u)

#define PIN_WIRE1_SDA  (26u)
#define PIN_WIRE1_SCL  (27u)

#define SERIAL_HOWMANY (3u)
#define SPI_HOWMANY    (2u)
#define WIRE_HOWMANY   (2u)

#include <generic/common.h>
