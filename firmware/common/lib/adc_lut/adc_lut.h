// ADC linearisation table, stored in its own flash partition.
//
// The ESP32 SAR ADC is markedly non-linear. firmware/adc_calibrate measures the
// curve by sweeping the hardware DAC into an ADC pin and inverts it into a
// 4096-entry lookup table mapping a raw reading to the value a linear converter
// would have produced.
//
// That table used to be printed as C source for the user to paste into a config
// header and rebuild. Under tool mode there is nothing to paste into: the tool
// and the robot firmware are the same image, selected by an env key. So the LUT
// lives in the `adclut` partition (firmware/common/partitions_lino.csv) exactly
// as the Wi-Fi keys live in `env` -- calibrating a board rewrites 12 KB of flash
// and never involves a compiler.
//
// Neither side holds the table in RAM. The writer streams it out in blocks; the
// reader maps it, so a lookup reads straight out of XIP-mapped flash.
#ifndef ADC_LUT_H
#define ADC_LUT_H

#include <Arduino.h>

// Which MCUs this whole facility exists on.
//
// Building the table means sweeping a hardware DAC into an ADC pin, and only the
// classic ESP32 and the ESP32-S2 have a DAC. The ESP32-S3, C3, C6 and H2 do not,
// and neither do the RP2040/RP2350 -- so on those boards there is no way to
// produce a table, and a table produced elsewhere would describe a different
// chip's ADC anyway. The store is therefore gated on exactly the same condition
// as the tool: one fact, stated once, so the reader and the generator cannot
// disagree about which boards are in scope.
//
// On every other MCU the accessors below compile to stubs, adcLinearize()
// returns its argument, and firmware/adc_calibrate must not be offered at all.
#if defined(CONFIG_IDF_TARGET_ESP32) || defined(CONFIG_IDF_TARGET_ESP32S2) || \
    (defined(ESP32) && !defined(CONFIG_IDF_TARGET_ESP32S3) && !defined(CONFIG_IDF_TARGET_ESP32C3) && \
     !defined(CONFIG_IDF_TARGET_ESP32C6) && !defined(CONFIG_IDF_TARGET_ESP32C2) && !defined(CONFIG_IDF_TARGET_ESP32H2))
#define ADC_LUT_SUPPORTED 1
#else
#define ADC_LUT_SUPPORTED 0
#endif

#define ADC_LUT_ENTRIES 4096

// Maps the `adclut` partition if it carries a table whose CRC32 matches its
// header. Safe to call when no table has ever been written.
void initAdcLut(void);

// true once initAdcLut() has found a valid table. When false adcLinearize()
// returns its argument unchanged, so an uncalibrated board still reads its ADC.
bool adcLutValid(void);

// raw 12-bit reading -> linearised 12-bit value.
uint16_t adcLinearize(uint16_t raw);

// Writer, used by the adc_calibrate tool. Erases the partition, takes the table
// in blocks of any size (the generator emits 256 entries at a time so it never
// holds more than 512 bytes of output), then commits the header. Committing
// last means an interrupted calibration leaves an invalid table, never a
// half-written one that reads as good.
bool adcLutBeginWrite(void);
bool adcLutWriteBlock(const int16_t *values, size_t count);
bool adcLutEndWrite(void);

#endif // ADC_LUT_H
