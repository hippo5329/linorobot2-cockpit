// Copyright (c) 2024 Thomas Chou, Linorobot contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * ADC Calibration Utility
 * Builds an ESP32 ADC Lookup Table (LUT) to linearize ADC readings.
 *
 * Based on original work from Helmut Weber & Henry Cheung.
 *
 * The sweep drives the hardware DAC through its 256 codes and records what the
 * ADC makes of each one. That measured curve is monotone and piecewise linear
 * between the 256 knots, so its inverse -- which is what a linearisation table
 * IS -- can be evaluated directly from the knots.
 *
 * The original built the inverse by materialising the curve at 4096 points
 * (`Results`, 16 KB), upsampling THAT five-fold into `Res2` (80 KB), and then,
 * for each of 4096 targets, scanning all 20480 entries of Res2 for the nearest
 * value: 98,308 bytes of static RAM and 84 million comparisons on a chip with
 * ~320 KB of DRAM. Both tables are pure functions of the 257 knots, so neither
 * needs to exist. Keeping only the knots costs 1,028 bytes, and because the
 * curve is monotone the inverse is one ascending walk over the knots instead of
 * a search. The result is exact rather than quantised to the 1/5-step grid.
 *
 * This matters beyond tidiness: under tool mode this file is linked into the
 * same image as the robot firmware, where 98 KB of static RAM would be a
 * permanent tax on every mode, not a cost paid only while calibrating.
 *
 * The finished table is written to the `adclut` flash partition in 512-byte
 * blocks, so the output is never held whole either, and it is not echoed over
 * serial: the firmware that reads it is the one that just stored it, so there
 * is nothing to paste into a header and no reason to spend 4096 lines of
 * terminal on it.
 */

#include <Arduino.h>

#include "config.h"
#include "tools.h"
// ADC_LUT_SUPPORTED decides which MCUs this tool exists on: the classic ESP32
// and the ESP32-S2, the only ones with the hardware DAC the sweep needs. It is
// defined in adc_lut.h so the generator and the store cannot disagree.
#include "adc_lut.h"
#include "mcu_env.h"

#ifndef BAUDRATE
#define BAUDRATE 921600
#endif

#ifndef BATTERY_PIN
#define BATTERY_PIN 33
#endif

#define ADC_PIN BATTERY_PIN

// GPIO wired to the ADC input for the sweep. Selectable from the config engine
// (config/custom/<robot>_config.h). Hardware DAC pins:
//   ESP32     -> GPIO25 (DAC1, default) or GPIO26 (DAC2)
//   ESP32-S2  -> GPIO17 (DAC1) or GPIO18 (DAC2)
#ifndef DAC_PIN
#if defined(CONFIG_IDF_TARGET_ESP32S2)
#define DAC_PIN 17
#else
#define DAC_PIN 25
#endif
#endif

// Each tool is its own translation unit with its own namespace, exposing only a
// setup/loop pair. File-scope names collide otherwise: several tools and the
// base firmware both define `battery_msg`, `setLed`, motor objects and so on,
// and under tool mode they are all linked into one image.
namespace adc_calibrate {

// DAC codes swept, and the ADC-domain span they are interpolated across. One
// DAC code is 16 ADC codes wide (4096 / 256).
static const int DAC_STEPS = 256;
static const int STEP_SPAN = ADC_LUT_ENTRIES / DAC_STEPS;

// Passes of the exponential moving average over the whole DAC range. The ADC is
// noisy enough that a single pass per code is not usable.
static const int SWEEP_PASSES = 500;
static const float EMA_ALPHA = 0.1f;

// Entries emitted per flash write. 256 int16 = 512 bytes, the only output
// buffer this tool owns.
static const int BLOCK_ENTRIES = 256;

// The measured curve: knot[k] is the ADC reading for DAC code k. knot[DAC_STEPS]
// closes the last segment at full scale.
//
// Allocated on entry and released on exit rather than held at file scope. Under
// tool mode every tool in the image pays its static cost in EVERY mode, including
// the robot firmware that will never calibrate anything, so a tool's working set
// belongs on the heap for as long as the tool is actually running. 1,028 bytes
// while calibrating, nothing the rest of the time.
static float *knot = NULL;

static uint8_t dac_pin = DAC_PIN;
static uint8_t adc_pin = ADC_PIN;

#if ADC_LUT_SUPPORTED

// Sweep the DAC and record the ADC's answer for every code.
static void measureCurve()
{
    for (int k = 0; k <= DAC_STEPS; k++)
        knot[k] = 0.0f;

    Serial.print(F("Test Linearity "));
    for (int pass = 0; pass < SWEEP_PASSES; pass++) {
        if (pass % 100 == 0) Serial.print(".");
        for (int k = 0; k < DAC_STEPS; k++) {
            dacWrite(dac_pin, k & 0xff);
            delayMicroseconds(100);
            knot[k] = (1.0f - EMA_ALPHA) * knot[k] + EMA_ALPHA * analogRead(adc_pin);
        }
    }
    Serial.println();

    // Close the final segment at full scale: DAC code 255 is not the top of the
    // ADC range, and the last segment has to reach it for the inverse to cover
    // every reading.
    knot[DAC_STEPS] = (float)(ADC_LUT_ENTRIES - 1);
}

// Half an LSB, added to the curve so that truncating the inverse rounds rather
// than floors. This is the original's `Results[i] = 0.5f + Results[i]`, kept so
// the emitted table keeps its rounding convention.
static void applyRoundingOffset()
{
    for (int k = 0; k <= DAC_STEPS; k++)
        knot[k] += 0.5f;
}

// A transfer curve cannot decrease; noise in the sweep can still make it dip.
// A dip would make the inverse ambiguous, and the original resolved that
// arbitrarily -- whichever grid point the linear scan happened to see first.
// Clamping to the running maximum makes the inverse well defined. Reports how
// far it had to reach, because a large correction means a bad measurement.
static void enforceMonotonic()
{
    float worst = 0.0f;
    int dips = 0;
    for (int k = 1; k <= DAC_STEPS; k++) {
        if (knot[k] < knot[k - 1]) {
            float delta = knot[k - 1] - knot[k];
            if (delta > worst) worst = delta;
            dips++;
            knot[k] = knot[k - 1];
        }
    }
    if (dips) {
        Serial.print(F("Curve was non-monotonic at "));
        Serial.print(dips);
        Serial.print(F(" of "));
        Serial.print(DAC_STEPS);
        Serial.print(F(" steps, largest dip "));
        Serial.print(worst, 2);
        Serial.println(F(" counts - clamped. A large dip means a noisy sweep."));
    }
}

// The inverse of the measured curve, evaluated at `reading`.
//
// `knot_index` walks forward across calls; callers feed readings in ascending
// order, so the whole inverse costs one pass over the knots rather than a
// search per target.
static float invert(int reading, int *knot_index)
{
    int k = *knot_index;
    while (k < DAC_STEPS && knot[k + 1] < (float)reading)
        k++;
    *knot_index = k;

    if ((float)reading <= knot[0])
        return 0.0f;
    if ((float)reading >= knot[DAC_STEPS])
        return (float)(ADC_LUT_ENTRIES - 1);

    float lo = knot[k];
    float hi = knot[k + 1];
    // A flat segment means several DAC codes read the same: any position in it
    // is as good as any other, so take the lowest.
    float fraction = (hi > lo) ? ((float)reading - lo) / (hi - lo) : 0.0f;
    return (float)STEP_SPAN * ((float)k + fraction);
}

// Build the table and stream it to flash. Nothing larger than one 512-byte
// block is held at a time, and the table is not echoed: it is 4096 values whose
// only consumer is the firmware that just stored them. A summary is printed so a
// bad sweep is still visible without reading 4096 numbers off a terminal.
static void generateLut()
{
    if (!adcLutBeginWrite()) {
        Serial.println(F("Cannot write the 'adclut' partition - calibration abandoned."));
        return;
    }

    int16_t block[BLOCK_ENTRIES];
    int knot_index = 0;

    for (int base = 0; base < ADC_LUT_ENTRIES; base += BLOCK_ENTRIES) {
        for (int i = 0; i < BLOCK_ENTRIES; i++) {
            int reading = base + i;
            int value;
            if (reading == 0) {
                value = 0;      // zero maps to zero by definition
            } else {
                float position = invert(reading, &knot_index);
                value = (int)position;
                if (value < 0) value = 0;
                if (value > ADC_LUT_ENTRIES - 1) value = ADC_LUT_ENTRIES - 1;
            }
            block[i] = (int16_t)value;
        }
        if (!adcLutWriteBlock(block, BLOCK_ENTRIES)) {
            Serial.println(F("Storing FAILED - re-run the calibration."));
            return;
        }
        // Eight evenly spaced samples, one per block: enough to see that the
        // table rises smoothly across the range and is not stuck or inverted.
        if ((base % (ADC_LUT_ENTRIES / 8)) == 0) {
            Serial.print(F("  raw "));
            Serial.print(base);
            Serial.print(F(" -> "));
            Serial.println(block[0]);
        }
    }

    if (adcLutEndWrite())
        Serial.println(F("Stored in the 'adclut' partition. It survives reflashing the "
                         "application, so this calibration does not need repeating."));
    else
        Serial.println(F("Committing FAILED - re-run the calibration."));
}

#endif // ADC_LUT_SUPPORTED

void setup_()
{
#if ADC_LUT_SUPPORTED
    initMcuEnv();
    knot = (float *)malloc((DAC_STEPS + 1) * sizeof(float));
    if (!knot) {
        Serial.println(F("Could not allocate the 1 KB curve buffer - not calibrating."));
        return;
    }
    // Pins are configuration, not build options: the env partition wins over the
    // generated header, as everywhere else.
    dac_pin = (uint8_t)envU16("dac_pin", DAC_PIN);
    adc_pin = (uint8_t)envU16("battery_pin", ADC_PIN);

    dacWrite(dac_pin, 0);   // Arduino-core pin-based DAC (ESP32 / ESP32-S2)
    analogReadResolution(12);
    Serial.print(F("DAC output pin: GPIO"));
    Serial.print(dac_pin);
    Serial.print(F("  ADC input pin: GPIO"));
    Serial.println(adc_pin);
    Serial.println(F("Wire the DAC pin to the ADC pin before starting."));
#endif
}

void loop_()
{
#if ADC_LUT_SUPPORTED
    if (!knot) {
        Serial.println(F("No curve buffer - nothing to do."));
        while (1) delay(1000);
    }
    measureCurve();
    applyRoundingOffset();
    enforceMonotonic();
    Serial.println(F("Inverting the curve .."));
    generateLut();
    // The table is in flash now; the curve it came from has no further use.
    free(knot);
    knot = NULL;
#else
    // Reached only if a dispatcher offered this tool on a board that cannot run
    // it; the app list is supposed to exclude it there.
    Serial.println(F("adc_calibrate is an ESP32-only tool: building the table needs a "
                     "hardware DAC, which the ESP32-S3 and the RP2040/RP2350 do not have."));
#endif

    while (1) {
        delay(1000);
    }
}

}  // namespace adc_calibrate

// Entry points for tool mode. The dispatcher in firmware/base calls these; it
// never sees the namespace.
extern "C" void adc_calibrate_setup(void) { adc_calibrate::setup_(); }
extern "C" void adc_calibrate_loop(void)  { adc_calibrate::loop_(); }
