// Copyright (c) 2026 Linorobot contributors
//
// The bus probe, as an application of the unified image.
//
// The scan and the WHO_AM_I table are NOT here any more: they moved to
// firmware/common/lib/i2c_probe so that `base` can run the same probe on a real
// robot and pick its drivers from the answer (main.cpp). What is left here is
// presentation -- the human table and the [I2C_JSON] line the Web UI Studio
// parses for 1-click driver selection -- which is the only part that was ever
// specific to the diagnostic.

#include <Arduino.h>
#include <Wire.h>
#include <stdio.h>
#include "config.h"
#include "tools.h"
#include "board_init.h"
#include "i2c_probe.h"
#include "mcu_env.h"

#if defined(I2C_SDA_OVERRIDE) && defined(I2C_SCL_OVERRIDE)
  #define I2C_SCAN_SDA I2C_SDA_OVERRIDE
  #define I2C_SCAN_SCL I2C_SCL_OVERRIDE
#elif defined(SDA_PIN) && defined(SCL_PIN)
  #define I2C_SCAN_SDA SDA_PIN
  #define I2C_SCAN_SCL SCL_PIN
#else
  // No macro from the config header: -1 is what initBoard() treats as "leave the
  // board's Wire default alone", so the fallback says exactly that.
  #define I2C_SCAN_SDA (-1)
  #define I2C_SCAN_SCL (-1)
#endif

namespace i2c_detect {

// found[] and num_found are locals of scanAndIdentify() now. Every use was
// already inside it, so file scope bought nothing and cost 388 bytes of .bss
// in a 124580-byte static segment -- paid for by every app in the image, since
// the tools are dispatched at runtime from one binary. This function runs on
// the tool's own loop task, which it owns outright, so the stack is free.
void scanAndIdentify() {
    I2CDevice found[I2C_PROBE_MAX];
    int num_found = 0;
    Serial.println("\n=======================================================");
    Serial.println("  Linorobot2 I2C Sensor Detection                       ");
    Serial.println("=======================================================");
    // Report the pins the bus is ACTUALLY on, which is what initBoard() resolved:
    // envInt("i2c_sda", SDA_PIN), env first and the macro only as a fallback. The
    // banner used to print the macro alone, so the prebuilt release image -- built
    // from a generated bare config, where both are -1 -- told every user of the
    // unified image "SDA:-1, SCL:-1" while happily scanning the env's pins and
    // finding their chip. Reading the same source as the bus is the whole point
    // of a diagnostic: a wiring check that reports the wrong wiring is worse than
    // none. `origin` says which answered, so a blank env still reads honestly.
    const int scan_sda = envInt("i2c_sda", I2C_SCAN_SDA);
    const int scan_scl = envInt("i2c_scl", I2C_SCAN_SCL);
    const char *origin = (envInt("i2c_sda", -32768) != -32768) ? "env" : "config header";
    if (scan_sda >= 0 && scan_scl >= 0)
        Serial.printf("Scanning I2C bus (SDA:%d, SCL:%d, from the %s)...\n",
                      scan_sda, scan_scl, origin);
    else
        Serial.printf("Scanning I2C bus (SDA:%d, SCL:%d, from the %s - "
                      "negative means this board's Wire default)...\n",
                      scan_sda, scan_scl, origin);

    num_found = i2cProbe(found, I2C_PROBE_MAX);

    if (num_found == 0)
        Serial.println("[-] No I2C devices detected on bus.");
    for (int i = 0; i < num_found; i++)
        Serial.printf(" [0x%02X] Device ACK received\n", found[i].addr);

    Serial.println("\n--- Identified Hardware Matrix ---");
    const char *detected_imu = "NONE";
    const char *detected_mag = "NONE";
    const char *detected_curr = "NONE";
    int identified = 0;

    for (int i = 0; i < num_found; i++) {
        if (strcmp(found[i].category, "unknown") == 0)
            continue;
        identified++;
        Serial.printf(" [%d] ADDR: 0x%02X | CATEGORY: %-8s | MODEL: %-10s | %s\n",
            identified, found[i].addr, found[i].category, found[i].model,
            found[i].desc);
        if (strcmp(found[i].category, "imu") == 0 && strcmp(detected_imu, "NONE") == 0)
            detected_imu = found[i].model;
        if (strcmp(found[i].category, "mag") == 0 && strcmp(detected_mag, "NONE") == 0)
            detected_mag = found[i].model;
        if (strcmp(found[i].category, "current") == 0 && strcmp(detected_curr, "NONE") == 0)
            detected_curr = found[i].model;
    }

    if (identified == 0)
        Serial.println("  (No known sensor signatures recognized)");

    // Machine-readable, for the Web UI. One line, so a reader can grep it out of
    // a serial log that also carries the human table above.
    Serial.print("\n[I2C_JSON] {");
    Serial.print("\"status\":\"ok\",");
    Serial.printf("\"imu\":\"%s\",", detected_imu);
    Serial.printf("\"mag\":\"%s\",", detected_mag);
    Serial.printf("\"current\":\"%s\",", detected_curr);
    Serial.print("\"devices\":[");
    for (int i = 0; i < num_found; i++) {
        if (i > 0) Serial.print(",");
        Serial.printf("{\"addr\":\"0x%02x\",\"category\":\"%s\",\"model\":\"%s\","
                      "\"driver\":\"%s\",\"desc\":\"%s\"}",
            found[i].addr, found[i].category, found[i].model,
            found[i].driver, found[i].desc);
    }
    Serial.println("]}");
    Serial.println("=======================================================\n");
}

void setup_() {
#if defined(I2C_SDA_OVERRIDE) && defined(I2C_SCL_OVERRIDE)
    // Explicit pins from the Web UI pinout — independent of the board config.
  #if defined(ARDUINO_ARCH_RP2040) || defined(PICO) || defined(PICO2)
    Wire.setSDA(I2C_SDA_OVERRIDE);
    Wire.setSCL(I2C_SCL_OVERRIDE);
    Wire.begin();
  #else
    Wire.begin(I2C_SDA_OVERRIDE, I2C_SCL_OVERRIDE);
  #endif
    Wire.setClock(400000);
#else
    // No explicit override: the bus is whatever the env partition and the
    // generated header say, exactly as the base firmware brings it up. A probe
    // that scanned a different bus from the firmware it is diagnosing would be
    // worse than no probe at all.
    initBoard();
#endif
    delay(1500);
    scanAndIdentify();
}

void loop_() {
    delay(5000);
    scanAndIdentify();
}

}  // namespace i2c_detect

extern "C" void i2c_detect_setup(void) { i2c_detect::setup_(); }
extern "C" void i2c_detect_loop(void)  { i2c_detect::loop_(); }
