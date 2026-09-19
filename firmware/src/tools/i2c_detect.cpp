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

#if defined(I2C_SDA_OVERRIDE) && defined(I2C_SCL_OVERRIDE)
  #define I2C_SCAN_SDA I2C_SDA_OVERRIDE
  #define I2C_SCAN_SCL I2C_SCL_OVERRIDE
#elif defined(SDA_PIN) && defined(SCL_PIN)
  #define I2C_SCAN_SDA SDA_PIN
  #define I2C_SCAN_SCL SCL_PIN
#endif

namespace i2c_detect {

I2CDevice found[I2C_PROBE_MAX];
int num_found = 0;

void scanAndIdentify() {
    Serial.println("\n=======================================================");
    Serial.println("  Linorobot2 I2C Sensor Detection                       ");
    Serial.println("=======================================================");
#if defined(I2C_SCAN_SDA) && defined(I2C_SCAN_SCL)
    Serial.printf("Scanning I2C bus (SDA:%d, SCL:%d)...\n", I2C_SCAN_SDA, I2C_SCAN_SCL);
#else
    Serial.println("Scanning I2C bus (pins from the env partition / config header)...");
#endif

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
        Serial.printf(" [%d] ADDR: 0x%02X | CATEGORY: %-8s | MODEL: %-10s | MACRO: %-18s | %s\n",
            identified, found[i].addr, found[i].category, found[i].model,
            found[i].macro, found[i].desc);
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
                      "\"driver\":\"%s\",\"macro\":\"%s\",\"desc\":\"%s\"}",
            found[i].addr, found[i].category, found[i].model,
            found[i].driver, found[i].macro, found[i].desc);
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
