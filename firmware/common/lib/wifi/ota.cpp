// Copyright (c) 2023 Thomas Chou
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
#include <Arduino.h>
#include "config.h"
#include "syslog.h"
#include "mcu_env.h"

#if defined(HAS_WIFI)
#include <ArduinoOTA.h>

// The env block over the air (2026-10-07). A Wi-Fi robot -- an ESP32 or ESP32-S3,
// the only boards on the udp4 transport -- leaves the computer after its first USB
// flash; from then on its env (the tool to run, the agent address, the sensor
// flags) is written with ArduinoOTA's FILESYSTEM command, as the same 4 KB image
// scripts/mcu_env.py builds for USB. `env` is a DATA/SPIFFS partition
// (partitions_lino.csv) and setPartitionLabel("env") points the filesystem command
// at it, so the core's Updater writes the block in place and checks its MD5; the
// board reboots into it, and the uploader confirms by the banner's `envcrc=`.

#include <WiFiUdp.h>

static WiFiUDP ping_udp;
static bool ping_started = false;
static void (*ping_format)(char *, size_t) = NULL;
static volatile bool stop_requested = false;
static volatile bool ident_requested = false;

bool hostStopRequested(void) { return stop_requested; }
void hostStopClear(void) { stop_requested = false; }
bool hostIdentRequested(void)
{
    if (!ident_requested) return false;
    ident_requested = false;
    return true;
}

void initPing(void (*format_banner)(char *buf, size_t n))
{
    const uint16_t port = envU16("ping_port", 3233);
    if (!port || !format_banner)
        return;
    ping_format = format_banner;
    ping_started = ping_udp.begin(port);
}

// One non-blocking look per call: parsePacket() is a recvfrom on a socket with
// nothing in it nearly always, which costs microseconds on the control loop.
static void runPing(void)
{
    if (!ping_started)
        return;
    if (ping_udp.parsePacket() <= 0)
        return;
    char req[16];
    const int r = ping_udp.read(req, sizeof(req) - 1);
    req[r > 0 ? r : 0] = '\0';
    const bool stop = strncmp(req, "lino-stop", 9) == 0;
    const bool ident = strncmp(req, "lino-ident", 10) == 0;
    if (!stop && !ident && strncmp(req, "lino?", 5) != 0)
        return;
    char line[224];
    if (stop) {
        stop_requested = true;
        strcpy(line, "lino-stop ok ");
        ping_format(line + 13, sizeof(line) - 13);
    } else if (ident) {
        ident_requested = true;
        strcpy(line, "lino-ident ok ");
        ping_format(line + 14, sizeof(line) - 14);
    } else {
        ping_format(line, sizeof(line));
    }
    ping_udp.beginPacket(ping_udp.remoteIP(), ping_udp.remotePort());
    ping_udp.write((const uint8_t *)line, strlen(line));
    ping_udp.endPacket();
}

// Only after initOta(): main.cpp calls that behind wifiWanted(), and handle()
// on a listener that was never begun is a call per loop() for nothing.
static bool ota_started = false;

static void (*ota_on_start)(void) = NULL;
static void (*ota_feed)(void) = NULL;

void feedWatchdogFromTool(void)
{
    if (ota_feed) ota_feed();
}

void initOta(void (*on_start)(void), void (*feed)(void))
{
    // Once only: main.cpp starts OTA (with its hooks) before any tool runs, and a
    // tool's own setup that calls this again must not restart it or clear them.
    if (ota_started)
        return;
    ota_on_start = on_start;
    ota_feed = feed;
    // The port is a robot fact like every other address: telemetry.ota_port in
    // the config, `ota_port` in the env. 3232 is ArduinoOTA's own default.
    ArduinoOTA.setPort(envU16("ota_port", 3232));

    const char *pwd = envGet("ota_password", NULL);
#if defined(OTA_PASSWORD)
    if (!pwd || pwd[0] == '\0') {
        pwd = OTA_PASSWORD;
    }
#endif
    if (pwd && pwd[0] != '\0') {
        ArduinoOTA.setPassword(pwd);
    }

    ArduinoOTA.onStart([]() {
      if (ota_on_start) ota_on_start();     // the wheels stop before the first byte
      if (ota_feed) ota_feed();
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH) {
	type = "sketch";
      } else {  // U_FS
	type = "filesystem";
      }

      // NOTE: if updating FS this would be the place to unmount FS using FS.end()
      Serial.println("Start updating " + type);
    });
    ArduinoOTA.onEnd([]() {
      Serial.println("\nEnd");
    });
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
      if (ota_feed) ota_feed();             // the whole transfer runs inside handle()
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    });
    ArduinoOTA.onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) {
	Serial.println("Auth Failed");
      } else if (error == OTA_BEGIN_ERROR) {
	Serial.println("Begin Failed");
      } else if (error == OTA_CONNECT_ERROR) {
	Serial.println("Connect Failed");
      } else if (error == OTA_RECEIVE_ERROR) {
	Serial.println("Receive Failed");
      } else if (error == OTA_END_ERROR) {
	Serial.println("End Failed");
      }
    });
#if defined(ESP32) || defined(ARDUINO_ARCH_ESP32)
    // The filesystem command writes the env partition (see the top of this file).
    ArduinoOTA.setPartitionLabel("env");
#endif
    // The robot's name on the network: ArduinoOTA starts mDNS under this hostname,
    // so the robot computer resolves `<robot>.local` from the name the user picked
    // the robot by (mcu_env writes `hostname` from robot.name, made DNS-safe).
    const char *host = envGet("hostname", "");
    if (host && host[0])
        ArduinoOTA.setHostname(host);
    ArduinoOTA.begin();
    ota_started = true;
}

void runOta(void)
{
    if (ota_started)
        ArduinoOTA.handle();
    runPing();
}

#endif // HAS_WIFI
