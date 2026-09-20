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

#if defined(USE_ARDUINO_OTA) && defined(USE_WIFI)
#include <ArduinoOTA.h>

// Only after initOta(): main.cpp calls that behind wifiWanted(), and handle()
// on a listener that was never begun is a call per loop() for nothing.
static bool ota_started = false;

void initOta(void)
{
    // The port is a robot fact like every other address: telemetry.ota_port in
    // the config, `ota_port` in the env. 3232 is ArduinoOTA's own default.
    ArduinoOTA.setPort(envU16("ota_port", 3232));
    // Port defaults to 3232
    // Port defaults to 8266
    // ArduinoOTA.setPort(8266);

    // Hostname defaults to esp3232-[MAC]
    // Hostname defaults to esp8266-[ChipID]
    // ArduinoOTA.setHostname("myesp8266");

    // No authentication by default
    // ArduinoOTA.setPassword("admin");

    // Password can be set with it's md5 value as well
    // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
    // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

    ArduinoOTA.onStart([]() {
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
    ArduinoOTA.begin();
    ota_started = true;
}

void runOta(void)
{
    if (ota_started)
        ArduinoOTA.handle();
}

#endif // USE_ARDUINO_OTA
