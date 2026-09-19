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
#include "wifis.h"
#include "mcu_env.h"
#include <string.h>

#ifdef WIFI_AP_LIST
#include <WiFi.h>
#include <WiFiMulti.h>
// How long setup() waits for the AP before carrying on without it, and how
// often loop() retries afterwards. The retry interval is not a politeness
// setting: each attempt on a disconnected radio blocks in scanNetworks().
#ifndef WIFI_CONNECT_TIMEOUT_MS
#define WIFI_CONNECT_TIMEOUT_MS 20000
#endif
#ifndef WIFI_RETRY_INTERVAL_MS
#define WIFI_RETRY_INTERVAL_MS 5000
#endif

#define EXECUTE_EVERY_N_MS(MS, X)  do { \
  static volatile unsigned long init = millis(); \
  if (millis() - init > MS) { X; init = millis();} \
} while (0)

const char *wifi_ap_list[][2] = WIFI_AP_LIST;
WiFiMulti wifiMulti;

bool wifiWanted(void)
{
    initMcuEnv();
    // udp4 cannot work without the radio, so the transport setting overrides
    // everything else -- a robot configured for Wi-Fi transport and wifi=0
    // would otherwise sit with no link and no explanation.
    const char *mode = envGet("transport", TRANSPORT_DEFAULT);
    if (strcasecmp(mode, "udp4") == 0 || strcasecmp(mode, "udp") == 0
        || strcasecmp(mode, "wifi") == 0)
        return true;
#ifdef WIFI_DEFAULT_ENABLED
    return envU16("wifi", WIFI_DEFAULT_ENABLED) != 0;
#else
    return envU16("wifi", 0) != 0;
#endif
}

void initWifis(void)
{
    if (!wifiWanted()) {
        // A serial robot must not block here. The connect loop below is
        // unbounded on purpose for a Wi-Fi robot, where there is nothing to do
        // until the radio associates -- but on a serial robot it would hang a
        // board that has no business touching the radio at all.
        Serial.println("[wifi] not requested this boot (env wifi=0, transport=serial)");
        return;
    }
#ifdef USE_MCU_ENV
    // The env partition wins over anything compiled in. A prebuilt image has an
    // empty WIFI_AP_LIST by design -- the credentials were never in it -- so on
    // those builds this is the only source there is.
    initMcuEnv();
    const char *env_ssid = envGet("wifi_ssid", NULL);
    if (env_ssid && *env_ssid) {
        wifiMulti.addAP(env_ssid, envGet("wifi_psk", ""));
        Serial.printf("[wifi] using SSID '%s' from the env partition\n", env_ssid);
    } else if (wifi_ap_list[0][0] == NULL) {
        // Nothing here and nothing compiled in: say so instead of spinning
        // silently in the connect loop below, which is what a user who has not
        // yet flashed an env block would otherwise see.
        Serial.println("[wifi] no wifi_ssid in the env partition and none compiled in.");
        Serial.println("[wifi] Flash one:  python3 scripts/mcu_env.py build --out env.bin");
        Serial.println("[wifi]             esptool write_flash 0x290000 env.bin");
    }
#endif
    for (int i = 0; wifi_ap_list[i][0] != NULL; i++) {
        wifiMulti.addAP(wifi_ap_list[i][0], wifi_ap_list[i][1]);
    }
    // Bounded, and it says which way it went. This loop used to be
    // `while (run() != WL_CONNECTED) delay(500);` with no way out: an AP that
    // did not accept the board stopped setup() dead -- no I2C scan, no banner
    // after the SSID line, no micro-ROS, no syslog (which needs this radio),
    // and no reset either, so the board looked bricked while it was in fact
    // waiting forever. Measured on the GenDrv, 2026-09-19: the console ended at
    // "[wifi] using SSID 'x' from the env partition" and stayed there, 0 resets
    // in 25 s, while the same board had been on that AP half an hour before.
    // A robot that cannot reach its AP should still boot, still show its
    // sensors, and keep trying -- runWifis() does that from loop().
    const uint32_t deadline = millis() + WIFI_CONNECT_TIMEOUT_MS;
    while (wifiMulti.run() != WL_CONNECTED) {
        if ((int32_t)(millis() - deadline) >= 0) {
            Serial.printf("[wifi] NOT connected after %lu s — carrying on without it and "
                          "retrying every %lu s. A udp4 robot has no agent until this "
                          "succeeds; check the AP, the credentials in the env partition, "
                          "and the antenna.\n",
                          (unsigned long)(WIFI_CONNECT_TIMEOUT_MS / 1000),
                          (unsigned long)(WIFI_RETRY_INTERVAL_MS / 1000));
            return;
        }
        delay(500);
    }
    Serial.println("WIFI connected");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    syslog(LOG_INFO, "%s ssid %s rssi %d ip %s", __FUNCTION__, WiFi.SSID(), WiFi.RSSI(),
	   WiFi.localIP().toString().c_str());
}

void runWifis(void)
{
    static uint8_t dis_bssid[6]; // check previous disconnected bssid to avoid repeated disconnection
    uint8_t *bssid;
    uint8_t bssidv[6];

    // The same guard as initWifis(), for the same reason: this runs from
    // loop(), and WiFiMulti::run() on a radio that is not connected does not
    // consult its AP list first -- it calls the blocking WiFi.scanNetworks(),
    // which takes seconds. A serial robot with wifi=0 went through that scan
    // on EVERY loop() iteration: measured on the GenDrv over the UART1
    // diagnostic, one loop() per ~7 s, one control-timer callback per loop,
    // /imu/data_raw at 0.14 Hz -- the "board connects, advertises six topics
    // and publishes nothing" stall. Not visible from the agent's side, because
    // the keepalive ping is also once per loop and that is enough to hold
    // the session. Answer cached: it is a boot-time fact and this is 50 Hz.
    static const bool wanted = wifiWanted();
    if (!wanted)
        return;

#ifdef WIFI_MONITOR
    EXECUTE_EVERY_N_MS(WIFI_MONITOR * 60 * 1000, syslog(LOG_INFO, "%s ssid %s rssi %d", \
							__FUNCTION__, WiFi.SSID(), WiFi.RSSI()));
#endif
#ifdef PICO // WiFi.BSSID api is different
    // when wifi signal is too weak, disconnect current ap and scan for strongest signal
    EXECUTE_EVERY_N_MS(2000, (WiFi.RSSI() < LOW_RSSI && (bssid = WiFi.BSSID(bssidv), memcmp(dis_bssid, bssid, 6))) ? \
		       (memcpy(dis_bssid, bssid, 6), WiFi.disconnect()) : 0);
#else
    // when wifi signal is too weak, disconnect current ap and scan for strongest signal
    EXECUTE_EVERY_N_MS(2000, (WiFi.RSSI() < LOW_RSSI && (bssid = WiFi.BSSID(), memcmp(dis_bssid, bssid, 6))) ? \
		       (memcpy(dis_bssid, bssid, 6), WiFi.disconnect()) : 0);
#endif
    // run() is cheap when the radio is associated and expensive when it is not:
    // on a disconnected radio it calls the blocking WiFi.scanNetworks() first
    // (seconds), which is the stall described above. Once connected that cost
    // is gone, so only the disconnected case is rate-limited -- and now that
    // initWifis() can return without a link, the disconnected case is reachable
    // for the whole life of the robot, not just between AP dropouts.
    if (WiFi.status() == WL_CONNECTED) {
        wifiMulti.run();
    } else {
        EXECUTE_EVERY_N_MS(WIFI_RETRY_INTERVAL_MS, wifiMulti.run());
    }
}
#endif // WIFI_AP_LIST
