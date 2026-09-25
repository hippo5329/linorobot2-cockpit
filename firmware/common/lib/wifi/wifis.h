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
#ifndef WIFIS_H
#define WIFIS_H

#include "config.h"
#ifndef LOW_RSSI
#define LOW_RSSI -75 // when wifi signal is too low, disconnect current ap and scan for strongest signal
#endif

// Credentials alone are not a radio.
//
// Compiled in wherever the silicon has a radio (HAS_WIFI, config.h), so that
// entering an AP list in the env turns Wi-Fi on with no rebuild. Without a
// radio there is no WiFi.h to compile against, so these stay stubs.
#if defined(HAS_WIFI)
void initWifis(void);
void runWifis(void);
// Whether this boot should bring the radio up at all. The Wi-Fi stack is
// compiled into every ESP32 build now -- that is what lets one binary serve
// both the serial and the udp4 robots -- so "has Wi-Fi code" and "should use
// Wi-Fi" stopped being the same question. Answered from the env (`wifi`), and
// forced true when the transport needs the radio to reach the agent at all.
bool wifiWanted(void);
#else
#define initWifis()
#define runWifis()
#define wifiWanted() false
#endif

#endif
