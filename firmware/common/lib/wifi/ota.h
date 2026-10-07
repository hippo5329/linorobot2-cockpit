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
#ifndef OTA_H
#define OTA_H

#include "config.h"

// The OTA responder is compiled in wherever there is a radio; `ota_port` in
// the env, and the radio being wanted at all, decide whether it runs.
#include <stddef.h>

// The ping responder (2026-10-07): UDP `ping_port` (env, default 3233; 0 disables)
// answers a datagram starting "lino?" with the firmware banner line, to the sender.
// It is how the robot computer asks a Wi-Fi robot that has left the USB cable what
// it is -- on demand, unicast to its address or broadcast to find it -- instead of
// waiting for the next syslog banner. Read-only: the banner carries no secret.
// Serviced from runOta(), so everywhere OTA stays responsive, so does this.
#if defined(HAS_WIFI)
void initOta(void);
void runOta(void);
void initPing(void (*format_banner)(char *buf, size_t n));
#else
#define initOta()
#define runOta()
#define initPing(f)
#endif

#endif
