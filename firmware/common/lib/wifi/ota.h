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
// initOta's hooks (2026-10-07): ArduinoOTA receives a whole image inside one
// handle() call, so loop() -- the motor command timeout and the watchdog feed --
// does not run until it is over. `on_start` stops the base before the first byte
// (the wheels would otherwise keep their last command for the whole transfer) and
// `feed` keeps the task watchdog fed on every progress step (an 8 s watchdog reset
// a classic ESP32 at 76 % of a 1.1 MB image). Either may be NULL.
#if defined(HAS_WIFI)
void initOta(void (*on_start)(void), void (*feed)(void));
void runOta(void);
void initPing(void (*format_banner)(char *buf, size_t n));
// The host's stop (2026-10-08): a datagram "lino-stop" to the ping port latches a
// flag a motion TOOL polls (test_acc halts its motors and stays halted until a new
// env is written). Answered "lino-stop ok <uid line>" so the sender knows it landed.
// Unauthenticated on purpose: it can only take motion away. The base application
// does not act on it; its wheels stop on the /cmd_vel timeout.
bool hostStopRequested(void);
// The watchdog feed initOta() was given (main.cpp's), for a tool that waits a long
// time outside loop(); a no-op without one.
void feedWatchdogFromTool(void);
#else
#define initOta(on_start, feed)
#define runOta()
#define initPing(f)
#define hostStopRequested() false
#define feedWatchdogFromTool()
#endif

#endif
