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
//
// Push lidar data to a UDP server, which will push laser scan message
// TODO: add PWM closed loop control the scan frequency
// TODO: add TCP for reliable data packet sequence
//
#include <Arduino.h>
#include "config.h"
#include "syslog.h"

#if defined(USE_LIDAR_UDP) && !defined(USE_FAKE_LD19)
#include <HardwareSerial.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#define BUFSIZE 512

HardwareSerial comm(LIDAR_SERIAL);
WiFiUDP udp;
uint8_t buf[BUFSIZE];

void rx_err_callback(hardwareSerial_error_t err)
{
  syslog(LOG_INFO, "%s err %d", __FUNCTION__, err);
}

size_t len = 0;
void rx_callback(void)
{
  size_t cc = comm.read(buf + len, BUFSIZE - len);
  // syslog(LOG_INFO, "%s recv %d", __FUNCTION__, cc);
  // No radio, no forwarding: beginPacket() below calls into lwIP, which has no
  // tcpip thread until the Wi-Fi stack starts one, and this runs from the UART
  // receive callback the moment a LiDAR is wired up. Keep draining the UART --
  // dropping the bytes is right, blocking the callback is not. The radio can
  // arrive after setup() now that initWifis() no longer waits for it.
  if (WiFi.status() != WL_CONNECTED) {
    len = 0;
    return;
  }
  // send larger UDP packet
  if ((len += cc) && (len > 128)) {
      udp.beginPacket(LIDAR_SERVER, LIDAR_PORT);
      udp.write(buf, len);
      udp.endPacket();
      // syslog(LOG_INFO, "%s send %d", __FUNCTION__, len);
      len = 0;
  }
}

void poweronLidar(void)
{
#ifdef LIDAR_POWEROFF
    digitalWrite(LIDAR_POWEROFF, LOW);
#endif
}

void poweroffLidar(void)
{
#ifdef LIDAR_POWEROFF
    digitalWrite(LIDAR_POWEROFF, HIGH);
#endif
}

// A generous default, because the cost of getting it wrong is asymmetric. An
// LD19 streams ~17.6 KB/s (375 packets/s of 47 bytes); at the old 1024 the
// buffer holds 58 ms of scan, and any control-loop iteration that overruns
// that drops bytes mid-packet -- which does not degrade the scan, it desyncs
// the framing until the next header. 4 KB buys ~230 ms, comfortably longer
// than any stall the control loop can produce, for 3 KB of an ESP32's 320 KB.
#ifndef LIDAR_RX_BUFFER_SIZE
#define LIDAR_RX_BUFFER_SIZE 4096
#endif

void initLidar(void) {
  pinMode(LIDAR_RXD, INPUT);
#ifdef LIDAR_POWEROFF
  pinMode(LIDAR_POWEROFF, OUTPUT);
#endif
  poweronLidar();
  comm.setRxBufferSize(LIDAR_RX_BUFFER_SIZE);
  comm.onReceiveError(rx_err_callback);
  comm.onReceive(rx_callback);
  comm.begin(LIDAR_BAUDRATE, SERIAL_8N1, LIDAR_RXD);
};
#else
void initLidar(void) {};
void poweronLidar(void) {};
void poweroffLidar(void) {};
#endif
