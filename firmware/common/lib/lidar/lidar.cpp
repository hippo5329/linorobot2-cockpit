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
#include "mcu_env.h"
#include <string.h>

// Forwarding a PHYSICAL LiDAR's bytes to a UDP server. The gate was
// `#if defined(USE_LIDAR_UDP) && !defined(USE_SIM_LD19)`, which made this an
// either/or decided by the build: an image that carried the emulator could
// never forward a real LiDAR, and an image that forwarded one could never
// simulate. Both now compile, and initLidar() picks at boot from `lidar_comm`
// and `sim_ld19`.
//
// The remaining gate is ARCHITECTURE: this needs WiFiUdp and a spare
// HardwareSerial, neither of which exists on a bare RP2040/RP2350.
#if defined(ESP32) || defined(ARDUINO_ARCH_ESP32)
#include <HardwareSerial.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Ticker.h>
#define BUFSIZE 512

#ifndef LIDAR_SERIAL
#define LIDAR_SERIAL 1
#endif
#ifndef LIDAR_RXD
#define LIDAR_RXD -1
#endif
#ifndef LIDAR_BAUDRATE
#define LIDAR_BAUDRATE 230400
#endif
// Read from the env at every use; these answer only for a board whose env has
// never been written. A released image names no host -- see sim_ld19.h.
#ifndef LIDAR_SERVER_DEFAULT
#define LIDAR_SERVER_DEFAULT IPAddress(192, 168, 1, 100)
#endif
#ifndef LIDAR_PORT_DEFAULT
#define LIDAR_PORT_DEFAULT 8889
#endif
#ifndef LIDAR_SERVER
#define LIDAR_SERVER envIP("lidar_ip", LIDAR_SERVER_DEFAULT)
#endif
#ifndef LIDAR_PORT
#define LIDAR_PORT   envU16("lidar_port", LIDAR_PORT_DEFAULT)
#endif
#ifndef LIDAR_POWEROFF
#define LIDAR_POWEROFF -1
#endif

HardwareSerial comm(LIDAR_SERIAL);
WiFiUDP udp;
uint8_t buf[BUFSIZE];

// Resolved in initLidar(): -1 for "not wired", and forwarding stays off unless
// this board is actually the tap for a real LiDAR.
static int  lidar_rx = -1;
static int  lidar_poweroff = -1;
static bool forwarding = false;

// Bytes the board sends the LiDAR once its UART is open (env lidar_init, hex;
// written for a model that needs telling -- the LDROBOT LD14P does not spin until
// it receives 54 A0 04 00 00 00 00 5E). Sent three times, a second apart: the
// LiDAR may still be booting from the gpio_out that powered it, and a start
// command to a running LiDAR is harmless. Needs lidar_tx, the pin wired to the
// LiDAR's RX; the firmware knows no model, only the bytes.
static uint8_t lidar_init_bytes[32];
static size_t  lidar_init_len = 0;
static int     lidar_init_sent = 0;
static Ticker  lidar_init_ticker;

static size_t parseHex(const char *hex, uint8_t *out, size_t max)
{
    size_t n = 0;
    while (hex && *hex && n < max)
    {
        while (*hex == ' ' || *hex == ':' || *hex == ',')
            hex++;
        if (!hex[0] || !hex[1])
            break;
        char pair[3] = {hex[0], hex[1], 0};
        char *end = NULL;
        long v = strtol(pair, &end, 16);
        if (end != pair + 2)
            return 0;   // not hex: send nothing rather than garbage
        out[n++] = (uint8_t)v;
        hex += 2;
    }
    return n;
}

static void sendLidarInit(void)
{
    comm.write(lidar_init_bytes, lidar_init_len);
    if (++lidar_init_sent >= 3)
        lidar_init_ticker.detach();
}



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
  if (!forwarding || WiFi.status() != WL_CONNECTED) {
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
    if (lidar_poweroff >= 0)
        digitalWrite(lidar_poweroff, LOW);
}

void poweroffLidar(void)
{
    if (lidar_poweroff >= 0)
        digitalWrite(lidar_poweroff, HIGH);
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

// The firmware's lidarUartOpen() hook (src/main.cpp) for the ESP32 family: a real LD19 on
// `rx_pin`, read by the raw_scan forwarder (pumpRealLidar) and sent to the robot computer as
// /raw_scan over micro-ROS -- lidar_comm `topic`, on the serial link (the GenDrv's 1.5 Mbaud)
// or over Wi-Fi. It is the UDP forwarder's own UART: the two are exclusive (initLidar()
// forwards only for lidar_comm udp), so in topic mode nothing else opens `comm`. Only the RX
// pin is used; the LD19 needs nothing sent to it. Same 4 KB buffer as the forwarder, for the
// same reason. Until 2026-10-08 only the UNO Q (Zephyr) had this hook, and a GenDrv set to
// topic with a real LD19 published no /raw_scan at all.
Stream *lidarUartOpen(int rx_pin, uint32_t baud)
{
  if (rx_pin < 0)
    return nullptr;
  comm.setRxBufferSize(LIDAR_RX_BUFFER_SIZE);
  comm.begin(baud, SERIAL_8N1, rx_pin, -1);
  return &comm;
}

void initLidar(void) {
  initMcuEnv();
  lidar_rx = envInt("lidar_rx", LIDAR_RXD);
  lidar_poweroff = envInt("lidar_poweroff", LIDAR_POWEROFF);

  if (lidar_poweroff >= 0)
    pinMode(lidar_poweroff, OUTPUT);
  poweronLidar();

  // Forward only when this board is the tap for a REAL LiDAR going out over
  // UDP. `sim_ld19` means the scan is synthesised instead, and the emulator
  // owns the same UART -- starting both would leave whichever began last
  // holding the pin.
  const char *comm_mode = envGet("lidar_comm", LIDAR_COMM_DEFAULT);
  const bool sim = envFlag("sim_ld19", SIM_LD19_DEFAULT);
  forwarding = (lidar_rx >= 0) && !sim
               && (strcasecmp(comm_mode, "udp") == 0
                   || strcasecmp(comm_mode, "udp_server") == 0);
  if (!forwarding) {
    Serial.printf("[lidar] UDP forwarder off (rx=%d sim_ld19=%d comm=%s)\n",
                  lidar_rx, (int)sim, comm_mode);
    return;
  }

  pinMode(lidar_rx, INPUT);
  comm.setRxBufferSize(LIDAR_RX_BUFFER_SIZE);
  comm.onReceiveError(rx_err_callback);
  comm.onReceive(rx_callback);
  const int lidar_tx = envInt("lidar_tx", -1);
  comm.begin(envU32("lidar_baud", LIDAR_BAUDRATE), SERIAL_8N1, lidar_rx, lidar_tx);
  Serial.printf("[lidar] forwarding a real LiDAR from GPIO %d to the UDP server\n", lidar_rx);
  lidar_init_len = (lidar_tx >= 0) ? parseHex(envGet("lidar_init", ""), lidar_init_bytes, sizeof(lidar_init_bytes)) : 0;
  if (lidar_init_len)
  {
    Serial.printf("[lidar] sending %u init bytes to the LiDAR on GPIO %d (x3, 1 s apart)\n",
                  (unsigned)lidar_init_len, lidar_tx);
    sendLidarInit();
    lidar_init_ticker.attach(1.0f, sendLidarInit);
  }
};
#else
void initLidar(void) {};
void poweronLidar(void) {};
void poweroffLidar(void) {};
#endif
