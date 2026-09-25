#include <Arduino.h>
#include "config.h"
#include "syslog.h"

#if defined(HAS_WIFI)
#include <WiFi.h>
#include <WiFiUdp.h>
#include <stdarg.h>
#include "mcu_env.h"

// Declared, not #included: syslog.h is included BY wifis.cpp, and a cycle
// between the two libraries confuses PlatformIO's LDF.
bool wifiWanted(void);

#ifndef SYSLOG_SERVER
#define SYSLOG_SERVER IPAddress(0, 0, 0, 0)
#endif
#ifndef SYSLOG_PORT
#define SYSLOG_PORT 5140
#endif
#ifndef DEVICE_HOSTNAME
#define DEVICE_HOSTNAME "linorobot2"
#endif
#ifndef APP_NAME
#define APP_NAME "hardware"
#endif

static WiFiUDP s_udp;
static IPAddress s_ip;
static uint16_t s_port = 0;

void initSyslog(void)
{
  initMcuEnv();
  s_ip = envIP("syslog_ip", SYSLOG_SERVER);
  s_port = envU16("syslog_port", SYSLOG_PORT);
}

void syslog(uint16_t priority, const char *fmt, ...)
{
  // No radio, no log. A UDP send ends in lwIP, which has no tcpip thread until
  // the Wi-Fi stack starts one: on a serial robot with the radio off that is an
  // abort and a boot loop, not a dropped line (measured on the GenDrv bench).
  // wifiWanted() first: WiFi.status() is itself a call into the CYW43 driver,
  // which must not happen on a W image running on a non-W board.
  if (!wifiWanted() || WiFi.status() != WL_CONNECTED)
    return;
  if (s_port == 0 || s_ip == IPAddress(0, 0, 0, 0))
    return;
  if ((priority & 0x03f8) == 0)
    priority |= LOG_KERN;
  char msg[192];
  va_list args;
  va_start(args, fmt);
  vsnprintf(msg, sizeof(msg), fmt, args);
  va_end(args);
  if (!s_udp.beginPacket(s_ip, s_port))
    return;
  s_udp.printf("<%u>1 - %s %s - - - \xEF\xBB\xBF%s", (unsigned)priority, DEVICE_HOSTNAME, APP_NAME, msg);
  s_udp.endPacket();
}
#endif
