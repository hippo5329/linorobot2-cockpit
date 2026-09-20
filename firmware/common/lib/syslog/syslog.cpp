#include <Arduino.h>
#include "config.h"

// Needs the radio: syslog is UDP. USE_SYSLOG says the config wants
// remote logging; USE_WIFI says this image has something to send it over.
#if defined(USE_SYSLOG) && defined(USE_WIFI)
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Syslog.h>
#include "mcu_env.h"

// Declared, not #included. syslog.h is included BY wifis.cpp, so including
// wifis.h here makes the `wifi` and `syslog` libraries mutually dependent, and
// PlatformIO's LDF (chain mode on the ESP32 bases) then fails to put the
// third-party Syslog library on the wifi library's include path:
//
//   common/lib/wifi/ota.cpp:16 -> common/lib/syslog/syslog.h:4
//   fatal error: Syslog.h: No such file or directory
//
// One declaration costs nothing and keeps the two libraries a DAG.
bool wifiWanted(void);
#ifndef DEVICE_HOSTNAME
#define DEVICE_HOSTNAME "linorobot2"
#endif
#ifndef APP_NAME
#define APP_NAME "hardware"
#endif
WiFiUDP udpClient;
Syslog syslogv(udpClient, SYSLOG_SERVER, SYSLOG_PORT, DEVICE_HOSTNAME, APP_NAME, LOG_KERN);
void initSyslog(void) {
  initMcuEnv();
  syslogv.server(envIP("syslog_ip", SYSLOG_SERVER), envU16("syslog_port", SYSLOG_PORT));
}

void syslog(uint16_t priority, const char *fmt, ...) {
  // No radio, no log. syslogv.vlogf() ends in WiFiUDP::beginPacket(), which
  // calls into lwIP -- and lwIP has no tcpip thread until the Wi-Fi stack
  // starts one. On a serial robot with the radio off that is not a dropped log
  // line, it is `assert failed: tcpip_send_msg_wait_sem ... (Invalid mbox)`,
  // an abort, and a boot loop.
  //
  // Measured on the GenDrv bench: the board dies in i2cProbeSelect's "INA219
  // detected" log, three lines after the I2C table it just printed, and the
  // only symptom upstream of the serial console is a micro-ROS session that
  // never appears.
  // wifiWanted() first: WiFi.status() is itself a call into the CYW43 driver,
  // and syslog() runs throughout setup(). On a W image running on a non-W
  // board with no AP list, asking the radio its status is exactly the thing
  // that must not happen. wifiWanted() is false there and this returns without
  // touching it. (It is cheap: one env lookup, and the env is cached.)
  if (!wifiWanted() || WiFi.status() != WL_CONNECTED)
    return;
  va_list args;
  va_start(args, fmt);
  syslogv.vlogf(priority, fmt, args);
  va_end(args);
};
#endif
