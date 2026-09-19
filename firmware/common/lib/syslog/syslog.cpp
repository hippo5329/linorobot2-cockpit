#include <Arduino.h>
#include "config.h"

#ifdef USE_SYSLOG
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Syslog.h>
#include "mcu_env.h"
#ifndef DEVICE_HOSTNAME
#define DEVICE_HOSTNAME "linorobot2"
#endif
#ifndef APP_NAME
#define APP_NAME "hardware"
#endif
WiFiUDP udpClient;
Syslog syslogv(udpClient, SYSLOG_SERVER, SYSLOG_PORT, DEVICE_HOSTNAME, APP_NAME, LOG_KERN);
void initSyslog(void) {
#ifdef USE_MCU_ENV
  initMcuEnv();
  syslogv.server(envIP("syslog_ip", SYSLOG_SERVER), envU16("syslog_port", SYSLOG_PORT));
#endif
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
  if (WiFi.status() != WL_CONNECTED)
    return;
  va_list args;
  va_start(args, fmt);
  syslogv.vlogf(priority, fmt, args);
  va_end(args);
};
#endif
