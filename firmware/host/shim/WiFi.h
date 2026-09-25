// WiFi.h for the HOST target: a link that is already up.
//
// The host has no radio to associate; its interface is configured by the
// operating system before the firmware starts. So the one question the firmware
// asks here -- sim_ld19.h's flushUdp() checking WiFi.status() before a datagram
// -- is answered the way a board's is once its radio has joined: connected.
// Only compiled into what includes it: the host build does not define HAS_WIFI,
// so wifis.cpp, ota.cpp and the syslog radio code are not part of it.
#ifndef LINO_HOST_WIFI_H
#define LINO_HOST_WIFI_H
#include <Arduino.h>
#include "WiFiUdp.h"

#define WL_CONNECTED 3
#define WL_DISCONNECTED 6

class _LinoHostWiFi
{
public:
    int status() const { return WL_CONNECTED; }
};
static _LinoHostWiFi WiFi;
#endif
