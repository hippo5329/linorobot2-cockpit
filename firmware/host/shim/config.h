// config.h for the HOST target: deliberately almost empty.
//
// On a board this is firmware/include/config.h, which includes the header
// generated for one robot (or, for a released image, the one generated for a
// silicon). The host target has NO compiled-in robot at all: it exists to read
// the same env image a board is flashed with, so anything it answered here would
// be a second source of truth for a value the env already carries -- and the
// first thing the pair would do is disagree.
//
// So this supplies only the three fallbacks the firmware needs a compile-time
// answer for, and each is the answer for a host rather than one borrowed from a
// robot config:
//
//   TRANSPORT_DEFAULT   udp4. On silicon this is what the board was generated
//                       for and the env overrides it; here udp4 is the only
//                       transport the target HAS (a host has no USB device port
//                       to be a serial client on), so it is not a preference.
//   AGENT_IP_DEFAULT    loopback, the one address that needs no configuration to
//                       be reachable and cannot accidentally be someone else's
//                       machine. A real run is expected to set agent_ip.
//   AGENT_PORT_DEFAULT  8888, micro_ros_agent's own default.
#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

#define TRANSPORT_DEFAULT   "udp4"
#define AGENT_IP_DEFAULT    IPAddress(127, 0, 0, 1)
#define AGENT_PORT_DEFAULT  8888

// And two the diagnostic counters ask for. Both are hardware answers, and a host
// has none of the hardware:
//
//   FAKE_LD19_DEFAULT  false. The scan emulator writes LD19 frames out a UART for
//                      a real LiDAR driver to read; there is no UART here. The env
//                      key `fake_ld19` still overrides it, which is why diag.cpp
//                      consults the env first -- this is only the answer for an
//                      image whose env was never written.
//   LIDAR_RXD          -1, the value the firmware already reads as "not wired".
#define FAKE_LD19_DEFAULT   false
#define LIDAR_RXD           (-1)

#endif // CONFIG_H
