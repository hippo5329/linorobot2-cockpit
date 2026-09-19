// One micro-ROS transport that is chosen at boot instead of at compile time.
//
// micro_ros_platformio picks a transport with `board_microros_transport` in
// platformio.ini, which is why `esp32` and `esp32_wifi` were two PlatformIO
// environments and two binaries for one board: serial framing in one, udp4 in
// the other. That is the last thing forcing a board to have more than one
// firmware, now that sensors and parameters come from the env partition.
//
// The transport layer is genuinely small. micro-ROS asks for four C functions --
// open, close, write, read -- plus a framing flag, and the difference between
// the two transports is entirely contained in them:
//
//     serial   framing true    write/read a Stream
//     udp4     framing false   write/read a WiFiUDP socket at agent_ip:agent_port
//
// So platformio.ini says `board_microros_transport = custom`, this file supplies
// the four functions, and they dispatch on a mode read from the env partition:
//
//     transport=serial     micro-ROS over USB serial
//     transport=udp4       micro-ROS over Wi-Fi to agent_ip:agent_port
//
// RP2040 and RP2350 have no Wi-Fi, so the UDP half compiles out there and the
// mode is always serial -- the same source, one honest branch.
#ifndef UROS_TRANSPORT_H
#define UROS_TRANSPORT_H

#include <Arduino.h>

// Selects the transport from the env partition and installs it. Call after
// initMcuEnv() and, for udp4, after initWifis() -- the socket needs an
// associated radio, and this does not bring the link up itself.
//
// Returns true when udp4 was installed, false for serial, so the caller can say
// which one it ended up on without reaching into the env again.
bool initUrosTransport(void);

// What the transport ended up being, for logging and for the LED/diagnostic
// paths that behave differently on a wireless link.
bool urosTransportIsUdp(void);

#endif // UROS_TRANSPORT_H
