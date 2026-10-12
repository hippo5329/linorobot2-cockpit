// The tool's terminal over Wi-Fi (2026-10-12, user: "for wifi we can stream terminal or
// use web"; "wifi tool can support picow pico2w, because they are low rate"). A Wi-Fi
// robot is usually off its cable, so the tool also listens on TCP `term_port` (2323):
// one client at a time, every console line copied to it, its lines read as commands.
// The board's OTA password, when it has one, is asked first -- the terminal drives
// motors. A cable robot (no radio) has the serial console only and none of this.
#ifndef TOOL_TERM_H
#define TOOL_TERM_H

#include <stddef.h>

// After the radio is up; a no-op without one.
void termBegin(void);
// One complete line from the client into `buf` (no newline): true when there is one.
// Also accepts a new client and runs the password exchange.
bool termLine(char *buf, size_t n);
// True once each time an authenticated client goes away (the shell stops the wheels).
bool termDropped(void);
// Close the server and any client (base carries on after the boot's tool window).
void termEnd(void);
// The address to print ("192.168.1.20:2323"), or "" when there is no terminal.
const char *termAddress(void);

#endif // TOOL_TERM_H
