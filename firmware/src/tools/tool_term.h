// The tool's terminal over Wi-Fi (2026-10-12, user: "for wifi we can stream terminal or
// use web"; "wifi tool can support picow pico2w, because they are low rate"). A Wi-Fi
// robot is usually off its cable, so the tool also listens on TCP `term_port` (2323):
// one client at a time, every console line copied to it, its lines read as commands.
// The login is md5(password:nonce) over a one-time nonce, as OTA does: the OTA password
// never crosses the network, and a board without one opens no terminal -- it drives motors. A cable robot (no radio) has the serial console only and none of this.
#ifndef TOOL_TERM_H
#define TOOL_TERM_H

#include <stddef.h>

// After the radio is up; a no-op without one.
void termBegin(void);
// millis() of the client's last byte (a NUL keep-alive counts), and a reset of it when a
// command starts: a drive started from the terminal stops when it goes quiet.
uint32_t termLastInput(void);
void termTouch(void);
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
