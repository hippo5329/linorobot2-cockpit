#include <Arduino.h>
#include "config.h"
#include "tool_term.h"
#include "mcu_env.h"
#include "wifis.h"
#include "syslog.h"

#if defined(HAS_WIFI) || defined(LINO_HOST)
#include <WiFi.h>

// The console's copy to the client: lino_console on a board, the host Serial on the Sim MCU.
static void consoleNet(Print *p)
{
#if defined(LINO_HOST)
    Serial.setNet(p);
#else
    lino_console.setNet(p);
#endif
}

// Allocated in termBegin(): base never builds a server, and a serial robot's tool
// never either.
static WiFiServer *server = nullptr;
static WiFiClient client;
static bool have_client = false;
static bool authed = false;
static int tries = 0;
static bool dropped = false;
static char rx[96];
static size_t rx_len = 0;
static char addr[32] = "";

static void hangUp(void);

static const char *password(void)
{
    const char *pwd = envGet("ota_password", NULL);
#if defined(OTA_PASSWORD)
    if (!pwd || !*pwd) pwd = OTA_PASSWORD;
#endif
    return (pwd && *pwd) ? pwd : NULL;
}

void termBegin(void)
{
#if defined(LINO_HOST)
    // The Sim MCU opens one only when asked (term_port): a gate runs several side by side.
    const uint16_t port = envU16("term_port", 0);
    if (server)
        return;
#else
    if (server || !wifiWanted() || WiFi.status() != WL_CONNECTED)
        return;
    const uint16_t port = envU16("term_port", 2323);
#endif
    if (!port)
        return;
    server = new WiFiServer(port);
    server->begin();
    snprintf(addr, sizeof(addr), "%s:%u", WiFi.localIP().toString().c_str(), (unsigned)port);
    Serial.printf("[tool] terminal on tcp %s%s\n", addr, password() ? " (OTA password)" : "");
    syslog(LOG_INFO, "tool terminal on tcp %s", addr);
}

const char *termAddress(void) { return addr; }

void termEnd(void)
{
    if (have_client) {
        client.print("[tool] no tool this boot -- micro-ROS\r\n");
        hangUp();
    }
    dropped = false;
    if (server) {
        server->stop();
        delete server;
        server = nullptr;
    }
    addr[0] = '\0';
}

bool termDropped(void)
{
    const bool d = dropped;
    dropped = false;
    return d;
}

static void hangUp(void)
{
    if (authed) {
        consoleNet(nullptr);
        dropped = true;
    }
    client.stop();
    have_client = false;
    authed = false;
    rx_len = 0;
}

static void greet(void)
{
    tries = 0;
    authed = (password() == nullptr);
    client.print("linorobot2 tool -- 'help' lists the commands\r\n");
    if (authed) {
        consoleNet(&client);
        client.print("tool> ");
    } else {
        client.print("password: ");
    }
}

// The password line: compared in full, three tries, then the line is dropped.
static void checkPassword(const char *line)
{
    const char *pwd = password();
    const size_t a = strlen(line), b = pwd ? strlen(pwd) : 0;
    unsigned diff = (unsigned)(a ^ b);
    for (size_t i = 0; i < a && i < b; i++) diff |= (unsigned)(line[i] ^ pwd[i]);
    if (pwd && diff == 0) {
        authed = true;
        consoleNet(&client);
        client.print("ok\r\ntool> ");
        syslog(LOG_INFO, "tool terminal: client %s in", client.remoteIP().toString().c_str());
        return;
    }
    if (++tries >= 3) {
        client.print("wrong password\r\n");
        syslog(LOG_WARNING, "tool terminal: 3 wrong passwords from %s", client.remoteIP().toString().c_str());
        hangUp();
        return;
    }
    client.print("wrong password\r\npassword: ");
}

bool termLine(char *buf, size_t n)
{
    if (!server)
        termBegin();          // the radio may associate after the tool started
    if (!server)
        return false;
    if (have_client && !client.connected())
        hangUp();
    // One client at a time; a second is told so and closed.
    WiFiClient incoming = server->available();   // accept() is newer than some cores
    if (incoming) {
        if (have_client) {
            incoming.print("busy: another terminal is connected\r\n");
            incoming.stop();
        } else {
            client = incoming;
            have_client = true;
            rx_len = 0;
            greet();
        }
    }
    if (!have_client)
        return false;
    while (client.available()) {
        const int c = client.read();
        if (c < 0) break;
        if (c == '\r') continue;
        if (c == '\n') {
            rx[rx_len] = '\0';
            rx_len = 0;
            if (!authed) {
                checkPassword(rx);
                return false;
            }
            strncpy(buf, rx, n - 1);
            buf[n - 1] = '\0';
            return true;
        }
        if (c == 0x04) {          // Ctrl-D: leave
            hangUp();
            return false;
        }
        if (rx_len < sizeof(rx) - 1)
            rx[rx_len++] = (char)c;
    }
    return false;
}

#else
void termBegin(void) {}
void termEnd(void) {}
bool termLine(char *, size_t) { return false; }
bool termDropped(void) { return false; }
const char *termAddress(void) { return ""; }
#endif
