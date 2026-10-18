#include <Arduino.h>
#include "config.h"
#include "tool_term.h"
#include "mcu_env.h"
#include "wifis.h"
#include "syslog.h"

#if defined(HAS_WIFI) || defined(LINO_HOST)
#include <WiFi.h>
#if defined(ARDUINO_ARCH_ESP32)
#include <lwip/sockets.h>
#endif

// The login is a challenge, like the ping port's lino-tool and like OTA itself: the board
// sends a one-time nonce, the client answers md5(password:nonce), and the OTA password --
// which also reflashes the robot -- never crosses the network. A board with no password
// opens no terminal: a robot on the network is never driven without one.
#define LOGIN_MS      10000      // to answer the challenge
#define LOCKOUT_MS    10000      // after three wrong answers, for that address
#define IDLE_MS       120000     // a logged-in client silent this long is gone (a half-open
                                 // link reads as connected; lino_tool.py sends a NUL a second)
#define WRITE_GRACE_MS 2000      // a send with no room: a burst, until it lasts this long

#if defined(LINO_HOST)
// The Sim MCU has no MD5Builder; RFC 1321, enough for one short string.
static void md5hex(const char *msg, char out[33])
{
    static const uint32_t K[64] = {
        0xd76aa478,0xe8c7b756,0x242070db,0xc1bdceee,0xf57c0faf,0x4787c62a,0xa8304613,0xfd469501,
        0x698098d8,0x8b44f7af,0xffff5bb1,0x895cd7be,0x6b901122,0xfd987193,0xa679438e,0x49b40821,
        0xf61e2562,0xc040b340,0x265e5a51,0xe9b6c7aa,0xd62f105d,0x02441453,0xd8a1e681,0xe7d3fbc8,
        0x21e1cde6,0xc33707d6,0xf4d50d87,0x455a14ed,0xa9e3e905,0xfcefa3f8,0x676f02d9,0x8d2a4c8a,
        0xfffa3942,0x8771f681,0x6d9d6122,0xfde5380c,0xa4beea44,0x4bdecfa9,0xf6bb4b60,0xbebfbc70,
        0x289b7ec6,0xeaa127fa,0xd4ef3085,0x04881d05,0xd9d4d039,0xe6db99e5,0x1fa27cf8,0xc4ac5665,
        0xf4292244,0x432aff97,0xab9423a7,0xfc93a039,0x655b59c3,0x8f0ccc92,0xffeff47d,0x85845dd1,
        0x6fa87e4f,0xfe2ce6e0,0xa3014314,0x4e0811a1,0xf7537e82,0xbd3af235,0x2ad7d2bb,0xeb86d391};
    static const uint8_t R[64] = {7,12,17,22,7,12,17,22,7,12,17,22,7,12,17,22,5,9,14,20,5,9,14,20,
        5,9,14,20,5,9,14,20,4,11,16,23,4,11,16,23,4,11,16,23,4,11,16,23,6,10,15,21,6,10,15,21,
        6,10,15,21,6,10,15,21};
    const size_t len = strlen(msg);
    const size_t total = ((len + 8) / 64 + 1) * 64;
    uint8_t buf[192] = {0};
    if (total > sizeof(buf)) { out[0] = '\0'; return; }
    memcpy(buf, msg, len);
    buf[len] = 0x80;
    const uint64_t bits = (uint64_t)len * 8;
    for (int i = 0; i < 8; i++) buf[total - 8 + i] = (uint8_t)(bits >> (8 * i));
    uint32_t h[4] = {0x67452301, 0xefcdab89, 0x98badcfe, 0x10325476};
    for (size_t off = 0; off < total; off += 64) {
        uint32_t w[16];
        for (int i = 0; i < 16; i++)
            w[i] = buf[off + 4*i] | (buf[off + 4*i + 1] << 8) | (buf[off + 4*i + 2] << 16) | ((uint32_t)buf[off + 4*i + 3] << 24);
        uint32_t a = h[0], b = h[1], c = h[2], d = h[3];
        for (int i = 0; i < 64; i++) {
            uint32_t f; int g;
            if (i < 16)      { f = (b & c) | (~b & d); g = i; }
            else if (i < 32) { f = (d & b) | (~d & c); g = (5 * i + 1) % 16; }
            else if (i < 48) { f = b ^ c ^ d;          g = (3 * i + 5) % 16; }
            else             { f = c ^ (b | ~d);       g = (7 * i) % 16; }
            const uint32_t t = d; d = c; c = b;
            const uint32_t x = a + f + K[i] + w[g];
            b = b + ((x << R[i]) | (x >> (32 - R[i])));
            a = t;
        }
        h[0] += a; h[1] += b; h[2] += c; h[3] += d;
    }
    for (int i = 0; i < 16; i++)
        snprintf(out + 2 * i, 3, "%02x", (unsigned)((h[i / 4] >> (8 * (i % 4))) & 0xff));
}
#else
#include <MD5Builder.h>
static void md5hex(const char *msg, char out[33])
{
    MD5Builder md5;
    md5.begin();
    md5.add((uint8_t *)msg, (uint16_t)strlen(msg));
    md5.calculate();
    md5.getChars(out);
}
#endif

static uint32_t nonceRandom(void)
{
#if defined(ARDUINO_ARCH_ESP32)
    return esp_random();
#elif defined(ARDUINO_ARCH_RP2040)
    return rp2040.hwrand32();
#else
    return (uint32_t)random(0x7fffffff) ^ (uint32_t)micros();
#endif
}

// An address as text, by its bytes: every core's IPAddress has operator[], the host's
// has no String to convert to.
static const char *ipText(const IPAddress &ip)
{
    static char t[16];
    snprintf(t, sizeof(t), "%u.%u.%u.%u", ip[0], ip[1], ip[2], ip[3]);
    return t;
}

// Allocated in termBegin(): base never builds a server, and a serial robot's tool
// never either.
static WiFiServer *server = nullptr;
static WiFiClient client;
static bool have_client = false;
static bool authed = false;
static int tries = 0;
static uint32_t login_t0 = 0;
static uint32_t locked_until = 0;
static IPAddress locked_ip;
static uint32_t write_fail_since = 0;
static bool dropped = false;
static uint32_t last_input = 0;
static char nonce[9] = "";
static char rx[96];
static size_t rx_len = 0;
static bool rx_cr = false;
static uint8_t rx_esc = 0;         // 1: after Esc, 2: inside its CSI/SS3 sequence
static char addr[32] = "";

static void hangUp(void);

// What the console copies to the client. Never waits on the network: a link without
// room for a line is a link that has gone, and a drive must not stall in a print.
class NetSink : public Print
{
public:
    size_t write(uint8_t c) override { return write(&c, 1); }
    size_t write(const uint8_t *buf, size_t n) override
    {
        if (!have_client || !authed) return 0;
#if defined(ARDUINO_ARCH_ESP32)
        const int fd = client.fd();
        const ssize_t w = fd >= 0 ? send(fd, buf, n, MSG_DONTWAIT) : -1;
#else
        const ssize_t w = (ssize_t)client.write(buf, n);
#endif
        if (w < (ssize_t)n) {
            // No room: a burst (`get`, an I2C table) or a link that has gone. A drive's own
            // stop does not wait on this -- its deadman reads input, not output.
            if (!write_fail_since) write_fail_since = millis();
            else if (millis() - write_fail_since > WRITE_GRACE_MS) { hangUp(); return 0; }
            return w > 0 ? (size_t)w : 0;
        }
        write_fail_since = 0;
        return n;
    }
    using Print::write;
};
static NetSink net_sink;

// The console's copy to the client: lino_console on a board, the host Serial on the Sim MCU.
static void consoleNet(Print *p)
{
#if defined(LINO_HOST)
    Serial.setNet(p);
#else
    lino_console.setNet(p);
#endif
}

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
    static uint32_t last_try = 0;
    if (server || (last_try && millis() - last_try < 1000))
        return;               // once a second at most: it is called from every poll
    last_try = millis();
#if defined(LINO_HOST)
    // The Sim MCU opens one only when asked (term_port): a gate runs several side by side.
    const uint16_t port = envU16("term_port", 0);
#else
    if (!wifiWanted() || WiFi.status() != WL_CONNECTED)
        return;
    const uint16_t port = envU16("term_port", 2323);
#endif
    if (!port)
        return;
    if (!password()) {
        static bool said = false;
        if (!said) {
            said = true;
            Serial.println("[tool] no terminal on the network: this board has no OTA password "
                           "(a USB flash of a Wi-Fi robot sets one)");
        }
        return;
    }
    server = new WiFiServer(port);
    server->begin();
#if defined(LINO_HOST)
    static bool bind_failed = false;
    if (!*server) {
        if (!bind_failed)
            Serial.printf("[tool] no terminal: tcp port %u is taken\n", (unsigned)port);
        bind_failed = true;
        delete server;
        server = nullptr;
        return;
    }
#endif
    snprintf(addr, sizeof(addr), "%s:%u", ipText(WiFi.localIP()), (unsigned)port);
    Serial.printf("[tool] terminal on tcp %s (OTA password, by challenge)\n", addr);
    syslog(LOG_INFO, "tool terminal on tcp %s", addr);
}

const char *termAddress(void) { return addr; }

uint32_t termLastInput(void) { return last_input; }
void termTouch(void) { last_input = millis(); }

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
    nonce[0] = '\0';
}

void termEnd(void)
{
    if (have_client && authed)
        client.print("[tool] no tool this boot -- micro-ROS\r\n");
    if (have_client)
        hangUp();
    dropped = false;
    if (server) {
        server->stop();
        delete server;
        server = nullptr;
    }
    addr[0] = '\0';
}

static void greet(void)
{
    login_t0 = millis();
#if defined(ARDUINO_ARCH_RP2040)
    client.setTimeout(20);           // a write waits at most this long for room
#endif
    snprintf(nonce, sizeof(nonce), "%08lx", (unsigned long)nonceRandom());
    client.print("linorobot2 tool -- answer with md5(password:nonce) (lino_tool.py does). A "
                 "drive started here stops after 3 s without input; lino_tool.py keeps it alive.\r\n");
    client.print("nonce ");
    client.print(nonce);
    client.print("\r\nlogin: ");
}

static void checkAnswer(const char *line)
{
    char want[96], md5[33];
    snprintf(want, sizeof(want), "%s:%s", password(), nonce);
    md5hex(want, md5);
    unsigned diff = strlen(line) == 32 ? 0u : 1u;
    for (size_t i = 0; i < 32 && line[i]; i++)
        diff |= (unsigned)(tolower((unsigned char)line[i]) ^ md5[i]);
    nonce[0] = '\0';
    if (diff == 0) {
        authed = true;
        tries = 0;
        last_input = millis();
        consoleNet(&net_sink);
        client.print("ok\r\ntool> ");
        syslog(LOG_INFO, "tool terminal: client %s in", ipText(client.remoteIP()));
        return;
    }
    client.print("wrong answer\r\n");
    if (++tries >= 3) {
        tries = 0;
        locked_until = millis() + LOCKOUT_MS;
        locked_ip = client.remoteIP();
        syslog(LOG_WARNING, "tool terminal: 3 wrong answers from %s, locked 10 s",
               ipText(client.remoteIP()));
    }
    hangUp();
}

bool termLine(char *buf, size_t n)
{
    if (!server)
        termBegin();          // the radio may associate after the tool started
    if (!server)
        return false;
#if !defined(LINO_HOST)
    if (have_client && WiFi.status() != WL_CONNECTED)
        hangUp();             // the radio went: the client cannot be there
#endif
    if (have_client && !client.connected())
        hangUp();
    if (have_client && !authed && millis() - login_t0 > LOGIN_MS) {
        client.print("too slow\r\n");
        hangUp();
    }
    if (have_client && authed && millis() - last_input > IDLE_MS) {
        client.print("[tool] idle 2 min: closing this terminal\r\n");
        hangUp();
    }
    // One client at a time; a second is told so and closed, as is anyone while locked.
    WiFiClient incoming = server->available();
    if (incoming) {
        if (have_client) {
            incoming.print("busy: another terminal is connected (an idle one closes after 2 min)\r\n");
            incoming.stop();
        } else if ((int32_t)(millis() - locked_until) < 0 && incoming.remoteIP() == locked_ip) {
            incoming.print("locked: too many wrong answers, try again shortly\r\n");
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
    while (have_client && client.available()) {
        const int c = client.read();
        if (c < 0) break;
        if (authed) last_input = millis();      // any byte, the keep-alive NUL included
        if (c == 0) continue;
        // An arrow key is Esc [ A: the stop has been given, the rest of it is swallowed.
        if (rx_esc == 2) { if (c >= 0x40 && c <= 0x7e) rx_esc = 0; continue; }
        if (rx_esc == 1) { rx_esc = 0; if (c == '[' || c == 'O') { rx_esc = 2; continue; } }
        if (c == 0x1b && authed) {         // Esc: stop, whatever was half typed (tools.cpp)
            rx_esc = 1;
            rx_len = 0;
            client.print("\r\n");
            strncpy(buf, "stop", n - 1);
            buf[n - 1] = '\0';
            return true;
        }
        if (c == '\n' && rx_cr) { rx_cr = false; continue; }
        rx_cr = (c == '\r');
        if (c == '\r' || c == '\n') {
            rx[rx_len] = '\0';
            rx_len = 0;
            if (!authed) {
                checkAnswer(rx);
                return false;
            }
            client.print("\r\n");
            strncpy(buf, rx, n - 1);
            buf[n - 1] = '\0';
            return true;
        }
        if (c == 0x04) {          // Ctrl-D: leave
            hangUp();
            return false;
        }
        if (c == 0x08 || c == 0x7f) {
            if (rx_len) { rx_len--; if (authed) client.print("\b \b"); }
            continue;
        }
        if (c >= 0x20 && c < 0x7f && rx_len < sizeof(rx) - 1) {
            rx[rx_len++] = (char)c;
            if (authed) client.write((uint8_t)c);        // the answer is not echoed
        }
    }
    return false;
}

#else
void termBegin(void) {}
void termEnd(void) {}
bool termLine(char *, size_t) { return false; }
bool termDropped(void) { return false; }
uint32_t termLastInput(void) { return millis(); }
void termTouch(void) {}
const char *termAddress(void) { return ""; }
#endif
