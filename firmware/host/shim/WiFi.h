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
#include <errno.h>
#include "WiFiUdp.h"

#define WL_CONNECTED 3
#define WL_DISCONNECTED 6

class _LinoHostWiFi
{
public:
    int status() const { return WL_CONNECTED; }
    IPAddress localIP() const { return IPAddress(127, 0, 0, 1); }
};
static _LinoHostWiFi WiFi;

// The tool's terminal (tool_term.cpp) on the Sim MCU, 2026-10-12: a POSIX TCP socket with
// the two classes' few methods it uses, so the Wi-Fi robot's walkthrough -- the password,
// the commands, a dropped client stopping the wheels -- runs on the host as on a board.
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <unistd.h>

class WiFiClient : public Print
{
public:
    WiFiClient() : fd_(-1) {}
    explicit WiFiClient(int fd) : fd_(fd) {}
    explicit operator bool() const { return fd_ >= 0; }
    bool connected()
    {
        if (fd_ < 0) return false;
        char c;
        const ssize_t r = recv(fd_, &c, 1, MSG_PEEK | MSG_DONTWAIT);
        return r > 0 || (r < 0 && (errno == EAGAIN || errno == EWOULDBLOCK));
    }
    int available()
    {
        int n = 0;
        return (fd_ >= 0 && ioctl(fd_, FIONREAD, &n) == 0) ? n : 0;
    }
    int read()
    {
        unsigned char c;
        return (fd_ >= 0 && recv(fd_, &c, 1, MSG_DONTWAIT) == 1) ? c : -1;
    }
    size_t write(uint8_t c) override { return write(&c, 1); }
    size_t write(const uint8_t *buf, size_t n) override
    {
        if (fd_ < 0) return 0;
        const ssize_t w = send(fd_, buf, n, MSG_NOSIGNAL | MSG_DONTWAIT);
        return w > 0 ? (size_t)w : 0;
    }
    using Print::write;
    void stop() { if (fd_ >= 0) close(fd_); fd_ = -1; }
    IPAddress remoteIP() const
    {
        struct sockaddr_in a = {};
        socklen_t l = sizeof(a);
        if (fd_ < 0 || getpeername(fd_, (struct sockaddr *)&a, &l) != 0) return IPAddress();
        return IPAddress(ntohl(a.sin_addr.s_addr));
    }
private:
    int fd_;
};

class WiFiServer
{
public:
    explicit WiFiServer(uint16_t port) : port_(port), fd_(-1) {}
    void begin()
    {
        fd_ = socket(AF_INET, SOCK_STREAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
        const int one = 1;
        setsockopt(fd_, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
        struct sockaddr_in a = {};
        a.sin_family = AF_INET;
        a.sin_port = htons(port_);
        // Loopback only: the Sim MCU runs beside a gate's other cells, and nothing but
        // this computer should drive it.
        a.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        if (bind(fd_, (struct sockaddr *)&a, sizeof(a)) != 0 || listen(fd_, 1) != 0) {
            close(fd_);
            fd_ = -1;
        }
    }
    WiFiClient available()
    {
        if (fd_ < 0) return WiFiClient();
        const int c = accept4(fd_, NULL, NULL, SOCK_CLOEXEC | SOCK_NONBLOCK);
        return c >= 0 ? WiFiClient(c) : WiFiClient();
    }
    void stop() { if (fd_ >= 0) close(fd_); fd_ = -1; }
    explicit operator bool() const { return fd_ >= 0; }   // listening
private:
    uint16_t port_;
    int fd_;
};
#endif
