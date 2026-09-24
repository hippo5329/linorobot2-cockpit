// WiFiUDP for the HOST target, over a plain POSIX socket. (IPAddress is in the
// shim's Arduino.h, where the real cores keep it.)
//
// firmware/common/lib/uros_transport/uros_transport.cpp is compiled UNMODIFIED
// against this. That file already chose the right shape for us: micro-ROS asks
// for four C functions -- open, close, write, read -- plus a framing flag, and
// the whole difference between the transports lives in them. The udp4 branch
// uses exactly nine WiFiUDP methods, so nine are implemented here and no more:
//
//     begin(port) stop() beginPacket(ip,port) write(buf,len)
//     endPacket() flush() parsePacket() available() read(buf,len)
//
// The consequence worth stating: the host target takes the SAME `transport=udp4`
// branch a board takes, reading the same `agent_ip`/`agent_port` env keys from the
// same blob scripts/mcu_env.py generates. It is not a re-implementation of the
// transport that happens to speak the same protocol; it is the firmware's
// transport with a different socket under it.
#ifndef LINO_HOST_WIFIUDP_H
#define LINO_HOST_WIFIUDP_H

#include <arpa/inet.h>
#include <cerrno>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include "Arduino.h"

// ---------------------------------------------------------------- WiFiUDP
class WiFiUDP
{
public:
    // begin() on a board binds the LOCAL port. The agent replies to whatever
    // source port the client sent from, so binding the agent's port number
    // locally is harmless and matches what the board does -- but bind failure
    // must not be silent, or every read would time out with no reason given.
    int begin(uint16_t local_port)
    {
        stop();
        _fd = ::socket(AF_INET, SOCK_DGRAM, 0);
        if (_fd < 0) { Serial.printf("[host-udp] socket: %s\n", strerror(errno)); return 0; }
        int on = 1;
        ::setsockopt(_fd, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on));
        struct sockaddr_in me = {};
        me.sin_family = AF_INET;
        me.sin_addr.s_addr = htonl(INADDR_ANY);
        // Port 0 lets the kernel choose, which is what we want when the agent
        // port is already bound by the agent in this same netns.
        me.sin_port = htons(0);
        if (::bind(_fd, (struct sockaddr *)&me, sizeof(me)) < 0) {
            Serial.printf("[host-udp] bind: %s\n", strerror(errno));
            stop();
            return 0;
        }
        (void)local_port;
        return 1;
    }

    void stop()
    {
        if (_fd >= 0) { ::close(_fd); _fd = -1; }
        _rx_len = _rx_pos = 0;
    }

    bool beginPacket(const IPAddress &ip, uint16_t port)
    {
        if (_fd < 0) return false;
        _dst = {};
        _dst.sin_family = AF_INET;
        _dst.sin_addr.s_addr = ip.asNetworkOrder();
        _dst.sin_port = htons(port);
        _tx_len = 0;
        return true;
    }

    // A board's WiFiUDP buffers into one datagram between beginPacket and
    // endPacket; so does this. The cap is the UDP MTU the client library was
    // built with (512) plus slack -- a write past it is a real fault, not
    // something to truncate quietly.
    size_t write(const uint8_t *buf, size_t len)
    {
        if (_fd < 0 || !buf) return 0;
        if (_tx_len + len > sizeof(_tx)) {
            Serial.printf("[host-udp] datagram would exceed %zu bytes; dropping\n", sizeof(_tx));
            return 0;
        }
        memcpy(_tx + _tx_len, buf, len);
        _tx_len += len;
        return len;
    }

    bool endPacket()
    {
        if (_fd < 0 || _tx_len == 0) return false;
        const ssize_t n = ::sendto(_fd, _tx, _tx_len, 0,
                                   (struct sockaddr *)&_dst, sizeof(_dst));
        const bool ok = (n == (ssize_t)_tx_len);
        if (!ok) Serial.printf("[host-udp] sendto: %s\n", strerror(errno));
        _tx_len = 0;
        return ok;
    }

    void flush() { _tx_len = 0; }

    // parsePacket() pulls ONE datagram into the buffer and returns its size, 0
    // when nothing is waiting. Non-blocking: uros_transport.cpp spins on it
    // against its own timeout, so blocking here would break that loop's contract.
    int parsePacket()
    {
        if (_fd < 0) return 0;
        if (_rx_pos < _rx_len) return (int)(_rx_len - _rx_pos);
        const ssize_t n = ::recvfrom(_fd, _rx, sizeof(_rx), MSG_DONTWAIT, nullptr, nullptr);
        if (n <= 0) return 0;
        _rx_len = (size_t)n;
        _rx_pos = 0;
        return (int)_rx_len;
    }

    int available() { return (int)(_rx_len - _rx_pos); }

    int read(uint8_t *buf, size_t len)
    {
        const size_t have = _rx_len - _rx_pos;
        const size_t n = len < have ? len : have;
        if (n == 0 || !buf) return 0;
        memcpy(buf, _rx + _rx_pos, n);
        _rx_pos += n;
        return (int)n;
    }

private:
    int _fd = -1;
    struct sockaddr_in _dst = {};
    uint8_t _tx[1500] = {};
    size_t _tx_len = 0;
    uint8_t _rx[1500] = {};
    size_t _rx_len = 0, _rx_pos = 0;
};

#endif // LINO_HOST_WIFIUDP_H
