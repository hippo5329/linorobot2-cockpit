#include <Arduino.h>
#include "config.h"
#include "uros_transport.h"
#include "mcu_env.h"
#include "diag.h"

#include <micro_ros_platformio.h>
#include <uxr/client/util/time.h>
#include <uxr/client/profile/transport/custom/custom_transport.h>

// Wi-Fi exists on the ESP32 family and nowhere else in this project. Guarding on
// the architecture rather than on a config macro keeps the RP2040/RP2350 builds
// free of a WiFiUDP they could never use, while the source stays single.
#if defined(ESP32) || defined(ARDUINO_ARCH_ESP32)
#define UROS_HAVE_UDP 1
#include <WiFi.h>
#include <WiFiUdp.h>
#endif

static bool uros_use_udp = false;

#ifdef UROS_HAVE_UDP
static WiFiUDP uros_udp;
static IPAddress uros_agent_ip;
static uint16_t uros_agent_port = 8888;
#endif

// The four functions micro-ROS calls. They are the whole transport: each one
// picks a branch on uros_use_udp, which was decided once at boot.
extern "C" {

bool platformio_transport_open(struct uxrCustomTransport *transport)
{
#ifdef UROS_HAVE_UDP
    if (uros_use_udp)
        return uros_udp.begin(uros_agent_port) == 1;
#endif
    (void)transport;
    return true;   // the Stream was opened by Serial.begin() in setup()
}

bool platformio_transport_close(struct uxrCustomTransport *transport)
{
#ifdef UROS_HAVE_UDP
    if (uros_use_udp) {
        uros_udp.stop();
        return true;
    }
#endif
    (void)transport;
    return true;
}

size_t platformio_transport_write(struct uxrCustomTransport *transport,
                                  const uint8_t *buf, size_t len, uint8_t *errcode)
{
    (void)errcode;
#ifdef UROS_HAVE_UDP
    if (uros_use_udp) {
        size_t sent = 0;
        if (uros_udp.beginPacket(uros_agent_ip, uros_agent_port)) {
            sent = uros_udp.write(buf, len);
            sent = uros_udp.endPacket() ? sent : 0;
        }
        uros_udp.flush();
        return sent;
    }
#endif
    Stream *stream = (Stream *)transport->args;
    const size_t n = stream->write(buf, len);
    diagCount(DIAG_TX_BYTES, (uint32_t)n);
    return n;
}

size_t platformio_transport_read(struct uxrCustomTransport *transport,
                                 uint8_t *buf, size_t len, int timeout, uint8_t *errcode)
{
    (void)errcode;
#ifdef UROS_HAVE_UDP
    if (uros_use_udp) {
        int64_t start = uxr_millis();
        while ((uxr_millis() - start) < (int64_t)timeout && uros_udp.parsePacket() == 0)
            delay(1);
        return uros_udp.available() ? uros_udp.read(buf, len) : 0;
    }
#endif
    Stream *stream = (Stream *)transport->args;
    stream->setTimeout(timeout);
    const size_t n = stream->readBytes((char *)buf, len);
    diagCount(DIAG_RX_CALLS);
    if (n == 0) diagCount(DIAG_RX_EMPTY); else diagCount(DIAG_RX_BYTES, (uint32_t)n);
    return n;
}

}   // extern "C"

bool urosTransportIsUdp(void) { return uros_use_udp; }

bool initUrosTransport(void)
{
    initMcuEnv();

#ifdef UROS_HAVE_UDP
    // TRANSPORT_DEFAULT is what the config was generated for; the env overrides
    // it. A board flashed with a prebuilt image and no env block still comes up
    // the way its config says.
    const char *mode = envGet("transport", TRANSPORT_DEFAULT);
    uros_use_udp = (strcasecmp(mode, "udp4") == 0 || strcasecmp(mode, "udp") == 0
                    || strcasecmp(mode, "wifi") == 0);
#else
    // No radio: serial is not a default here, it is the only possibility. Saying
    // so is better than silently ignoring a transport=udp4 that can never work.
    const char *mode = envGet("transport", "serial");
    if (strcasecmp(mode, "serial") != 0)
        Serial.printf("[uros] env asks for transport '%s', but this board has no "
                      "Wi-Fi — using serial\n", mode);
    uros_use_udp = false;
#endif

#ifdef UROS_HAVE_UDP
    if (uros_use_udp) {
        uros_agent_ip = envIP("agent_ip", AGENT_IP_DEFAULT);
        uros_agent_port = envU16("agent_port", AGENT_PORT_DEFAULT);
        Serial.print("[uros] transport udp4 -> ");
        Serial.print(uros_agent_ip);
        Serial.printf(":%u\n", (unsigned)uros_agent_port);
        // framing false: UDP datagrams carry their own boundaries, so the
        // stream framer that serial needs would only add overhead.
        rmw_uros_set_custom_transport(false, NULL,
                                      platformio_transport_open,
                                      platformio_transport_close,
                                      platformio_transport_write,
                                      platformio_transport_read);
        return true;
    }
#endif

    Serial.println("[uros] transport serial");
    // framing true: a byte stream has no message boundaries of its own.
    rmw_uros_set_custom_transport(true, (void *)&Serial,
                                  platformio_transport_open,
                                  platformio_transport_close,
                                  platformio_transport_write,
                                  platformio_transport_read);
    return false;
}
