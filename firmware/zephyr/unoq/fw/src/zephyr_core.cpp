// The Arduino runtime for the UNO Q (Zephyr) target: the core's globals, time, the
// Serial and Wire bodies, and main() -> setup() once, loop() for ever. Every line of
// robot behaviour is the firmware's own (src/main.cpp, common/lib), compiled unmodified.
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

#include <ctype.h>
#include <zephyr/device.h>
#include <zephyr/drivers/hwinfo.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include "uart_link.h"

_LinoZephyrSerial Serial;
TwoWire Wire;
TwoWire Wire1;
SPIClass SPI;

// --- time -------------------------------------------------------------------------
uint64_t _lino_zephyr_now_us() { return k_ticks_to_us_floor64(k_uptime_ticks()); }
void delay(unsigned long ms) { k_msleep((int32_t)ms); }
void delayMicroseconds(unsigned int us) { k_busy_wait(us); }
void yield() { k_yield(); }
static unsigned int irq_key;
void noInterrupts() { irq_key = irq_lock(); }
void interrupts() { irq_unlock(irq_key); }

// --- Serial: the micro-ROS link ------------------------------------------------------
void _LinoZephyrSerial::begin(unsigned long baud) { linkBegin((uint32_t)baud); }

size_t _LinoZephyrSerial::write(const uint8_t *buf, size_t n)
{
    // Text (the banner, the boot log) is mirrored to the RAM console; a micro-ROS
    // frame always carries non-text bytes and is not.
    bool text = n > 0;
    for (size_t i = 0; i < n && text; i++)
        text = isprint(buf[i]) || buf[i] == '\n' || buf[i] == '\r' || buf[i] == '\t';
    if (text)
        printk("%.*s", (int)n, (const char *)buf);
    return linkWrite(buf, n);
}

void _LinoZephyrSerial::flush() { linkFlush(100); }
int _LinoZephyrSerial::available() { return linkAvailable(); }
int _LinoZephyrSerial::read()
{
    uint8_t c;
    return linkRead(&c, 1, 0) == 1 ? c : -1;
}
size_t _LinoZephyrSerial::readBytes(char *buf, size_t n)
{
    return linkRead((uint8_t *)buf, n, (int)getTimeout());
}

// --- Wire: the Qwiic bus (i2c4) -------------------------------------------------------
static const struct device *const qwiic = DEVICE_DT_GET(DT_NODELABEL(i2c4));

void TwoWire::setClock(uint32_t freq)
{
    const uint32_t speed = freq >= 1000000 ? I2C_SPEED_FAST_PLUS
                         : freq >= 400000  ? I2C_SPEED_FAST : I2C_SPEED_STANDARD;
    (void)i2c_configure(qwiic, I2C_MODE_CONTROLLER | I2C_SPEED_SET(speed));
}

size_t TwoWire::write(uint8_t c)
{
    if (tx_len_ >= sizeof(tx_)) return 0;
    tx_[tx_len_++] = c;
    return 1;
}

size_t TwoWire::write(const uint8_t *buf, size_t n)
{
    size_t i = 0;
    while (i < n && write(buf[i])) i++;
    return i;
}

// Arduino's codes: 0 ok, 2 address NACK, 4 other error.
uint8_t TwoWire::endTransmission(bool stop)
{
    if (!device_is_ready(qwiic)) return 4;
    if (!stop) {          // keep it for a repeated-start read
        tx_held_ = true;
        return 0;
    }
    tx_held_ = false;
    int rc;
    if (tx_len_ == 0) {   // an address probe: a 1-byte read is what every STM32 I2C can do
        uint8_t b;
        rc = i2c_read(qwiic, &b, 1, addr_);
    } else {
        rc = i2c_write(qwiic, tx_, tx_len_, addr_);
    }
    tx_len_ = 0;
    return rc == 0 ? 0 : 2;
}

uint8_t TwoWire::requestFrom(uint8_t addr, size_t n, bool stop)
{
    (void)stop;
    rx_len_ = rx_pos_ = 0;
    if (n > sizeof(rx_)) n = sizeof(rx_);
    if (!device_is_ready(qwiic) || n == 0) return 0;
    int rc;
    if (tx_held_ && tx_len_ > 0 && addr == addr_)
        rc = i2c_write_read(qwiic, addr, tx_, tx_len_, rx_, n);
    else
        rc = i2c_read(qwiic, rx_, n, addr);
    tx_held_ = false;
    tx_len_ = 0;
    if (rc != 0) return 0;
    rx_len_ = n;
    return (uint8_t)n;
}

// --- identity, for the banner's uid= -----------------------------------------------
extern "C" void linoZephyrUid(char *buf, size_t n)
{
    uint8_t id[12] = {};
    const ssize_t got = hwinfo_get_device_id(id, sizeof(id));
    size_t o = 0;
    for (ssize_t i = 0; i < got && o + 3 <= n; i++)
        o += snprintf(buf + o, n - o, "%02X", id[i]);
    if (o < n) buf[o] = '\0';
}

// --- the env block, in the last 8 KB flash page (app.overlay) -----------------------
extern "C" const uint8_t *linoZephyrEnvBase(void)
{
    return (const uint8_t *)(DT_REG_ADDR(DT_CHOSEN(zephyr_flash)) +
                             DT_REG_ADDR(DT_NODELABEL(lino_env_partition)));
}

void setup();
void loop();

int main()
{
    setup();
    for (;;)
        loop();
}
