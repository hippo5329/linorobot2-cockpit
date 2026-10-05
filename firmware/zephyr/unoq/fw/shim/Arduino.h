// Arduino.h for the UNO Q (Zephyr) target: the firmware's own sources -- src/main.cpp,
// common/lib, src/tools -- compiled unmodified on the STM32U585, as firmware/host does
// on the robot computer. Derived from firmware/host/shim/Arduino.h; what differs is the
// platform under it: time and sleep from the Zephyr kernel, Serial on the lpuart1 link
// (uart_link.cpp), no sockets. Pins are inert for now -- the robot is the simulated one
// until the motor/encoder/ADC mapping lands -- exactly as on the host.
#ifndef LINO_ZEPHYR_ARDUINO_H
#define LINO_ZEPHYR_ARDUINO_H

// The host target identifies itself the way ESP32 and RP2040 do -- by a macro the
// per-MCU branches in the firmware can test (mcu_env.cpp's loader is the first).
// It is defined HERE rather than passed on the command line because every one of
// our translation units includes Arduino.h, so this cannot be forgotten for one
// file; a -DLINO_HOST that reached only some of them would give one object a
// different view of the tree than the next, which is the silent divergence this
// whole target exists to avoid.
#ifndef LINO_ZEPHYR
#define LINO_ZEPHYR 1
#endif

#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <strings.h>   // strcasecmp: glibc declares it in string.h, picolibc only here
#include <cmath>
#include <ctime>
#include <string>

// ---------------------------------------------------------------- time
// A board's millis()/micros() count from boot. clock_gettime(CLOCK_MONOTONIC)
// is the same quantity; the epoch is taken at first call so the numbers start
// near zero as they do on a board, which matters because sim_wheel.h stores
// `unsigned long` timestamps and compares differences.
// Time and sleep from the Zephyr kernel (the host shim used CLOCK_MONOTONIC).
uint64_t _lino_zephyr_now_us();
inline unsigned long micros() { return (unsigned long)_lino_zephyr_now_us(); }
inline unsigned long millis() { return (unsigned long)(_lino_zephyr_now_us() / 1000ULL); }
void delay(unsigned long ms);
void delayMicroseconds(unsigned int us);
inline long map(long x, long in_min, long in_max, long out_min, long out_max)
{
    if (in_max == in_min) return out_min;
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Mixed argument types, as the core's macro allows: constrain(float, int, float)
// in kinematics.cpp, constrain(double, float, float) in pid.cpp. The result is
// the value's own type.
template <class T, class L, class H>
inline T constrain(T v, L lo, H hi) { return v < (T)lo ? (T)lo : (v > (T)hi ? (T)hi : v); }

// random(max) and random(min,max), matching Arduino's half-open ranges. The
// simulated encoder and gyro noise come through here, so a host run and a board
// run are the same KIND of random, not the same sequence -- which is correct: the
// board's is not reproducible either.
inline long random(long howbig) { return howbig <= 0 ? 0 : (long)(::rand() % howbig); }
inline long random(long howsmall, long howbig)
{
    return howbig <= howsmall ? howsmall : howsmall + random(howbig - howsmall);
}
inline void randomSeed(unsigned long seed) { ::srand((unsigned)seed); }

// min/max as FUNCTIONS, the way ArduinoCore-API (and so arduino-pico) defines
// them for C++: mixed argument types allowed, result the common type. As macros
// they break every standard header that says numeric_limits<T>::max() -- which
// the whole firmware reaches through <cmath> and <limits> once main.cpp is here.
template <class T, class L>
inline auto min(const T &a, const L &b) -> decltype((b < a) ? b : a) { return (b < a) ? b : a; }
template <class T, class L>
inline auto max(const T &a, const L &b) -> decltype((b < a) ? b : a) { return (a < b) ? b : a; }

// ---------------------------------------------------------------- Print/Stream
// The firmware passes `Stream *` around -- diag.cpp keeps its output as one, and
// uros_transport.cpp's serial branch casts transport->args back to one -- so the
// shim reproduces the real class shape rather than a single Serial object:
//
//     Print    write + the print/println/printf family
//     Stream   Print + available/read/peek/flush        (Arduino's own hierarchy)
//
// Getting this shape right is what lets those files compile UNMODIFIED. A shim
// that offered only a concrete Serial would have forced a host #ifdef into each
// of them, and an #ifdef in the firmware is a divergence between board and host
// -- exactly what this target exists to detect rather than create.
class IPAddress;

class Print
{
public:
    virtual ~Print() {}
    virtual size_t write(uint8_t c) = 0;
    virtual size_t write(const uint8_t *buf, size_t n)
    {
        size_t out = 0;
        for (size_t i = 0; i < n; i++) out += write(buf[i]);
        return out;
    }
    size_t write(const char *s) { return s ? write((const uint8_t *)s, strlen(s)) : 0; }

    size_t print(const char *s) { return s ? write((const uint8_t *)s, strlen(s)) : 0; }
    size_t print(char c) { return write((uint8_t)c); }
    size_t print(int v) { return printf("%d", v); }
    size_t print(unsigned v) { return printf("%u", v); }
    size_t print(long v) { return printf("%ld", v); }
    size_t print(unsigned long v) { return printf("%lu", v); }
    size_t print(double v, int digits = 2) { return printf("%.*f", digits, v); }
    size_t print(long v, int base) { return base == 16 ? printf("%lx", v) : printf("%ld", v); }
    size_t print(unsigned long v, int base) { return base == 16 ? printf("%lx", v) : printf("%lu", v); }
    size_t print(int v, int base) { return print((long)v, base); }
    size_t print(unsigned v, int base) { return print((unsigned long)v, base); }
    size_t print(uint8_t v, int base) { return print((unsigned long)v, base); }
    size_t print(const IPAddress &ip);          // defined below, once IPAddress is

    size_t println() { return print("\r\n"); }
    template <typename T> size_t println(T v) { return print(v) + println(); }
    template <typename T> size_t println(T v, int fmt) { return print(v, fmt) + println(); }

    // ESP32 Print has printf; the RP2040 core has it too, and the firmware uses
    // it heavily, so it belongs on Print rather than on the concrete port.
    int printf(const char *fmt, ...)
    {
        va_list ap;
        va_start(ap, fmt);
        char buf[512];
        const int n = vsnprintf(buf, sizeof(buf), fmt, ap);
        va_end(ap);
        if (n <= 0) return n;
        const size_t len = (size_t)n < sizeof(buf) ? (size_t)n : sizeof(buf) - 1;
        write((const uint8_t *)buf, len);
        return (int)len;
    }
};

class Stream : public Print
{
public:
    virtual int available() { return 0; }
    virtual int read() { return -1; }
    virtual int peek() { return -1; }
    virtual void flush() {}
    virtual size_t readBytes(char *, size_t) { return 0; }
    // uros_transport.cpp sets it on the serial branch. Stored and ignored: the
    // host has no serial peer, and its udp4 read() runs its own timeout loop
    // against uxr_millis() rather than leaning on the Stream.
    void setTimeout(unsigned long ms) { _timeout = ms; }
    unsigned long getTimeout() const { return _timeout; }

protected:
    unsigned long _timeout = 1000;
};

// Serial is the micro-ROS link: lpuart1 <-> the QRB2210's /dev/ttyHS1, through the
// interrupt-driven queues in uart_link.cpp -- as on the boards, where Serial is both
// the transport and the console before the agent connects, so the boot banner
// reaches Linux on the same port. Text written here is also copied to the RAM
// console (read over SWD), binary micro-ROS frames are not.
class _LinoZephyrSerial : public Stream
{
public:
    void begin(unsigned long baud = 0);
    void end() {}
    operator bool() const { return true; }
    size_t write(uint8_t c) override { return write(&c, 1); }
    size_t write(const uint8_t *buf, size_t n) override;
    using Print::write;
    void flush() override;
    int available() override;
    int read() override;
    int peek() override { return -1; }
    size_t readBytes(char *buf, size_t n) override;
};
extern _LinoZephyrSerial Serial;

// ---------------------------------------------------------------- IPAddress
// In the real Arduino cores IPAddress is part of the CORE and arrives with
// Arduino.h, not with the networking library -- mcu_env.h relies on exactly
// that: it declares envIP() and includes only <Arduino.h>. So it lives here.
class IPAddress
{
public:
    IPAddress() : _v(0) {}
    IPAddress(uint8_t a, uint8_t b, uint8_t c, uint8_t d)
        : _v(((uint32_t)a << 24) | ((uint32_t)b << 16) | ((uint32_t)c << 8) | d) {}
    explicit IPAddress(uint32_t host_order) : _v(host_order) {}

    // The env stores an address as text, so fromString is the path actually used
    // (mcu_env's envIP calls it). Returns false on anything malformed rather than
    // silently yielding 0.0.0.0, which would send the session into the void.
    bool fromString(const char *s)
    {
        unsigned a, b, c, d;
        char tail;
        if (!s || sscanf(s, "%u.%u.%u.%u%c", &a, &b, &c, &d, &tail) != 4 ||
            a > 255 || b > 255 || c > 255 || d > 255)
            return false;
        _v = (a << 24) | (b << 16) | (c << 8) | d;
        return true;
    }
    uint32_t asHostOrder() const { return _v; }
    uint32_t asNetworkOrder() const { return __builtin_bswap32(_v); }
    uint8_t operator[](int i) const { return (uint8_t)((_v >> (8 * (3 - i))) & 0xFF); }
    bool operator==(const IPAddress &o) const { return _v == o._v; }

    // Serial.print(ip) in uros_transport.cpp's udp4 branch.
    const char *c_str() const
    {
        snprintf(_txt, sizeof(_txt), "%u.%u.%u.%u",
                 (unsigned)(*this)[0], (unsigned)(*this)[1],
                 (unsigned)(*this)[2], (unsigned)(*this)[3]);
        return _txt;
    }

private:
    uint32_t _v;
    mutable char _txt[16];
};

// Serial.print(ip) in uros_transport.cpp's udp4 branch. On a real core this comes
// from IPAddress being Printable; one overload is the whole of what we need.
inline size_t Print::print(const IPAddress &ip) { return print(ip.c_str()); }

// ---------------------------------------------------------------- misc types
typedef uint8_t byte;
typedef bool boolean;
inline uint16_t word(uint8_t h, uint8_t l) { return (uint16_t)((h << 8) | l); }

// ---------------------------------------------------------------- constants
#ifndef PI
#define PI         3.1415926535897932384626433832795
#endif
#define HALF_PI    1.5707963267948966192313216916398
#define TWO_PI     6.283185307179586476925286766559
#define DEG_TO_RAD 0.017453292519943295769236907684886
#define RAD_TO_DEG 57.295779513082320876798154814105
// F() keeps a string in flash on AVR; on every 32-bit core it is the string,
// and so is PROGMEM data.
#define F(s) (s)
#define PROGMEM
#define pgm_read_byte(addr) (*(const uint8_t *)(addr))
#define pgm_read_word(addr) (*(const uint16_t *)(addr))
#define DEC 10
#define HEX 16
#define OCT 8
#define BIN 2

// ---------------------------------------------------------------- GPIO
// A host has no pins. What the firmware must see is what a board with NOTHING
// WIRED sees: every write lands and reads back, an input reads its pull, and no
// edge ever arrives. The Sim MCU is exactly that board -- every pin -1 and every
// device simulated -- so none of this is on its hot path; it exists so the same
// sources compile and a stray call behaves like silicon rather than crashing.
#define LOW 0
#define HIGH 1
#define INPUT 0x0
#define OUTPUT 0x1
#define INPUT_PULLUP 0x2
#define INPUT_PULLDOWN 0x3
#define CHANGE 1
#define FALLING 2
#define RISING 3

// Pins are the UNO Q's Arduino header numbers -- D0-D13 = 0-13, A0-A5 = 14-19 -- plus
// the on-board RGB LED3 (active low) as 20 red, 21 green, 22 blue. pins.cpp maps them to
// Zephyr's board file (boards/arduino/uno_q/arduino_r3_connector.dtsi). PWM is on the
// timer pins D3, D6 (TIM3) and D9, D10 (TIM4); analogRead on A0-A5 (ADC1).
#define LINO_UNOQ_A0 14
#define LED_BUILTIN 21
void pinMode(int pin, int mode);
void digitalWrite(int pin, int v);
int digitalRead(int pin);
void analogWrite(int pin, int value);
void analogWriteResolution(int bits);
void analogWriteFrequency(int pin, uint32_t hz);
inline void analogWriteFrequency(uint32_t hz) { analogWriteFrequency(-1, hz); }
inline void analogWriteFreq(uint32_t hz) { analogWriteFrequency(-1, hz); }
void analogWriteRange(uint32_t range);
int analogRead(int pin);
void analogReadResolution(int bits);
inline int digitalPinToInterrupt(int pin) { return pin; }
void attachInterrupt(int pin, void (*isr)(void), int mode);
void attachInterrupt(int pin, void (*isr)(void *), int mode, void *arg);
void detachInterrupt(int pin);
void noInterrupts();
void interrupts();
void yield();

#endif // LINO_ZEPHYR_ARDUINO_H
