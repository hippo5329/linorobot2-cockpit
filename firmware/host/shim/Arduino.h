// Arduino.h for the HOST target: enough of the Arduino surface that the
// firmware's own sources compile natively, unmodified.
//
// WHY A SHIM AND NOT A PORT. The host target exists to run micro-ROS over UDP4
// like a board does -- that is the one layer scripts/fake_base_node.py cannot
// test, and its own SKILL.md lists "micro-ROS, the serial and Wi-Fi transports"
// among what it removes. If the host build reimplemented the model or the
// transport, it would be a second copy of both, and a second copy of the wheel
// model is the one thing that would make the instrument lie
// (tests/test_fake_base_node.py exists to prevent exactly that). So instead of
// porting the firmware to Linux, this emulates the small Arduino surface the
// firmware actually uses, and the firmware sources are compiled as they are.
//
// The surface is small because it was measured rather than guessed:
//   kinematics.h  no Arduino calls at all (includes Arduino.h and uses none)
//   PID.h         none
//   odometry.h    none (its micro_ros_utilities/nav_msgs includes are satisfied
//                 by the micro-ROS host client library)
//   fake_wheel.h  micros(), random(), map()
//   uros_transport.cpp  Serial.print*/printf, and the WiFiUDP surface in WiFiUdp.h
//
// Anything a future firmware change needs will fail to COMPILE here, which is the
// right failure: a silent divergence between board and host is what this whole
// target exists to avoid.
#ifndef LINO_HOST_ARDUINO_H
#define LINO_HOST_ARDUINO_H

#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <string>

// ---------------------------------------------------------------- time
// A board's millis()/micros() count from boot. clock_gettime(CLOCK_MONOTONIC)
// is the same quantity; the epoch is taken at first call so the numbers start
// near zero as they do on a board, which matters because fake_wheel.h stores
// `unsigned long` timestamps and compares differences.
inline uint64_t _lino_host_now_us()
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    static const uint64_t base =
        (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)ts.tv_nsec / 1000ULL;
    const uint64_t now = (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)ts.tv_nsec / 1000ULL;
    return now - base;
}

inline unsigned long micros() { return (unsigned long)_lino_host_now_us(); }
inline unsigned long millis() { return (unsigned long)(_lino_host_now_us() / 1000ULL); }
inline void delay(unsigned long ms)
{
    struct timespec ts = {(time_t)(ms / 1000), (long)((ms % 1000) * 1000000L)};
    nanosleep(&ts, nullptr);
}
inline void delayMicroseconds(unsigned int us)
{
    struct timespec ts = {(time_t)(us / 1000000), (long)((us % 1000000) * 1000L)};
    nanosleep(&ts, nullptr);
}

// ---------------------------------------------------------------- maths
// Arduino's map() is integer arithmetic and truncates; fake_wheel.h relies on
// that, so this must NOT be a floating-point convenience.
inline long map(long x, long in_min, long in_max, long out_min, long out_max)
{
    if (in_max == in_min) return out_min;
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

template <typename T> inline T constrain(T v, T lo, T hi) { return v < lo ? lo : (v > hi ? hi : v); }

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

#ifndef min
#define min(a, b) ((a) < (b) ? (a) : (b))
#endif
#ifndef max
#define max(a, b) ((a) > (b) ? (a) : (b))
#endif

// ---------------------------------------------------------------- Serial
// The firmware logs through Serial. On a host that is stdout, line-buffered so a
// crash does not swallow the last lines -- the boot banner is how a board says
// which build it is running, and the same is true here.
class _LinoHostSerial
{
public:
    void begin(unsigned long = 0) {}
    operator bool() const { return true; }
    size_t print(const char *s) { return s ? fputs(s, stdout), fflush(stdout), strlen(s) : 0; }
    size_t print(int v) { return printf("%d", v); }
    size_t print(unsigned v) { return printf("%u", v); }
    size_t print(float v) { return printf("%f", v); }
    size_t println() { return printf("\n"); }
    size_t println(const char *s) { return printf("%s\n", s ? s : ""); }
    size_t println(int v) { return printf("%d\n", v); }
    int printf(const char *fmt, ...)
    {
        va_list ap;
        va_start(ap, fmt);
        const int n = vfprintf(stdout, fmt, ap);
        va_end(ap);
        fflush(stdout);
        return n;
    }
    // Stream-ish members uros_transport.cpp touches on the SERIAL branch. The
    // host target is udp4 only, so these exist to compile, and say so loudly
    // rather than pretending to be a serial link.
    size_t write(const uint8_t *, size_t)
    {
        fputs("[host] Serial.write() on the host target: transport must be udp4\n", stderr);
        return 0;
    }
    size_t readBytes(char *, size_t) { return 0; }
    int available() { return 0; }
    void flush() { fflush(stdout); }
};

extern _LinoHostSerial Serial;

// ---------------------------------------------------------------- misc types
typedef uint8_t byte;
typedef bool boolean;

#endif // LINO_HOST_ARDUINO_H
