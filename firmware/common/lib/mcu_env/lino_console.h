// The board's console -- and, on an ESP32-S3, WHICH port that is.
//
// The S3 images are built with ARDUINO_USB_CDC_ON_BOOT=1, so `Serial` is the
// chip's native USB (HW CDC/JTAG on GPIO 19/20) and UART0 (GPIO 43/44) is
// `Serial0`, which nothing here used. That fits the DevKit; it does not fit a
// board whose only USB is a bridge on UART0 -- the Yahboom YB-EET01 puts a
// CP2102 on 43/44 and breaks out no native USB at all -- and on such a board the
// released image printed its banner, and ran micro-ROS, into a port that does
// not exist.
//
// The MCU is the same, so the choice belongs to the env, not the build: the
// `console` key (`usb`, the default, or `uart0`) picks the Stream at boot.
// Every `Serial.` in this tree then goes through lino_console, which forwards
// to HWCDC Serial or HardwareSerial Serial0 -- the macro at the bottom is what
// makes that happen without touching 250 call sites, and it is scoped to the
// translation units that include this header (every one of ours; the Arduino
// core and the vendored libraries are compiled separately and keep the real
// object). On a board without CDC-on-boot there is nothing to choose and this
// header defines nothing.
#ifndef LINO_CONSOLE_H
#define LINO_CONSOLE_H

#include <Arduino.h>

// 2026-10-07: every ESP32 gets the wrapper, not only the S3, for one more job --
// the syslog TEE. A Wi-Fi robot (ESP32/ESP32-S3 on udp4) leaves the USB cable after
// its first flash, and a diagnostic tool's whole output is its console. With the tee
// on, each complete console line also goes to syslog, where the cockpit's Monitor
// streams it (the robot computer is the syslog sink). main.cpp turns it on for a
// tool on a Wi-Fi robot only: the base application's console is the micro-ROS
// serial transport on a serial robot, and its frames must never be split into
// log lines. On a classic ESP32 there is nothing to select; the wrapper forwards
// to the UART Serial.
// 2026-10-12: the RP2 boards get it too, for the tool's network terminal (user: "wifi
// tool can support picow pico2w, because they are low rate"): every console line of a
// tool also goes to the connected terminal client (setNet). The micro-ROS transport
// keeps the real SerialUSB there (linoConsoleStream()), so base's bytes take the path
// they always took.
#if (defined(ARDUINO_USB_CDC_ON_BOOT) && ARDUINO_USB_CDC_ON_BOOT) || defined(ESP32) || defined(ARDUINO_ARCH_RP2040)
#if defined(ARDUINO_USB_CDC_ON_BOOT) && ARDUINO_USB_CDC_ON_BOOT
#define LINO_CONSOLE_SELECTABLE 1
#else
#define LINO_CONSOLE_SELECTABLE 0
#endif
#define LINO_CONSOLE_WRAPPED 1
class LinoConsole : public Stream
{
    private:
        bool uart0_ = false;
        bool tee_ = false;
        Print *net_ = nullptr;
        char line_[160];
        size_t len_ = 0;
#if LINO_CONSOLE_SELECTABLE
        Stream &s() { return uart0_ ? static_cast<Stream &>(Serial0) : static_cast<Stream &>(Serial); }
#else
        Stream &s() { return static_cast<Stream &>(Serial); }
#endif
        void teeByte(uint8_t c);
    public:
        // Read the env key. Called once, at the top of setup(), before the
        // first print; the flash partition is readable by then.
        void selectFromEnv();
        void select(bool use_uart0) { uart0_ = use_uart0; }
        bool isUart0() const { return uart0_; }
        const char *name() const { return uart0_ ? "uart0" : "usb"; }
        // Copy every complete line to syslog as well (a tool on a Wi-Fi robot).
        void teeToSyslog(bool on) { tee_ = on; len_ = 0; }
        // Copy every byte to the tool's terminal client as well (NULL: none).
        void setNet(Print *p) { net_ = p; }
        // The port itself, unwrapped: the micro-ROS transport writes here.
        Stream &raw() { return s(); }
        void begin(unsigned long baud);
        void end();
        size_t setRxBufferSize(size_t n);
        size_t setTxBufferSize(size_t n);
        // A bridge is always "open"; only the native port waits for a host.
#if LINO_CONSOLE_SELECTABLE
        operator bool() { return uart0_ ? true : (bool)Serial; }
#elif defined(ARDUINO_ARCH_RP2040)
        operator bool() { return (bool)Serial; }   // the USB port: a host has it open
#else
        operator bool() { return true; }      // a classic ESP32's UART: always "open"
#endif
        int available() override { return s().available(); }
        int read() override { return s().read(); }
        int peek() override { return s().peek(); }
        void flush() override { s().flush(); }
        size_t write(uint8_t c) override
        {
            if (tee_) teeByte(c);
            if (net_) net_->write(c);
            return s().write(c);
        }
        size_t write(const uint8_t *buf, size_t n) override
        {
            if (tee_)
                for (size_t i = 0; i < n; i++) teeByte(buf[i]);
            if (net_) net_->write(buf, n);
            return s().write(buf, n);
        }
        using Print::write;
};
extern LinoConsole lino_console;
inline Stream &linoConsoleStream() { return lino_console.raw(); }
// From here on, in every translation unit that includes this header, `Serial`
// is the console wrapper.
#define Serial lino_console
#else
#define LINO_CONSOLE_SELECTABLE 0
#define LINO_CONSOLE_WRAPPED 0
inline Stream &linoConsoleStream() { return Serial; }
#endif
#endif // LINO_CONSOLE_H
