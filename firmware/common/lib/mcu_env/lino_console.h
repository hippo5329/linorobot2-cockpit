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

#if defined(ARDUINO_USB_CDC_ON_BOOT) && ARDUINO_USB_CDC_ON_BOOT
#define LINO_CONSOLE_SELECTABLE 1

class LinoConsole : public Stream
{
    private:
        bool uart0_ = false;
        Stream &s() { return uart0_ ? static_cast<Stream &>(Serial0) : static_cast<Stream &>(Serial); }

    public:
        // Read the env key. Called once, at the top of setup(), before the
        // first print; the flash partition is readable by then.
        void selectFromEnv();
        void select(bool uart0) { uart0_ = uart0; }
        bool isUart0() const { return uart0_; }
        const char *name() const { return uart0_ ? "uart0" : "usb"; }

        void begin(unsigned long baud);
        void end();
        size_t setRxBufferSize(size_t n);
        size_t setTxBufferSize(size_t n);
        // A bridge is always "open"; only the native port waits for a host.
        operator bool() { return uart0_ ? true : (bool)Serial; }

        int available() override { return s().available(); }
        int read() override { return s().read(); }
        int peek() override { return s().peek(); }
        void flush() override { s().flush(); }
        size_t write(uint8_t c) override { return s().write(c); }
        size_t write(const uint8_t *buf, size_t n) override { return s().write(buf, n); }
        using Print::write;
};

extern LinoConsole lino_console;

// From here on, in every translation unit that includes this header, `Serial`
// is the selectable console.
#define Serial lino_console

#else
#define LINO_CONSOLE_SELECTABLE 0
#endif

#endif // LINO_CONSOLE_H
