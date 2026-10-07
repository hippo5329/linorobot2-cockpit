#include "lino_console.h"

#if LINO_CONSOLE_WRAPPED
#include "mcu_env.h"
#include "syslog.h"

// This file must see the REAL objects, so the macro is undone here only.
#undef Serial

LinoConsole lino_console;

#if LINO_CONSOLE_SELECTABLE
void LinoConsole::selectFromEnv()
{
    const char *c = envGet("console", "usb");
    uart0_ = (strcasecmp(c, "uart0") == 0);
}

void LinoConsole::begin(unsigned long baud)
{
    if (uart0_) Serial0.begin(baud);
    else        Serial.begin(baud);
}

void LinoConsole::end()
{
    if (uart0_) Serial0.end();
    else        Serial.end();
}

size_t LinoConsole::setRxBufferSize(size_t n)
{
    return uart0_ ? Serial0.setRxBufferSize(n) : Serial.setRxBufferSize(n);
}

size_t LinoConsole::setTxBufferSize(size_t n)
{
    return uart0_ ? Serial0.setTxBufferSize(n) : Serial.setTxBufferSize(n);
}
#else
// A classic ESP32: one console, the UART, so nothing to select.
void LinoConsole::selectFromEnv() {}
void LinoConsole::begin(unsigned long baud) { Serial.begin(baud); }
void LinoConsole::end() { Serial.end(); }
size_t LinoConsole::setRxBufferSize(size_t n) { return Serial.setRxBufferSize(n); }
size_t LinoConsole::setTxBufferSize(size_t n) { return Serial.setTxBufferSize(n); }
#endif

// A line at a time: on a newline, or when the buffer is full (a long line goes
// out in pieces rather than not at all). '\r' is dropped. syslog() writes to its
// own UDP socket, never to the console, so this cannot recurse.
void LinoConsole::teeByte(uint8_t c)
{
    if (c == '\r')
        return;
    if (c != '\n')
        line_[len_++] = (char)c;
    if (c == '\n' || len_ >= sizeof(line_) - 1) {
        if (len_) {
            line_[len_] = '\0';
            syslog(LOG_INFO, "%s", line_);
        }
        len_ = 0;
    }
}
#endif
