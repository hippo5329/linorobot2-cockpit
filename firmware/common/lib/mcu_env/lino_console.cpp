#include "lino_console.h"

#if LINO_CONSOLE_SELECTABLE
#include "mcu_env.h"

// This file must see the REAL objects, so the macro is undone here only.
#undef Serial

LinoConsole lino_console;

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
#endif
