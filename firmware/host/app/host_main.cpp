// The Arduino runtime's own main(), for the host: setup() once, then loop()
// for ever. Nothing else lives here -- every line of behaviour is the
// firmware's (src/main.cpp and common/lib), compiled unmodified against
// ../shim. A board's core does exactly this after its own hardware init.
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

// The core's global objects, which a board's core library defines.
_LinoHostSerial Serial;
TwoWire Wire;
TwoWire Wire1;
SPIClass SPI;

void setup();
void loop();

int main()
{
    setup();
    for (;;)
        loop();
}
