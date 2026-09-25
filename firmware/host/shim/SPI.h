// SPI.h for the HOST target: a bus with nothing on it (see Wire.h).
#ifndef LINO_HOST_SPI_H
#define LINO_HOST_SPI_H
#include <Arduino.h>
#define MSBFIRST 1
#define LSBFIRST 0
#define SPI_MODE0 0
#define SPI_MODE1 1
#define SPI_MODE2 2
#define SPI_MODE3 3
class SPISettings
{
public:
    SPISettings() {}
    SPISettings(uint32_t, uint8_t, uint8_t) {}
};
class SPIClass
{
public:
    void begin() {}
    void end() {}
    void beginTransaction(SPISettings) {}
    void endTransaction() {}
    uint8_t transfer(uint8_t) { return 0xFF; }
};
extern SPIClass SPI;
#endif
