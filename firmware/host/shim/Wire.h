// Wire.h for the HOST target: an I2C bus with nothing on it.
//
// That is the honest model of the Sim MCU -- a board with no chip fitted -- and
// it is what the firmware's own probing already handles: an address that does
// not ACK (endTransmission() == 2) is "no device", so the sensor factory falls
// back to the simulated drivers exactly as a bare board's does. A shim that
// ACKed would make every driver believe a chip was there and read zeros.
#ifndef LINO_HOST_WIRE_H
#define LINO_HOST_WIRE_H
#include <Arduino.h>

class TwoWire : public Stream
{
public:
    bool begin() { return true; }
    bool begin(int, int, uint32_t = 0) { return true; }
    bool begin(uint8_t) { return true; }
    void end() {}
    void setClock(uint32_t hz) { _clock = hz; }
    uint32_t getClock() const { return _clock; }
    bool setSDA(int) { return true; }
    bool setSCL(int) { return true; }
    bool setPins(int, int) { return true; }
    void setTimeOut(uint16_t) {}
    void setTimeout(uint16_t) {}

    void beginTransmission(uint8_t) {}
    void beginTransmission(int) {}
    // 2: address sent, NACK received -- nobody home.
    uint8_t endTransmission(bool = true) { return 2; }
    size_t requestFrom(uint8_t, size_t, bool = true) { return 0; }
    size_t requestFrom(int, int) { return 0; }
    size_t requestFrom(int, int, int) { return 0; }
    uint8_t requestFrom(uint8_t, uint8_t) { return 0; }
    uint8_t requestFrom(uint8_t, uint8_t, uint8_t) { return 0; }

    size_t write(uint8_t) override { return 1; }
    size_t write(int) { return 1; }
    size_t write(const uint8_t *, size_t n) override { return n; }
    using Print::write;
    int available() override { return 0; }
    int read() override { return -1; }
    int peek() override { return -1; }

private:
    uint32_t _clock = 100000;
};

extern TwoWire Wire;
extern TwoWire Wire1;
#endif
