// Wire for the UNO Q (Zephyr) target: TwoWire on the Qwiic bus (i2c4).
//
// Arduino's model is a buffered transaction: beginTransmission/write queue bytes,
// endTransmission(false) keeps the bus for a repeated-start read, and requestFrom
// collects into a buffer that read()/available() drain. Mapped onto Zephyr's
// i2c_write / i2c_write_read / i2c_read. Wire1 is the same bus: the board brings out
// one I2C to the robot (Qwiic), and the firmware only names Wire1 for boards with two.
#ifndef LINO_ZEPHYR_WIRE_H
#define LINO_ZEPHYR_WIRE_H
#include <Arduino.h>

class TwoWire : public Stream
{
public:
    void begin() {}
    void begin(int, int) {}
    void begin(int, int, uint32_t freq) { setClock(freq); }
    bool setPins(int, int) { return true; }
    bool setSDA(int) { return true; }
    bool setSCL(int) { return true; }
    void end() {}
    void setClock(uint32_t freq);
    void setTimeout(uint16_t) {}
    void setTimeOut(uint16_t) {}
    void beginTransmission(uint8_t addr) { addr_ = addr; tx_len_ = 0; }
    void beginTransmission(int addr) { beginTransmission((uint8_t)addr); }
    uint8_t endTransmission(bool stop = true);
    uint8_t requestFrom(uint8_t addr, size_t n, bool stop = true);
    uint8_t requestFrom(int addr, int n) { return requestFrom((uint8_t)addr, (size_t)n, true); }
    uint8_t requestFrom(int addr, int n, int stop) { return requestFrom((uint8_t)addr, (size_t)n, stop != 0); }
    size_t write(uint8_t c) override;
    size_t write(int c) { return write((uint8_t)c); }   // Wire.write(0x00): as Arduino's
    size_t write(const uint8_t *buf, size_t n) override;
    using Print::write;
    int available() override { return (int)(rx_len_ - rx_pos_); }
    int read() override { return rx_pos_ < rx_len_ ? rx_[rx_pos_++] : -1; }
    int peek() override { return rx_pos_ < rx_len_ ? rx_[rx_pos_] : -1; }
    size_t readBytes(char *buf, size_t n) override
    {
        size_t i = 0;
        while (i < n && rx_pos_ < rx_len_) buf[i++] = (char)rx_[rx_pos_++];
        return i;
    }
private:
    uint8_t addr_ = 0;
    uint8_t tx_[128];
    size_t tx_len_ = 0;
    bool tx_held_ = false;       // endTransmission(false): the next requestFrom is a write-read
    uint8_t rx_[128];
    size_t rx_len_ = 0, rx_pos_ = 0;
};
extern TwoWire Wire;
extern TwoWire Wire1;
#endif
