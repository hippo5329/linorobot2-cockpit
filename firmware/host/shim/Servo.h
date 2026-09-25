// Servo.h for the HOST target: the ESC/servo motor driver's output, stored.
// The Sim MCU drives no motor (sim_wheel.h models the wheels), so the pulse
// goes nowhere -- as it would on a board with nothing on the pin.
#ifndef LINO_HOST_SERVO_H
#define LINO_HOST_SERVO_H
#include <Arduino.h>
class Servo
{
public:
    int attach(int pin) { _pin = pin; return 1; }
    int attach(int pin, int, int) { _pin = pin; return 1; }
    void detach() { _pin = -1; }
    bool attached() const { return _pin >= 0; }
    void write(int v) { _us = v; }
    void writeMicroseconds(int us) { _us = us; }
    int read() const { return _us; }
    int readMicroseconds() const { return _us; }
    void setPeriodHertz(int) {}
private:
    int _pin = -1;
    int _us = 1500;
};
#endif
