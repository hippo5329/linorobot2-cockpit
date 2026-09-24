// Drivetrain built from the env partition instead of from #defines.
//
// Pins, wheel geometry, PID gains, the motor driver type and whether the wheels
// are real or simulated were all compile-time facts, which is why two ESP32
// robots that differ only in wiring needed two firmwares. They come out of the
// env partition now, each falling back to what the config was generated for, so
// a board with a blank env behaves exactly as its YAML says while the same
// binary can be rewired by rewriting 4 KB of flash.
//
// Everything here is built in setup(), never at static-init: the flash
// partition API is not usable that early, and a global would have to take its
// values from macros -- which is the thing being removed.
#ifndef HW_FACTORY_H
#define HW_FACTORY_H

#include "encoder_interface.h"
#include "motor_interface.h"
#include "kinematics.h"
#include "pid.h"

// index is 1..4, matching MOTOR1_* .. MOTOR4_* and the env keys m1_* .. m4_*.
EncoderInterface *createEncoder(int index);
MotorInterface   *createMotor(int index);
PID              *createPID(void);
Kinematics       *createKinematics(void);

// Whether this boot simulates the wheels. Read once; the control loop and the
// simulated IMU both need the same answer.
bool wheelsAreSim(void);

#endif // HW_FACTORY_H
