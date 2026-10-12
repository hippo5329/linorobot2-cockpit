// The tool's hardware, brought up once when the tool boots (2026-10-12: one tool, many
// commands). The drivetrain is main.cpp's -- initDrivetrain() builds it for every mode,
// first thing in setup() -- so the commands share those motors, encoders and PIDs; what
// base builds later, after the tool has taken over (the IMU, the magnetometer, battery,
// range and barometer), is built here, once. Each command used to build all of it again
// in its own setup(): two commands in a row would have created every motor twice.
#ifndef TOOL_HW_H
#define TOOL_HW_H

#include <Arduino.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/magnetic_field.h>
#include <sensor_msgs/msg/battery_state.h>
#include <sensor_msgs/msg/range.h>
#include "motor.h"
#include "encoder.h"
#include "pid.h"
#include "kinematics.h"
#include "imu.h"
#include "mag.h"

// main.cpp's drivetrain.
extern EncoderInterface *motor_encoders[4];
extern MotorInterface   *motor_controllers[4];
extern PID              *motor_pids[4];
extern Kinematics       *kinematics;

namespace tool_hw {

extern IMUInterface *imu;        // NULL: none fitted (real parts only)
extern bool imu_ok;              // init() answered
extern MAGInterface *mag;
extern bool mag_ok;
extern const char *imu_name;     // what the bus and the env settled on
extern const char *mag_name;
extern bool env_ok;              // a barometer answered
extern unsigned total_motors;    // 2 on a differential drive, else 4

extern sensor_msgs__msg__Imu *imu_msg;
extern sensor_msgs__msg__MagneticField *mag_msg;
extern sensor_msgs__msg__BatteryState *battery_msg;
extern sensor_msgs__msg__Range *range_msg;

// Once, from toolSetup(). Prints what it found, in the lines test_sensors always
// printed ("[+] IMU initialized successfully." ...): a host reading them still can.
void init(void);
// The same lines again, for `sensors` and `status`.
void report(void);

}  // namespace tool_hw

#endif // TOOL_HW_H
