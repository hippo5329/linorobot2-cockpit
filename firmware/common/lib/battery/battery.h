#ifndef BATTERY_H
#define BATTERY_H

#include <sensor_msgs/msg/battery_state.h>
sensor_msgs__msg__BatteryState getBattery();
void initBattery();
// True when a voltage can actually be reported: the INA219 answered the I2C
// probe in initBattery(), or the board has a divider on an ADC pin.
bool batteryPresent();
void getBatteryPercentage(sensor_msgs__msg__BatteryState *msg);
// The simulated pack (env `sim_battery`): a battery voltage sensor for a board
// with none wired. main.cpp feeds it the wheel model's load before each read, so
// the voltage sags when the robot accelerates and the charge drains as it drives.
bool batteryIsSim();
void setSimPackLoad(float sag_divisor, float amps);

#endif
