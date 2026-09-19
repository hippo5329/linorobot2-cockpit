#ifndef BATTERY_H
#define BATTERY_H

#include <sensor_msgs/msg/battery_state.h>
sensor_msgs__msg__BatteryState getBattery();
void initBattery();
// True when a voltage can actually be reported: the INA219 answered the I2C
// probe in initBattery(), or the board has a divider on an ADC pin.
bool batteryPresent();
void getBatteryPercentage(sensor_msgs__msg__BatteryState *msg);

#endif
