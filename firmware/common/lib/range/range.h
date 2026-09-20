#ifndef RANGE_H
#define RANGE_H

#include <sensor_msgs/msg/range.h>
sensor_msgs__msg__Range getRange();
// allow_hardware false (fake mode) drops the pins: no interrupt is attached
// and nothing is driven, whatever the env says.
void initRange(bool allow_hardware = true);
// True when both pins resolved to a real GPIO, so /range carries a measurement
// rather than the +INF an unwired board would publish forever.
bool rangePresent();

#endif
