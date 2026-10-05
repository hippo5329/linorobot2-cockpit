// The UNO Q's two wheels: PWM out, quadrature counts in -- or, with `sim_wheel`, a
// wheel model fed the same PWM and counted the same way, so one image runs both and
// the choice is the env's.
#pragma once
#include <stdbool.h>

#define DRIVE_MOTORS 2

bool  driveInit(void);
// Signed command in [-pwmMax(), pwmMax()]; 0 lets the bridge coast.
void  driveSpin(int motor, int pwm);
int   drivePwmMax(void);
// Wheel RPM over the time since the previous call (control loop only); advances
// the wheel model first when it is on.
void  driveMeasure(float dt_s, float rpm_out[DRIVE_MOTORS]);
bool  driveSimulated(void);
