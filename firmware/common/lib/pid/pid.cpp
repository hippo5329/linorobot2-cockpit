// Copyright (c) 2021 Juan Miguel Jimeno
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "Arduino.h"
#include "pid.h"
#include <math.h>

PID::PID(float min_val, float max_val, float kp, float ki, float kd):
    min_val_(min_val),
    max_val_(max_val),
    kp_(kp),
    ki_(ki),
    kd_(kd),
    integral_(0.0),
    derivative_(0.0),
    prev_error_(0.0)
{
}

double PID::compute(float setpoint, float measured_value)
{
    double error;
    double pid;

    error = setpoint - measured_value;
    integral_ += error;
    derivative_ = error - prev_error_;

    // Anti-windup. `integral_` was a pure accumulator with no bound, and the
    // only thing that ever reset it was `setpoint == 0 && error == 0` -- exact
    // float equality, which measurement noise makes essentially unreachable.
    //
    // So once the loop saturated it stayed there. Caught on the bench
    // 2026-09-20 with an instrumented build, on a board commanded to stand
    // still:
    //
    //   cmd=0.00,0.00 req=0.0,0.0 rpm=-139.5,138.4 pwm=-1023,1023
    //
    // Both wheels pinned at opposite rails, at the 140 rpm maximum, with a
    // zero request -- and raising the request to 31.4 rpm changed nothing,
    // because the integral term alone was already past the rail. The robot
    // spun on the spot and Nav2 could not drive it.
    //
    // Clamping the integral's CONTRIBUTION (not the raw sum) keeps the term
    // meaningful across different ki: the loop can still hold any output the
    // actuator can reach, and it leaves saturation on the first tick the error
    // reverses, instead of after unwinding a debt it spent minutes building.
    //
    // The bound is the larger rail, so the integral alone can still drive the
    // output all the way to either end -- and no further, which is what makes
    // the exit immediate.
    if (ki_ != 0.0f)
    {
        const double limit = fmax(fabs((double)max_val_), fabs((double)min_val_));
        const double i_max = limit / (double)fabs(ki_);
        if (integral_ > i_max)  integral_ = i_max;
        if (integral_ < -i_max) integral_ = -i_max;
    }

    // A standing still request with the wheel actually stopped: forget the
    // history rather than carry it into the next move. Compared with a
    // tolerance now, because `== 0` on a noisy measurement never fired.
    if (setpoint == 0.0f && fabs(error) < 0.5)
    {
        integral_ = 0;
        derivative_ = 0;
    }

    pid = (kp_ * error) + (ki_ * integral_) + (kd_ * derivative_);
    prev_error_ = error;

    return constrain(pid, min_val_, max_val_);
}

void PID::updateConstants(float kp, float ki, float kd)
{
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}
