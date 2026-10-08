// Copyright (c) 2026 Linorobot contributors
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
#ifndef BASE_IDENT_H
#define BASE_IDENT_H

#include "pid.h"
#include "kinematics.h"

// The drivetrain identification, run INSIDE the base application (2026-10-08, user:
// "since we unify the firmware, we can call test_acc with base firmware mode?").
//
// test_acc is a separate app: switching to it is an env write and a reboot, micro-ROS
// drops, and the robot computer sees nothing of the run but syslog. This is the same
// measurement as a state machine advanced one step per control tick from moveBase(),
// so micro-ROS, /odom, /imu and the LiDAR stay up throughout and the robot computer
// tracks the run from the room as it happens.
//
// Started by "lino-ident" on the ping port (ota.h). Each step drives every wheel at
// once, forward and then the same backward, so the robot goes out and comes back:
//   dead zone   PWM ramped until each wheel's encoder moves
//   plant       full PWM for 0.6 s: steady rpm, tau, K
//   loop        the robot's own PID at 20 / 55 / 90 % of top, 1.5 s each
//   runs        1 s straight sprints and half-PWM spins, both ways, at 100 / 50 / 25 %
// It reports to syslog in test_acc's exact formats (IDENT ..., MAX PWM, MAX VEL, time
// to 0.9x max vel), so drivetrain_report.py reads either. It stops -- motors to zero,
// "IDENT aborted: <why>" -- on "lino-stop", on any non-zero /cmd_vel (whoever is
// driving takes over), and on a simulated-wheel board refuses to start.

// One control tick. `cur` is each wheel's measured rpm this tick. Returns true when the
// identification drove the motors this tick (pwm[] is what to spin, the PID was not
// used); false when it is idle and the base runs as usual. `operator_cmd` is a non-zero
// /cmd_vel received in the last 200 ms.
bool baseIdentTick(const float cur[4], int pwm[4], PID *const pids[4], Kinematics *kin,
                   unsigned motors, int pwm_max, bool operator_cmd, bool sim_wheels);

bool baseIdentActive(void);

#endif
