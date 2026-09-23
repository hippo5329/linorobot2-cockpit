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

#ifndef KINEMATICS_H
#define KINEMATICS_H

#include "Arduino.h"

#define RPM_TO_RPS 1/60

class Kinematics
{
    public:
        enum base {DIFFERENTIAL_DRIVE, SKID_STEER, MECANUM};

        base base_platform_;

        struct rpm
        {
            float motor1;
            float motor2;
            float motor3;
            float motor4;
        };
        
        struct velocities
        {
            float linear_x;
            float linear_y;
            float angular_z;
        };

        struct pwm
        {
            int motor1;
            int motor2;
            int motor3;
            int motor4;
        };
        // wheels_x_distance is the WHEELBASE (front-to-rear axle spacing) and
        // angular_scale the skid-steer scrub correction. Both default to the
        // values that reproduce the old behaviour exactly, so a 2wd base is
        // unaffected whether or not a caller passes them.
        Kinematics(base robot_base, int motor_max_rpm, float max_rpm_ratio,
                   float motor_operating_voltage, float motor_power_max_voltage,
                   float wheel_diameter, float wheels_y_distance,
                   float wheels_x_distance = 0.0, float angular_scale = 1.0);
        // The radius the wheels' tangential speed acts at when the base turns.
        // Exposed so a diagnostic can print what the firmware actually
        // believes, rather than what the config meant.
        float getRotationRadius();
        velocities getVelocities(float rpm1, float rpm2, float rpm3, float rpm4);
        rpm getRPM(float linear_x, float linear_y, float angular_z);
        float getMaxRPM();
        // Rescale the ceiling to the voltage the motors are ACTUALLY seeing.
        // At construction max_rpm_ is derived from a static config voltage; a
        // 3S pack sags several volts under load, so the real top speed drops
        // with it. When the firmware is told the live bus voltage (INA219 or a
        // divider), this recomputes the ceiling from it. Opt-in: nothing calls
        // this unless `rpm_track_voltage` is set, so the default behaviour is
        // unchanged.
        void setMeasuredVoltage(float measured_voltage);

    private:
        rpm calculateRPM(float linear_x, float linear_y, float angular_z);
        int getTotalWheels(base robot_base);
        static float rotationRadius(base robot_base, float wheels_y_distance,
                                    float wheels_x_distance, float angular_scale);

        int motor_max_rpm_;
        float max_rpm_ratio_;
        float motor_operating_voltage_;
        float max_rpm_;
        // Neither axle dimension is kept. Both are folded into
        // rotation_radius_ at construction and nothing else reads them, so
        // storing them would leave two unused fields and a second place for
        // the track and the radius derived from it to disagree.
        float rotation_radius_;
        float pwm_res_;
        float wheel_circumference_;
        int total_wheels_;
};

#endif