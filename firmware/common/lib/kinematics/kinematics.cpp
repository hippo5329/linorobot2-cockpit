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
#include "kinematics.h"

// The radius the wheels' tangential speed acts at when the base turns.
//
// This used to be lr/2 for every base type, written once inline in
// calculateRPM() and once more in getVelocities(). It is right for a
// differential drive and wrong for the other two, and because the SAME wrong
// radius is used to command the wheels and to read them back, a simulated
// robot is perfectly self-consistent: sim wheels turn the command into an
// RPM and the RPM back into a velocity, the error cancels, and odom agrees
// with cmd_vel to three decimals. Every bench result to date is sim mode, so
// nothing on the bench can see this. It appears the first time real wheels
// touch a real floor.
//
//   DIFFERENTIAL_DRIVE   lr/2
//       The ICR is the midpoint of the drive axle. Unchanged.
//
//   MECANUM              (lr + fr)/2
//       The textbook (lx + ly). A mecanum wheel's rollers take the wheelbase
//       into the yaw term, so ignoring fr under-commands the turn by a factor
//       of (1 + fr/lr) -- on the shipped mecanum reference, lr 0.271 and fr
//       0.18, that is 1.66x. Ask for 1.0 rad/s and a real base gives 0.60.
//
//   SKID_STEER           lr/2 * angular_scale
//       Geometrically lr/2, like a differential drive: the four wheels sit at
//       +/-lr/2 and the wheelbase does not enter the ideal model. What enters
//       is that four driven wheels CANNOT pivot without sliding sideways. The
//       scrub moves the real centres of rotation outboard, so the base turns
//       slower than the ideal for the same wheel speeds -- equivalent to a
//       track wider than the one you can measure with a tape.
//
//       How much wider depends on the tyres and the floor, so there is no
//       honest default but 1.0 (the ideal). A real 4-wheel skid base measures
//       roughly 1.1-1.5. It is a measurement, not a constant: command a 360
//       degree turn, divide the angle the base actually turned by the angle
//       odom claims, and put the ratio in kinematics.angular_scale.
float Kinematics::rotationRadius(base robot_base, float wheels_y_distance,
                                 float wheels_x_distance, float angular_scale)
{
    if (robot_base == MECANUM)
        return (wheels_y_distance + wheels_x_distance) / 2.0;
    if (robot_base == SKID_STEER)
        return (wheels_y_distance / 2.0) * angular_scale;
    return wheels_y_distance / 2.0;
}

Kinematics::Kinematics(base robot_base, int motor_max_rpm, float max_rpm_ratio,
                       float motor_operating_voltage, float motor_power_max_voltage,
                       float wheel_diameter, float wheels_y_distance,
                       float wheels_x_distance, float angular_scale):
    base_platform_(robot_base),
    motor_max_rpm_(motor_max_rpm),
    max_rpm_ratio_(max_rpm_ratio),
    motor_operating_voltage_(motor_operating_voltage),
    rotation_radius_(rotationRadius(robot_base, wheels_y_distance,
                                    wheels_x_distance, angular_scale)),
    wheel_circumference_(PI * wheel_diameter),
    total_wheels_(getTotalWheels(robot_base))
{
    motor_power_max_voltage = constrain(motor_power_max_voltage, 0, motor_operating_voltage);
    max_rpm_ =  ((motor_power_max_voltage / motor_operating_voltage) * motor_max_rpm) * max_rpm_ratio;
}

void Kinematics::setMeasuredVoltage(float measured_voltage)
{
    if (motor_operating_voltage_ <= 0.0f)
        return;
    measured_voltage = constrain(measured_voltage, 0, motor_operating_voltage_);
    max_rpm_ = ((measured_voltage / motor_operating_voltage_) * motor_max_rpm_) * max_rpm_ratio_;
}

Kinematics::rpm Kinematics::calculateRPM(float linear_x, float linear_y, float angular_z)
{

    float tangential_vel = angular_z * rotation_radius_;

    //convert m/s to m/min
    float linear_vel_x_mins = linear_x * 60.0;
    float linear_vel_y_mins = linear_y * 60.0;
    //convert rad/s to rad/min
    float tangential_vel_mins = tangential_vel * 60.0;

    float x_rpm = linear_vel_x_mins / wheel_circumference_;
    float y_rpm = linear_vel_y_mins / wheel_circumference_;
    float tan_rpm = tangential_vel_mins / wheel_circumference_;

    // Scale the whole request down until the busiest wheel fits, so an
    // unreachable command comes out as the SAME motion more slowly.
    //
    // This replaces two special cases that between them missed the one a
    // mecanum base actually drives in. They were:
    //
    //     if (|x| + |y| >= max && angular_z == 0)        ... scale x and y
    //     else if (|x| + |tan| >= max && linear_y == 0)  ... scale x and tan
    //
    // Each guard demands that one of the three components be EXACTLY zero.
    // A differential drive always has linear_y exactly zero -- getRPM() assigns
    // it -- so the second branch fires and the behaviour below is identical for
    // 2wd and skid. A mecanum base is the case where all three are non-zero at
    // once, which is the whole point of having one, and `angular_z == 0` is
    // float equality on a controller output: Nav2 sends 1e-4 rad/s, not 0.
    // Neither branch fires, nothing is scaled, and the four constrain() calls
    // below clip whichever wheels are over the rail.
    //
    // Clipping one wheel is not "slower": it changes the ratio between the
    // wheels, and for a mecanum the ratio IS the direction. A base asked to
    // strafe diagonally at more than it can do would curve off the commanded
    // heading instead of tracking it at reduced speed -- and keep its odometry
    // straight-faced about it, because getVelocities() reads the clipped wheels
    // back as whatever motion they really describe.
    //
    // The general rule subsumes both old cases exactly: with y == 0 the peak is
    // |x| + |tan|, with tan == 0 it is |x| + |y|.
    float peak = 0.0;
    for (int i = 0; i < 4; i++)
    {
        // The same four combinations the motors are assigned below. The signs
        // are what decides which wheel is worst, so they cannot be summarised
        // as |x| + |y| + |tan| -- that would scale a base that is nowhere near
        // its limit.
        float w = (i == 0) ? (x_rpm - y_rpm - tan_rpm)
                : (i == 1) ? (x_rpm + y_rpm + tan_rpm)
                : (i == 2) ? (x_rpm + y_rpm - tan_rpm)
                           : (x_rpm - y_rpm + tan_rpm);
        w = fabs(w);
        if (w > peak) peak = w;
    }
    if (peak > max_rpm_ && peak > 0.0)
    {
        float vel_scaler = max_rpm_ / peak;

        x_rpm *= vel_scaler;
        y_rpm *= vel_scaler;
        tan_rpm *= vel_scaler;
    }

    Kinematics::rpm rpm;

    //calculate for the target motor RPM and direction
    //front-left motor
    rpm.motor1 = x_rpm - y_rpm - tan_rpm;
    rpm.motor1 = constrain(rpm.motor1, -max_rpm_, max_rpm_);

    //front-right motor
    rpm.motor2 = x_rpm + y_rpm + tan_rpm;
    rpm.motor2 = constrain(rpm.motor2, -max_rpm_, max_rpm_);

    //rear-left motor
    rpm.motor3 = x_rpm + y_rpm - tan_rpm;
    rpm.motor3 = constrain(rpm.motor3, -max_rpm_, max_rpm_);

    //rear-right motor
    rpm.motor4 = x_rpm - y_rpm + tan_rpm;
    rpm.motor4 = constrain(rpm.motor4, -max_rpm_, max_rpm_);

    return rpm;
}

Kinematics::rpm Kinematics::getRPM(float linear_x, float linear_y, float angular_z)
{
    if(base_platform_ == DIFFERENTIAL_DRIVE || base_platform_ == SKID_STEER)
    {
        linear_y = 0;
    }

    return calculateRPM(linear_x, linear_y, angular_z);;
}

Kinematics::velocities Kinematics::getVelocities(float rpm1, float rpm2, float rpm3, float rpm4)
{
    Kinematics::velocities vel;
    float average_rps_x;
    float average_rps_y;
    float average_rps_a;

    if(base_platform_ == DIFFERENTIAL_DRIVE)
    {
        rpm3 = 0.0;
        rpm4 = 0.0;
    }
 
    //convert average revolutions per minute to revolutions per second
    average_rps_x = ((float)(rpm1 + rpm2 + rpm3 + rpm4) / total_wheels_) / 60.0; // RPM
    vel.linear_x = average_rps_x * wheel_circumference_; // m/s

    //convert average revolutions per minute in y axis to revolutions per second
    average_rps_y = ((float)(-rpm1 + rpm2 + rpm3 - rpm4) / total_wheels_) / 60.0; // RPM
    if(base_platform_ == MECANUM)
        vel.linear_y = average_rps_y * wheel_circumference_; // m/s
    else
        vel.linear_y = 0;

    //convert average revolutions per minute to revolutions per second
    average_rps_a = ((float)(-rpm1 + rpm2 - rpm3 + rpm4) / total_wheels_) / 60.0;
    // The SAME radius that turned the command into wheel speeds turns them back,
    // so a base cannot report a yaw rate it was never asked to produce.
    vel.angular_z =  (average_rps_a * wheel_circumference_) / rotation_radius_; //  rad/s

    return vel;
}

int Kinematics::getTotalWheels(base robot_base)
{
    switch(robot_base)
    {
        case DIFFERENTIAL_DRIVE:    return 2;
        case SKID_STEER:            return 4;
        case MECANUM:               return 4;
        default:                    return 2;
    }
}

float Kinematics::getRotationRadius()
{
    return rotation_radius_;
}

float Kinematics::getMaxRPM()
{
    return max_rpm_;
}