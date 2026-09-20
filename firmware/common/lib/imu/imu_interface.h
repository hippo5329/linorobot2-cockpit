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

#ifndef IMU_INTERFACE
#define IMU_INTERFACE

#include <Arduino.h>
#include <math.h>
#include <sensor_msgs/msg/imu.h>
#include "mcu_env.h"

#ifndef ACCEL_COV
#define ACCEL_COV { 0.00001, 0.00001, 0.00001 }
#endif
#ifndef GYRO_COV
#define GYRO_COV { 0.00001, 0.00001, 0.00001 }
#endif
#ifndef ORI_COV
#define ORI_COV { 0.00001, 0.00001, 0.00001 }
#endif

// How much the gyro may wander during the startup average before the result is
// called untrustworthy, in rad/s of standard deviation per axis.
//
// The wiki tells the operator the robot must stand still for calibration, and
// that is the right instruction -- but nothing checked it, so a robot that was
// being carried, or whose wheels were driven, calibrated against its own motion
// and then subtracted that motion from every reading for the rest of the run.
// A part at rest sits around 0.005-0.01 rad/s; 0.05 (about 2.9 deg/s) is well
// clear of noise and well under any real handling.
#ifndef GYRO_CAL_MAX_STDDEV
#define GYRO_CAL_MAX_STDDEV 0.05f
#endif

class IMUInterface
{
    protected:
        // Value-initialised: these objects are heap-allocated (createIMU does
        // `new`), and the constructor hands frame_id to
        // micro_ros_string_utilities_set, which only reallocates when
        // size > capacity. On a reset that is not a power cycle the heap still
        // holds the previous run's bytes, so a garbage capacity skips the
        // realloc and memcpy writes through a garbage data pointer --
        // StoreProhibited, in setup(), before the agent is ever reached.
        sensor_msgs__msg__Imu imu_msg_{};
        const float g_to_accel_ = 9.81;
        const float mgauss_to_utesla_ = 0.1;
        const float utesla_to_tesla_ = 0.000001;

        // Covariance is a property of the SENSOR, and which sensor is fitted is
        // an env key -- so the values are too. The macros stay as the fallback
        // for a board with a blank env; envFloatVec leaves them alone when the
        // key is absent. A scalar in the env expands to all three axes.
        float accel_cov[3] = ACCEL_COV;
        float gyro_cov[3] = GYRO_COV;
        float ori_cov[3] = ORI_COV;
        const int sample_size_ = 40;

        // Value-initialised, for the same reason imu_msg_ above is: every
        // concrete IMU declares its own default constructor, so `new T()` is
        // value-initialisation of a class WITH a user-provided constructor --
        // which does NOT zero the members it does not mention. calibrateGyro()
        // then accumulated onto whatever the heap held, and on a warm reset
        // that is the previous run's bytes, not zeros.
        geometry_msgs__msg__Vector3 gyro_cal_{};

        // True when the board was moving during the last calibrateGyro().
        bool gyro_cal_suspect_ = false;

        void calibrateGyro()
        {
            geometry_msgs__msg__Vector3 gyro;

            gyro_cal_.x = 0.0;
            gyro_cal_.y = 0.0;
            gyro_cal_.z = 0.0;

            // Sum and sum-of-squares in one pass, so the spread of the samples
            // is known as well as their mean. The mean alone cannot tell a
            // stationary board from one being carried: both average to *a*
            // number, and only one of them is a bias.
            double sq[3] = { 0.0, 0.0, 0.0 };

            for(int i=0; i<sample_size_; i++)
            {
                gyro = readGyroscope();
                gyro_cal_.x += gyro.x;
                gyro_cal_.y += gyro.y;
                gyro_cal_.z += gyro.z;
                sq[0] += (double)gyro.x * gyro.x;
                sq[1] += (double)gyro.y * gyro.y;
                sq[2] += (double)gyro.z * gyro.z;

                delay(50);
            }

            const float n = (float)sample_size_;
            gyro_cal_.x = gyro_cal_.x / n;
            gyro_cal_.y = gyro_cal_.y / n;
            gyro_cal_.z = gyro_cal_.z / n;

            const double mean[3] = { gyro_cal_.x, gyro_cal_.y, gyro_cal_.z };
            gyro_cal_suspect_ = false;
            float worst = 0.0f;
            for (int a = 0; a < 3; a++)
            {
                // var = E[x^2] - E[x]^2, clamped: rounding can push a
                // genuinely-zero variance a hair below zero.
                double var = sq[a] / n - mean[a] * mean[a];
                const float sd = (var > 0.0) ? (float)sqrt(var) : 0.0f;
                if (sd > worst)
                    worst = sd;
                if (sd > (float)GYRO_CAL_MAX_STDDEV)
                    gyro_cal_suspect_ = true;
            }

            if (gyro_cal_suspect_)
            {
                Serial.printf("[imu] gyro calibration is SUSPECT: samples varied by "
                              "%.4f rad/s (limit %.4f) -- the robot moved while "
                              "calibrating, so this bias is its motion, not its offset. "
                              "Stand it still and reset.\n",
                              worst, (float)GYRO_CAL_MAX_STDDEV);
            }
            else
            {
                Serial.printf("[imu] gyro bias %.4f %.4f %.4f rad/s (spread %.4f, still)\n",
                              gyro_cal_.x, gyro_cal_.y, gyro_cal_.z, worst);
            }
        }

    public:
        IMUInterface()
        {
            imu_msg_.header.frame_id = micro_ros_string_utilities_set(imu_msg_.header.frame_id, "imu_link");
            applyEnvCovariance();
        }

        // The env's values, if it carries any. Called from the constructor
        // rather than left to each concrete IMU: every one of them inherits
        // this, and a sensor that forgot the call would publish the firmware's
        // 1e-5 placeholder while the config said otherwise -- which the EKF
        // reads as "this IMU is almost perfect".
        void applyEnvCovariance()
        {
            envFloatVec("accel_cov", accel_cov, 3);
            envFloatVec("gyro_cov", gyro_cov, 3);
            envFloatVec("ori_cov", ori_cov, 3);
        }

        virtual geometry_msgs__msg__Vector3 readAccelerometer() = 0;
        virtual geometry_msgs__msg__Vector3 readGyroscope() = 0;
        virtual bool startSensor() = 0;

        bool init()
        {
            bool sensor_ok = startSensor();
            if(sensor_ok)
                calibrateGyro();

            return sensor_ok;
        }

        // Whether the startup average was taken on a board that was moving.
        // The reading is still published -- refusing to publish would be worse
        // than publishing a known-doubtful bias -- but the caller can say so.
        bool gyroCalSuspect() const { return gyro_cal_suspect_; }

        sensor_msgs__msg__Imu getData()
        {
            imu_msg_.angular_velocity = readGyroscope();
            // Gyro bias is removed HERE, for every driver, and nowhere else.
            //
            // This used to be skipped under `#ifndef USE_MPU6050_IMU`, because
            // that one driver also asked its chip to self-calibrate. Two
            // problems: the macro is emitted by nothing since the sensor
            // factory replaced compile-time driver selection, so the guard was
            // dead and the subtraction happened anyway; and it was a BUILD-time
            // answer to a question that is now decided at boot, by the I2C
            // probe, so it could not have been right on a board whose IMU is
            // detected rather than declared. One place, one rule: init() runs
            // calibrateGyro() after startSensor(), and this subtracts it.
            imu_msg_.angular_velocity.x -= gyro_cal_.x;
            imu_msg_.angular_velocity.y -= gyro_cal_.y;
            imu_msg_.angular_velocity.z -= gyro_cal_.z;

            if(imu_msg_.angular_velocity.x > -0.01 && imu_msg_.angular_velocity.x < 0.01 )
                imu_msg_.angular_velocity.x = 0;

            if(imu_msg_.angular_velocity.y > -0.01 && imu_msg_.angular_velocity.y < 0.01 )
                imu_msg_.angular_velocity.y = 0;

            if(imu_msg_.angular_velocity.z > -0.01 && imu_msg_.angular_velocity.z < 0.01 )
                imu_msg_.angular_velocity.z = 0;

            imu_msg_.angular_velocity_covariance[0] = gyro_cov[0];
            imu_msg_.angular_velocity_covariance[4] = gyro_cov[1];
            imu_msg_.angular_velocity_covariance[8] = gyro_cov[2];

            imu_msg_.linear_acceleration = readAccelerometer();
            imu_msg_.linear_acceleration_covariance[0] = accel_cov[0];
            imu_msg_.linear_acceleration_covariance[4] = accel_cov[1];
            imu_msg_.linear_acceleration_covariance[8] = accel_cov[2];

            imu_msg_.orientation_covariance[0] = ori_cov[0];
            imu_msg_.orientation_covariance[4] = ori_cov[1];
            imu_msg_.orientation_covariance[8] = ori_cov[2];

#ifdef IMU_TWEAK
            IMU_TWEAK
#endif
            return imu_msg_;
        }
};

#endif
