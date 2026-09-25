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
#include <micro_ros_utilities/string_utilities.h>
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

#ifndef IMU_SAMPLE_AGE_MAX_US
// The most a stamp may ever be moved backwards. Two sample periods at the
// slowest rate this firmware publishes (50 Hz -> 20 ms) with room to spare: a
// larger age is a fault, not a late read, and a fault must not be allowed to
// place a message at an arbitrary time.
#define IMU_SAMPLE_AGE_MAX_US 50000
#endif


class IMUInterface
{
    private:
        uint32_t age_n_ = 0;
        uint64_t age_sum_us_ = 0;
        uint32_t age_min_us_ = 0xFFFFFFFFu;
        uint32_t age_max_us_ = 0;

    protected:

        // How old the sample just read is, in microseconds, ACCORDING TO THE
        // CHIP. Modern MEMS parts keep a monotonic counter and put its value
        // in the FIFO beside the sample -- ICM-42670 (16-bit, ~1 us tick),
        // LSM6DSOX (32-bit, 25 us tick), BMI270 (24-bit) -- so the sample can
        // say when it was taken rather than when it was fetched.
        //
        // An AGE, not the counter itself, and that is the design decision
        // worth defending. The chip's oscillator is not the MCU's and drifts
        // against it, so turning a raw counter into a ROS stamp means keeping
        // a linear fit that has to be re-anchored forever. An age is only ever
        // used across one sample interval (~20 ms), where even a 2 % clock
        // error is 0.4 us -- below the quantisation of everything downstream.
        //
        // 0 means "this driver cannot say", which is the default and is not
        // the same as "the sample is new": see sampleAgeUs().
        virtual uint32_t chipSampleAgeUs() { return 0; }


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
        // Whether this part computes its own orientation.
        //
        // A BNO085 runs a full AHRS on the chip and returns a quaternion, so it
        // does its best and the firmware's filter stands aside. Everything else
        // returns accel and gyro (and a field, if a magnetometer answered) and
        // the board fuses them -- since 2026-09-25, in ahrs.h, instead of
        // shipping imu/data_raw to imu_filter_madgwick and depending on the pair
        // surviving a best-effort link.
        //
        // Default false, so a new driver is fused rather than silently trusted
        // to have filled in a quaternion it never touched. That failure would be
        // an identity orientation published as a heading.
        virtual bool hasFusedOrientation() { return false; }
        IMUInterface()
        {
            imu_msg_.header.frame_id = micro_ros_string_utilities_set(imu_msg_.header.frame_id, "imu_link");
            applyEnvCovariance();
        }

        // Re-stamp the frame with the robot's namespace. Separate from the
        // constructor for the reason applyEnvCovariance() documents: a global
        // is built before the flash partition API is usable, and the env reads
        // back empty there.
        void applyEnvFrames()
        {
            imu_msg_.header.frame_id =
                micro_ros_string_utilities_set(imu_msg_.header.frame_id, envPrefixed("imu_link"));
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
            // A valid identity rotation, for every driver that does not compute
            // one. The field is zero-initialised, which is (0,0,0,0) -- not
            // "no rotation" but NOT A ROTATION: norm 0, so anything that
            // normalises divides by zero and RViz rejects the message outright.
            //
            // It mattered little while this message was only imu/data_raw,
            // because madgwick ignored the orientation of its input. It matters
            // now: the base publishes imu/data itself, and that is the topic
            // consumers read. Measured on the bench's
            // Pico 2 with an LSM6DSOX, 2026-09-23: /imu/data at 48.9 Hz with a
            // quaternion norm of 0.0000.
            //
            // w = 1 and the rest zero is the honest value: this driver does not
            // know the orientation, and identity is what "no correction" looks
            // like to a consumer. The EKF does not fuse it either way -- with no
            // magnetometer bringup.launch.py clears imu0_config[5].
            //
            // Drivers with real on-chip fusion (BNO085) overwrite all four.
            imu_msg_.orientation.x = 0.0;
            imu_msg_.orientation.y = 0.0;
            imu_msg_.orientation.z = 0.0;
            imu_msg_.orientation.w = 1.0;

            bool sensor_ok = startSensor();
            if(sensor_ok)
                calibrateGyro();

            return sensor_ok;
        }

        // Whether the startup average was taken on a board that was moving.
        // The reading is still published -- refusing to publish would be worse
        // than publishing a known-doubtful bias -- but the caller can say so.
        bool gyroCalSuspect() const { return gyro_cal_suspect_; }

        // ---- when the sample was taken --------------------------------------
        //
        // The publisher stamps every message with one getTime() taken AFTER
        // every sensor has been read, so an IMU sample carried the time the
        // MCU got round to publishing it. That is the jitter this removes:
        // unknown, load-dependent, and invisible -- and the EKF fuses each
        // message at its stamp, so it lands directly in the estimate.
        //
        // ONE source: the chip's own counter, when the driver can read it.
        //
        // There used to be a second -- the DATA_RDY edge, timed in an ISR. The
        // whole interrupt path is gone (2026-09-24), and the reasoning is worth
        // keeping because it applies to any "interrupt that only sets a flag":
        // the ISR could not read the bus (the ESP32 Arduino I2C driver takes a
        // FreeRTOS mutex with portMAX_DELAY, which is illegal from an ISR), so
        // the read happened later in the publish path -- and by then more
        // samples may have arrived, which means the precise edge time belonged
        // to an UNCERTAIN sample. A precise time paired with the wrong sample
        // is not an improvement.
        //
        // The chip's counter has none of that problem: it is latched with the
        // sample, by the part, and comes back in the same read.
        //
        // 0 when the driver has no counter to read. The caller must treat 0 as
        // "no correction", never as "brand new".
        uint32_t sampleAgeUs()
        {
            const uint32_t chip = chipSampleAgeUs();
            if (chip)
                return chip > IMU_SAMPLE_AGE_MAX_US ? IMU_SAMPLE_AGE_MAX_US : chip;
            return 0;
        }

        // Running statistics on the correction, so the bench can see the thing
        // this feature exists to remove rather than take it on trust. The
        // SPREAD is the number that matters: a constant age is a constant
        // offset and harms nothing, while a varying one is the jitter that
        // lands in the EKF's fusion.
        void noteSampleAge(uint32_t age_us)
        {
            if (!age_us)
                return;
            age_n_++;
            age_sum_us_ += age_us;
            if (age_us < age_min_us_) age_min_us_ = age_us;
            if (age_us > age_max_us_) age_max_us_ = age_us;
        }
        uint32_t ageCount() const { return age_n_; }
        uint32_t ageMinUs()  const { return age_n_ ? age_min_us_ : 0; }
        uint32_t ageMaxUs()  const { return age_n_ ? age_max_us_ : 0; }
        uint32_t ageMeanUs() const { return age_n_ ? (uint32_t)(age_sum_us_ / age_n_) : 0; }
        // Where the number came from. One source now -- the chip's own
        // counter -- but the report still names it, because "no correction" and
        // "a correction of zero" are different facts and a line that does not
        // distinguish them is not evidence.
        const char *ageSource()
        {
            return chipSampleAgeUs() ? "chip timestamp" : "none - stamped at publish";
        }

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
