// Copyright (c) 2021 Juan Miguel Jimeno
// Copyright (c) 2023 Thomas Chou
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

#ifndef MAG_INTERFACE
#define MAG_INTERFACE

#include <sensor_msgs/msg/magnetic_field.h>
#include "mcu_env.h"

#ifndef MAG_COV
#define MAG_COV { 0.00001, 0.00001, 0.00001 }
#endif

class MAGInterface
{
    protected:
        // Value-initialised; see imu_interface.h for why.
        sensor_msgs__msg__MagneticField mag_msg_{};
        // See imu_interface.h: the env wins, the macro is the fallback.
        float mag_cov[3] = MAG_COV;

    public:
        MAGInterface()
        {
            mag_msg_.header.frame_id = micro_ros_string_utilities_set(mag_msg_.header.frame_id, "imu_link");
            envFloatVec("mag_cov", mag_cov, 3);
        }

        // The robot's namespace, applied after the env is readable -- see
        // IMUInterface::applyEnvFrames().
        void applyEnvFrames()
        {
            mag_msg_.header.frame_id =
                micro_ros_string_utilities_set(mag_msg_.header.frame_id, envPrefixed("imu_link"));
        }

        virtual geometry_msgs__msg__Vector3 readMagnetometer() = 0;
        virtual bool startSensor() = 0;

        bool init()
        {
            bool sensor_ok = startSensor();
            return sensor_ok;
        }

        sensor_msgs__msg__MagneticField getData()
        {
            mag_msg_.magnetic_field = readMagnetometer();
            mag_msg_.magnetic_field_covariance[0] = mag_cov[0];
            mag_msg_.magnetic_field_covariance[4] = mag_cov[1];
            mag_msg_.magnetic_field_covariance[8] = mag_cov[2];

            return mag_msg_;
        }
};

#endif
