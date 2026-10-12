// Copyright (c) 2026 Thomas Chou
// Copyright (c) 2026 Paul Bouchier
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
#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <micro_ros_utilities/string_utilities.h>
#include <stdio.h>
#include <math.h>
#include "i2c_probe.h"
#include "tool_hw.h"

#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/magnetic_field.h>
#include <sensor_msgs/msg/battery_state.h>
#include <sensor_msgs/msg/range.h>

#include "config.h"
#include "tools.h"
#include "syslog.h"
#include "imu.h"
#include "mag.h"
#include "battery.h"
#include "range.h"
#include "env.h"
#include "wifis.h"
#include "ota.h"
#include "board_init.h"
#include "sensor_factory.h"
#include "mcu_env.h"
#include "i2c_probe.h"

#ifndef BAUDRATE
#define BAUDRATE 921600
#endif

#ifndef RAD_TO_DEG
#define RAD_TO_DEG (180.0f / M_PI)
#endif

// Sensor diagnostics. Serial, initBoard(), the radio and OTA are the
// dispatcher's job (see tools.h); everything below is only this tool's work.
namespace test_sensors {

// The tool's sensors and buffers, built once at boot (tool_hw.cpp) -- by the same probe
// and adoption rule as base, so this command and the robot firmware never disagree
// about what is fitted. Real chips only: `none`, `sim` or nothing on the bus is none.
using tool_hw::imu;
using tool_hw::mag;
using tool_hw::imu_msg;
using tool_hw::mag_msg;
using tool_hw::battery_msg;
using tool_hw::range_msg;

static unsigned long lastLogTime = 0;
// What was found, again every 5 s: a host that opened the port after the tool started (an
// RP2's USB drops what nobody reads) still sees the "[+] IMU initialized" lines.
static unsigned long lastReport = 0;

// `sensors`: the header and what was found, then the stream until stop.
void begin()
{
    Serial.println("\n==========================================");
    Serial.println("   Linorobot2 Hardware Sensor Diagnostics ");
    Serial.println("==========================================");
    tool_hw::report();
    Serial.println("Starting real-time sensor stream (50 Hz poll, 1 Hz output)...\n");
    lastLogTime = 0;
    lastReport = millis();
}

bool step()
{
    if (!imu_msg) return false;   // tool_hw could not allocate; nothing to run

    // Poll sensors at 50 Hz (20ms interval) to keep IMU state machines (e.g. BNO085) running smoothly
    delay(20);
    if (imu) *imu_msg = imu->getData();   // otherwise the zeroed message: no IMU
    if (mag) *mag_msg = mag->getData();

#ifdef MAG_BIAS
    const float mag_bias[3] = MAG_BIAS;
    mag_msg->magnetic_field.x -= mag_bias[0];
    mag_msg->magnetic_field.y -= mag_bias[1];
    mag_msg->magnetic_field.z -= mag_bias[2];
#endif

    *battery_msg = getBattery();
    *range_msg = getRange();

    unsigned long currentTime = millis();
    if (currentTime - lastReport >= 5000)
    {
        lastReport = currentTime;
        tool_hw::report();
    }
    if (currentTime - lastLogTime >= 1000)
    {
        lastLogTime = currentTime;

        // Convert quaternion to Euler angles (Roll, Pitch, Yaw) if orientation is available
        float qx = imu_msg->orientation.x;
        float qy = imu_msg->orientation.y;
        float qz = imu_msg->orientation.z;
        float qw = imu_msg->orientation.w;
        // Only a chip that fuses on its own (a BNO085) reports a rotation here. The others
        // leave the identity -- w=1 -- and printed "RPY 0.0 0.0 0.0" in place of their
        // magnetometer (a QMI8658 robot, 2026-10-12).
        bool has_orientation = (qx != 0.0f || qy != 0.0f || qz != 0.0f);
        (void)qw;

        float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
        if (has_orientation)
        {
            float sinr_cosp = 2.0f * (qw * qx + qy * qz);
            float cosr_cosp = 1.0f - 2.0f * (qx * qx + qy * qy);
            roll = atan2(sinr_cosp, cosr_cosp) * RAD_TO_DEG;

            float sinp = 2.0f * (qw * qy - qz * qx);
            if (fabs(sinp) >= 1.0f)
                pitch = copysign(90.0f, sinp);
            else
                pitch = asin(sinp) * RAD_TO_DEG;

            float siny_cosp = 2.0f * (qw * qz + qx * qy);
            float cosy_cosp = 1.0f - 2.0f * (qy * qy + qz * qz);
            yaw = atan2(siny_cosp, cosy_cosp) * RAD_TO_DEG;
        }

        if (!imu)
        {
            // No IMU: say so, not 0.00 everywhere (zeros read as a broken chip).
            if (mag)
                Serial.printf("ACC none (no IMU) | MAG [uT] X:%5.2f Y:%5.2f Z:%5.2f\n",
                    mag_msg->magnetic_field.x * 1000000.0f, mag_msg->magnetic_field.y * 1000000.0f,
                    mag_msg->magnetic_field.z * 1000000.0f);
            else
                Serial.println("ACC none (no IMU) | MAG none (no magnetometer)");
        }
        else if (has_orientation)
        {
            Serial.printf("ACC [m/s^2] X:%5.2f Y:%5.2f Z:%5.2f | GYR [rad/s] X:%5.2f Y:%5.2f Z:%5.2f | RPY [deg] R:%5.1f P:%5.1f Y:%5.1f\n",
                imu_msg->linear_acceleration.x, imu_msg->linear_acceleration.y, imu_msg->linear_acceleration.z,
                imu_msg->angular_velocity.x, imu_msg->angular_velocity.y, imu_msg->angular_velocity.z,
                roll, pitch, yaw
            );
        }
        else
        {
            Serial.printf("ACC [m/s^2] X:%5.2f Y:%5.2f Z:%5.2f | GYR [rad/s] X:%5.2f Y:%5.2f Z:%5.2f | MAG [uT] X:%5.2f Y:%5.2f Z:%5.2f\n",
                imu_msg->linear_acceleration.x, imu_msg->linear_acceleration.y, imu_msg->linear_acceleration.z,
                imu_msg->angular_velocity.x, imu_msg->angular_velocity.y, imu_msg->angular_velocity.z,
                mag_msg->magnetic_field.x * 1000000.0f, mag_msg->magnetic_field.y * 1000000.0f,
                mag_msg->magnetic_field.z * 1000000.0f
            );
        }

        // Unconditional: USE_INA219 was in this condition, and the INA219 is
        // compiled in unconditionally and found by probing 0x40-0x45 -- so a
        // board whose current monitor was detected rather than declared printed
        // no battery line at all, in the tool you run to check the battery.
        // BATTERY_PIN and TRIG_PIN are still genuine compile-time facts; a board
        // without them reads 0 here, which is what a diagnostic should show.
        // A sonar that is not fitted reads none: getRange() keeps its seeded
        // max_range (4.84 m on a robot with no sonar looked like a clear reading).
        if (batteryPresent()) Serial.printf("  BAT: %5.2fV", battery_msg->voltage);
        else                  Serial.print("  BAT: none (no battery sense)");
        if (rangePresent()) Serial.printf(" | RANGE: %5.2fm\n", range_msg->range);
        else                Serial.print(" | RANGE: none\n");
        // One tagged line for a program (the Web UI's readouts): never parse the prose.
        Serial.print("[TOOL_JSON] {\"cmd\":\"sensors\"");
        if (imu)
            Serial.printf(",\"acc\":[%.2f,%.2f,%.2f],\"gyr\":[%.3f,%.3f,%.3f]",
                imu_msg->linear_acceleration.x, imu_msg->linear_acceleration.y, imu_msg->linear_acceleration.z,
                imu_msg->angular_velocity.x, imu_msg->angular_velocity.y, imu_msg->angular_velocity.z);
        if (has_orientation)
            Serial.printf(",\"rpy\":[%.1f,%.1f,%.1f]", roll, pitch, yaw);
        if (mag)
            Serial.printf(",\"mag_ut\":[%.2f,%.2f,%.2f]", mag_msg->magnetic_field.x * 1e6f,
                mag_msg->magnetic_field.y * 1e6f, mag_msg->magnetic_field.z * 1e6f);
        if (batteryPresent()) Serial.printf(",\"bat\":%.2f", battery_msg->voltage);
        if (rangePresent())   Serial.printf(",\"range\":%.2f", range_msg->range);
        Serial.print("}\n");
        // envOk() is false when nothing answered the probe, so this prints only
        // on a board that actually has the chip -- no macro needed.
        if (envOk())
        {
            EnvData env = readEnv();
            if (env.valid)
            {
                Serial.printf("  ENV: %7.2f Pa | %5.2f C%s\n",
                    env.pressure, env.temperature,
                    envHasHumidity() ? " (BME280)" : " (BMP280)");
            }
        }

        syslog(LOG_INFO, "ACC %5.2f %5.2f %5.2f GYR %5.2f %5.2f %5.2f MAG %5.2f %5.2f %5.2f BAT %5.2fV",
            imu_msg->linear_acceleration.x, imu_msg->linear_acceleration.y, imu_msg->linear_acceleration.z,
            imu_msg->angular_velocity.x, imu_msg->angular_velocity.y, imu_msg->angular_velocity.z,
            mag_msg->magnetic_field.x * 1000000.0f, mag_msg->magnetic_field.y * 1000000.0f,
            mag_msg->magnetic_field.z * 1000000.0f,
            battery_msg->voltage
        );
    }
    return true;
}

}  // namespace test_sensors

extern "C" void test_sensors_begin(void) { test_sensors::begin(); }
extern "C" bool test_sensors_step(void)  { return test_sensors::step(); }
