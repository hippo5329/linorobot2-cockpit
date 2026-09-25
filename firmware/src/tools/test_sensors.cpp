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

sensor_msgs__msg__Imu *imu_msg = nullptr;
sensor_msgs__msg__MagneticField *mag_msg = nullptr;
sensor_msgs__msg__BatteryState *battery_msg = nullptr;
sensor_msgs__msg__Range *range_msg = nullptr;

// Pointers, built in setup() from what is on the bus -- not the macro types the
// config header chose. A tool compiled against `IMU` reported the SIM driver's
// zeros on a board with a real MPU6050 answering at 0x68, because the image it
// lives in was built for a robot whose config says `imu: SIM`. One image serves
// every board of a family (AGENTS.md §10), so a diagnostic that takes its sensor
// from the build is diagnosing the build, not the board.
IMUInterface *imu = nullptr;
MAGInterface *mag = nullptr;

static unsigned long lastLogTime = 0;

void setup_()
{
    // Allocated when this tool runs, not statically: every tool's buffers
    // used to sit in .bss at once, paid by whichever app was actually booted.
    if (!imu_msg)     imu_msg     = (sensor_msgs__msg__Imu *)calloc(1, sizeof(*imu_msg));
    if (!mag_msg)     mag_msg     = (sensor_msgs__msg__MagneticField *)calloc(1, sizeof(*mag_msg));
    if (!battery_msg) battery_msg = (sensor_msgs__msg__BatteryState *)calloc(1, sizeof(*battery_msg));
    if (!range_msg)   range_msg   = (sensor_msgs__msg__Range *)calloc(1, sizeof(*range_msg));
    if (!imu_msg || !mag_msg || !battery_msg || !range_msg) {
        Serial.println("[test_sensors] out of memory for the message buffers");
        return;
    }

    delay(2000);
    Serial.println("\n==========================================");
    Serial.println("   Linorobot2 Hardware Sensor Diagnostics ");
    Serial.println("==========================================");
    Serial.println("Scanning I2C bus...");
    i2cScanTable();  // default range from 0x03 to 0x77

    // The same probe and the same adoption rule the robot firmware uses, so this
    // tool and `base` can never disagree about what is fitted.
    initMcuEnv();
    const char *imu_name = envGet("imu", defaultIMUName());
    const char *mag_name = envGet("mag", defaultMAGName());
    i2cProbeSelect(&imu_name, &mag_name);
    imu = createIMU(imu_name);
    mag = createMAG(mag_name);

    Serial.println("Initializing IMU & Magnetometer...");
    bool imu_ok = imu->init();
    if (!imu_ok)
    {
        Serial.println("[-] IMU initialization FAILED!");
    }
    else
    {
        Serial.println("[+] IMU initialized successfully.");
    }

    bool mag_ok = mag->init();
    if (!mag_ok)
    {
        Serial.println("[-] Magnetometer initialization FAILED or not detected.");
    }
    else
    {
        Serial.println("[+] Magnetometer initialized successfully.");
    }

    initBattery();
    initRange();
    // Unconditional, like every other sensor this tool reports on. It used to be
    // #if defined(USE_BMP280), which meant the one application you run TO FIND
    // OUT what is on the bus could only see a barometer the config had already
    // named -- and the config naming it was exactly the assumption under test.
    // initEnv() probes 0x76/0x77 and returns false when nothing answers.
    if (!initEnv())
    {
        Serial.println("[ ] No BMP280/BME280 on the bus.");
    }
    else
    {
        Serial.printf("[+] %s environmental sensor initialized successfully.\n", envHasHumidity() ? "BME280" : "BMP280");
    }

    initBoardLate();
    syslog(LOG_INFO, "%s Ready %lu", __FUNCTION__, millis());
    Serial.println("Starting real-time sensor stream (50 Hz poll, 1 Hz output)...\n");
}

void loop_()
{
    if (!imu_msg) return;   // setup_ could not allocate; nothing to run

    // Poll sensors at 50 Hz (20ms interval) to keep IMU state machines (e.g. BNO085) running smoothly
    delay(20);
    *imu_msg = imu->getData();
    *mag_msg = mag->getData();

#ifdef MAG_BIAS
    const float mag_bias[3] = MAG_BIAS;
    mag_msg->magnetic_field.x -= mag_bias[0];
    mag_msg->magnetic_field.y -= mag_bias[1];
    mag_msg->magnetic_field.z -= mag_bias[2];
#endif

    *battery_msg = getBattery();
    *range_msg = getRange();

    unsigned long currentTime = millis();
    if (currentTime - lastLogTime >= 1000)
    {
        lastLogTime = currentTime;

        // Convert quaternion to Euler angles (Roll, Pitch, Yaw) if orientation is available
        float qx = imu_msg->orientation.x;
        float qy = imu_msg->orientation.y;
        float qz = imu_msg->orientation.z;
        float qw = imu_msg->orientation.w;
        bool has_orientation = (qw != 0.0f || qx != 0.0f || qy != 0.0f || qz != 0.0f);

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

        if (has_orientation)
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
        Serial.printf("  BAT: %5.2fV | RANGE: %5.2fm\n", battery_msg->voltage, range_msg->range);
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

}

}  // namespace test_sensors

extern "C" void test_sensors_setup(void) { test_sensors::setup_(); }
extern "C" void test_sensors_loop(void)  { test_sensors::loop_(); }
