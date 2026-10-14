#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <micro_ros_utilities/string_utilities.h>
#include "config.h"
#include "tool_hw.h"
#include "i2c_probe.h"
#include "sensor_factory.h"
#include "battery.h"
#include "range.h"
#include "env.h"
#include "lidar.h"
#include "board_init.h"
#include "mcu_env.h"
#include "syslog.h"
#include "tools.h"

namespace tool_hw {

IMUInterface *imu = nullptr;
bool imu_ok = false;
MAGInterface *mag = nullptr;
bool mag_ok = false;
const char *imu_name = "none";
const char *mag_name = "none";
bool env_ok = false;
unsigned total_motors = 4;

sensor_msgs__msg__Imu *imu_msg = nullptr;
sensor_msgs__msg__MagneticField *mag_msg = nullptr;
sensor_msgs__msg__BatteryState *battery_msg = nullptr;
sensor_msgs__msg__Range *range_msg = nullptr;

void report(void)
{
    if (imu)
        Serial.println(imu_ok ? "[+] IMU initialized successfully." : "[-] IMU initialization FAILED!");
    else
        Serial.println("[ ] No IMU.");
    if (mag)
        Serial.println(mag_ok ? "[+] Magnetometer initialized successfully."
                              : "[-] Magnetometer initialization FAILED or not detected.");
    else
        Serial.println("[ ] No magnetometer.");
    if (env_ok)
        Serial.printf("[+] %s environmental sensor initialized successfully.\n",
                      envHasHumidity() ? "BME280" : "BMP280");
    else
        Serial.println("[ ] No BMP280/BME280 on the bus.");
}

void init(void)
{
    // Allocated here, not statically: base never runs this.
    imu_msg     = (sensor_msgs__msg__Imu *)calloc(1, sizeof(*imu_msg));
    mag_msg     = (sensor_msgs__msg__MagneticField *)calloc(1, sizeof(*mag_msg));
    battery_msg = (sensor_msgs__msg__BatteryState *)calloc(1, sizeof(*battery_msg));
    range_msg   = (sensor_msgs__msg__Range *)calloc(1, sizeof(*range_msg));
    if (!imu_msg || !mag_msg || !battery_msg || !range_msg)
        Serial.println("[tool] out of memory for the sensor messages");

    if (kinematics && kinematics->base_platform_ == Kinematics::DIFFERENTIAL_DRIVE)
        total_motors = 2;

    // The LiDAR's UDP forwarder, as in base: the robot computer can watch the room
    // while a command drives. With no real LiDAR in the env it stays off.
    initLidar();

    // The same probe and adoption rule as base, so the tool and the robot firmware
    // never disagree about what is fitted. Real chips only (envRealOnly). A board whose
    // env names no I2C bus is not scanned: the core's default pins may be a motor's.
    if (toolWalk()) {
        // The Sim MCU's walkthrough: its simulated chips, as base builds them.
        Serial.println("[tool] walkthrough mode: the Sim MCU's simulated parts stand in for chips");
        imu_name = envGet("imu", "sim");
        mag_name = envGet("mag", "sim");
        if ((imu = createIMU(imu_name)))
            imu_ok = imu->init();
        if ((mag = createMAG(mag_name)))
            mag_ok = mag->init();
    } else if (toolI2cUnassigned()) {
        Serial.println("[tool] no I2C bus in the env: no IMU, magnetometer or barometer");
    } else {
        Serial.println("Scanning I2C bus...");
        i2cScanTable();
        imu_name = envGet("imu", defaultIMUName());
        mag_name = envGet("mag", defaultMAGName());
        i2cProbeSelect(&imu_name, &mag_name);
        if ((imu = createIMU(imu_name)))
            imu_ok = imu->init();
        if ((mag = createMAG(mag_name)))
            mag_ok = mag->init();
    }
    initBattery();
    initRange();
    env_ok = !toolI2cUnassigned() && initEnv();
    initBoardLate();
    report();
    syslog(LOG_INFO, "%s tool ready %lu", __FUNCTION__, millis());
}

}  // namespace tool_hw
