#include <Arduino.h>
#include <string.h>
// The interfaces build their header frame_id with this, and main.cpp happened to
// include it first. A translation unit that pulls the interfaces in on its own
// has to say so itself.
#include <micro_ros_utilities/string_utilities.h>
#include "config.h"
#include "default_imu.h"
#include "default_mag.h"
#include "sensor_factory.h"

// The table is the single place a sensor name is tied to a class. Adding a
// driver means adding its row here and nothing else -- the generator, the env
// tool and the YAML all carry the same lowercase string.
struct IMUEntry { const char *name; IMUInterface *(*make)(void); };
struct MAGEntry { const char *name; MAGInterface *(*make)(void); };

template <typename T> static IMUInterface *makeIMU(void) { return new T(); }
template <typename T> static MAGInterface *makeMAG(void) { return new T(); }

static const IMUEntry IMU_TABLE[] = {
    {"fake",     makeIMU<FakeIMU>},
    {"gy85",     makeIMU<GY85IMU>},
    {"mpu6050",  makeIMU<MPU6050IMU>},
    {"mpu9150",  makeIMU<MPU6050IMU>},   // same silicon as far as this driver is concerned
    {"mpu9250",  makeIMU<MPU9250IMU>},
    {"qmi8658",  makeIMU<QMI8658IMU>},
    {"lsm6dsox", makeIMU<LSM6DSOXIMU>},
    {"icm20948", makeIMU<ICM20948IMU>},
    {"bno085",   makeIMU<BNO085IMU>},
};

static const MAGEntry MAG_TABLE[] = {
    {"fake",     makeMAG<FakeMAG>},
    {"hmc5883l", makeMAG<HMC5883LMAG>},
    {"ak8963",   makeMAG<AK8963MAG>},
    {"ak8975",   makeMAG<AK8975MAG>},
    {"ak09918",  makeMAG<AK09918MAG>},
    {"qmc5883l", makeMAG<QMC5883LMAG>},
    {"icm20948", makeMAG<ICM20948MAG>},
};

const char *defaultIMUName(void)
{
#ifdef IMU_DEFAULT_NAME
    return IMU_DEFAULT_NAME;
#else
    return "fake";
#endif
}

const char *defaultMAGName(void)
{
#ifdef MAG_DEFAULT_NAME
    return MAG_DEFAULT_NAME;
#else
    return "fake";
#endif
}

IMUInterface *createIMU(const char *name)
{
    if (name && *name) {
        for (unsigned i = 0; i < sizeof(IMU_TABLE) / sizeof(IMU_TABLE[0]); i++) {
            if (strcasecmp(name, IMU_TABLE[i].name) == 0) {
                Serial.printf("[sensors] IMU: %s\n", IMU_TABLE[i].name);
                return IMU_TABLE[i].make();
            }
        }
        Serial.printf("[sensors] unknown IMU '%s' — falling back to fake\n", name);
    }
    return new FakeIMU();
}

MAGInterface *createMAG(const char *name)
{
    if (name && *name) {
        for (unsigned i = 0; i < sizeof(MAG_TABLE) / sizeof(MAG_TABLE[0]); i++) {
            if (strcasecmp(name, MAG_TABLE[i].name) == 0) {
                Serial.printf("[sensors] MAG: %s\n", MAG_TABLE[i].name);
                return MAG_TABLE[i].make();
            }
        }
        Serial.printf("[sensors] unknown MAG '%s' — falling back to fake\n", name);
    }
    return new FakeMAG();
}
