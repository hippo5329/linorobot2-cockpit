// Runtime sensor selection, driven by the env partition.
//
// The IMU and magnetometer a board carries -- and whether they are real at all
// -- used to be a compile-time fact: gen_firmware_header.py emitted
// USE_MPU6050_IMU or USE_SIM_IMU, and imu.h turned that into `#define IMU
// <class>`. One robot, one binary, and a bench board running sim sensors
// needed a different build from the same board with a real IMU soldered on.
//
// Every driver class in default_imu.h and default_mag.h is already compiled
// unconditionally -- the macro only ever chose which one to NAME -- so nothing
// new has to be built to pick between them at run time. These factories do the
// picking, from a string that comes out of the env partition:
//
//     imu=mpu6050     mag=qmc5883l      a real board
//     imu=sim        mag=sim          the same binary on a bare bench
//
// The names are lowercase and match the config/<robot>_config.yaml `sensors.imu`
// and `sensors.mag` values, so the YAML, the env block and the firmware all
// spell a sensor the same way.
//
// An unknown or empty name falls back to the sim driver rather than failing to
// boot: a board that reports simulated data over a working micro-ROS link can
// be diagnosed, one stuck in a fatal init loop cannot.
#ifndef SENSOR_FACTORY_H
#define SENSOR_FACTORY_H

#include "imu_interface.h"
#include "mag_interface.h"

// Both return a heap instance that lives for the life of the program. Call
// after initMcuEnv(), which means from setup() -- not from a static
// initialiser, where the flash partition API is not yet usable.
IMUInterface *createIMU(const char *name);
MAGInterface *createMAG(const char *name);

// The name this firmware was generated for, used when the env says nothing.
// Keeps a board with a blank env behaving exactly as its config says.
const char *defaultIMUName(void);
const char *defaultMAGName(void);

#endif // SENSOR_FACTORY_H
