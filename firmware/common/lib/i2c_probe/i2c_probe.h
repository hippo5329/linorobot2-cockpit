// What is actually on the I2C bus, asked once and answered the same way for
// every application in the image.
//
// This table used to live inside firmware/src/tools/i2c_detect.cpp, where only
// the diagnostic could reach it. That put the robot firmware in the odd position
// of being the one program on the board that could NOT tell what was wired to
// it: `base` took the IMU and magnetometer names from the env partition, and if
// those named the wrong chip it went into the fatal "IMU init failed" flash-code
// loop -- while a tool sitting in the same image, two flash pages away, could
// have read the right answer off the bus in 30 ms.
//
// So the identification moved here and both callers share it:
//
//   i2c_detect  prints the table and the [I2C_JSON] line the Web UI parses.
//   base        probes at boot on a real robot, prints what it found, and hands
//               the driver names to the sensor factories (main.cpp).
//
// `driver` is the name sensor_factory.cpp knows, which is NOT always derivable
// from `model`: MPU6500 is driven by the MPU6050 driver, an ADXL345 or ITG3200
// is one half of a GY85, and a BNO055 is a different chip from the BNO085 this
// image drives -- so its driver field is empty rather than a plausible guess.
// An empty driver means "recognised, but this image cannot drive it", which is
// a different and much more useful answer than "not found".
#ifndef I2C_PROBE_H
#define I2C_PROBE_H

#include <Arduino.h>

struct I2CDevice {
    uint8_t     addr;
    const char *category;  // "imu", "mag", "current", "env", "unknown"
    const char *model;     // "QMI8658", "AK09918", ... "?" when unidentified
    const char *driver;    // sensor_factory name ("qmi8658"), "" if none
    const char *macro;     // legacy USE_* macro, for the Web UI's config studio
    const char *desc;
};

// Sixteen is what the original tool allowed and no bus in this project comes
// close; a 17th device is dropped rather than overflowing.
#define I2C_PROBE_MAX 16

// Scan the assignable addresses 0x08..0x77 and identify what answers. Wire must already be begun
// (initBoard(), or a tool's explicit pin override). Returns the count written.
// Every address that ACKs appears in the result, identified or not -- an
// unexpected device is exactly the thing worth seeing.
int i2cProbe(I2CDevice *out, int max_devices);

// The first device of a category, or NULL. Categories are the strings above.
const I2CDevice *i2cProbeFind(const I2CDevice *devs, int count, const char *category);

// One line per device on Serial, in the same shape the diagnostic prints.
void i2cProbePrint(const I2CDevice *devs, int count);

// Scan, print what answered, and replace *imu_name / *mag_name with the drivers
// for what is actually on the bus. Names not matched by anything on the bus are
// left alone, so a board whose sensor did not answer keeps its configured name
// and fails in init() with a message, rather than silently becoming something
// else. Both callers -- the robot firmware and test_sensors -- go through this,
// so a tool cannot disagree with the firmware it is diagnosing about what is
// fitted. The pointers may be NULL.
void i2cProbeSelect(const char **imu_name, const char **mag_name);

#endif // I2C_PROBE_H
