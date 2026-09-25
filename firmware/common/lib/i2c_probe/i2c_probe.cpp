#include "lino_console.h"
#include <Arduino.h>
#include <Wire.h>
#include <string.h>
#include "i2c_probe.h"
#include "syslog.h"

namespace {

uint8_t readRegister8(uint8_t addr, uint8_t reg)
{
    Wire.beginTransmission(addr);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return 0xFF;
    Wire.requestFrom((int)addr, 1);
    if (Wire.available()) return Wire.read();
    return 0xFF;
}

struct Sink {
    I2CDevice *out;
    int        max;
    int        count;

    void add(uint8_t addr, const char *category, const char *model,
             const char *driver, const char *desc)
    {
        if (count < max)
            out[count++] = { addr, category, model, driver, desc };
    }
};

// Identify one address that has already ACKed. Kept as a single function so the
// order of the checks -- which matters, several chips share an address -- is
// visible in one place.
void identify(Sink &sink, uint8_t addr)
{
    // QMI8658 / LSM6DSOX share 0x6A/0x6B and are told apart by which register
    // answers with which signature, not by the address.
    if (addr == 0x6B || addr == 0x6A) {
        if (readRegister8(addr, 0x00) == 0x05) {
            sink.add(addr, "imu", "QMI8658", "qmi8658",
                     "QMI8658 6-Axis IMU (Acc+Gyr)");
            return;
        }
        if (readRegister8(addr, 0x0F) == 0x6C) {
            sink.add(addr, "imu", "LSM6DSOX", "lsm6dsox",
                     "LSM6DSOX 6-Axis IMU (Acc+Gyr)");
            return;
        }
    }

    // The InvenSense family, all at 0x68/0x69, separated by WHO_AM_I at 0x75 --
    // except the ICM-20948 and the ITG3200, whose id register is 0x00.
    if (addr == 0x68 || addr == 0x69) {
        const uint8_t who = readRegister8(addr, 0x75);
        if (who == 0x68) {
            sink.add(addr, "imu", "MPU6050", "mpu6050",
                     "MPU6050 6-Axis IMU");
            return;
        }
        if (who == 0x71 || who == 0x73) {
            sink.add(addr, "imu", "MPU9250", "mpu9250",
                     "MPU9250 9-Axis IMU (Acc+Gyr+Mag)");
            return;
        }
        if (who == 0x70) {
            // Same driver as the 6050: the difference is not one this image acts on.
            sink.add(addr, "imu", "MPU6500", "mpu6050",
                     "MPU6500 6-Axis IMU");
            return;
        }
        if (who == 0x67) {
            // TDK's newer 6-axis part; the Yahboom YB-EET01 V2.0 carries one
            // where its documentation says QMI8658.
            sink.add(addr, "imu", "ICM42670", "icm42670",
                     "ICM-42670-P 6-Axis IMU (Acc+Gyr)");
            return;
        }
        const uint8_t who0 = readRegister8(addr, 0x00);
        if (who0 == 0xEA) {
            sink.add(addr, "imu", "ICM20948", "icm20948",
                     "ICM-20948 9-Axis IMU (Acc+Gyr+Mag)");
            // One chip, two roles. The ICM-20948's AK09916 magnetometer hangs off
            // the IMU's INTERNAL auxiliary bus, so it never ACKs a scan of the
            // main bus -- probing alone would report a 9-axis part as 6-axis and
            // i2cProbeSelect() would leave mag_name at whatever the config said,
            // usually "sim". /imu/mag would then never publish and nothing would
            // say why. Register the magnetometer here, against the same address:
            // ICM20948MAG reaches it through the IMU exactly as this implies.
            sink.add(addr, "mag", "AK09916", "icm20948",
                     "AK09916 magnetometer (inside the ICM-20948)");
            return;
        }
        if (who0 == 0x68) {
            sink.add(addr, "imu", "GY85", "gy85",
                     "ITG3200 Gyroscope (GY85 component)");
            return;
        }
    }

    if (addr == 0x4A || addr == 0x4B) {
        sink.add(addr, "imu", "BNO085", "bno085",
                 "BNO085/BNO080 9-DOF Robotic IMU");
        return;
    }

    if (addr == 0x28 || addr == 0x29) {
        if (readRegister8(addr, 0x00) == 0xA0) {
            // Recognised, but a BNO055 is not a BNO085 and this image carries no
            // driver for it. Empty driver, so the caller keeps its configured
            // sensor instead of being handed one that cannot work.
            sink.add(addr, "imu", "BNO055", "",
                     "BNO055 9-DOF IMU (no driver in this image)");
            return;
        }
    }

    if (addr == 0x53) {
        if (readRegister8(addr, 0x00) == 0xE5) {
            sink.add(addr, "imu", "GY85", "gy85",
                     "ADXL345 Accelerometer (GY85)");
            return;
        }
    }

    // Magnetometers. The AK09918/AK8963/AK8975 all sit at 0x0C and answer on
    // different id registers; QMC5883L is at 0x0D and HMC5883L at 0x1E.
    if (addr >= 0x0C && addr <= 0x0F) {
        const uint8_t wia2 = readRegister8(addr, 0x01);
        const uint8_t wia  = readRegister8(addr, 0x00);
        if (wia2 == 0x09 || addr == 0x0C) {
            sink.add(addr, "mag", "AK09918", "ak09918",
                     "AK09918 3-Axis Precision Magnetometer");
            return;
        }
        if (wia == 0x48) {
            sink.add(addr, "mag", "AK8963", "ak8963",
                     "AK8963/AK8975 3-Axis Magnetometer");
            return;
        }
        if (addr == 0x0D) {
            sink.add(addr, "mag", "QMC5883L", "qmc5883l",
                     "QMC5883L 3-Axis Compass");
            return;
        }
    }

    if (addr == 0x1E) {
        if (readRegister8(addr, 10) == 'H' && readRegister8(addr, 11) == '4') {
            sink.add(addr, "mag", "HMC5883L", "hmc5883l",
                     "HMC5883L 3-Axis Digital Compass");
            return;
        }
    }

    if (addr >= 0x40 && addr <= 0x45) {
        sink.add(addr, "current", "INA219", "ina219",
                 "INA219 High-Side DC Current & Power Sensor");
        return;
    }

    if (addr == 0x76 || addr == 0x77) {
        const uint8_t id = readRegister8(addr, 0xD0);
        if (id == 0x58 || id == 0x60) {
            sink.add(addr, "env", "BMP280", "bmp280",
                     "BMP280/BME280 Environmental Barometer");
            return;
        }
    }

    // It answered and we do not know what it is. That is worth printing: a bus
    // with an unexpected device on it is a different problem from an empty one
    // -- and the three registers most parts keep their identity in are worth
    // printing with it, so the operator can name the chip from the log rather
    // than from a second tool. (The Yahboom YB-EET01's IMU answered at 0x68
    // with none of the InvenSense WHO_AM_I values; this line is how it gets
    // identified.)
    static char unknown_desc[4][48];
    static int  unknown_n = 0;
    const char *desc = "unidentified device";
    if (unknown_n < 4) {
        snprintf(unknown_desc[unknown_n], sizeof(unknown_desc[0]),
                 "unidentified: reg00=%02X reg0F=%02X reg75=%02X",
                 readRegister8(addr, 0x00), readRegister8(addr, 0x0F), readRegister8(addr, 0x75));
        desc = unknown_desc[unknown_n++];
    }
    sink.add(addr, "unknown", "?", "", desc);
}

}  // namespace

int i2cProbe(I2CDevice *out, int max_devices)
{
    Sink sink{out, max_devices, 0};
    // 0x08..0x77: the 7-bit addresses a device may own. 0x00-0x07 and
    // 0x78-0x7F are reserved by the I2C specification (general call, start
    // byte, CBUS, 10-bit addressing, device ID), and 0x7E in particular is
    // the I3C broadcast address: an I3C-capable part on the bus -- the
    // ICM-42670-P is one, with I3C on by default -- obeys a 0x7E+W and did
    // not answer the I2C transaction that followed.
    for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0)
            identify(sink, addr);
    }
    i2cProbeFoldComposites(out, sink.count);
    return sink.count;
}

// A 9-axis part is ONE chip, and the scan cannot see that on its own.
//
// The ICM-20948 carries an AK09918 on its internal auxiliary bus. The comment
// on its branch above says that bus never ACKs a main-bus scan -- true from a
// power-on reset, and false for the rest of the board's life: the first thing
// ICM20948IMU::startSensor() does is set INT_PIN_CFG BYPASS_EN, which wires
// the AK09918 onto the main bus at 0x0C, and it stays there until something
// clears the bit. So on any board that has run this firmware once, a rescan
// sees 0x0C answer and identifies it as a STANDALONE AK09918.
//
// That is what the bench Pico 2 reported on 2026-09-23:
//
//   [0x0C] mag AK09918  driver ak09918      <- the ICM-20948's own, unlabelled
//   [0x68] imu ICM20948 driver icm20948
//   [0x68] mag AK09916  driver icm20948
//
// and i2cProbeFind() takes the FIRST match, so the magnetometer was selected
// as a separate chip at the lower address. It happened to work -- both drivers
// talk to 0x0C through the same bypass -- but only because IMU init runs
// before MAG init and turns the bypass on. Reorder those two, or fit the part
// on a board whose IMU is disabled, and the magnetometer goes silent with the
// bus still ACKing at 0x0C and nothing to say why.
//
// So fold it: when the composite part is present, the satellite address is
// ITS magnetometer, named for the chip it lives in.
void i2cProbeFoldComposites(I2CDevice *devs, int count)
{
    bool has_icm20948 = false;
    for (int i = 0; i < count; i++)
        if (strcmp(devs[i].category, "imu") == 0 && strcmp(devs[i].driver, "icm20948") == 0)
            has_icm20948 = true;
    if (!has_icm20948)
        return;
    for (int i = 0; i < count; i++) {
        if (strcmp(devs[i].category, "mag") != 0 || devs[i].addr != 0x0C)
            continue;
        if (strcmp(devs[i].driver, "icm20948") == 0)
            continue;                       // already the composite's entry
        devs[i].model  = "AK09918";
        devs[i].driver = "icm20948";
        devs[i].desc   = "AK09918 magnetometer (inside the ICM-20948, via bypass at 0x0C)";
    }
}

const I2CDevice *i2cProbeFind(const I2CDevice *devs, int count, const char *category)
{
    for (int i = 0; i < count; i++)
        if (strcmp(devs[i].category, category) == 0)
            return &devs[i];
    return NULL;
}

void i2cProbePrint(const I2CDevice *devs, int count)
{
    if (count == 0) {
        Serial.println("[i2c] no device answered on the bus");
        return;
    }
    for (int i = 0; i < count; i++)
        Serial.printf("[i2c] 0x%02X  %-8s %-10s %s\n",
                      devs[i].addr, devs[i].category, devs[i].model, devs[i].desc);
}


// Take the detected sensor's driver when there is one, and say so. Three cases,
// and the two that are not a silent success are the ones worth printing:
// nothing answered (keep the configured name and let init() report it), or
// something answered that this image cannot drive -- a BNO055, say, where the
// table leaves `driver` empty rather than handing back a plausible wrong one.
static const char *adopt(const char *what, const I2CDevice *dev, const char *configured)
{
    if (!dev)
        return configured;
    if (!dev->driver || !*dev->driver) {
        Serial.printf("[i2c] %s: found %s at 0x%02X, but this image has no driver "
                      "for it - keeping %s\n", what, dev->model, dev->addr, configured);
        return configured;
    }
    if (!configured || strcasecmp(dev->driver, configured) != 0) {
        Serial.printf("[i2c] %s: the bus says %s at 0x%02X, the config says %s - "
                      "using %s\n", what, dev->model, dev->addr,
                      configured && *configured ? configured : "nothing", dev->driver);
        syslog(LOG_WARNING, "%s %s detected %s at 0x%02X overriding configured %s %lu",
               __FUNCTION__, what, dev->model, dev->addr,
               configured && *configured ? configured : "none", millis());
    } else {
        Serial.printf("[i2c] %s: %s at 0x%02X, as configured\n",
                      what, dev->model, dev->addr);
    }
    return dev->driver;
}

void i2cProbeSelect(const char **imu_name, const char **mag_name)
{
    I2CDevice devs[I2C_PROBE_MAX];
    const int n = i2cProbe(devs, I2C_PROBE_MAX);
    Serial.printf("[i2c] %d device(s) on the bus\n", n);
    i2cProbePrint(devs, n);

    if (imu_name)
        *imu_name = adopt("IMU", i2cProbeFind(devs, n, "imu"), *imu_name);
    if (mag_name)
        *mag_name = adopt("MAG", i2cProbeFind(devs, n, "mag"), *mag_name);

    const I2CDevice *cur = i2cProbeFind(devs, n, "current");
    if (cur)
        syslog(LOG_INFO, "%s %s current monitor at 0x%02X %lu",
               __FUNCTION__, cur->model, cur->addr, millis());
}


// The classic i2cdetect grid, 0x03..0x77, for the diagnostic tools. Our own:
// it is one beginTransmission per address, and the library it replaced was a
// dependency for exactly this.
void i2cScanTable(void)
{
    Serial.println("     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f");
    for (int row = 0; row < 0x80; row += 16) {
        Serial.printf("%02x:", row);
        for (int col = 0; col < 16; col++) {
            const int addr = row + col;
            if (addr < 0x03 || addr > 0x77) { Serial.print("   "); continue; }
            Wire.beginTransmission((uint8_t)addr);
            const bool ack = Wire.endTransmission() == 0;
            if (ack) Serial.printf(" %02x", addr); else Serial.print(" --");
        }
        Serial.println();
    }
}
