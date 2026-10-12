// The BNO series: one chip, two roles.
//
// A BNO08x (BNO080/085/086, CEVA's SH-2 protocol) and a BNO055 (Bosch, registers)
// each carry an accelerometer, a gyro AND a magnetometer behind one I2C address.
// This firmware drives an IMU and a magnetometer as two objects (createIMU,
// createMAG), so the chip itself lives here, once, and both drivers talk to it:
// BNO085IMU / BNO055IMU in default_imu.h, BNO085MAG / BNO055MAG in default_mag.h.
//
// Order-independent by construction: start() is idempotent (whichever role starts
// first runs begin(), the other finds it started), and each role only ADDS the
// reports it needs. The magnetometer does not need the IMU driver to be the same
// chip: it polls the chip itself, so a BNO used only as a compass works too.
//
// The rule both follow, the same as every other 9-axis part here: when the field
// is in use, THIS firmware fuses it (ahrs.h), so the robot's own magnetometer
// calibration (mag_bias / mag_soft in the env, from magnetometer_calibration)
// applies. A BNO08x without its field keeps the chip's own fusion (the game
// rotation vector), exactly as before.

#ifndef LINO_BNO_CHIPS_H
#define LINO_BNO_CHIPS_H

#include <Arduino.h>
#include <Wire.h>
#include "syslog.h"

// Note: Sparkfun library redefines I2C_BUFFER_LENGTH, so we undefine it first
#ifdef I2C_BUFFER_LENGTH
#undef I2C_BUFFER_LENGTH
#endif
#include <SparkFun_BNO080_Arduino_Library.h>

// ---------------------------------------------------------------- BNO08x (SH-2)
class Bno08xChip
{
    public:
        static const int kReportMs = 20;    // 50 Hz, the rate every ROS IMU here runs at

        // The roles. Each adds its reports; a chip already running re-arms at once.
        bool wantImu() { want_imu_ = true;  return start(); }
        bool wantMag() { want_mag_ = true;  return start(); }
        bool magWanted() const { return want_mag_; }

        // The game rotation vector is the chip's orientation from gyro + accel only.
        // It is the published orientation exactly when the field is NOT in use.
        bool chipFusesOrientation() const { return !want_mag_; }

        bool start()
        {
            if (started_) {
                enableReports();
                return true;
            }
            // The bus is initBoard()'s (env pins and clock); a second Wire.begin()
            // here reset an RP2's clock to the core default. 0x4B is the SparkFun /
            // Adafruit default; 0x4A with the address pad bridged.
            const uint8_t addrs[2] = {0x4B, 0x4A};
            for (uint8_t i = 0; i < 2 && !started_; i++) {
                if (!ack(addrs[i]))
                    continue;
                if (dev_.begin(addrs[i], Wire)) {
                    addr_ = addrs[i];
                    started_ = true;
                }
            }
            // Neither ACKed -- a chip still booting may not yet. begin() at the
            // default is what this driver always did, and it waits for the chip.
            if (!started_ && dev_.begin(0x4B, Wire)) {
                addr_ = 0x4B;
                started_ = true;
            }
            if (!started_) {
                syslog(LOG_ERR, "%s BNO08x init fail %lu", __FUNCTION__, millis());
                return false;
            }
            syslog(LOG_INFO, "%s BNO08x at 0x%02X %lu", __FUNCTION__, addr_, millis());
            enableReports();
            return true;
        }

        // The SparkFun library parses ONE packet per getReadings() call, and
        // dataAvailable() is nothing but getReadings() != 0 -- so calling both
        // consumes two packets and keeps the second. The state machine this
        // replaced did exactly that, and treated a single empty poll as a lost
        // link (begin(), i.e. a chip reset). A standalone sketch doing the same on
        // an RP2350 at 400 kHz (2026-10-03) counted a fraction of what the chip
        // sent: accel ~38 Hz, gyro ~37 Hz, game rotation ~18 Hz.
        //
        // So: one getReadings() per packet, drain what is queued on every call,
        // keep the last good sample across quiet polls, re-arm the reports when
        // the chip says it reset, and re-run begin() only after a real silence.
        // Both roles call this; a second call in the same cycle finds the queue
        // empty and costs one short read.
        void poll()
        {
            const unsigned long now = millis();
            if (!started_) {
                if (now - lastRecoveryMs_ >= kRearmAfterMs) {
                    lastRecoveryMs_ = now;
                    rebegin();
                }
                return;
            }
            for (uint8_t i = 0; i < kMaxPacketsPerPoll; i++) {
                const uint16_t id = dev_.getReadings();
                if (id == 0) break;
                if (id == SENSOR_REPORTID_ACCELEROMETER || id == SENSOR_REPORTID_GYROSCOPE ||
                    id == SENSOR_REPORTID_GAME_ROTATION_VECTOR) {
                    lastReportMs_ = now;
                    haveImu_ = true;
                } else if (id == SENSOR_REPORTID_MAGNETIC_FIELD) {
                    lastReportMs_ = now;
                    haveMag_ = true;
                }
            }
            // The chip resets itself now and then and comes back with every report
            // off -- and with its calibration settings back at their defaults.
            if (dev_.hasReset()) {
                syslog(LOG_INFO, "%s BNO08x reported a reset, re-arming reports %lu", __FUNCTION__, now);
                enableReports();
                return;
            }
            const unsigned long quiet = now - (lastReportMs_ > lastRecoveryMs_ ? lastReportMs_ : lastRecoveryMs_);
            if (quiet >= kRebeginAfterMs) {
                syslog(LOG_INFO, "%s BNO08x silent %lu ms, begin() again %lu", __FUNCTION__, quiet, now);
                started_ = false;
                // A frozen last sample would look like a healthy, motionless sensor.
                haveImu_ = haveMag_ = false;
                lastRecoveryMs_ = now - kRearmAfterMs;   // retry begin() on this very poll
                poll();
            } else if (quiet >= kRearmAfterMs && now - lastRecoveryMs_ >= kRearmAfterMs) {
                syslog(LOG_INFO, "%s BNO08x silent %lu ms, re-arming reports %lu", __FUNCTION__, quiet, now);
                enableReports();
            }
        }

        bool haveImu() const { return haveImu_; }
        bool haveMag() const { return haveMag_; }
        BNO080 &dev() { return dev_; }

    private:
        BNO080 dev_;
        uint8_t addr_ = 0;
        bool want_imu_ = false;
        bool want_mag_ = false;
        bool started_ = false;
        bool haveImu_ = false;
        bool haveMag_ = false;
        unsigned long lastReportMs_ = 0;
        unsigned long lastRecoveryMs_ = 0;

        static const uint8_t kMaxPacketsPerPoll = 32;
        static const unsigned long kRearmAfterMs = 1000;    // silent this long: re-send the report config
        static const unsigned long kRebeginAfterMs = 3000;  // silent this long: the link is gone, begin() again

        static bool ack(uint8_t addr)
        {
            Wire.beginTransmission(addr);
            return Wire.endTransmission() == 0;
        }

        void rebegin()
        {
            if (addr_ && dev_.begin(addr_, Wire)) {
                started_ = true;
                enableReports();
            }
        }

        void enableReports()
        {
            dev_.hasReset();   // clear the flag the reset we just caused would leave
            if (want_imu_) {
                // Calibrated gyro (rad/s) and accelerometer (m/s^2, gravity included).
                // Interval 0 turns a report off: the IMU usually starts first and
                // turned it on, and nothing reads it once the board fuses the field.
                dev_.enableGameRotationVector(chipFusesOrientation() ? kReportMs : 0);
                dev_.enableGyro(kReportMs);
                dev_.enableAccelerometer(kReportMs);
            }
            if (want_mag_) {
                // The calibrated magnetic field, uT. "Calibrated" by the chip's own
                // dynamic calibration (ME) -- which keeps moving its hard-iron estimate
                // while the robot drives, under a calibration we measured on the host.
                // So the magnetometer's ME calibration is switched OFF: the field the
                // chip reports is then the raw field less a FIXED offset (whatever it
                // last saved), and a fixed offset is exactly what mag_bias absorbs.
                // Accel and gyro keep the dynamic calibration the chip runs by default.
                // ME settings live in RAM, so this goes again after every chip reset.
                dev_.enableMagnetometer(kReportMs);
                for (uint8_t x = 3; x < 12; x++)
                    dev_.shtpData[x] = 0;
                dev_.shtpData[3] = 1;   // P0 accelerometer dynamic calibration: on
                dev_.shtpData[4] = 1;   // P1 gyroscope dynamic calibration: on
                dev_.shtpData[5] = 0;   // P2 magnetometer dynamic calibration: OFF
                dev_.sendCommand(COMMAND_ME_CALIBRATE);
            }
            lastRecoveryMs_ = millis();
            syslog(LOG_INFO, "%s BNO08x reports enabled (imu %d, mag %d) @ %d ms %lu", __FUNCTION__,
                   (int)want_imu_, (int)want_mag_, kReportMs, millis());
        }
};

// One per image: an inline function's static is one object in every unit.
inline Bno08xChip &bno08xChip()
{
    static Bno08xChip chip;
    return chip;
}

// ---------------------------------------------------------------- BNO055 (registers)
//
// NOT RUN ON SILICON: there is no BNO055 on the bench. Written from the Bosch
// datasheet (BST-BNO055-DS000), compiled and unit-tested only.
//
// Run in AMG mode -- accelerometer, magnetometer, gyro, NO on-chip fusion -- so
// the board's AHRS fuses it like any 9-axis part and the robot's own
// magnetometer calibration applies. Units set to m/s^2 and rad/s (UNIT_SEL).
class Bno055Chip
{
    public:
        static const uint8_t REG_CHIP_ID = 0x00;   // 0xA0
        static const uint8_t REG_PAGE_ID = 0x07;
        static const uint8_t REG_ACC_X_LSB = 0x08; // acc 0x08..0x0D, mag 0x0E..0x13, gyr 0x14..0x19
        static const uint8_t REG_MAG_X_LSB = 0x0E;
        static const uint8_t REG_UNIT_SEL = 0x3B;
        static const uint8_t REG_OPR_MODE = 0x3D;
        static const uint8_t REG_PWR_MODE = 0x3E;
        static const uint8_t MODE_CONFIG = 0x00;
        static const uint8_t MODE_AMG = 0x07;
        // Output scales in the units UNIT_SEL selects (datasheet 3.6.4).
        static constexpr double ACC_LSB_PER_MS2 = 100.0;
        static constexpr double GYR_LSB_PER_RPS = 900.0;
        static constexpr double MAG_LSB_PER_UT = 16.0;

        bool start()
        {
            if (started_)
                return true;
            // After power-on the chip needs ~650 ms before it answers.
            for (unsigned long t0 = millis(); !addr_ && millis() - t0 < 800; delay(20)) {
                if (r8(0x28, REG_CHIP_ID) == 0xA0) addr_ = 0x28;
                else if (r8(0x29, REG_CHIP_ID) == 0xA0) addr_ = 0x29;
            }
            if (!addr_) {
                syslog(LOG_ERR, "%s BNO055 init fail %lu", __FUNCTION__, millis());
                return false;
            }
            w8(REG_OPR_MODE, MODE_CONFIG);   // settings only change in CONFIG mode
            delay(25);                       // any -> CONFIG: 19 ms
            w8(REG_PAGE_ID, 0);
            w8(REG_PWR_MODE, 0x00);          // normal power
            delay(10);
            w8(REG_UNIT_SEL, 0x02);          // acc m/s^2 (bit0 0), gyro rad/s (bit1 1), Windows axes
            w8(REG_OPR_MODE, MODE_AMG);
            delay(10);                       // CONFIG -> any: 7 ms
            started_ = true;
            syslog(LOG_INFO, "%s BNO055 at 0x%02X, AMG mode %lu", __FUNCTION__, addr_, millis());
            return true;
        }

        bool read(uint8_t reg, uint8_t *buf, uint8_t n)
        {
            Wire.beginTransmission(addr_); Wire.write(reg);
            if (Wire.endTransmission(false) != 0) return false;
            if (Wire.requestFrom((int)addr_, (int)n) != (int)n) return false;
            for (uint8_t i = 0; i < n; i++) buf[i] = Wire.read();
            return true;
        }

        static int16_t le16(const uint8_t *b) { return (int16_t)((uint16_t)b[1] << 8 | b[0]); }

    private:
        uint8_t addr_ = 0;
        bool started_ = false;

        void w8(uint8_t reg, uint8_t val)
        {
            Wire.beginTransmission(addr_); Wire.write(reg); Wire.write(val); Wire.endTransmission();
        }
        static uint8_t r8(uint8_t addr, uint8_t reg)
        {
            Wire.beginTransmission(addr); Wire.write(reg);
            if (Wire.endTransmission(false) != 0) return 0;
            if (Wire.requestFrom((int)addr, 1) != 1) return 0;
            return Wire.read();
        }
};

inline Bno055Chip &bno055Chip()
{
    static Bno055Chip chip;
    return chip;
}

#endif // LINO_BNO_CHIPS_H
