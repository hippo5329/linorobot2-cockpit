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

#ifndef DEFAULT_IMU
#define DEFAULT_IMU

//include IMU base interface
#include "imu_interface.h"

//include sensor API headers
#include "I2Cdev.h"
#include "ADXL345.h"
#include "ITG3200.h"
#include "HMC5883L.h"
#include "MPU6050.h"
#include "MPU9250.h"

#include "syslog.h"

class GY85IMU: public IMUInterface 
{
    private:
        //constants specific to the sensor
        const float accel_scale_ = 1 / 256.0;
        const float gyro_scale_ = 1 / 14.375;

        // driver objects to be used
        ADXL345 accelerometer_;
        ITG3200 gyroscope_;

        // returned vector for sensor reading
        geometry_msgs__msg__Vector3 accel_;
        geometry_msgs__msg__Vector3 gyro_;

    public:
        GY85IMU()
        {
            // accel_cov_ = 0.001; //you can overwrite the convariance values here
            // gyro_cov_ = 0.001; //you can overwrite the convariance values here
        }

        bool startSensor() override
        {
            // here you can override startSensor() function and use the sensor's driver API
            // to initialize and test the sensor's connection during boot time
            Wire.begin();
            bool ret;
            accelerometer_.initialize();
            ret = accelerometer_.testConnection();
            if(!ret)
                return false;

            gyroscope_.initialize();
            ret = gyroscope_.testConnection();
            if(!ret)
                return false;

            return true;
        }

        geometry_msgs__msg__Vector3 readAccelerometer() override
        {
            // here you can override readAccelerometer function and use the sensor's driver API
            // to grab the data from accelerometer and return as a Vector3 object
            int16_t ax, ay, az;
            
            accelerometer_.getAcceleration(&ax, &ay, &az);

            accel_.x = ax * (double) accel_scale_ * g_to_accel_;
            accel_.y = ay * (double) accel_scale_ * g_to_accel_;
            accel_.z = az * (double) accel_scale_ * g_to_accel_;

            return accel_;
        }

        geometry_msgs__msg__Vector3 readGyroscope() override
        {
            // here you can override readAccelerometer function and use the sensor's driver API
            // to grab the data from gyroscope and return as a Vector3 object
            int16_t gx, gy, gz;

            gyroscope_.getRotation(&gx, &gy, &gz);

            gyro_.x = gx * (double) gyro_scale_ * DEG_TO_RAD;
            gyro_.y = gy * (double) gyro_scale_ * DEG_TO_RAD;
            gyro_.z = gz * (double) gyro_scale_ * DEG_TO_RAD;

            return gyro_;
        }
};


class MPU6050IMU: public IMUInterface 
{
    private:
        const float accel_scale_ = 1 / 16384.0;
        const float gyro_scale_ = 1 / 131.0;

        MPU6050 accelgyro_;

        geometry_msgs__msg__Vector3 accel_;
        geometry_msgs__msg__Vector3 gyro_;

    public:
        MPU6050IMU()
        {
        }

        bool startSensor() override
        {
            Wire.begin();
            bool ret;
            accelgyro_.initialize();
            ret = accelgyro_.testConnection();
            if(!ret)
                return false;

            // Accelerometer only. The gyro's bias is removed once, in
            // IMUInterface::calibrateGyro() -- calling the library's
            // CalibrateGyro() here as well made this the one driver that
            // corrected the same error twice, and it is the slow half of the
            // two (15 sampling loops against the chip).
            accelgyro_.CalibrateAccel();
            return true;
        }

        // The MPU6050's INT pin, as a data-ready strobe: active high, push-pull,
        // a 50 us pulse per new sample, cleared by any register read so a slow
        // reader cannot wedge it high. Register names are i2cdevlib's.
        bool enableDataReadyInterrupt() override
        {
            accelgyro_.setInterruptMode(0);
            accelgyro_.setInterruptDrive(0);
            accelgyro_.setInterruptLatch(0);
            accelgyro_.setInterruptLatchClear(1);
            accelgyro_.setIntDataReadyEnabled(true);
            return true;
        }

        geometry_msgs__msg__Vector3 readAccelerometer() override
        {
            int16_t ax, ay, az;
            
            accelgyro_.getAcceleration(&ax, &ay, &az);

            accel_.x = ax * (double) accel_scale_ * g_to_accel_;
            accel_.y = ay * (double) accel_scale_ * g_to_accel_;
            accel_.z = az * (double) accel_scale_ * g_to_accel_;

            return accel_;
        }

        geometry_msgs__msg__Vector3 readGyroscope() override
        {
            int16_t gx, gy, gz;

            accelgyro_.getRotation(&gx, &gy, &gz);

            gyro_.x = gx * (double) gyro_scale_ * DEG_TO_RAD;
            gyro_.y = gy * (double) gyro_scale_ * DEG_TO_RAD;
            gyro_.z = gz * (double) gyro_scale_ * DEG_TO_RAD;

            return gyro_;
        }
};

class MPU9250IMU: public IMUInterface 
{
    private:
        const float accel_scale_ = 1 / 16384.0;
        const float gyro_scale_ = 1 / 131.0;

        MPU9250 accelgyro_;

        geometry_msgs__msg__Vector3 accel_;
        geometry_msgs__msg__Vector3 gyro_;

    public:
        MPU9250IMU()
        {
        }

        bool startSensor() override
        {
            Wire.begin();
            bool ret;
            accelgyro_.initialize();
            ret = accelgyro_.testConnection();
            if(!ret)
                return false;

            return true;
        }

        geometry_msgs__msg__Vector3 readAccelerometer() override
        {
            int16_t ax, ay, az;
            
            accelgyro_.getAcceleration(&ax, &ay, &az);

            accel_.x = ax * (double) accel_scale_ * g_to_accel_;
            accel_.y = ay * (double) accel_scale_ * g_to_accel_;
            accel_.z = az * (double) accel_scale_ * g_to_accel_;

            return accel_;
        }

        geometry_msgs__msg__Vector3 readGyroscope() override
        {
            int16_t gx, gy, gz;

            accelgyro_.getRotation(&gx, &gy, &gz);

            gyro_.x = gx * (double) gyro_scale_ * DEG_TO_RAD;
            gyro_.y = gy * (double) gyro_scale_ * DEG_TO_RAD;
            gyro_.z = gz * (double) gyro_scale_ * DEG_TO_RAD;

            return gyro_;
        }
};

class FakeIMU: public IMUInterface 
{
    private:
        geometry_msgs__msg__Vector3 accel_;
        geometry_msgs__msg__Vector3 gyro_;

    public:
        FakeIMU()
        {
        }

        bool startSensor() override
        {
            return true;
        }

        geometry_msgs__msg__Vector3 readAccelerometer() override
        {
            return accel_;
        }

        geometry_msgs__msg__Vector3 readGyroscope() override
        {
            return gyro_;
        }
};

// ---------------------------------------------------------------------------
// QMI8658 / QMI8658A / QMI8658C (QST 6-axis, I2C). Self-contained Wire driver.
//
// This replaced the 650-line QST/Waveshare reference driver that used to sit
// beside this file. That one carried module-level globals, a 2.2 s on-demand
// gyro calibration on every boot that calibrateGyro() then repeated, a
// hard-coded 0x6B, three register transactions per sample, and a getData()
// override that the IMUInterface* the sketch holds never reached -- so its
// bias handling was dead code. What is left is what the base needs:
//
//   * WHO_AM_I (0x00) reads 0x05 at 0x6B (SA0 high: Yahboom YB-EET01, the
//     GenDrv) or 0x6A (SA0 low); both are tried.
//   * +/-8 g at 4096 LSB/g, +/-1024 dps at 32 LSB/dps, both at 224.2 Hz with
//     the on-chip low-pass at 13.37 % of ODR (~30 Hz): the base publishes at
//     50 Hz, so the filter sits at its Nyquist and each publish sees a
//     settled, oversampled value rather than one raw 224 Hz sample.
//   * ONE burst per sample, 0x2D..0x40: STATUSINT, STATUS0, STATUS1, the
//     24-bit sample counter, temperature, AX..GZ. Reading STATUS0/STATUSINT
//     is what clears the chip's DATA_RDY line, so the burst both fetches the
//     sample and re-arms the interrupt for the next one. readGyroscope() does
//     the burst; readAccelerometer() hands back the accel half of it, which
//     is the order IMUInterface::getData() calls them in.
//   * DATA_RDY on request (enableDataReadyInterrupt()): the chip drives DRDY
//     on INT2 in its normal mode and on INT1 in SyncSample mode; a board that
//     breaks out "INT" rarely says which, so both are enabled and both modes
//     turned on. Whichever line the board routed rises once per sample.
//     Neither pin is driven until asked -- a board with the line unwired (the
//     GenDrv) keeps them high-impedance.
//
// Datasheet: QMI8658A rev 1.x, register map section 9. Field names below are
// the datasheet's.
// ---------------------------------------------------------------------------
class QMI8658IMU : public IMUInterface
{
    private:
        static const uint8_t REG_WHO_AM_I  = 0x00;   // -> 0x05
        static const uint8_t REG_REVISION  = 0x01;
        static const uint8_t REG_CTRL1     = 0x02;   // SIM | ADDR_AI | BE | INT2_EN | INT1_EN | FIFO_INT_SEL | - | SensorDisable
        static const uint8_t REG_CTRL2     = 0x03;   // aST | aFS[2:0] | aODR[3:0]
        static const uint8_t REG_CTRL3     = 0x04;   // gST | gFS[2:0] | gODR[3:0]
        static const uint8_t REG_CTRL5     = 0x06;   // - | gLPF_MODE[1:0] | gLPF_EN | - | aLPF_MODE[1:0] | aLPF_EN
        static const uint8_t REG_CTRL7     = 0x08;   // syncSmpl | - | DRDY_DIS | gSN | - | - | gEN | aEN
        static const uint8_t REG_STATUSINT = 0x2D;   // CmdDone | ... | Avail | Locked
        static const uint8_t REG_STATUS0   = 0x2E;   // gDA | aDA
        static const uint8_t REG_RESET     = 0x60;   // write 0xB0

        static const uint8_t CTRL1_ADDR_AI = 0x40;   // auto-increment the register address in a burst
        static const uint8_t CTRL1_BE      = 0x20;   // "big endian" in QST's naming: L byte at the lower address
        static const uint8_t CTRL1_INT2_EN = 0x10;
        static const uint8_t CTRL1_INT1_EN = 0x08;
        static const uint8_t CTRL7_SYNC    = 0x80;   // SyncSample: DRDY on INT1 as well
        static const uint8_t CTRL7_GEN     = 0x02;
        static const uint8_t CTRL7_AEN     = 0x01;

        // CTRL2: aFS 010 (+/-8 g) | aODR 0101 (224.2 Hz)
        static const uint8_t CTRL2_8G_224HZ      = 0x25;
        // CTRL3: gFS 110 (+/-1024 dps) | gODR 0101 (224.2 Hz)
        static const uint8_t CTRL3_1024DPS_224HZ = 0x65;
        // CTRL5: gLPF mode 3 + enable, aLPF mode 3 + enable (13.37 % of ODR)
        static const uint8_t CTRL5_LPF_BOTH      = 0x77;

        static constexpr float ACCEL_LSB_PER_G   = 4096.0f;
        static constexpr float GYRO_LSB_PER_DPS  = 32.0f;
        static constexpr float DEG_TO_RAD_F      = 0.017453292519943295f;

        // One burst: 0x2D .. 0x40 inclusive.
        static const uint8_t BURST_FIRST = REG_STATUSINT;
        static const uint8_t BURST_LEN   = 20;

        uint8_t addr_ = 0x6B;
        uint8_t revision_ = 0;
        uint8_t ctrl1_ = CTRL1_ADDR_AI | CTRL1_BE;
        uint8_t ctrl7_ = CTRL7_GEN | CTRL7_AEN;

        geometry_msgs__msg__Vector3 accel_{};
        geometry_msgs__msg__Vector3 gyro_{};
        uint32_t sample_count_ = 0;       // the chip's 24-bit counter, last read
        float temperature_c_ = 0.0f;
        uint32_t bursts_ok_ = 0;
        uint32_t bursts_failed_ = 0;

        void writeReg(uint8_t reg, uint8_t val)
        {
            Wire.beginTransmission(addr_);
            Wire.write(reg);
            Wire.write(val);
            Wire.endTransmission();
        }

        // 0xFF on a NACK, so a missing chip never reads as WHO_AM_I 0x05
        // (0x00 would be a plausible register value; 0xFF is what an idle
        // bus reads as).
        uint8_t readReg(uint8_t reg)
        {
            Wire.beginTransmission(addr_);
            Wire.write(reg);
            if (Wire.endTransmission(false) != 0)
                return 0xFF;
            if (Wire.requestFrom((int)addr_, 1) != 1)
                return 0xFF;
            return Wire.read();
        }

        bool readBlock(uint8_t reg, uint8_t *buf, uint8_t len)
        {
            Wire.beginTransmission(addr_);
            Wire.write(reg);
            if (Wire.endTransmission(false) != 0)
                return false;
            const uint8_t got = Wire.requestFrom((int)addr_, (int)len);
            for (uint8_t i = 0; i < got; i++)
            {
                const uint8_t b = Wire.read();
                if (i < len) buf[i] = b;
            }
            return got == len;
        }

        static int16_t le16(const uint8_t *p) { return (int16_t)((uint16_t)p[1] << 8 | p[0]); }

        // The burst. Fills accel_, gyro_, the sample counter and the
        // temperature; leaves the previous sample in place on a bus error so
        // a single failed transaction does not publish zeros.
        bool sample()
        {
            uint8_t b[BURST_LEN];
            if (!readBlock(BURST_FIRST, b, BURST_LEN))
            {
                bursts_failed_++;
                return false;
            }
            bursts_ok_++;
            // b[0] STATUSINT, b[1] STATUS0, b[2] STATUS1,
            // b[3..5] TIMESTAMP_L/M/H, b[6..7] TEMP_L/H,
            // b[8..13] AX..AZ, b[14..19] GX..GZ
            sample_count_ = (uint32_t)b[5] << 16 | (uint32_t)b[4] << 8 | b[3];
            temperature_c_ = (float)le16(&b[6]) / 256.0f;

            accel_.x = (float)le16(&b[8])  / ACCEL_LSB_PER_G * g_to_accel_;
            accel_.y = (float)le16(&b[10]) / ACCEL_LSB_PER_G * g_to_accel_;
            accel_.z = (float)le16(&b[12]) / ACCEL_LSB_PER_G * g_to_accel_;

            gyro_.x = (float)le16(&b[14]) / GYRO_LSB_PER_DPS * DEG_TO_RAD_F;
            gyro_.y = (float)le16(&b[16]) / GYRO_LSB_PER_DPS * DEG_TO_RAD_F;
            gyro_.z = (float)le16(&b[18]) / GYRO_LSB_PER_DPS * DEG_TO_RAD_F;
            return true;
        }

    protected:
        // Both pins, both modes: see the header comment. The base attaches
        // its ISR on RISING; the chip re-asserts the line after every sample
        // once the previous one's status has been read, which the burst does.
        bool enableDataReadyInterrupt() override
        {
            ctrl1_ |= CTRL1_INT1_EN | CTRL1_INT2_EN;
            ctrl7_ |= CTRL7_SYNC;               // DRDY_DIS stays 0: INT2 too
            writeReg(REG_CTRL1, ctrl1_);
            writeReg(REG_CTRL7, ctrl7_);
            return readReg(REG_CTRL7) == ctrl7_;
        }

    public:
        QMI8658IMU() {}

        bool startSensor() override
        {
            Wire.begin();

            bool found = false;
            const uint8_t cand[2] = { 0x6B, 0x6A };
            for (uint8_t i = 0; i < 2 && !found; i++)
            {
                addr_ = cand[i];
                found = readReg(REG_WHO_AM_I) == 0x05;
            }
            if (!found)
                return false;

            writeReg(REG_RESET, 0xB0);          // soft reset: every CTRL back to default
            delay(20);
            revision_ = readReg(REG_REVISION);

            ctrl1_ = CTRL1_ADDR_AI | CTRL1_BE;  // INT pins stay high-Z until asked
            ctrl7_ = CTRL7_GEN | CTRL7_AEN;
            writeReg(REG_CTRL1, ctrl1_);
            writeReg(REG_CTRL2, CTRL2_8G_224HZ);
            writeReg(REG_CTRL3, CTRL3_1024DPS_224HZ);
            writeReg(REG_CTRL5, CTRL5_LPF_BOTH);
            writeReg(REG_CTRL7, ctrl7_);
            delay(100);                         // filters settle; first samples land

            // Prove the configuration took and the chip is producing data
            // before init() starts averaging the gyro on it.
            if (readReg(REG_CTRL7) != ctrl7_)
                return false;
            return sample();
        }

        // getData() calls this first: it is the bus transaction.
        geometry_msgs__msg__Vector3 readGyroscope() override
        {
            sample();
            return gyro_;
        }

        // ... and this second, off the same burst.
        geometry_msgs__msg__Vector3 readAccelerometer() override
        {
            return accel_;
        }

        uint8_t address() const { return addr_; }
        uint8_t revision() const { return revision_; }
        uint32_t sampleCount() const { return sample_count_; }
        float temperatureC() const { return temperature_c_; }
        uint32_t burstsOk() const { return bursts_ok_; }
        uint32_t burstsFailed() const { return bursts_failed_; }
};

// ---------------------------------------------------------------------------
// LSM6DSOX (STMicroelectronics 6-axis, I2C / SPI). Self-contained Wire
// driver (no external library). Supports I2C Fast-Mode-Plus up to 1 MHz.
// I2C address 0x6A (SDO/SA0 low) or 0x6B (high).
// Datasheet: gyro/accel output registers are contiguous 0x22..0x2D, so each
// axis set is one burst read. The chip's built-in timestamp counter is
// enabled (CTRL10_C TIMESTAMP_EN) and exposed via readTimestampSec().
// ---------------------------------------------------------------------------
class LSM6DSOXIMU : public IMUInterface
{
    private:
        static const uint8_t REG_WHO_AM_I  = 0x0F;   // -> 0x6C
        static const uint8_t REG_CTRL1_XL  = 0x10;   // accel ODR / full-scale
        static const uint8_t REG_CTRL2_G   = 0x11;   // gyro  ODR / full-scale
        static const uint8_t REG_CTRL3_C   = 0x12;   // BDU / IF_INC / SW_RESET
        static const uint8_t REG_CTRL10_C  = 0x19;   // TIMESTAMP_EN (bit 5)
        static const uint8_t REG_OUTX_L_G  = 0x22;   // gyro  X..Z (6 bytes)
        static const uint8_t REG_OUTX_L_A  = 0x28;   // accel X..Z (6 bytes)
        static const uint8_t REG_TIMESTAMP0 = 0x40;  // 32-bit sample counter (LSB first)

        uint8_t addr_ = 0x6A;

        // CTRL1_XL 0x40 -> 104 Hz, +/-2 g   : 0.061 mg/LSB
        // CTRL2_G  0x4C -> 104 Hz, +/-2000 dps : 70 mdps/LSB
        const float accel_scale_ = 0.061e-3f;   // g per LSB
        const float gyro_scale_  = 70.0e-3f;    // dps per LSB
        // LSM6DSOX built-in timestamp: 1 LSB ~= 25 us (internal 40 kHz clock).
        const double timestamp_lsb_s_ = 25.0e-6;

        geometry_msgs__msg__Vector3 accel_;
        geometry_msgs__msg__Vector3 gyro_;

        void writeReg(uint8_t reg, uint8_t val)
        {
            Wire.beginTransmission(addr_);
            Wire.write(reg);
            Wire.write(val);
            Wire.endTransmission();
        }

        uint8_t readReg(uint8_t reg)
        {
            Wire.beginTransmission(addr_);
            Wire.write(reg);
            if (Wire.endTransmission(false) != 0)
                return 0;
            Wire.requestFrom((int)addr_, 1);
            return Wire.available() ? Wire.read() : 0;
        }

        bool readBlock(uint8_t reg, uint8_t *buf, uint8_t len)
        {
            Wire.beginTransmission(addr_);
            Wire.write(reg);
            if (Wire.endTransmission(false) != 0)
                return false;
            uint8_t got = Wire.requestFrom((int)addr_, (int)len);
            for (uint8_t i = 0; i < len && i < got; i++)
                buf[i] = Wire.read();
            return got == len;
        }

    public:
        LSM6DSOXIMU() {}

        bool startSensor() override
        {
            Wire.begin();
            Wire.setClock(1000000);   // LSM6DSOX I2C Fast-Mode-Plus (1 MHz)

            bool found = false;
            const uint8_t cand[2] = { 0x6A, 0x6B };
            for (uint8_t i = 0; i < 2; i++)
            {
                addr_ = cand[i];
                if (readReg(REG_WHO_AM_I) == 0x6C) { found = true; break; }
            }
            if (!found)
                return false;

            writeReg(REG_CTRL3_C, 0x01);    // SW_RESET
            delay(20);
            writeReg(REG_CTRL3_C, 0x44);    // BDU=1, IF_INC=1
            writeReg(REG_CTRL1_XL, 0x40);   // accel 104 Hz, +/-2 g
            writeReg(REG_CTRL2_G,  0x4C);   // gyro  104 Hz, +/-2000 dps
            writeReg(REG_CTRL10_C, 0x20);   // TIMESTAMP_EN — run the built-in sample counter
            delay(100);                     // let the digital filters settle
            return true;
        }

        // Built-in hardware timestamp: 32-bit free-running counter latched with
        // each sample, 1 LSB ~= 25 us. Use this for sample timing instead of
        // host millis() (roll-over ~29.8 h; caller handles wrap on the delta).
        uint32_t readTimestampRaw()
        {
            uint8_t b[4] = {0};
            readBlock(REG_TIMESTAMP0, b, 4);
            return (uint32_t)b[0] | ((uint32_t)b[1] << 8) |
                   ((uint32_t)b[2] << 16) | ((uint32_t)b[3] << 24);
        }

        double readTimestampSec() { return readTimestampRaw() * timestamp_lsb_s_; }

        geometry_msgs__msg__Vector3 readAccelerometer() override
        {
            uint8_t b[6] = {0};
            readBlock(REG_OUTX_L_A, b, 6);
            int16_t ax = (int16_t)(b[0] | (b[1] << 8));
            int16_t ay = (int16_t)(b[2] | (b[3] << 8));
            int16_t az = (int16_t)(b[4] | (b[5] << 8));
            accel_.x = ax * (double)accel_scale_ * g_to_accel_;
            accel_.y = ay * (double)accel_scale_ * g_to_accel_;
            accel_.z = az * (double)accel_scale_ * g_to_accel_;
            return accel_;
        }

        geometry_msgs__msg__Vector3 readGyroscope() override
        {
            uint8_t b[6] = {0};
            readBlock(REG_OUTX_L_G, b, 6);
            int16_t gx = (int16_t)(b[0] | (b[1] << 8));
            int16_t gy = (int16_t)(b[2] | (b[3] << 8));
            int16_t gz = (int16_t)(b[4] | (b[5] << 8));
            gyro_.x = gx * (double)gyro_scale_ * DEG_TO_RAD;
            gyro_.y = gy * (double)gyro_scale_ * DEG_TO_RAD;
            gyro_.z = gz * (double)gyro_scale_ * DEG_TO_RAD;
            return gyro_;
        }
};

// ---------------------------------------------------------------------------
// ICM-20948 (TDK InvenSense 9-axis). Self-contained I2C driver — accel + gyro
// here; the on-chip AK09916 magnetometer is exposed at 0x0C via I2C bypass
// (see ICM20948MAG in default_mag.h). Low noise / low drift. I2C address
// 0x68 (AD0 low) or 0x69 (high). Register banks selected via REG_BANK_SEL.
// ---------------------------------------------------------------------------
class ICM20948IMU: public IMUInterface
{
    private:
        uint8_t addr_ = 0x68;
        const float accel_scale_ = 1.0f / 16384.0f;   // ±2 g  -> g / LSB
        const float gyro_scale_  = 1.0f / 131.0f;     // ±250 dps -> dps / LSB

        geometry_msgs__msg__Vector3 accel_;
        geometry_msgs__msg__Vector3 gyro_;

        void w8(uint8_t reg, uint8_t val)
        {
            Wire.beginTransmission(addr_); Wire.write(reg); Wire.write(val); Wire.endTransmission();
        }
        bool rN(uint8_t reg, uint8_t *buf, uint8_t n)
        {
            Wire.beginTransmission(addr_); Wire.write(reg);
            if (Wire.endTransmission(false) != 0) return false;
            if (Wire.requestFrom((int)addr_, (int)n) != (int)n) return false;
            for (uint8_t i = 0; i < n; i++) buf[i] = Wire.read();
            return true;
        }
        uint8_t r8(uint8_t reg) { uint8_t v = 0; rN(reg, &v, 1); return v; }
        void bank(uint8_t b) { w8(0x7F, b << 4); }   // REG_BANK_SEL

    public:
        ICM20948IMU() {}

        bool startSensor() override
        {
            Wire.begin();
            const uint8_t cand[2] = { 0x68, 0x69 };
            for (int i = 0; i < 2; i++)
            {
                addr_ = cand[i];
                bank(0);
                if (r8(0x00) != 0xEA) continue;   // WHO_AM_I
                w8(0x06, 0x80); delay(20);        // PWR_MGMT_1: device reset
                w8(0x06, 0x01);                   // PWR_MGMT_1: auto clock, wake
                w8(0x07, 0x00);                   // PWR_MGMT_2: accel + gyro enabled
                w8(0x0F, 0x02);                   // INT_PIN_CFG: BYPASS_EN -> AK09916 @ 0x0C
                bank(2);
                w8(0x01, 0x00);                   // GYRO_CONFIG_1: ±250 dps, DLPF off
                w8(0x14, 0x00);                   // ACCEL_CONFIG : ±2 g,    DLPF off
                bank(0);
                return true;
            }
            return false;
        }

        geometry_msgs__msg__Vector3 readAccelerometer() override
        {
            uint8_t b[6];
            if (rN(0x2D, b, 6))                   // ACCEL_XOUT_H (big-endian)
            {
                accel_.x = (int16_t)(b[0] << 8 | b[1]) * (double)accel_scale_ * g_to_accel_;
                accel_.y = (int16_t)(b[2] << 8 | b[3]) * (double)accel_scale_ * g_to_accel_;
                accel_.z = (int16_t)(b[4] << 8 | b[5]) * (double)accel_scale_ * g_to_accel_;
            }
            return accel_;
        }

        geometry_msgs__msg__Vector3 readGyroscope() override
        {
            uint8_t b[6];
            if (rN(0x33, b, 6))                   // GYRO_XOUT_H
            {
                gyro_.x = (int16_t)(b[0] << 8 | b[1]) * (double)gyro_scale_ * DEG_TO_RAD;
                gyro_.y = (int16_t)(b[2] << 8 | b[3]) * (double)gyro_scale_ * DEG_TO_RAD;
                gyro_.z = (int16_t)(b[4] << 8 | b[5]) * (double)gyro_scale_ * DEG_TO_RAD;
            }
            return gyro_;
        }
};

// Note: Sparkfun library redefines I2C_BUFFER_LENGTH, so we undefine it for this class
#ifdef I2C_BUFFER_LENGTH
#undef I2C_BUFFER_LENGTH
#endif
#include <SparkFun_BNO080_Arduino_Library.h>

class BNO085IMU: public IMUInterface 
{
    private:
        BNO080 bno085_;
        const int bno085UpdateRateMs = 20;   // 50Hz update rate (standard for ROS IMU messages)
        const float accel_cov_ = 0.01;
        const float gyro_cov_ = 0.001;
        const float ori_xy_cov_ = 0.01;
        const float ori_z_cov_ = 0.05;

        unsigned long nextUpdateTime = 0;

        geometry_msgs__msg__Vector3 accel_;
        geometry_msgs__msg__Vector3 gyro_;

        // State Machine Enumeration
        enum IMUState {
        STATE_DISCONNECTED,
        STATE_SEND_CONFIG,
        STATE_CHECK_RESET,
        STATE_VALIDATE_STREAM,
        STATE_RUNNING
        };

        // Global State Variables for the IMU State Machine
        IMUState imuState = STATE_DISCONNECTED;
        unsigned long stateTimer = 0;
        int validPacketCount = 0;
        int configAttempts = 0;

    public:
        BNO085IMU()
        {
        }

        bool startSensor() override
        {
            Wire.begin();
            if (bno085_.begin() == 0){
                // Serial.println("bno085_init fail");
                syslog(LOG_ERR, "%s BNO085 IMU init fail %lu", __FUNCTION__, millis());
                imuState = STATE_DISCONNECTED;
                return false;
            }
            syslog(LOG_INFO, "%s BNO085 IMU init success %lu", __FUNCTION__, millis());
            imuState = STATE_SEND_CONFIG;

            return true;
        }

        geometry_msgs__msg__Vector3 readAccelerometer() override
        {
            accel_.x = bno085_.getAccelX();
            accel_.y = bno085_.getAccelY();
            accel_.z = bno085_.getAccelZ();
            return accel_;
        }

        geometry_msgs__msg__Vector3 readGyroscope() override
        {
            gyro_.x = bno085_.getGyroX() * DEG_TO_RAD;
            gyro_.y = bno085_.getGyroY() * DEG_TO_RAD;
            gyro_.z = bno085_.getGyroZ() * DEG_TO_RAD;
            return gyro_;
        }

        sensor_msgs__msg__Imu getData()
        {
            if (!runIMUStateMachine()) {
                logImuDataUnavailable();
                return imu_msg_;
            }
            imu_msg_.angular_velocity = readGyroscope();

            if(imu_msg_.angular_velocity.x > -0.01 && imu_msg_.angular_velocity.x < 0.01 )
                imu_msg_.angular_velocity.x = 0;

            if(imu_msg_.angular_velocity.y > -0.01 && imu_msg_.angular_velocity.y < 0.01 )
                imu_msg_.angular_velocity.y = 0;

            if(imu_msg_.angular_velocity.z > -0.01 && imu_msg_.angular_velocity.z < 0.01 )
                imu_msg_.angular_velocity.z = 0;

            imu_msg_.angular_velocity_covariance[0] = gyro_cov_;
            imu_msg_.angular_velocity_covariance[4] = gyro_cov_;
            imu_msg_.angular_velocity_covariance[8] = gyro_cov_;

            imu_msg_.linear_acceleration = readAccelerometer();
            imu_msg_.linear_acceleration_covariance[0] = accel_cov_;
            imu_msg_.linear_acceleration_covariance[4] = accel_cov_;
            imu_msg_.linear_acceleration_covariance[8] = accel_cov_;

            imu_msg_.orientation.x = bno085_.getQuatI();
            imu_msg_.orientation.y = bno085_.getQuatJ();
            imu_msg_.orientation.z = bno085_.getQuatK();
            imu_msg_.orientation.w = bno085_.getQuatReal();

            imu_msg_.orientation_covariance[0] = ori_xy_cov_;
            imu_msg_.orientation_covariance[4] = ori_xy_cov_;
            imu_msg_.orientation_covariance[8] = ori_z_cov_;

            return imu_msg_;
        }

        // The BNO085 IMU has an I2C interface that doesn't work well with the ESP32.
        // To work around this, we implement a state machine to manage the IMU's initialization and data streaming.
        bool runIMUStateMachine()
        {
        switch (imuState) {

          case STATE_DISCONNECTED:
            if (millis() - stateTimer >= 500) {
                stateTimer = millis();
                if (bno085_.begin() == true) {
                syslog(LOG_INFO, "%s [I2C] Link achieved. Moving to configuration step.", __FUNCTION__);
                imuState = STATE_SEND_CONFIG;
                }
            }
            break;

          case STATE_SEND_CONFIG:
            configAttempts++;
            syslog(LOG_INFO, "%s [CONFIG] Transmitting 6-DOF Profile (Try # %d)...", __FUNCTION__, configAttempts);

            bno085_.hasReset(); // Clear historical reset tracking bits

            bno085_.enableGameRotationVector(bno085UpdateRateMs);
            bno085_.enableGyro(bno085UpdateRateMs);
            bno085_.enableAccelerometer(bno085UpdateRateMs);
            bno085_.endCalibration(); // Anchor our saved physical Tare profile

            stateTimer = millis();
            imuState = STATE_CHECK_RESET;
            break;

          case STATE_CHECK_RESET:
            if (millis() - stateTimer >= 400) {
                if (bno085_.hasReset()) {
                syslog(LOG_INFO, "%s [WARNING] Reset flag caught during parsing. Cyclical retry...", __FUNCTION__);
                imuState = STATE_SEND_CONFIG;
                } else {
                syslog(LOG_INFO, "%s [VALIDATION] Checking telemetry stream integrity...", __FUNCTION__);
                validPacketCount = 0;
                stateTimer = millis();
                imuState = STATE_VALIDATE_STREAM;
                }
            }
            break;

          case STATE_VALIDATE_STREAM:
            // Note: myIMU.dataAvailable() internally executes and evaluates getReadings()
            if (bno085_.dataAvailable() == true) {
                // 2. Verify the active packet type matches our navigation profile.
                // This isolates the actual 6-DOF frame and strips out 0.06 diagnostic responses.
                if (bno085_.getReadings() == SENSOR_REPORTID_GAME_ROTATION_VECTOR) {
                    float testYaw = bno085_.getYaw();
                    if (!isnan(testYaw)) {
                        validPacketCount++;
                    }
                }
            }

            if (validPacketCount >= 10) {
                syslog(LOG_INFO, "%s [SUCCESS] Navigation data streams verified numeric!", __FUNCTION__);
                imuState = STATE_RUNNING;
            }
            else if (millis() - stateTimer >= 1500) {
                syslog(LOG_INFO, "%s [TIMEOUT] Stream unpopulated or stuck on NaN. Soft resetting...", __FUNCTION__);
                bno085_.softReset();
                stateTimer = millis();
                imuState = STATE_CHECK_RESET;
            }
            break;

          case STATE_RUNNING:
            if (bno085_.dataAvailable() == true) {
                // Enforce the layout check in real-time execution to prevent background packets from causing spikes
                if (bno085_.getReadings() == SENSOR_REPORTID_GAME_ROTATION_VECTOR) {
// Uncomment the following line to log the IMU data to syslog for debugging purposes
// #define DEBUG_BNO085
#ifdef DEBUG_BNO085
                    float roll = bno085_.getRoll() * RAD_TO_DEG;
                    float pitch = bno085_.getPitch() * RAD_TO_DEG;
                    float yaw = bno085_.getYaw() * RAD_TO_DEG;

                    if (millis() >= nextUpdateTime) {
                        syslog(LOG_INFO, "%s BNO085 IMU data read complete %lu, roll: %0.2f, pitch: %0.2f, yaw: %0.2f", __FUNCTION__, millis(), roll, pitch, yaw);
                        nextUpdateTime = millis() + 500;
                    }
#endif
                }
                return true;  // IMU is fully initialized and running and we can use its data
            } else {
                syslog(LOG_INFO, "%s [WARNING] Data stream interrupted. Revalidating...", __FUNCTION__);
                imuState = STATE_DISCONNECTED;
            }
            break;
        }
        return false;  // if we don't return true from STATE_RUNNING, we are not fully initialized yet
        }

        void logImuDataUnavailable()
        {
            static unsigned long lastLogTime = 0;
            unsigned long currentTime = millis();
            if (currentTime - lastLogTime >= 1000) { // Log every 1 second
                syslog(LOG_INFO, "%s BNO085 IMU data not available %lu", __FUNCTION__, currentTime);
                lastLogTime = currentTime;
            }
        }
};

#endif
//ADXL345 https://www.sparkfun.com/datasheets/Sensors/Accelerometer/ADXL345.pdf
//HMC8553L https://cdn-shop.adafruit.com/datasheets/HMC5883L_3-Axis_Digital_Compass_IC.pdf
//ITG320 https://www.sparkfun.com/datasheets/Sensors/Gyro/PS-ITG-3200-00-01.4.pdf


//MPU9150 https://www.invensense.com/wp-content/uploads/2015/02/PS-MPU-9250A-01-v1.1.pdf
//MPU9250 https://www.invensense.com/wp-content/uploads/2015/02/MPU-9150-Datasheet.pdf
//MPU6050 https://store.invensense.com/datasheets/invensense/MPU-6050_DataSheet_V3%204.pdf

//http://www.sureshjoshi.com/embedded/invensense-imus-what-to-know/
//https://stackoverflow.com/questions/19161872/meaning-of-lsb-unit-and-unit-lsb
