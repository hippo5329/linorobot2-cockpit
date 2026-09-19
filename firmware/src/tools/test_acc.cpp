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
#include <stdio.h>
#include <i2cdetect.h>

#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/magnetic_field.h>
#include <sensor_msgs/msg/battery_state.h>
#include <sensor_msgs/msg/range.h>
#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/vector3.h>

#include "config.h"
#include "tools.h"
#include "hw_factory.h"
#include "syslog.h"
#include "motor.h"
#include "kinematics.h"
#include "pid.h"
#include "odometry.h"
#include "imu.h"
#include "mag.h"
#define ENCODER_USE_INTERRUPTS
#define ENCODER_OPTIMIZE_INTERRUPTS
#include "encoder.h"
#include "lidar.h"
#include "wifis.h"
#include "ota.h"
#include "board_init.h"
#include "sensor_factory.h"
#include "mcu_env.h"
#include "i2c_probe.h"

#ifndef BAUDRATE
#define BAUDRATE 921600
#endif

// LED_ACTIVE is defined at most once (no #else branch): a board with an
// addressable status LED sets LED_PIN >= 0; LED_PIN -1 (e.g. Waveshare GenDrv)
// leaves it undefined and the LED writes below compile out.
#if defined(LED_PIN) && (LED_PIN) >= 0
#define LED_ACTIVE
#endif

// Accelerometer calibration.
//
// The Encoder and Motor objects used to be constructed at file scope. In a
// unified image that runs their constructors at boot for EVERY mode -- touching
// motor pins before initBoard() has set the bus up, in a robot whose user asked
// for a sensor scan. They come from hw_factory now, built in setup_() like the
// robot firmware's, which also means this tool drives the wiring the env
// partition describes rather than whatever the header was generated for.
//
// Serial, initBoard() and the radio belong to the dispatcher (see tools.h).
namespace test_acc {

EncoderInterface *motor1_encoder = NULL, *motor2_encoder = NULL,
                 *motor3_encoder = NULL, *motor4_encoder = NULL;
MotorInterface   *motor1_controller = NULL, *motor2_controller = NULL,
                 *motor3_controller = NULL, *motor4_controller = NULL;

// Only imu_msg is used here. The other five came along from main.cpp when this
// tool was split out and were never referenced -- 1056 bytes of .bss in an
// image whose static segment is 124580 bytes total, carried on every board.
sensor_msgs__msg__Imu imu_msg;



PID motor1_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID motor2_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID motor3_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID motor4_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);

Kinematics kinematics(
    Kinematics::LINO_BASE,
    MOTOR_MAX_RPM,
    MAX_RPM_RATIO,
    MOTOR_OPERATING_VOLTAGE,
    MOTOR_POWER_MAX_VOLTAGE,
    WHEEL_DIAMETER,
    LR_WHEELS_DISTANCE
);

// No `Odometry odometry;` here: it was declared and never referenced, 728
// bytes of .bss out of a 124580-byte static segment, on every board and
// every app. This tool measures wheel velocity through Kinematics; it never
// built an odometry estimate.

// Pointers, built in setup_() from what is on the bus -- not the macro types the
// config header chose. This tool runs on a real robot with real motors, and its
// whole output is the IMU's measured acceleration set against the wheel
// odometry's, so the IMU has to be the chip that is actually answering. One
// image serves every board of a family and the generated header is whatever
// robot was last generated (AGENTS.md S12), so a build-time `IMU` here reports
// through whichever driver that header happened to name: the motors spin, the
// encoders read, the velocity and acceleration columns fill with real numbers,
// and the one column the test exists to produce is taken from the wrong chip.
// Same rule and the same two calls as test_sensors and `base`.
IMUInterface *imu = nullptr;
MAGInterface *mag = nullptr;
unsigned total_motors = 4;

void setup_()
{
    for (int i = 1; i <= 4; i++) {
        EncoderInterface **enc = (i == 1) ? &motor1_encoder : (i == 2) ? &motor2_encoder
                               : (i == 3) ? &motor3_encoder : &motor4_encoder;
        MotorInterface **mot = (i == 1) ? &motor1_controller : (i == 2) ? &motor2_controller
                             : (i == 3) ? &motor3_controller : &motor4_controller;
        *enc = createEncoder(i);
        *mot = createMotor(i);
    }
#ifdef LED_ACTIVE
    pinMode(LED_PIN, OUTPUT);
#endif
    // I2C bus and boot-time output pins, from the env partition falling back to
    // the generated header -- see firmware/common/lib/board_init. This replaced
    // the board-specific init macro, a fragment of setup() living in a config.
    initBoard();

    initWifis();
    initOta();

    i2cdetect();

    // The same probe and the same adoption rule the robot firmware uses, so this
    // tool and `base` can never disagree about what is fitted.
    initMcuEnv();
    const char *imu_name = envGet("imu", defaultIMUName());
    const char *mag_name = envGet("mag", defaultMAGName());
    i2cProbeSelect(&imu_name, &mag_name);
    imu = createIMU(imu_name);
    mag = createMAG(mag_name);

    if (!imu->init())
        Serial.println("[-] IMU initialization FAILED -- IMU ACC below is not a measurement.");
    else
        Serial.printf("[+] IMU %s initialized.\n", imu_name);
    mag->init();

    if(Kinematics::LINO_BASE == Kinematics::DIFFERENTIAL_DRIVE)
    {
        total_motors = 2;
    }
    motor1_encoder->getRPM();
    motor2_encoder->getRPM();
    motor3_encoder->getRPM();
    motor4_encoder->getRPM();

    initBoardLate();
    syslog(LOG_INFO, "%s Ready %lu", __FUNCTION__, millis());
    delay(2000);
}

unsigned runs = 12;
const unsigned ticks = 20;
const float dt = ticks * 0.001f;
const unsigned run_time = 1000; // 1s
const unsigned buf_size = run_time / ticks * 4;
// The 2400-byte velocity trace is a LOCAL in loop_(), passed to the two
// functions that touch it -- see loop_(). It used to be a file-scope array,
// which put it in .bss, where it was charged against a static segment of just
// 124580 bytes (memory.ld: 0x2c200 - 0xdb5c) and paid for by every app in the
// image, since the tools are dispatched at runtime from one binary. A tool
// owns the whole 8 KB loop-task stack while it runs -- it never enters the
// base micro-ROS loop -- so 2400 bytes of it costs nothing that is otherwise
// in use.
// No batt[] here any more: it was written once per sample and never read, so
// it was 800 bytes of static RAM recording something nobody looked at. If a
// battery trace is wanted alongside the velocity trace, add it back WITH the
// code that prints it.
float imu_max_acc_x, imu_min_acc_x;
unsigned idx = 0;

void record(unsigned n, Kinematics::velocities *buf) {
    for (unsigned i = 0; i < n; i++, idx++) {
        float rpm1 = motor1_encoder->getRPM();
        float rpm2 = motor2_encoder->getRPM();
        float rpm3 = motor3_encoder->getRPM();
        float rpm4 = motor4_encoder->getRPM();
        imu_msg = imu->getData();
        float imu_acc_x = imu_msg.linear_acceleration.x;
        if (imu_acc_x > imu_max_acc_x) imu_max_acc_x = imu_acc_x;
        if (imu_acc_x < imu_min_acc_x) imu_min_acc_x = imu_acc_x;

        if (idx < buf_size) {
            buf[idx] = kinematics.getVelocities(rpm1, rpm2, rpm3, rpm4);
        }
        delay(ticks);
        runWifis();
        runOta();
    }
}

void dump_record(const Kinematics::velocities *buf) {
    float max_vel_x = 0, min_vel_x = 0, max_acc_x = 0, min_acc_x = 0;
    float max_vel_y = 0, min_vel_y = 0, max_acc_y = 0, min_acc_y = 0;
    float max_vel_z = 0, min_vel_z = 0, max_acc_z = 0, min_acc_z = 0;
    float dist = 0;
    for (idx = 0; idx < buf_size; idx++) {
        float vel_x = buf[idx].linear_x;
        float vel_y = buf[idx].linear_y;
        float vel_z = buf[idx].angular_z;
        if (vel_x > max_vel_x) max_vel_x = vel_x;
        if (vel_x < min_vel_x) min_vel_x = vel_x;
        if (vel_y > max_vel_y) max_vel_y = vel_y;
        if (vel_y < min_vel_y) min_vel_y = vel_y;
        if (vel_z > max_vel_z) max_vel_z = vel_z;
        if (vel_z < min_vel_z) min_vel_z = vel_z;
    }
    for (idx = 0; idx < buf_size; idx++) {
        unsigned prev = idx ? (idx - 1) : 0;
        float acc_x = (buf[idx].linear_x - buf[prev].linear_x) / dt;
        float acc_y = (buf[idx].linear_y - buf[prev].linear_y) / dt;
        float acc_z = (buf[idx].angular_z - buf[prev].angular_z) / dt;
        if (acc_x > max_acc_x) max_acc_x = acc_x;
        if (acc_x < min_acc_x) min_acc_x = acc_x;
        if (acc_y > max_acc_y) max_acc_y = acc_y;
        if (acc_y < min_acc_y) min_acc_y = acc_y;
        if (acc_z > max_acc_z) max_acc_z = acc_z;
        if (acc_z < min_acc_z) min_acc_z = acc_z;
    }
    if (runs & 1) {
        for (idx = buf_size / 4; idx < buf_size / 2; idx++)
            dist += buf[idx].linear_x * dt;
        for (idx = 0; idx < buf_size / 4; idx++)
            if (buf[idx].linear_x > max_vel_x * 0.9f) break;
    } else {
        for (idx = buf_size / 4; idx < buf_size / 2; idx++)
            dist += buf[idx].angular_z * dt;
        for (idx = 0; idx < buf_size / 4; idx++)
            if (buf[idx].angular_z > max_vel_z * 0.9f) break;
    }

    Serial.printf("MAX VEL %6.2f %6.2f m/s  %6.2f rad/s\n", max_vel_x, max_vel_y, max_vel_z);
    Serial.printf("MIN VEL %6.2f %6.2f m/s  %6.2f rad/s\n", min_vel_x, min_vel_y, min_vel_z);
    Serial.printf("MAX ACC %6.2f %6.2f m/s2  %6.2f rad/s2\n", max_acc_x, max_acc_y, max_acc_z);
    Serial.printf("MIN ACC %6.2f %6.2f m/s2  %6.2f rad/s2\n", min_acc_x, min_acc_y, min_acc_z);
    Serial.printf("IMU ACC %6.2f %6.2f m/s2\n", imu_max_acc_x, imu_min_acc_x);
    Serial.printf("time to 0.9x max vel %6.2f sec\n", idx * dt);
    Serial.printf("distance to stop %6.2f %s\n", dist, (runs & 1) ? "m" : "rad");

    syslog(LOG_INFO, "MAX VEL %6.2f %6.2f m/s  %6.2f rad/s", max_vel_x, max_vel_y, max_vel_z);
    syslog(LOG_INFO, "MIN VEL %6.2f %6.2f m/s  %6.2f rad/s", min_vel_x, min_vel_y, min_vel_z);
    syslog(LOG_INFO, "MAX ACC %6.2f %6.2f m/s2  %6.2f rad/s2", max_acc_x, max_acc_y, max_acc_z);
    syslog(LOG_INFO, "MIN ACC %6.2f %6.2f m/s2  %6.2f rad/s2", min_acc_x, min_acc_y, min_acc_z);
    syslog(LOG_INFO, "IMU ACC %6.2f %6.2f m/s2", imu_max_acc_x, imu_min_acc_x);
}

void loop_() {
    // The velocity trace lives here, on the stack, for exactly as long as the
    // test runs. 2400 bytes of the loop task's 8192, and this tool is the only
    // thing on that task: selecting an app replaces the base loop, it does not
    // run alongside it.
    Kinematics::velocities buf[buf_size] = {};
    const int pwm_max = (1 << PWM_BITS) - 1;
    float current_pwm_max = pwm_max;
    float current_pwm_min = -current_pwm_max;

    while (runs > 0) {
        runs--;
        idx = 0;
        imu_max_acc_x = 0;
        imu_min_acc_x = 0;
#ifdef LED_ACTIVE
        digitalWrite(LED_PIN, HIGH);
#endif
        motor1_controller->spin((runs & 1) ? current_pwm_max : current_pwm_min);
        motor2_controller->spin(current_pwm_max);
        motor3_controller->spin((runs & 1) ? current_pwm_max : current_pwm_min);
        motor4_controller->spin(current_pwm_max);
        record(run_time / ticks, buf);

#ifdef LED_ACTIVE
        digitalWrite(LED_PIN, LOW);
#endif
        motor1_controller->spin(0);
        motor2_controller->spin(0);
        motor3_controller->spin(0);
        motor4_controller->spin(0);
        record(run_time / ticks, buf);

#ifdef LED_ACTIVE
        digitalWrite(LED_PIN, HIGH);
#endif
        motor1_controller->spin((runs & 1) ? current_pwm_min : current_pwm_max);
        motor2_controller->spin(current_pwm_min);
        motor3_controller->spin((runs & 1) ? current_pwm_min : current_pwm_max);
        motor4_controller->spin(current_pwm_min);
        record(run_time / ticks, buf);

#ifdef LED_ACTIVE
        digitalWrite(LED_PIN, LOW);
#endif
        motor1_controller->spin(0);
        motor2_controller->spin(0);
        motor3_controller->spin(0);
        motor4_controller->spin(0);
        record(run_time / ticks, buf);

        Serial.printf("MAX PWM %6.1f %6.1f\n", current_pwm_max, current_pwm_min);
        syslog(LOG_INFO, "MAX PWM %6.1f %6.1f", current_pwm_max, current_pwm_min);
        dump_record(buf);
        if ((runs & 3) == 0) {
            current_pwm_max /= 2;
            current_pwm_min /= 2;
        }
    }

    delay(100);
}

}  // namespace test_acc

extern "C" void test_acc_setup(void) { test_acc::setup_(); }
extern "C" void test_acc_loop(void)  { test_acc::loop_(); }
