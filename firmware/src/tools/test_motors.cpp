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

#ifndef BAUDRATE
#define BAUDRATE 921600
#endif

// Motor diagnostics and calibration.
//
// The Encoder and Motor objects used to be constructed at file scope. In a
// unified image that runs their constructors at boot for EVERY mode -- touching
// motor pins before initBoard() has set the bus up, in a robot whose user asked
// for a sensor scan. They come from hw_factory now, built in setup_() like the
// robot firmware's, which also means this tool drives the wiring the env
// partition describes rather than whatever the header was generated for.
//
// Serial, initBoard() and the radio belong to the dispatcher (see tools.h).
namespace test_motors {

EncoderInterface *motor1_encoder = NULL, *motor2_encoder = NULL,
                 *motor3_encoder = NULL, *motor4_encoder = NULL;
MotorInterface   *motor1_controller = NULL, *motor2_controller = NULL,
                 *motor3_controller = NULL, *motor4_controller = NULL;

nav_msgs__msg__Odometry odom_msg;
sensor_msgs__msg__Imu imu_msg;
sensor_msgs__msg__MagneticField mag_msg;
geometry_msgs__msg__Twist twist_msg;
sensor_msgs__msg__BatteryState battery_msg;
sensor_msgs__msg__Range range_msg;

#if defined(LED_PIN) && (LED_PIN) >= 0
#define LED_ACTIVE
#endif

void setLed(int value)
{
#ifdef LED_ACTIVE
    digitalWrite(LED_PIN, value);
#else
    (void)value;
#endif
}

int getLed(void)
{
#ifdef LED_ACTIVE
    return digitalRead(LED_PIN);
#else
    return 0;
#endif
}

void initLed(void)
{
#ifdef LED_ACTIVE
    pinMode(LED_PIN, OUTPUT);
#endif
}



PID motor1_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID motor2_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID motor3_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID motor4_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);

Odometry odometry;
IMU imu;
MAG mag;
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
    initLed();
    // I2C bus and boot-time output pins, from the env partition falling back to
    // the generated header -- see firmware/common/lib/board_init. This replaced
    // the board-specific init macro, a fragment of setup() living in a config.
    initBoard();

    initWifis();
    initOta();
    i2cdetect();  // default range from 0x03 to 0x77
    imu.init();
    mag.init();

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
}

void loop_() {
    static unsigned tk = 0; // tick
    const unsigned run_time = 8; // run time of each motor
    const unsigned cycle = run_time * total_motors;
    unsigned current_motor = tk / run_time % total_motors;
    unsigned direction = tk / cycle % 2; // 0 forward, 1 reverse
    const int pwm_max = (1 << PWM_BITS) - 1;
    static float max_rpm, stopping;

    setLed(direction ? LOW : HIGH);
    motor1_controller->spin((current_motor == 0) ? (direction ? -pwm_max : pwm_max) : 0);
    motor2_controller->spin((current_motor == 1) ? (direction ? -pwm_max : pwm_max) : 0);
    motor3_controller->spin((current_motor == 2) ? (direction ? -pwm_max : pwm_max) : 0);
    motor4_controller->spin((current_motor == 3) ? (direction ? -pwm_max : pwm_max) : 0);

    delay(1000);
    float current_rpm1 = motor1_encoder->getRPM();
    float current_rpm2 = motor2_encoder->getRPM();
    float current_rpm3 = motor3_encoder->getRPM();
    float current_rpm4 = motor4_encoder->getRPM();
    if (current_motor == 0 && tk % run_time == run_time - 1) max_rpm = current_rpm1;
    if (current_motor == 1 && tk % run_time == 0) stopping = current_rpm1;
    if (current_motor == 1 && tk % run_time == run_time - 1) max_rpm = current_rpm2;
    if (total_motors == 2 && current_motor == 0 && tk % run_time == 0) stopping = current_rpm2;
    if (current_motor == 2 && tk % run_time == 0) stopping = current_rpm2;
    if (current_motor == 2 && tk % run_time == run_time - 1) max_rpm = current_rpm3;
    if (current_motor == 3 && tk % run_time == 0) stopping = current_rpm3;
    if (current_motor == 3 && tk % run_time == run_time - 1) max_rpm = current_rpm4;
    if (total_motors == 4 && current_motor == 0 && tk % run_time == 0) stopping = current_rpm4;
    if (tk && tk % run_time == 0) {
        float max_linear_speed = max_rpm / 60.0 * PI * WHEEL_DIAMETER; // m/s = rps * circumference

        Serial.printf("MOTOR%d SPEED %6.2f m/s STOP %6.3f m\n", current_motor ? current_motor : total_motors,
	       max_linear_speed, max_linear_speed * stopping / max_rpm);
        syslog(LOG_INFO, "MOTOR%d SPEED %6.2f m/s STOP %6.3f m\n", current_motor ? current_motor : total_motors,
	       max_linear_speed, max_linear_speed * stopping / max_rpm);
    }
    Serial.printf("MOTOR%d %s RPM %8.1f %8.1f %8.1f %8.1f\n",
	   current_motor + 1, direction ? "REV" : "FWD",
	   current_rpm1, current_rpm2, current_rpm3, current_rpm4);
    syslog(LOG_INFO, "MOTOR%d %s RPM %8.1f %8.1f %8.1f %8.1f\n",
	   current_motor + 1, direction ? "REV" : "FWD",
	   current_rpm1, current_rpm2, current_rpm3, current_rpm4);
    tk++;
}

}  // namespace test_motors

extern "C" void test_motors_setup(void) { test_motors::setup_(); }
extern "C" void test_motors_loop(void)  { test_motors::loop_(); }
