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
#include "i2c_probe.h"

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
#include "sensor_factory.h"
#include "mcu_env.h"
#define ENCODER_USE_INTERRUPTS
#define ENCODER_OPTIMIZE_INTERRUPTS
#include "encoder.h"
#include "lidar.h"
#include "wifis.h"
#include "ota.h"
#include "board_init.h"
#include "tool_hw.h"
#include "battery.h"

#ifndef BAUDRATE
#define BAUDRATE 921600
#endif

// Motor diagnostics and calibration. The tool's `motors` / `motor` commands
// (tools.cpp). The motors and encoders are main.cpp's: initDrivetrain() builds them
// from the env partition for every mode, first thing in setup(), so a command drives the
// wiring the env describes and two commands in a row never build them twice.
//
// Serial, initBoard() and the radio belong to the dispatcher (see tools.h).
namespace test_motors {

// main.cpp's drivetrain (initDrivetrain(), every mode): the tool's commands share it.
EncoderInterface *&motor1_encoder = motor_encoders[0], *&motor2_encoder = motor_encoders[1],
                 *&motor3_encoder = motor_encoders[2], *&motor4_encoder = motor_encoders[3];
MotorInterface   *&motor1_controller = motor_controllers[0], *&motor2_controller = motor_controllers[1],
                 *&motor3_controller = motor_controllers[2], *&motor4_controller = motor_controllers[3];

// All six of these came from main.cpp when this tool was split out and not one
// of them was referenced here -- 1376 bytes of .bss, on every board, for
// nothing.

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



// No PIDs here: this tool drives open loop, and the four it declared were never
// used. The base type, the wheel and the PWM width come from the env, like
// main.cpp's: the header's macros named whichever robot the image was generated
// for, so a 0.056 m wheel was reported through the header's diameter
// (a tracked GenDrv robot, 2026-10-08: 181 RPM printed as 0.95 m/s).

// No `Odometry odometry;` here: declared, never referenced, 728 bytes of
// .bss out of a 124580-byte static segment. A motor test has no use for an
// odometry estimate.
// The bus, the IMU and the radio are the tool's (tool_hw.cpp): this command only drives.
static int encPin[4][2];
static unsigned total_motors = 4;

static void readEncPins(void)
{
    static const char *k[4][2] = {{"m1_enc_a", "m1_enc_b"}, {"m2_enc_a", "m2_enc_b"},
                                  {"m3_enc_a", "m3_enc_b"}, {"m4_enc_a", "m4_enc_b"}};
    for (int m = 0; m < 4; m++)
        for (int c = 0; c < 2; c++) encPin[m][c] = envInt(k[m][c], -1);
}

// The drive: a quarter of full PWM unless the env's tm_pct says otherwise. A first run on a new
// robot turns the wheels slowly, where a wrong wire, a reversed motor or a loose wheel is seen
// before it is felt; full power is one env write away (tm_pct=100).
static int testPwm(void)
{
    const int full = (1 << envU16("pwm_bits", PWM_BITS)) - 1;
    int pct = envInt("tm_pct", 25);
    if (pct < 1) pct = 1;
    if (pct > 100) pct = 100;
    return full * pct / 100;
}

// The encoder lines, read directly: each pin polled every 100 us for the second the motor runs,
// its level changes counted. Independent of the Encoder class and its pin interrupts, so a
// line that moves while the count stays 0 is the counting, and a line that never moves is the
// wiring or the encoder's supply (a mecanum robot's first run, 2026-10-11: two wheels turned
// and every count read 0).
static void pollEncoders(unsigned ms, unsigned edges[4][2], int level[4][2])
{
    for (int m = 0; m < 4; m++)
        for (int c = 0; c < 2; c++) {
            edges[m][c] = 0;
            level[m][c] = encPin[m][c] >= 0 ? digitalRead(encPin[m][c]) : -1;
        }
    const unsigned long t0 = millis();
    while (millis() - t0 < ms) {
        for (int m = 0; m < 4; m++)
            for (int c = 0; c < 2; c++) {
                if (encPin[m][c] < 0) continue;
                const int v = digitalRead(encPin[m][c]);
                if (v != level[m][c]) { edges[m][c]++; level[m][c] = v; }
            }
        delayMicroseconds(100);
        // A second is long to wait for `stop` with a wheel turning.
        if ((millis() - t0) % 10 == 0) {
            linoWdtFeed();
            if (toolStopRequested())
                toolAbort(toolStopReason());
        }
    }
}

// `motor <1-4|all> <pct> [secs]` (the shell sets tm_only / tm_pct / tm_secs): one motor,
// or all four (tm_only=5, the encoders' interrupt load at its highest), at tm_pct % --
// signed, minus is reverse -- printing its RPM and the polled encoder lines every second,
// so a person beside the robot can say which wheel turns and which way. Read every
// second, so `set tm_pct 50` while it runs changes its power. Stops on `stop`, or after
// tm_secs (120 when absent; 0 = until stop).
static unsigned long one_t0 = 0;

// The pack under load: a thin lead or a tired pack shows as sag beside the RPM. NAN when the
// robot has no battery sense.
static float packVolts(void) { return batteryPresent() ? getBattery().voltage : NAN; }

static void printBat(float v)
{
    if (isnan(v)) Serial.print("  BAT none");
    else          Serial.printf("  BAT %5.2f V", v);
}

// One tagged line a second for a program (the Web UI's readouts): never parse the prose.
static void toolJson(const char *cmd, int wheel, const char *dir, int pct, const float rpm[4],
                     unsigned edges[4][2], float bat)
{
    Serial.printf("[TOOL_JSON] {\"cmd\":\"%s\",\"wheel\":%d,\"dir\":\"%s\",\"pct\":%d,"
                  "\"rpm\":[%.1f,%.1f,%.1f,%.1f]", cmd, wheel, dir, pct, rpm[0], rpm[1], rpm[2], rpm[3]);
    if (edges)
        Serial.printf(",\"enc\":[[%u,%u],[%u,%u],[%u,%u],[%u,%u]]", edges[0][0], edges[0][1],
                      edges[1][0], edges[1][1], edges[2][0], edges[2][1], edges[3][0], edges[3][1]);
    if (isnan(bat)) Serial.print(",\"bat\":null}\n");
    else            Serial.printf(",\"bat\":%.2f}\n", bat);
}

static int onePwm(void)
{
    const int full = (1 << envU16("pwm_bits", PWM_BITS)) - 1;
    int pct = envInt("tm_pct", 25);
    if (pct > 100) pct = 100;
    if (pct < -100) pct = -100;
    return full * pct / 100;
}

void oneBegin(void)
{
    readEncPins();
    total_motors = tool_hw::total_motors;
    one_t0 = millis();
    const int only = envInt("tm_only", 0);
    const long secs = envInt("tm_secs", 120);
    if (only == 5) Serial.printf("MOTORS ALL at %d %%", envInt("tm_pct", 25));
    else           Serial.printf("MOTOR %d at %d %%", only, envInt("tm_pct", 25));
    if (secs > 0) Serial.printf(" for %ld s -- 'stop' ends it\n", secs);
    else          Serial.println(" until 'stop'");
}

bool oneStep(void)
{
    const int only = envInt("tm_only", 0);
    if (only < 1 || only > 5) return false;
    const long secs = envInt("tm_secs", 120);
    const bool running = secs <= 0 || millis() - one_t0 < 1000UL * (unsigned long)secs;
    MotorInterface *mot[4] = {motor1_controller, motor2_controller, motor3_controller, motor4_controller};
    EncoderInterface *enc[4] = {motor1_encoder, motor2_encoder, motor3_encoder, motor4_encoder};
    const int pwm = onePwm();
    const char *dir = pwm < 0 ? "REV" : "FWD";
    for (int m = 0; m < 4; m++)
        mot[m]->spin(running && (only == 5 || m == only - 1) ? pwm : 0);
    if (!running) return false;
    if (only == 5) {
        // Interrupts only: no polling beside them, so the counts show what the pin interrupts
        // keep up with on their own with every encoder line toggling.
        toolWait(1000);
        float r[4];
        for (int m = 0; m < 4; m++) r[m] = enc[m]->getRPM();
        const float bat = packVolts();
        Serial.printf("MOTORS ALL %s RPM %8.1f %8.1f %8.1f %8.1f", dir, r[0], r[1], r[2], r[3]);
        printBat(bat);
        Serial.println();
        toolJson("motor", 5, dir, envInt("tm_pct", 25), r, nullptr, bat);
        return true;
    }
    unsigned edges[4][2];
    int level[4][2];
    pollEncoders(1000, edges, level);
    float r[4];
    for (int m = 0; m < 4; m++) r[m] = enc[m]->getRPM();
    const float bat = packVolts();
    // The wheel's RPM, the pack, then how often each encoder line (A/B) changed level in that
    // second, every wheel's: only the running wheel's pair should move.
    Serial.printf("MOTOR %d %s  RPM %7.1f", only, dir, r[only - 1]);
    printBat(bat);
    Serial.printf("  ENC A/B  m1 %u/%u  m2 %u/%u  m3 %u/%u  m4 %u/%u\n",
                  edges[0][0], edges[0][1], edges[1][0], edges[1][1],
                  edges[2][0], edges[2][1], edges[3][0], edges[3][1]);
    toolJson("motor", only, dir, envInt("tm_pct", 25), r, edges, bat);
    return true;
}

// `enc`: the motors off, each encoder's count once a second -- the count as the firmware
// reads it, with the config's invert. Turn a wheel by hand: forward must count up. Ten
// marked turns give counts_per_rev (the difference over ten).
static int32_t enc_last[4];

void encBegin(void)
{
    EncoderInterface *enc[4] = {motor1_encoder, motor2_encoder, motor3_encoder, motor4_encoder};
    for (int m = 0; m < 4; m++) { motor_controllers[m]->spin(0); enc_last[m] = enc[m]->count(); }
    Serial.println("ENC counts, motors off: turn a wheel by hand -- forward must count up. "
                   "'stop' ends it.");
}

bool encStep(void)
{
    toolWait(1000);
    EncoderInterface *enc[4] = {motor1_encoder, motor2_encoder, motor3_encoder, motor4_encoder};
    int32_t c[4];
    for (int m = 0; m < 4; m++) c[m] = enc[m]->count();
    Serial.printf("ENC  m1 %ld (%+ld)  m2 %ld (%+ld)  m3 %ld (%+ld)  m4 %ld (%+ld)\n",
                  (long)c[0], (long)(c[0] - enc_last[0]), (long)c[1], (long)(c[1] - enc_last[1]),
                  (long)c[2], (long)(c[2] - enc_last[2]), (long)c[3], (long)(c[3] - enc_last[3]));
    Serial.printf("[TOOL_JSON] {\"cmd\":\"enc\",\"count\":[%ld,%ld,%ld,%ld]}\n",
                  (long)c[0], (long)c[1], (long)c[2], (long)c[3]);
    for (int m = 0; m < 4; m++) enc_last[m] = c[m];
    return true;
}

// `motors [pct]`: each motor in turn, forward 8 s, then the same in reverse, then done.
static unsigned tk = 0;

void begin(void)
{
    readEncPins();
    total_motors = tool_hw::total_motors;
    tk = 0;
}

bool step() {
    // One pass forward and one in reverse, then done: a test that never ended was a robot
    // left driving its wheels after everyone walked away.
    if (tk >= 2 * 8 * total_motors) return false;
    const unsigned run_time = 8; // run time of each motor
    const unsigned cycle = run_time * total_motors;
    unsigned current_motor = tk / run_time % total_motors;
    unsigned direction = tk / cycle % 2; // 0 forward, 1 reverse
    const int pwm_max = testPwm();
    static float max_rpm, stopping;
    if (tk == 0) {
        Serial.printf("MOTOR TEST pwm %d of %d (tm_pct %d)\n", pwm_max,
                      (1 << envU16("pwm_bits", PWM_BITS)) - 1, envInt("tm_pct", 25));
    }

    setLed(direction ? LOW : HIGH);
    motor1_controller->spin((current_motor == 0) ? (direction ? -pwm_max : pwm_max) : 0);
    motor2_controller->spin((current_motor == 1) ? (direction ? -pwm_max : pwm_max) : 0);
    motor3_controller->spin((current_motor == 2) ? (direction ? -pwm_max : pwm_max) : 0);
    motor4_controller->spin((current_motor == 3) ? (direction ? -pwm_max : pwm_max) : 0);

    unsigned edges[4][2];
    int level[4][2];
    pollEncoders(1000, edges, level);
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
        float max_linear_speed = max_rpm / 60.0 * PI * envFloat("wheel_d", WHEEL_DIAMETER); // m/s = rps * circumference

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
    // Level changes per line in that second (A/B) and the levels at its end.
    char lv[4][3];
    for (int m = 0; m < 4; m++)
        for (int c = 0; c < 2; c++) lv[m][c] = level[m][c] < 0 ? '-' : (char)('0' + level[m][c]);
    for (int m = 0; m < 4; m++) lv[m][2] = '\0';
    Serial.printf("ENC POLL %u/%u %u/%u %u/%u %u/%u  levels %s %s %s %s\n",
           edges[0][0], edges[0][1], edges[1][0], edges[1][1],
           edges[2][0], edges[2][1], edges[3][0], edges[3][1], lv[0], lv[1], lv[2], lv[3]);
    {
        const float r[4] = {current_rpm1, current_rpm2, current_rpm3, current_rpm4};
        toolJson("motors", (int)current_motor + 1, direction ? "REV" : "FWD", envInt("tm_pct", 25),
                 r, edges, packVolts());
    }
    tk++;
    return true;
}

}  // namespace test_motors

extern "C" void test_motors_begin(void)     { test_motors::begin(); }
extern "C" bool test_motors_step(void)      { return test_motors::step(); }
extern "C" void test_motors_one_begin(void) { test_motors::oneBegin(); }
extern "C" bool test_motors_one_step(void)  { return test_motors::oneStep(); }
extern "C" void test_motors_enc_begin(void) { test_motors::encBegin(); }
extern "C" bool test_motors_enc_step(void)  { return test_motors::encStep(); }
