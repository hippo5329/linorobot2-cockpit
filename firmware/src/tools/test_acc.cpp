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
sensor_msgs__msg__Imu *imu_msg = nullptr;



// The gains, the kinematics and the PWM width come from the env, through the
// factory, like main.cpp. They were file-scope objects built from the generated
// header's macros: one image serves every robot of a family, so on a real robot
// (ts100_gendrv, 2026-10-08) the closed-loop steps ran the header's gains, the
// velocity table used a 0.043 m wheel and 180 RPM, and the base type was the
// header's -- an identification of a robot that does not exist.
PID *motor_pids[4] = {};
Kinematics *kinematics = nullptr;

// The PWM width the motors were built with (createMotor reads the same key).
static int pwmMax() { return (1 << envU16("pwm_bits", PWM_BITS)) - 1; }

// The console. On a Wi-Fi robot main.cpp tees a tool's console to syslog, so a
// robot on the ground, off its cable, still reports every line.
#define REPORT(...) Serial.printf(__VA_ARGS__)

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
bool imu_ok = false;     // init() answered: only then is a gyro heading anything
MAGInterface *mag = nullptr;
unsigned total_motors = 4;

// Set when the env says the wheels are simulated. loop_() then prints where to
// get the answer instead of driving nothing and tabulating the result.
bool sim_wheels = false;

// The host's stop (ota.h, 2026-10-08: the TS100 ran into a wall during this test
// and nothing could stop it short of the env). Every wait in this tool goes
// through waitMs(), which services the radio -- so the stop datagram is read
// within milliseconds, mid-step -- and halts on it. Halted means motors off and
// nothing else, for good: the radio and OTA stay up so the host can write
// app=base, which is the only way out. Not a return: every caller would have to
// check, and one that forgot would drive on.
static void halt(const char *why)
{
    motor1_controller->spin(0);
    motor2_controller->spin(0);
    motor3_controller->spin(0);
    motor4_controller->spin(0);
    Serial.printf("[test_acc] STOPPED: %s -- motors off; write app=base to leave\n", why);
    for (;;) {
        runWifis();
        runOta();
        feedWatchdogFromTool();
        delay(20);
    }
}

// The robot's heading, from the IMU's gyro -- not from the wheels (user, 2026-10-08:
// "use imu heading to return home"). A track skids in a turn, so the wheels' yaw is
// what they were told, not what the body did: run 3 ended 0.6 deg off by its wheels
// and 10 deg off by the room. getData() is bias-corrected with a 0.01 rad/s deadband;
// the board's Z is up (test_sensors: +9 m/s2), so yaw rate is the chip's Z. Sampled
// from every wait, ~5 ms apart, so a 4.7 rad/s spin is integrated in 1.4 deg steps.
static float imu_heading = 0.0f;
static uint32_t heading_us = 0;
static void trackHeading(void)
{
    if (!imu || !imu_ok) return;
    const uint32_t now = micros();
    const float wz = (float)imu->getData().angular_velocity.z;
    if (heading_us) imu_heading += wz * (float)(now - heading_us) * 1e-6f;
    heading_us = now;
}

static void waitMs(uint32_t ms)
{
    const uint32_t end = millis() + ms;
    for (;;) {
        trackHeading();
        runWifis();
        runOta();
        if (hostStopRequested())
            halt("stop from the host");
        const int32_t left = (int32_t)(end - millis());
        if (left <= 0)
            return;
        delay(left < 5 ? (uint32_t)left : 5u);
    }
}

// Where the robot is, from its own wheels (user, 2026-10-08: "change it so that it
// return to home"). EVERY encoder read in this tool goes through readRPM(): getRPM()
// is the ticks since the previous call over the time since then, so rpm x interval
// is exactly the travel, and a read that bypassed this would lose its interval's.
// The position is the wheels' straight-line travel laid along the IMU heading; the
// wheels' own yaw (odom_th) is kept only to print beside it. A skid turns the body
// without moving it, so it costs the heading, which the gyro has, and not the
// position. The host's LiDAR before/after comparison is the judge of both.
static float wheel_net_rev[4] = {};
static uint32_t wheel_read_us[4] = {};
static float home_x = 0.0f, home_y = 0.0f, odom_th = 0.0f;

static EncoderInterface *encN(int i)
{
    return (i == 0) ? motor1_encoder : (i == 1) ? motor2_encoder
         : (i == 2) ? motor3_encoder : motor4_encoder;
}

static float readRPM(int i)
{
    const float rpm = encN(i)->getRPM();
    const uint32_t now = micros();
    if (wheel_read_us[i]) {
        const float dt = (float)(now - wheel_read_us[i]) * 1e-6f;
        wheel_net_rev[i] += rpm / 60.0f * dt;
        if (kinematics) {
            float r[4] = {0.0f, 0.0f, 0.0f, 0.0f};
            r[i] = rpm;
            const Kinematics::velocities v = kinematics->getVelocities(r[0], r[1], r[2], r[3]);
            const float th = imu_ok ? imu_heading : odom_th;
            odom_th += v.angular_z * dt;
            home_x += (v.linear_x * cosf(th) - v.linear_y * sinf(th)) * dt;
            home_y += (v.linear_x * sinf(th) + v.linear_y * cosf(th)) * dt;
        }
    }
    wheel_read_us[i] = now;
    return rpm;
}

static float wheelNetM(int i)
{
    return wheel_net_rev[i] * (float)PI * envFloat("wheel_d", WHEEL_DIAMETER);
}

static void reportHome(const char *phase)
{
    REPORT("HOME %s x=%.3f y=%.3f yaw_imu=%.1f yaw_wheels=%.1f net_m=%.3f,%.3f,%.3f,%.3f\n", phase,
           home_x, home_y, imu_heading * 57.29578f, odom_th * 57.29578f,
           wheelNetM(0), wheelNetM(1), wheelNetM(2), wheelNetM(3));
}

void setup_()
{
    // Allocated when this tool runs, not statically (see test_sensors).
    if (!imu_msg) imu_msg = (sensor_msgs__msg__Imu *)calloc(1, sizeof(*imu_msg));
    if (!imu_msg) {
        Serial.println("[test_acc] out of memory for the IMU message buffer");
        return;
    }

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
    initOta(NULL, NULL);   // a no-op: main.cpp started OTA, with its hooks
    // The LiDAR's UDP forwarder, as in `base` (user, 2026-10-08: "include lidar udp in
    // the test loop"): the robot computer tracks the whole trajectory from the room while
    // this tool drives. It forwards from the UART's receive callback, so the tool's loops
    // do not have to service it; with no real LiDAR in the env it stays off.
    initLidar();

    i2cScanTable();

    // The same probe and the same adoption rule the robot firmware uses, so this
    // tool and `base` can never disagree about what is fitted.
    initMcuEnv();
    const char *imu_name = envGet("imu", defaultIMUName());
    const char *mag_name = envGet("mag", defaultMAGName());
    i2cProbeSelect(&imu_name, &mag_name);
    imu = createIMU(imu_name);
    mag = createMAG(mag_name);

    imu_ok = imu->init();
    if (!imu_ok)
        Serial.println("[-] IMU initialization FAILED -- IMU ACC below is not a measurement.");
    else
        Serial.printf("[+] IMU %s initialized.\n", imu_name);
    mag->init();

    for (int i = 0; i < 4; i++) motor_pids[i] = createPID();
    kinematics = createKinematics();

    // A simulated-wheel board has nothing to measure. Checked here, after
    // initMcuEnv(), because it is the env that decides -- not the build.
    sim_wheels = wheelsAreSim();

    if (kinematics->base_platform_ == Kinematics::DIFFERENTIAL_DRIVE)
    {
        total_motors = 2;
    }
    for (int i = 0; i < 4; i++) readRPM(i);   // the baseline: travel counts from here

    initBoardLate();
    syslog(LOG_INFO, "%s Ready %lu", __FUNCTION__, millis());
    waitMs(2000);
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
        float rpm1 = readRPM(0);
        float rpm2 = readRPM(1);
        float rpm3 = readRPM(2);
        float rpm4 = readRPM(3);
        *imu_msg = imu->getData();
        float imu_acc_x = imu_msg->linear_acceleration.x;
        if (imu_acc_x > imu_max_acc_x) imu_max_acc_x = imu_acc_x;
        if (imu_acc_x < imu_min_acc_x) imu_min_acc_x = imu_acc_x;

        if (idx < buf_size) {
            buf[idx] = kinematics->getVelocities(rpm1, rpm2, rpm3, rpm4);
        }
        waitMs(ticks);
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


// Spin the motors. Four calls, one place, because five call sites drifted.
//
// It does NOT drive the simulated wheels, and that is deliberate: this tool is
// for a robot with motors on it. SimEncoder::feed() was called here for a
// while so that a simulation-mode run produced a full table instead of zeros -- but
// the table it produced was a measurement OF THE SIMULATOR, taken on an MCU,
// over a serial line, after a flash. The simulator is a host-side model whose
// constants live in sim_wheel.h, so the honest way to read it is to run it on
// the host: scripts/drivetrain_report.py steps the same equations on
// test_acc's own 20 ms / 1 s profile and prints the same four lines in
// milliseconds, with no board involved.
//
// So a simulated-wheel board is refused in setup_() rather than answered. What is
// left here is the real measurement: a real motor's real acceleration, which is
// the only thing a board can tell you that the model cannot.
static void driveAll(int pwm1, int pwm2, int pwm3, int pwm4)
{
    motor1_controller->spin(pwm1);
    motor2_controller->spin(pwm2);
    motor3_controller->spin(pwm3);
    motor4_controller->spin(pwm4);
}

// ==============================================================================
// Identification: what a host needs to TUNE this robot, measured on this robot.
//
// The open-loop table below (MAX VEL / MAX ACC / time to 0.9x) is a step
// response, and scripts/drivetrain_report.py already fits a first-order plant to
// it and hands back IMC gains. Two things that aggregate table cannot answer,
// and both of them decide whether the gains are right:
//
//   PER WHEEL. The table averages four wheels into one velocity. Four real
//   wheels have different friction, different gearboxes and, on a used robot,
//   different wear -- and the model assumes they do not differ. If they do, one
//   set of gains is wrong for three of them, and nothing anywhere would say so.
//
//   CLOSED LOOP. A plant fit says what the motor does when you shove it. It does
//   not say whether the PID around it is stable, and the simulated check on the
//   host cannot either: it knows nothing about this robot's backlash, its
//   encoder quantisation or the load on its wheels. Overshoot and setpoint
//   crossings have to be measured with the loop actually closed.
//
// Everything here is accumulated ONLINE -- running peak, crossing count,
// settling time -- and never stored as a trace. A per-wheel trace at the control
// rate would be several kilobytes on a board whose whole static segment is
// 124580 bytes, and the summary is what gets used anyway.
//
// The output is deliberately machine-readable and one fact per line, because it
// is read by a script and pasted into issues by people:
//
//   IDENT gains kp=... ki=... kd=... pwm_max=... rate_hz=...
//   IDENT deadzone wheel=1 pwm=... duty=...
//   IDENT plant wheel=1 steady_rpm=... tau_ms=... K=...
//   IDENT loop wheel=1 sp=... overshoot=... crossings=... settle_ms=... err=...
// ==============================================================================
namespace ident {

const unsigned TICK_MS = 20;            // CONTROL_TIMER: the loop's own period

EncoderInterface *enc(int i)
{
    return (i == 0) ? motor1_encoder : (i == 1) ? motor2_encoder
         : (i == 2) ? motor3_encoder : motor4_encoder;
}

MotorInterface *mot(int i)
{
    return (i == 0) ? motor1_controller : (i == 1) ? motor2_controller
         : (i == 2) ? motor3_controller : motor4_controller;
}

PID *pid(int i)
{
    return motor_pids[i];
}

void stopAll()
{
    for (unsigned i = 0; i < total_motors; i++) mot(i)->spin(0);
    waitMs(700);                          // let the wheels actually stop
}

// The identification drives ALL wheels at once, the same command on each: a step
// is the robot driving straight ahead, then the same straight back, and every
// wheel is measured in parallel, on its own encoder (user, 2026-10-08: "test acc
// should fast forward, fast backward, then rotate in both direction ... Fix this
// with odom"). One wheel at a time, forward only, was a robot pivoting on one
// track and then the other -- crawling ahead in arcs, ~2 m into a wall. The
// reverse half reports as `<kind>_rev`, which drivetrain_report.py's parser does
// not read; wheel odometry (readRPM) brings the robot home after the phase.

static void spinAll(const int *pwm)
{
    for (unsigned i = 0; i < total_motors; i++) mot(i)->spin(pwm[i]);
}

// The smallest PWM that turns each wheel. SIM_WHEEL_STALL_DUTY is a guess at 4%;
// this is the real number, and it differs per wheel because stiction does. All
// wheels ramp together; each one's figure is where its OWN encoder first moves.
void deadzone()
{
    const int pwm_max = pwmMax();
    for (int dir = 1; dir >= -1; dir -= 2) {
        int found[4] = {-1, -1, -1, -1};
        unsigned left = total_motors;
        for (int pwm = 0; pwm <= pwm_max / 2 && left; pwm += pwm_max / 100) {
            int cmd[4];
            for (unsigned i = 0; i < 4; i++) cmd[i] = dir * pwm;
            spinAll(cmd);
            waitMs(120);
            // Two samples: the first getRPM() after a stop can still carry the
            // previous motion on a filtered encoder.
            for (unsigned i = 0; i < total_motors; i++) readRPM(i);
            waitMs(80);
            for (unsigned i = 0; i < total_motors; i++)
                if (found[i] < 0 && fabs(readRPM(i)) > 2.0) { found[i] = pwm; left--; }
        }
        stopAll();
        for (unsigned i = 0; i < total_motors; i++)
            REPORT("IDENT %s wheel=%u pwm=%d duty=%.3f\n", dir > 0 ? "deadzone" : "deadzone_rev",
                   i + 1, found[i], found[i] < 0 ? -1.0 : (float)found[i] / (float)pwm_max);
    }
}

// Open-loop step: the plant the PID has to control. Full PWM on every wheel for
// 0.6 s -- the robot's own straight-line sprint -- then the same backwards. Short
// for the floor's sake (user, 2026-10-08: "or we need to shorten motor run"): the
// TS100's tau is 20-40 ms, so 0.6 s is >10 tau and the last sample is steady.
//
// tau is the 63.2% crossing, which is the definition for a first-order step, and
// K is rpm per PWM count -- the units the loop gain is the inverse of.
void plant()
{
    const int pwm_max = pwmMax();
    for (int dir = 1; dir >= -1; dir -= 2) {
        static float samples[4][30];
        float peak[4] = {0, 0, 0, 0};
        int cmd[4];
        for (unsigned i = 0; i < total_motors; i++) readRPM(i);
        for (unsigned i = 0; i < 4; i++) cmd[i] = dir * pwm_max;
        spinAll(cmd);
        for (unsigned t = 0; t < 30; t++) {          // 0.6 s at 20 ms
            waitMs(TICK_MS);
            for (unsigned i = 0; i < total_motors; i++) {
                samples[i][t] = fabs(readRPM(i));
                if (samples[i][t] > peak[i]) peak[i] = samples[i][t];
            }
        }
        stopAll();
        for (unsigned i = 0; i < total_motors; i++) {
            const float steady = samples[i][29];
            unsigned tau_ticks = 0;
            for (unsigned t = 0; t < 30; t++) {
                if (samples[i][t] >= steady * 0.632f) { tau_ticks = t; break; }
            }
            REPORT("IDENT %s wheel=%u steady_rpm=%.1f tau_ms=%u K=%.5f peak_rpm=%.1f\n",
                   dir > 0 ? "plant" : "plant_rev", i + 1, steady, tau_ticks * TICK_MS,
                   pwm_max > 0 ? steady / (float)pwm_max : 0.0f, peak[i]);
        }
    }
}

// Closed loop: is the PID in this config actually stable on each wheel? Every
// wheel's own loop runs at once, the robot driving straight at the setpoint.
//
// Overshoot alone is not enough -- a loop can creep past the setpoint once and
// settle, or cross it repeatedly by a hair and never settle. The crossing count
// is the second test, and it is the one that catches ringing. 1.5 s, not 3: at
// 90% of top that is ~0.5 m of floor each way (user, 2026-10-08: "or we need to
// shorten motor run"), and the TS100's loops settled in ~0.5 s; the steady error
// is still the last 0.5 s.
void loopStep(float magnitude_rpm)
{
    const unsigned ticks = 75;                       // 1.5 s
    for (int dir = 1; dir >= -1; dir -= 2) {
        const float setpoint_rpm = dir * magnitude_rpm;
        // Start from a known state. The integral carries between steps, so the
        // second setpoint would be measured on a loop that is already wound --
        // and the overshoot figure would belong to the previous step.
        //
        // Done through the PID's OWN zeroing path (`setpoint == 0` with the
        // wheel stopped clears integral and derivative) rather than by adding a
        // reset method: this is the state the robot is in every time it stops,
        // so the identification starts where real driving starts.
        for (unsigned t = 0; t < 30; t++) {
            for (unsigned i = 0; i < total_motors; i++)
                mot(i)->spin((int)pid(i)->compute(0.0f, readRPM(i)));
            waitMs(TICK_MS);
        }
        float peak[4] = {0, 0, 0, 0}, last[4] = {0, 0, 0, 0}, err_sum[4] = {0, 0, 0, 0};
        int crossings[4] = {0, 0, 0, 0};
        unsigned settle_tick[4] = {ticks, ticks, ticks, ticks};
        bool was_below[4] = {true, true, true, true};
        for (unsigned t = 0; t < ticks; t++) {
            for (unsigned i = 0; i < total_motors; i++) {
                const float rpm = readRPM(i);
                mot(i)->spin((int)pid(i)->compute(setpoint_rpm, rpm));
                if (fabs(rpm) > peak[i]) peak[i] = fabs(rpm);
                const bool below = rpm < setpoint_rpm;
                if (t > 0 && below != was_below[i]) crossings[i]++;
                was_below[i] = below;
                // Settling: the LAST tick outside the 2% band, so a curve that
                // wanders back out is not called settled at its first touch.
                if (fabs(rpm - setpoint_rpm) > fabs(setpoint_rpm) * 0.02f)
                    settle_tick[i] = t + 1;
                if (t >= ticks - 25) err_sum[i] += (setpoint_rpm - rpm);   // last 0.5 s
                last[i] = rpm;
            }
            waitMs(TICK_MS);
        }
        stopAll();
        for (unsigned i = 0; i < total_motors; i++) {
            const float over = (peak[i] - fabs(setpoint_rpm)) / fabs(setpoint_rpm);
            // -1, not 0, for "never settled". Zero reads as "settled instantly",
            // which is the opposite of what it means and is exactly the wheel you
            // most want to notice.
            REPORT("IDENT %s wheel=%u sp=%.1f overshoot=%.3f crossings=%d "
                   "settle_ms=%d err=%.2f final=%.1f\n",
                   dir > 0 ? "loop" : "loop_rev", i + 1, setpoint_rpm, over, crossings[i],
                   settle_tick[i] >= ticks ? -1 : (int)(settle_tick[i] * TICK_MS),
                   err_sum[i] / 25.0f, last[i]);
        }
    }
}

// Where the wheels and the gyro think the robot is, printed after each phase. The
// robot does NOT drive itself home: that moved to the robot computer (user,
// 2026-10-08: "move home and trajectory to ros2"), which tracks the whole run from
// the room with the LiDAR this tool now forwards and drives home on /cmd_vel once the
// base app is back. These lines are the board's witness beside it -- on tracks the
// wheels' yaw is fiction (run 4: 49.8 deg by the gyro, 0.4 by the wheels).
void reportPhase(const char *phase)
{
    reportHome(phase);
}

void run()
{
    REPORT("IDENT gains kp=%.4f ki=%.4f kd=%.4f pwm_max=%d rate_hz=%u\n",
                  envFloat("kp", K_P), envFloat("ki", K_I), envFloat("kd", K_D), pwmMax(),
                  1000 / TICK_MS);
    REPORT("IDENT robot base=%d wheels=%u max_rpm=%d ratio=%.3f wheel_d=%.4f\n",
                  (int)kinematics->base_platform_, total_motors, (int)envU16("max_rpm", MOTOR_MAX_RPM),
                  envFloat("rpm_ratio", MAX_RPM_RATIO), envFloat("wheel_d", WHEEL_DIAMETER));
    stopAll();
    deadzone();
    plant();
    // Three setpoints, because a wheel loop is not linear: stiction and the
    // dead zone dominate at a crawl, the PWM rail and the pack's sag near the
    // top. Gains that are calm at cruise and ring at a crawl are not calm.
    const float top = (float)envU16("max_rpm", MOTOR_MAX_RPM) * envFloat("rpm_ratio", MAX_RPM_RATIO);
    loopStep(top * 0.20f);
    loopStep(top * 0.55f);
    loopStep(top * 0.90f);
    stopAll();
    reportPhase("ident");
    REPORT("IDENT done\n");
}

}  // namespace ident

void loop_() {
    if (!imu_msg) return;   // setup_ could not allocate; nothing to run

    // Refuse rather than answer. With simulated wheels there is no motor to
    // accelerate: every number below would be a property of the simulated
    // drivetrain in sim_wheel.h, measured the hard way. The host runs that
    // model directly, on this tool's own 20 ms / 1 s profile, from the robot's
    // config -- including the terms a board cannot vary without a reflash
    // (mass, gear efficiency, pack sag, driver losses).
    //
    // Saying so once and stopping is the point. Printing a table would invite
    // somebody to tune a velocity smoother from a number that describes a
    // simulator, and the wiki tells people to tune it from this output.
    if (sim_wheels) {
        Serial.println("[test_acc] the env says these wheels are simulated "
                       "(sim_wheel=1), so there is nothing here to measure.");
        Serial.println("[test_acc] the same model, run on the host, with this "
                       "robot's config:");
        Serial.println("[test_acc]     python3 scripts/drivetrain_report.py "
                       "--params <robot>_config.yaml");
        Serial.println("[test_acc] or open the Config Studio's Kinematics HUD. "
                       "For a real measurement, flash a robot with motors.");
        syslog(LOG_INFO, "test_acc refused: sim_wheel=1, nothing to measure");
        delay(10000);
        return;
    }

    // Identification first, while the motors are cold and the pack is full:
    // once, not once per run, because the twelve runs below halve the PWM as
    // they go and a plant fitted across different step sizes is not a plant.
    static bool identified = false;
    if (!identified) {
        identified = true;
        ident::run();
    }

    // The velocity trace lives here, on the stack, for exactly as long as the
    // test runs. 2400 bytes of the loop task's 8192, and this tool is the only
    // thing on that task: selecting an app replaces the base loop, it does not
    // run alongside it.
    Kinematics::velocities buf[buf_size] = {};
    const int pwm_max = pwmMax();
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
        // The spins (even runs) at half the straight runs' PWM: at full PWM the
        // TS100's tracks skidded the body 10 deg further than they said (2026-10-08).
        const float spin = (runs & 1) ? 1.0f : 0.5f;
        driveAll((runs & 1) ? current_pwm_max : current_pwm_min * spin, current_pwm_max * spin,
                 (runs & 1) ? current_pwm_max : current_pwm_min * spin, current_pwm_max * spin);
        record(run_time / ticks, buf);

#ifdef LED_ACTIVE
        digitalWrite(LED_PIN, LOW);
#endif
        driveAll(0, 0,
                 0, 0);
        record(run_time / ticks, buf);

#ifdef LED_ACTIVE
        digitalWrite(LED_PIN, HIGH);
#endif
        driveAll((runs & 1) ? current_pwm_min : current_pwm_max * spin, current_pwm_min * spin,
                 (runs & 1) ? current_pwm_min : current_pwm_max * spin, current_pwm_min * spin);
        record(run_time / ticks, buf);

#ifdef LED_ACTIVE
        digitalWrite(LED_PIN, LOW);
#endif
        driveAll(0, 0,
                 0, 0);
        record(run_time / ticks, buf);

        Serial.printf("MAX PWM %6.1f %6.1f\n", current_pwm_max * spin, current_pwm_min * spin);
        dump_record(buf);
        if ((runs & 3) == 0) {
            current_pwm_max /= 2;
            current_pwm_min /= 2;
        }
    }
    // The runs come back by construction (each drives out and the same back), but
    // a track that runs faster one way still walks; take out what is left, once.
    static bool homed = false;
    if (!homed) {
        homed = true;
        ident::reportPhase("runs");
        Serial.println("[test_acc] done -- write app=base to leave");
    }

    waitMs(100);
}

}  // namespace test_acc

extern "C" void test_acc_setup(void) { test_acc::setup_(); }
extern "C" void test_acc_loop(void)  { test_acc::loop_(); }
