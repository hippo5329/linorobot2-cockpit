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
#include <Arduino.h>
#ifdef ESP32
#include <esp_system.h>   // esp_reset_reason(), see printResetReason()
#endif
#include <micro_ros_platformio.h>
#include <stdio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <micro_ros_utilities/string_utilities.h>

#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/magnetic_field.h>
#include <sensor_msgs/msg/battery_state.h>
#include <sensor_msgs/msg/range.h>
#include <sensor_msgs/msg/fluid_pressure.h>
#include <sensor_msgs/msg/temperature.h>
#include <sensor_msgs/msg/relative_humidity.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/u_int8_multi_array.h>
#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/twist_stamped.h>
#include <geometry_msgs/msg/vector3.h>

#include "config.h"
#include "syslog.h"
#include "motor.h"
#include "kinematics.h"
#include "pid.h"
#include "odometry.h"
#include "imu.h"
#include "mag.h"
#include "sensor_factory.h"
#include "uros_transport.h"
#include "board_init.h"
#include "tools/tools.h"
#include "hw_factory.h"
#include "mcu_env.h"
#if defined(ESP32) || defined(ARDUINO_ARCH_ESP32)
#include <esp_bt.h>
static esp_err_t bt_release_err = ESP_FAIL;
static uint32_t  bt_heap_before = 0, bt_heap_after = 0;
#endif
#include "i2c_probe.h"
#define ENCODER_USE_INTERRUPTS
#define ENCODER_OPTIMIZE_INTERRUPTS
#include "encoder.h"
#include "fake_wheel.h"
#include "fake_ld19.h"
// The synthetic scan can reach the host three ways, and running two at once
// wastes a link that has no headroom to spare -- so exactly one is chosen, at
// BOOT rather than at build time. The publisher below is compiled into every
// image that has the emulator; whether it is created and fed is decided by
// `lidar_comm` in the env, defaulting to LIDAR_COMM_DEFAULT (the build's own
// comm_mode).
//
// This used to be a compile-time gate keyed on LIDAR_RXD and USE_LIDAR_UDP,
// which made the transport a property of the image: the released esp32 image
// is built `udp`, so flashing it onto the gendrv bench -- wired for a serial
// LD19 into a USB-serial bridge -- gave a board that could not be told to use
// the UART, whatever its env said.
#include "battery.h"
#include "range.h"
#include "env.h"
#include "lidar.h"
#include "wifis.h"
#include "ota.h"
#include "diag.h"

// The local set_microros_net_transports() that used to live here is gone.
// It duplicated the UDP transport and depended on micro_ros_agent_locator,
// a struct that only exists when board_microros_transport = wifi. With the
// transport set to `custom` both paths live in
// firmware/common/lib/uros_transport and are chosen at boot instead.

#ifndef LIDAR_RXD
#define LIDAR_RXD -1
#endif
#ifndef LIDAR_BAUDRATE
#define LIDAR_BAUDRATE 230400
#endif

#ifndef NODE_NAME
#define NODE_NAME "linorobot_base_node"
#endif
#ifndef TOPIC_PREFIX
#define TOPIC_PREFIX
#endif
#ifndef CONTROL_TIMER
#define CONTROL_TIMER 20 // 50Hz
#endif
#ifndef BATTERY_TIMER
#define BATTERY_TIMER 2000 // 2 sec
#endif
#ifndef RANGE_TIMER
#define RANGE_TIMER 100 // 10Hz
#endif
#ifndef ENV_TIMER
#define ENV_TIMER 1000 // 1Hz (BMP280/BME280 pressure/temperature/humidity)
#endif

#ifndef RCCHECK
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){rclErrorLoop();}}
#endif
// A discarded return is how a robot goes quiet without anyone noticing. Every
// rcl_publish() in this firmware is wrapped in RCSOFTCHECK, and the empty body
// below meant a board whose publishes all failed looked exactly like a board
// that was publishing -- session up, topics advertised, no data and no
// complaint. Count the failures and say so once a second: rate-limited, because
// a failing publish at 50 Hz would otherwise flood the very link that is sick.
extern void rcSoftFail(int line, int code);
#ifndef RCSOFTCHECK
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; \
    if((temp_rc != RCL_RET_OK)){ rcSoftFail(__LINE__, (int)temp_rc); }}
#endif
#define EXECUTE_EVERY_N_MS(MS, X)  do { \
  static volatile int64_t init = -1; \
  if (init == -1) { init = uxr_millis();} \
  if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
} while (0)

// Same as EXECUTE_EVERY_N_MS but the FIRST fire is delayed by PHASE ms. All
// plain EXECUTE_EVERY_N_MS blocks seed their timer on the same publishData()
// call, so low-rate publishers (battery, sonar, barometer) keep landing on the
// same 50 Hz control cycle and burst into one serialization window together
// with imu/odom/mag. Giving each a distinct PHASE (a multiple of CONTROL_TIMER,
// and not a common divisor of the periods) spreads them onto different cycles.
#define EXECUTE_EVERY_N_MS_PHASED(MS, PHASE, X)  do { \
  static volatile int64_t init = -1; \
  if (init == -1) { init = uxr_millis() - (int64_t)(MS) + (int64_t)(PHASE);} \
  if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
} while (0)

// Whether /imu/mag exists is decided at BOOT, not at build time. A released
// image is built for an MCU, not for a robot, so "does this robot have a
// magnetometer" cannot be a macro: the answer is whichever chip answered the
// I2C probe, plus fake wheel mode, which synthesises a real field from the
// simulated heading -- hard-iron bias and all -- and is exactly what a
// calibration run needs even though no chip is present.
static bool publish_mag = false;
// QoS of the 50 Hz sensor topics (odom/unfiltered, imu/data_raw, imu/mag):
// best effort, the ROS convention for sensor data, and measured to matter.
// A reliable rmw_publish on a serial link waits in
// uxr_run_session_until_confirm_delivery for an agent round trip per
// message; on the GenDrv, one core, that was 42 Hz at 1.5 Mbaud and 25/33 Hz
// at 921600 -- best effort holds 50/50 at both, and imu_filter_madgwick
// and robot_localization subscribe best-effort already, so the launch tree
// receives (measured: /imu/data 49.97 Hz, /odom 49.0 Hz). A best-effort
// writer does NOT match a reliable subscriber: `qos: reliable` in the
// config (env best_effort=0) is for a consumer that insists on it. Needs
// UCLIENT_CUSTOM_TRANSPORT_MTU >= 1024 (the user metas) so an Odometry
// message fits one best-effort frame.
static bool best_effort = true;

rcl_publisher_t odom_publisher;
rcl_publisher_t imu_publisher;
rcl_publisher_t mag_publisher;
#ifdef USE_STAMPED_CMD_VEL
rcl_subscription_t twist_stamped_subscriber;
geometry_msgs__msg__TwistStamped twist_stamped_msg;
static char twist_stamped_frame_id[64];
#endif
rcl_subscription_t twist_subscriber;
rcl_publisher_t battery_publisher;
// The forward hazard stop. `USE_SAFETY_STOP` guarded all of this and was
// emitted by nothing -- not gen_firmware_header.py, not platformio.ini, not any
// config -- so the reflex has never been compiled into a single image. It is an
// env flag now (`safety_stop`), and it defaults OFF: it brakes the robot, and a
// behaviour that has never run on hardware should be asked for explicitly
// rather than arrive with a firmware update.
rcl_publisher_t safety_stop_publisher;
std_msgs__msg__Bool safety_stop_msg;
bool safety_stopped = false;
static bool safety_stop_on = false;
static float safety_stop_range = 0.25f;   // metres ahead before forward motion is cut
rcl_publisher_t range_publisher;
rcl_publisher_t pressure_publisher;
rcl_publisher_t temperature_publisher;
rcl_publisher_t humidity_publisher;
sensor_msgs__msg__FluidPressure pressure_msg;
sensor_msgs__msg__Temperature temperature_msg;
sensor_msgs__msg__RelativeHumidity humidity_msg;
static bool env_present = false;
static bool publish_env = false;
static bool publish_battery = false;

nav_msgs__msg__Odometry odom_msg;
sensor_msgs__msg__Imu imu_msg;
sensor_msgs__msg__MagneticField mag_msg;
geometry_msgs__msg__Twist twist_msg;
sensor_msgs__msg__BatteryState battery_msg;
sensor_msgs__msg__Range range_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t control_timer;

unsigned long long time_offset = 0;
unsigned long prev_cmd_time = 0;
unsigned long prev_odom_update = 0;
float prev_voltage;

// USE_ESP32_DUAL_CORE is a CAPABILITY, not a choice: it says this silicon has a
// second core to pin a control loop to. Whether to actually use it is the env
// key `dual_core`, read at boot into the flag below.
//
// It has to be a run-time decision because the critical section it installs is
// not free. controlTask holds portENTER_CRITICAL(&controlMux) around moveBase()
// every CONTROL_TIMER ms, and on an ESP32 that disables interrupts on core 0 --
// the core the Wi-Fi driver and LwIP run on. On a released image, built for an
// MCU rather than for a robot, baking that in means every robot on that image
// pays for it whether or not its control loop needs the isolation.
#if defined(ESP32) && !defined(CONFIG_FREERTOS_UNICORE)
#define USE_ESP32_DUAL_CORE 1
TaskHandle_t controlTaskHandle = NULL;
portMUX_TYPE controlMux = portMUX_INITIALIZER_UNLOCKED;
void controlTask(void *pvParameters);
#ifdef USE_DUAL_CORE
static const bool DUAL_CORE_DEFAULT = true;
#else
static const bool DUAL_CORE_DEFAULT = false;
#endif
static bool dual_core = false;
#endif

enum states 
{
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

// Compiled in unconditionally, and that is the point: whether THIS BOARD has
// wheels is a fact about the board, and a board is a configuration, not a build
// (AGENTS.md §10). It used to be `#ifdef USE_FAKE_WHEEL`, which meant the bare
// bench module and the same module with an IMU soldered on needed two different
// binaries of the same firmware -- and no env key could undo the difference,
// because an #ifdef had already removed the other path at compile time.
// It costs one small object in images that never simulate anything.
FakeIMUFromWheels fake_imu;

// wheelsAreFake() reads the env; it is read once in setup() and used from the
// control loop, which runs on the other core on ESP32.
static bool fake_wheels = false;
FakeLD19 fake_ld19;
// Whether the emulator runs at all is a robot fact, not an image fact: a
// prebuilt image is built from a fake-mode reference, and every real robot
// that flashes it would otherwise raycast a room and stream it (env key
// fake_ld19; the compiled-in default is on, so a blank env keeps the bench).
static bool fake_lidar_on = false;

// /sonar, decided at boot rather than by the build.
//
// It used to take `#if defined(ECHO_PIN) || (defined(USE_FAKE_SONAR) &&
// defined(USE_FAKE_LD19))`. Neither half was ever true in a shipped image: no
// reference config carried sonar pins, so ECHO_PIN was never defined, and
// USE_FAKE_SONAR is emitted by nothing at all. The topic has therefore never
// been published by any release, on either path.
//
// publish_range is now rangePresent() -- both pins resolved from the env -- or
// the simulated cone, which follows the emulator. range_fake says which.
static bool publish_range = false;
static bool range_fake = false;
static FakeLD19::CommMode fake_lidar_comm = FakeLD19::COMM_SERIAL;
rcl_publisher_t raw_scan_publisher;
std_msgs__msg__UInt8MultiArray raw_scan_msg;
// Heap, and only on a board that actually publishes raw_scan. Held statically
// this was 512 bytes of .bss on every image, including every serial robot whose
// scan leaves over a UART and never touches it. Allocated by initRawScan() when
// the emulator is turned on, never freed -- rcl_publish() reads it for the rest
// of the run.
#define RAW_SCAN_BATCH_CAP 512
static uint8_t *raw_scan_batch = NULL;
static size_t raw_scan_batch_len = 0;
bool raw_scan_pub_ready = false;

void initRawScan(void)
{
    if (!raw_scan_batch)
        raw_scan_batch = (uint8_t *)malloc(RAW_SCAN_BATCH_CAP);
    if (!raw_scan_batch)
        Serial.println("[lidar] no heap for the raw_scan batch — raw_scan off");
}

void onRawScanPacket(const uint8_t *pkt, size_t len)
{
    if (!raw_scan_batch) return;
    if (raw_scan_batch_len + len <= RAW_SCAN_BATCH_CAP)
    {
        memcpy(raw_scan_batch + raw_scan_batch_len, pkt, len);
        raw_scan_batch_len += len;
    }
}

void flushRawScan()
{
    if (raw_scan_batch && raw_scan_batch_len > 0)
    {
        if (raw_scan_pub_ready && state == AGENT_CONNECTED)
        {
            raw_scan_msg.data.data = raw_scan_batch;
            raw_scan_msg.data.size = raw_scan_batch_len;
            raw_scan_msg.data.capacity = RAW_SCAN_BATCH_CAP;
            (void)rcl_publish(&raw_scan_publisher, &raw_scan_msg, NULL);
        }
        raw_scan_batch_len = 0;
    }
}

// The drivetrain is built in setup() from the env partition, not here. Pins,
// wheel geometry, PID gains, the motor driver type and whether the wheels are
// real or simulated used to be macros, which is why two ESP32 robots differing
// only in wiring needed two firmwares. The flash partition holding those values
// cannot be read during static initialisation, so these are pointers.
//
// Indexed 1..4 through the arrays; the named pointers are what the control loop
// reads, kept so the loop stays legible.
EncoderInterface *motor_encoders[4] = {nullptr, nullptr, nullptr, nullptr};
MotorInterface   *motor_controllers[4] = {nullptr, nullptr, nullptr, nullptr};
PID              *motor_pids[4] = {nullptr, nullptr, nullptr, nullptr};
Kinematics       *kinematics = nullptr;

#define motor1_encoder    (*motor_encoders[0])
#define motor2_encoder    (*motor_encoders[1])
#define motor3_encoder    (*motor_encoders[2])
#define motor4_encoder    (*motor_encoders[3])
#define motor1_controller (*motor_controllers[0])
#define motor2_controller (*motor_controllers[1])
#define motor3_controller (*motor_controllers[2])
#define motor4_controller (*motor_controllers[3])
#define motor1_pid        (*motor_pids[0])
#define motor2_pid        (*motor_pids[1])
#define motor3_pid        (*motor_pids[2])
#define motor4_pid        (*motor_pids[3])

static void initDrivetrain(void)
{
    for (int i = 1; i <= 4; i++)
    {
        motor_encoders[i - 1]    = createEncoder(i);
        motor_controllers[i - 1] = createMotor(i);
        motor_pids[i - 1]        = createPID();
    }
    kinematics = createKinematics();
}

Odometry odometry;

// Pointers, constructed in setup() rather than here. The concrete driver is
// named by the env partition, and the flash partition API is not usable during
// static initialisation -- a global built here would have to be chosen at
// compile time, which is exactly what this removes.
IMUInterface *imu = nullptr;
MAGInterface *mag = nullptr;
// Set in setup() once the name is resolved (config, then env, then the bus).
static bool imu_is_fake = false;

#ifndef BAUDRATE
#define BAUDRATE 921600
#endif

// How long setup() waits for a USB-CDC host before printing (see setup()).
#ifndef BOOT_SERIAL_WAIT_MS
#define BOOT_SERIAL_WAIT_MS 2000
#endif

// micro-ROS runs over this port, and at 921600 (1500000 on the GenDrv rig) the
// old 1 KB buffers hold 11 ms and 6.8 ms of wire respectively -- less than half
// a control period. An overrun does not slow the link down, it corrupts an XRCE
// frame, which the agent then discards, which looks like an intermittently
// unreliable robot rather than a buffer that is too small. 4 KB each costs
// 8 KB of an ESP32's 320 KB and makes the failure impossible at these rates.
#ifndef SERIAL_RX_BUFFER
#define SERIAL_RX_BUFFER 4096
#endif
#ifndef SERIAL_TX_BUFFER
#define SERIAL_TX_BUFFER 4096
#endif

// ---------------------------------------------------------------------------
// LED helpers.
//
// Boards without an addressable status LED (e.g. Waveshare GenDrv) set
// #define LED_PIN -1 in their config header. On such boards the helpers
// compile to no-ops so no spurious GPIO is touched and nothing is written.
//
// The guard is positive (enabled when defined && >= 0). The preprocessor
// folds integer constants in #if expressions:
//   - #define LED_PIN 25      -> 25 >= 0  true  -> LED enabled
//   - #define LED_PIN -1      -> -1 >= 0  false -> LED disabled (no wiring)
//   - #define LED_PIN LED_BUILTIN -> LED_BUILTIN is a non-macro identifier
//     (a static const in the core pins_arduino.h), so the preprocessor treats
//     it as 0 -> 0 >= 0 true  -> LED enabled, and digitalWrite(LED_BUILTIN,..)
//     still resolves to the core pin at link time.
//   - LED_PIN undefined       -> disabled (no reference, safe to compile)
// ---------------------------------------------------------------------------
// The pin is read from the env, with the generated header as the fallback -- a
// board is a configuration, not a build (AGENTS.md §10). It used to be a pure
// `#if defined(LED_PIN) && (LED_PIN) >= 0`, which meant that whether a board
// could blink was decided by whichever robot's header the image happened to be
// compiled from: the one pico2 image serving a bench module and an assembled
// robot could light the LED on neither or both, never on the one that has it.
//
// -1 keeps the old meaning exactly: no LED wired, every helper a no-op, no
// spurious GPIO touched. That matters on boards like the Waveshare GenDrv.
#ifndef LED_PIN
#define LED_PIN -1
#endif

static int led_pin = -1;

static inline void ledInit() {
    initMcuEnv();
    const char *value = envGet("led", NULL);
    led_pin = (value && *value) ? (int)strtol(value, NULL, 10) : (int)(LED_PIN);
    if (led_pin < 0)
        return;
    pinMode(led_pin, OUTPUT);
    digitalWrite(led_pin, LOW);
}

static inline void ledWrite(bool level) {
    if (led_pin < 0)
        return;
    digitalWrite(led_pin, level ? HIGH : LOW);
}

static inline bool ledRead() {
    return led_pin >= 0 && digitalRead(led_pin) == HIGH;
}

// Forward declarations
void flashLED(int n_times);
void rclErrorLoop();
bool syncTime();
struct timespec getTime();
bool createEntities();
bool destroyEntities();
void fullStop();
void moveBase();
void publishData();
void controlCallback(rcl_timer_t * timer, int64_t last_call_time);
void twistCallback(const void * msgin);
#ifdef USE_STAMPED_CMD_VEL
void twistStampedCallback(const void * msgin);
#endif

static AppMode app_mode = APP_BASE;

// ---------------------------------------------------------------------------
// RP2040/RP2350 hardware watchdog.
//
// This exists for one specific failure, not as general belt-and-braces. The
// 1200-baud touch is handled inside the core, in SerialUSB::checkSerialReset()
// (arduino-pico cores/rp2040/SerialUSB.cpp): it disables USBCTRL_IRQ, resets
// the USB block, calls reset_usb_boot() -- and then sits in
//
//     while (1); // WDT will fire here
//
// On RP2350 reset_usb_boot() is not the RP2040 ROM call any more; it is
// rom_reboot(REBOOT2_FLAG_REBOOT_TYPE_BOOTSEL | NO_RETURN_ON_SUCCESS, 10, ...),
// declared noreturn and with its result discarded. When that reboot does not
// happen the board stays in that while(1) with USB torn down and its IRQ off:
// still enumerated from the host's side, mute at every baud, and answering no
// control transfer -- which is why the host sees ETIMEDOUT (Errno 110) on the
// DTR ioctl.
//
// The core's comment assumes a watchdog is running. Nothing in this image armed
// one, so there was nothing to fire. Arming it is what makes that comment true:
// a failed BOOTSEL request costs a reboot instead of a walk to the bench.
//
// What the watchdog does NOT do is reach BOOTSEL. It converts the hang into a
// reboot back into THIS application, so the board comes back healthy -- it
// enumerates, it answers a micro-ROS agent -- and still refuses the next touch.
// A 2026-09-19 release matrix hit that on four boards, two RP2040 and two
// RP2350, none of them mute. The recovery is a USB device reset
// (USBDEVFS_RESET, `usbreset.py --vid 2e8a`), after which the very next touch
// succeeds; a RESET button press is the fallback, not the only way back.
#if defined(ARDUINO_ARCH_RP2040)
// Long enough that no normal blocking stretch reaches it -- entity creation
// over a live serial link is well under a second, and SerialUSB::write() gives
// up on an undrained host after 1 s -- and short enough that the recovery is a
// pause rather than an outage. 8388 ms is the RP2040 ceiling.
#define RP2_WDT_TIMEOUT_MS 8000
static inline void wdtBegin() { rp2040.wdt_begin(RP2_WDT_TIMEOUT_MS); }
static inline void wdtFeed()  { rp2040.wdt_reset(); }
#else
static inline void wdtBegin() {}
static inline void wdtFeed()  {}
#endif

// An env key read as a boolean. hw_factory has its own copy of this three-liner
// and it stays there: one is file-static in a library, this one is file-static
// in main, and exporting it would put a fourth spelling of `env` in the public
// headers for no gain.

// What this board is running, printed before anything else it does.
//
// Everything that describes the ROBOT now lives in the env partition and can be
// rewritten without a compiler (AGENTS.md §10). What the env deliberately cannot
// describe is the IMAGE -- the ROS 2 distro is fixed at link time, and so is
// every line of code -- so "which build is on this board?" had no answer at all
// from the outside. The only safe assumption was "not the one I just compiled",
// which made a reflash the first step of every run; and a reflash is the one
// step that can brick a robot that is already assembled.
//
// So the board says it, unprompted, in one greppable line:
//
//     [fw] linorobot2_hardware app=base built=2026-09-16 git=6aa607f
//
// `app` is the sub-firmware this boot selected, `built` is the compile date and
// `git` the 7-character revision of the tree it came from (a trailing '+' means
// that tree had uncommitted edits, so the revision alone does not identify it).
// The last two come from firmware/common/build_stamp.py through the build flags.
// scripts/mcu_probe.py parses exactly this line -- keep the key=value shape.
#ifndef FW_GIT_REV
#define FW_GIT_REV "unknown"      // built outside a git tree; say so rather than guess
#endif
#ifndef FW_ROS_DISTRO
#define FW_ROS_DISTRO "unknown"   // no board_microros_distro; never guess one
#endif
#ifndef FW_BUILD_DATE
#define FW_BUILD_DATE "unknown"
#endif
static void printBanner(void)
{
    // distro is here and not in the env partition because it cannot be there:
    // it picks the micro_ros library at link time (AGENTS.md §10). It is also
    // the only field that distinguishes the two halves of the release matrix,
    // so mcu_probe.py needs it to avoid calling a jazzy board up_to_date on a
    // lyrical run.
    Serial.printf("\n[fw] linorobot2_hardware app=%s distro=%s built=%s git=%s%s\n",
                  toolName(app_mode), FW_ROS_DISTRO, FW_BUILD_DATE, FW_GIT_REV,
                  mcuEnvValid() ? "" : " (env blank or invalid - using header defaults)");
}

// Why the board restarted. The ROM banner cannot answer this: it prints
// `rst:0xc (SW_CPU_RESET)` for a panic, for a brownout and for a plain
// esp_restart() alike, and the brownout handler's own message is routinely lost
// because the supply is collapsing while it tries to send it. So a board that
// browns out the instant the Wi-Fi PHY powers up boot-loops with NOTHING on the
// console after the last line the app printed -- which reads as a firmware hang
// at exactly that line and sends the reader into the code. It cost a long
// session on 2026-09-16 to identify from the outside; esp_reset_reason() says
// it in one word, so the board now says it unprompted, every boot.
static void printResetReason(void)
{
#ifdef ESP32
    const char *why;
    switch (esp_reset_reason()) {
    case ESP_RST_POWERON:  why = "power-on";                       break;
    case ESP_RST_EXT:      why = "external reset pin";             break;
    case ESP_RST_SW:       why = "software restart";               break;
    case ESP_RST_PANIC:    why = "PANIC (exception/abort)";        break;
    case ESP_RST_INT_WDT:  why = "interrupt watchdog";             break;
    case ESP_RST_TASK_WDT: why = "task watchdog";                  break;
    case ESP_RST_WDT:      why = "other watchdog";                 break;
    case ESP_RST_DEEPSLEEP: why = "deep sleep wake";               break;
    case ESP_RST_BROWNOUT: why = "BROWNOUT - the supply sagged, "
                                 "not a firmware fault";           break;
    case ESP_RST_SDIO:     why = "SDIO";                           break;
    default:               why = "unknown";                        break;
    }
    Serial.printf("[boot] last reset: %s\n", why);
#if defined(ESP32) || defined(ARDUINO_ARCH_ESP32)
    Serial.printf("[boot] cpu %u MHz\n", (unsigned)getCpuFrequencyMhz());
    // ESP_ERR_INVALID_STATE (259) is the NORMAL answer on this core: the Arduino
    // framework already releases the controller when no BT stack is linked, so
    // there is nothing left to hand back. Measured on the GenDrv 2026-09-20 --
    // heap 243492 before and after. The call stays as a guard in case a future
    // core stops doing it; it must not read as a fault when it is working.
    if (bt_release_err == ESP_OK && bt_heap_after > bt_heap_before)
        Serial.printf("[mem] Bluetooth controller released: %lu KB of DRAM back "
                      "(heap %lu -> %lu)\n",
                      (unsigned long)((bt_heap_after - bt_heap_before) / 1024),
                      (unsigned long)bt_heap_before, (unsigned long)bt_heap_after);
    else if (bt_release_err == ESP_ERR_INVALID_STATE)
        Serial.printf("[mem] heap %lu bytes (Bluetooth already released by the core)\n",
                      (unsigned long)bt_heap_after);
    else
        Serial.printf("[mem] heap %lu bytes (Bluetooth release returned %d)\n",
                      (unsigned long)bt_heap_after, (int)bt_release_err);
#endif
#endif
}

// Rate-limited reporter for the RCSOFTCHECK macro above. Serial is the
// micro-ROS link on most boards, so a message there would corrupt the stream
// it is describing -- syslog goes out over Wi-Fi when there is any, and the
// counter is reported on the next boot line otherwise.
static uint32_t rc_soft_fails = 0;
static int32_t  rc_soft_last_line = 0;
static int32_t  rc_soft_last_code = 0;

void rcSoftFail(int line, int code)
{
    rc_soft_fails++;
    diagCount(DIAG_PUBFAIL);
    rc_soft_last_line = line;
    rc_soft_last_code = code;
    EXECUTE_EVERY_N_MS(1000, {
        syslog(LOG_WARNING, "rcl call failed %lu times, last main.cpp:%ld rc=%ld",
               (unsigned long)rc_soft_fails, (long)rc_soft_last_line,
               (long)rc_soft_last_code);
    });
}

void setup() 
{
    ledInit();

#if defined(ESP32) || defined(ARDUINO_ARCH_ESP32)
    // Hand back the Bluetooth controller's DRAM. Nothing in this firmware uses
    // Bluetooth -- there is no BT stack, no BLE, no pairing, not one reference
    // anywhere in the tree -- but the ESP32 reserves roughly 64 KB of DRAM for
    // its controller whether or not it is ever initialised. On a chip whose
    // DRAM is fixed at 320 KB by the linker script and the silicon, and cannot
    // be enlarged by any build setting, that is a fifth of the budget held for
    // a radio we never switch on.
    //
    // It must happen before anything else claims heap, and it is irreversible
    // for this boot -- which is exactly right here: a board that finds it needs
    // Bluetooth has bigger problems than this call.
    // Measured here and PRINTED later: Serial.begin() has not run yet, so a
    // printf at this point goes nowhere -- which is exactly what happened the
    // first time this shipped, and the release looked like it had not worked.
    {
        const uint32_t before = ESP.getFreeHeap();
        bt_release_err = esp_bt_controller_mem_release(ESP_BT_MODE_BTDM);
        bt_heap_before = before;
        bt_heap_after = ESP.getFreeHeap();
    }
#endif

    // Which application this boot runs, read from the env partition before
    // anything is sized for it. The 4+4 KB UART rings below exist to carry
    // micro-ROS traffic; a diagnostic that prints a line a second needs neither,
    // and in a unified image that memory would otherwise be spent in every mode.
    app_mode = toolSelect();
    const bool micro_ros = (app_mode == APP_BASE);
#ifdef ESP32
    Serial.end();
    if (micro_ros) {
        Serial.setRxBufferSize(SERIAL_RX_BUFFER);
#ifndef ARDUINO_USB_CDC_ON_BOOT
        Serial.setTxBufferSize(SERIAL_TX_BUFFER);
#endif
    }
#endif
    // The rate is an env key, not a build setting. A released image is built
    // for an MCU, not for a robot (scripts/build_prebuilt.py), so the one thing
    // that must match the agent on the other end of the cable cannot be baked
    // in: a GenDrv carrying four I2C sensors at 50 Hz needs 1.5 Mbaud, a bare
    // DevKit is fine at 921600, and both run the same binary. BAUDRATE from the
    // generated header stays the fallback for a board with a blank env.
    Serial.begin(envU32("baud", BAUDRATE));
    // The second UART, when the env names a pin for it (diag_tx). Opened
    // this early so a boot that never reaches the agent still reports.
    if (micro_ros) diagBegin();

    // Wait briefly for a host to open the port, on boards whose Serial IS the
    // USB device.
    //
    // On an ESP32 with a USB-UART bridge the bridge stays enumerated across a
    // reset, so the boot lines land in whatever monitor is attached. On an
    // RP2040/RP2350 -- and an S3 with CDC-on-boot -- the port IS the MCU: it
    // disappears on reboot, reappears about a second later, and everything
    // written before the host opens it is discarded by the core. That silently
    // made the most useful output in this firmware unobservable: the banner, the
    // reset reason and the I2C listing are all printed in the first
    // milliseconds, and any monitor, including our own flasher, connects long
    // after them. It reads as a firmware that prints nothing.
    //
    // Bounded, so a robot that boots with nothing attached loses at most this
    // much; `boot_serial_wait` in the env sets it (0 disables the wait).
#if defined(ARDUINO_ARCH_RP2040) || defined(ARDUINO_ARCH_RP2350) || defined(ARDUINO_USB_CDC_ON_BOOT)
    {
        const uint32_t wait_ms = envU16("boot_serial_wait", BOOT_SERIAL_WAIT_MS);
        for (uint32_t t0 = millis(); !Serial && (millis() - t0) < wait_ms; )
            delay(10);
        if (Serial)
            delay(50);   // let the host's terminal finish opening before the first line
    }
#endif
    printBanner();
    printResetReason();

    // I2C bus and any GPIO the board must drive at boot. This replaced the
    // BOARD_INIT macro, which was a block of setup() pasted into the config
    // header -- see firmware/common/lib/board_init. Note the bus pins were
    // only ever honoured through that macro, so a config carrying SDA_PIN /
    // SCL_PIN and no BOARD_INIT silently ran the bus on the core defaults.
    initBoard();

    // Before the sensors: fullStop() and the control loop both dereference
    // these, and a fatal sensor init below reaches fullStop() on its way out.
    initDrivetrain();
    // Read once. Every use below -- the sensor path, the pose reset, the control
    // loop -- asks this rather than the compiler, so one image serves a bare
    // bench module and the same board with an IMU on it.
    fake_wheels = wheelsAreFake();
#ifdef USE_ESP32_DUAL_CORE
    dual_core = envFlag("dual_core", DUAL_CORE_DEFAULT);
    // Dual core is the SERIAL robot's tool: it takes moveBase() off the core
    // that services micro-ROS, and on the GenDrv over 1.5 Mbaud with four real
    // sensors that is the difference between 43 and 49 Hz. It is not for a
    // robot with the radio on. controlTask's critical section disables
    // interrupts on core 0, where the Wi-Fi driver and lwIP live; measured
    // with wifi=1 the same binary froze inside a publish for 6 s and dropped
    // /imu/data_raw to 4 Hz. So the radio wins: wifi=1 or transport=udp4
    // means single core, whatever dual_core says.
    if (dual_core && wifiWanted()) {
        dual_core = false;
        Serial.println("[core] dual_core=1 ignored: the radio is on (wifi=1 or "
                       "transport=udp4). Dual core is for a serial robot with "
                       "wifi=0.");
    }
    Serial.printf("[core] control loop on %s\n",
                  dual_core ? "core 0 (dual core)" : "the micro-ROS core");
#endif

    initWifis();
    // After initWifis, not before: the syslog server address comes out of the
    // env partition, and there is nowhere to send a log line until the radio is
    // associated anyway.
    initSyslog();
    // Only when the radio actually came up. ArduinoOTA.begin() ends in lwIP
    // (mDNS + a UDP socket) and asserts the same way syslog did when there is
    // no tcpip thread. initWifis() returns early for a serial robot, so this
    // used to abort every ESP32 that flashed a released image -- which compiles
    // OTA in, because the image is built from a Wi-Fi robot's config.
    if (wifiWanted())
        initOta();

    // Everything above is what every application needs: the bus, the drivetrain,
    // the radio, a log sink. Everything below it is micro-ROS, which only the
    // robot firmware uses -- so a tool takes over here.
    if (!micro_ros) {
        toolSetup(app_mode);
        return;
    }
    // Which IMU and magnetometer this board has is read from the env partition,
    // falling back to what the config was generated for. This is the first use
    // of the env, so initMcuEnv() runs here -- it is safe now and was not during
    // static initialisation.
    initMcuEnv();
    const char *imu_name = envGet("imu", defaultIMUName());
    const char *mag_name = envGet("mag", defaultMAGName());

    // On a real robot, ask the bus before believing the config.
    //
    // The names above are a description of the robot written by a human, and the
    // failure mode when that description is wrong is the worst one this firmware
    // has: imu->init() returns false, setup() enters the fatal `flashLED(3)`
    // loop, and the board never reaches micro-ROS at all -- so the Cockpit sees
    // a board that will not connect and nothing that says why. Meanwhile the
    // i2c_detect application, two flash pages away in this same image, could
    // have read the right answer off the bus in 30 ms.
    //
    // So it reads it here. Detection wins over the configured name when this
    // image carries a driver for what answered, because the bus is the ground
    // truth and the YAML is a claim about it -- and the override is printed, so
    // a config that disagrees with the hardware is visible rather than silently
    // routed around.
    //
    // Skipped only on a board that is fake all the way through: no wheels, no
    // IMU, no magnetometer. That is the bare bench module, where the bus is
    // empty by definition and the simulated IMU is computed from the simulated
    // wheels anyway.
    //
    // Keying this on wheelsAreFake() alone was wrong, and the bench board that
    // exists to test this feature is exactly the counter-example: an RP2350 with
    // a real MPU6050 on GP0/GP1 and no drivetrain at all. Real sensors are a
    // fact about the BUS; fake wheels are a fact about the drivetrain.
    // `i2c_scan` in the env forces the answer either way.
    const bool all_fake = wheelsAreFake()
                          && strcasecmp(imu_name, "fake") == 0
                          && strcasecmp(mag_name, "fake") == 0;
    if (envFlag("i2c_scan", !all_fake))
        i2cProbeSelect(&imu_name, &mag_name);

    imu = createIMU(imu_name);
    mag = createMAG(mag_name);
    // Which driver was actually built, after the probe has had its say. The
    // simulated IMU has no gyro of its own, so the loop below takes yaw rate
    // from the odometry instead -- that used to be `#ifdef USE_FAKE_IMU`, which
    // asked the BUILD a question only the boot can answer, and got it wrong on
    // any board whose IMU was chosen by detection rather than by the config.
    imu_is_fake = (strcasecmp(imu_name, "fake") == 0);

    // A real magnetometer answered the bus, or fake wheels are synthesising a
    // field to calibrate against. Either way there is something to publish; a
    // FakeMAG standing in for absent hardware has nothing to say and the topic
    // stays off the wire.
    publish_mag = envFlag("pub_mag",
                              (strcasecmp(mag_name, "fake") != 0) || fake_wheels);

    if (fake_wheels) {
        // A bare module has nothing on the I2C bus, so probing it would fail and
        // the fatal loops below would trap the board before it ever connects. The
        // simulated IMU and magnetometer are computed from the simulated wheels
        // anyway, and would overwrite whatever a real sensor returned -- so skip
        // the hardware entirely and just prepare the two messages.
        fake_imu.initMsgs(imu_msg, mag_msg);
    } else {
        if (!imu->init()) // take IMU failure as fatal
        {
            Serial.println("IMU init failed");
            syslog(LOG_INFO, "%s IMU init failed %lu", __FUNCTION__, millis());
            while (1)
            {
                flashLED(3); // flash 3 times
                runWifis();
                runOta();
            }
        }
        if (!mag->init()) // take mag failure as fatal
        {
            Serial.println("MAG init failed");
            syslog(LOG_INFO, "%s MAG init failed %lu", __FUNCTION__, millis());
            while (1)
            {
                flashLED(4); // flash 4 times
                runWifis();
                runOta();
            }
        }
    }
    initBattery();
    // Fake mode masks the sonar. `fake_ld19` is read directly rather than
    // waiting for fake_lidar_on below, because initRange() has to happen before
    // the LiDAR block and the answer is the same either way.
    const bool sonar_faked = fake_wheels || envFlag("fake_ld19", FAKE_LD19_DEFAULT);
    initRange(!sonar_faked);
    env_present = initEnv();
    // Three states, carried by one optional env key:
    //
    //   key absent  auto    -- publish what answered the I2C probe
    //   pub_x=0     disable -- never publish, even though the chip is there.
    //                          Six publishers at 50 Hz do not fit through a
    //                          921600 serial link, so this is a real knob.
    //   pub_x=1     enable  -- publish even if the probe missed it, so a wiring
    //                          fault reads as a dead topic rather than as a
    //                          topic that was never configured.
    publish_env = envFlag("pub_env", env_present);
    publish_battery = envFlag("pub_battery", batteryPresent());
    best_effort = envFlag("best_effort", true);
    if (env_present)
    {
        pressure_msg.header.frame_id = micro_ros_string_utilities_set(pressure_msg.header.frame_id, "base_link");
        temperature_msg.header.frame_id = micro_ros_string_utilities_set(temperature_msg.header.frame_id, "base_link");
        humidity_msg.header.frame_id = micro_ros_string_utilities_set(humidity_msg.header.frame_id, "base_link");
        // { pressure Pa^2, temperature C^2, humidity (0..1)^2 }. The env wins;
        // the macro, where a build defines one, is the fallback.
        {
            float env_cov[3] = {0.0f, 0.0f, 0.0f};
            bool have_cov = false;
#ifdef ENV_COV
            const float compiled[3] = ENV_COV;
            env_cov[0] = compiled[0];
            env_cov[1] = compiled[1];
            env_cov[2] = compiled[2];
            have_cov = true;
#endif
            have_cov = envFloatVec("env_cov", env_cov, 3) || have_cov;
            if (have_cov)
            {
                pressure_msg.variance = env_cov[0];
                temperature_msg.variance = env_cov[1];
                humidity_msg.variance = env_cov[2];
            }
        }
        syslog(LOG_INFO, "%s %s ready @ 1Hz %lu", __FUNCTION__, envHasHumidity() ? "BME280" : "BMP280", millis());
    }
    else
    {
        syslog(LOG_WARNING, "%s BMP280/BME280 not found (0x76/0x77) %lu", __FUNCTION__, millis());
    }
    // The globals read their env here, not in their constructors: static
    // initialisation runs before the flash partition API is usable.
    odometry.applyEnvCovariance();
    initLidar(); // after wifi connected
    fake_lidar_on = envFlag("fake_ld19", true);
    if (fake_lidar_on)
    {
        // The mode, before begin(): it decides whether a UART is opened, which
        // sink is armed and how many packets a step() may emit.
        fake_lidar_comm = FakeLD19::parseCommMode(envGet("lidar_comm", NULL),
                                                  FakeLD19::parseCommMode(LIDAR_COMM_DEFAULT,
                                                                          FakeLD19::COMM_SERIAL));
        fake_ld19.setCommMode(fake_lidar_comm);
        Serial.printf("[lidar] comm=%s (default %s)\n",
                      fake_lidar_comm == FakeLD19::COMM_SERIAL ? "serial"
                      : fake_lidar_comm == FakeLD19::COMM_UDP ? "udp" : "topic",
                      LIDAR_COMM_DEFAULT);
        if (fake_lidar_comm == FakeLD19::COMM_TOPIC)
        {
            initRawScan();
            fake_ld19.setPacketCallback(onRawScanPacket);
        }
        // Where on the robot the scan is taken from: geometry.laser.x, the
        // same number the URDF puts the laser frame at.
        {
            const char *x_env = envGet("lidar_x", NULL);
            if (x_env && *x_env)
                fake_ld19.setOffsetX((float)atof(x_env));
        }
        // Which pin the simulated scan goes out of, and how fast, are wiring
        // facts about one board -- so they come from the env with the generated
        // header as the fallback, like every other pin. -1 means no UART: the
        // scan then leaves over UDP or micro-ROS instead.
        const int rx = envInt("lidar_rx", LIDAR_RXD);
        if (rx >= 0)
            fake_ld19.begin(rx, envU32("lidar_baud", LIDAR_BAUDRATE));
        else
            fake_ld19.begin();
    }
    else
        Serial.println("[lidar] fake_ld19=0: the LiDAR emulator is off (a real LiDAR on this robot)");

    // /sonar: a real HC-SR04 if the env named both pins, else the simulated
    // cone when the emulator is running to raycast it. `fake_sonar` can turn
    // the simulated one off on a bench that does not want it; it cannot
    // conjure one without the emulator.
    safety_stop_on = envFlag("safety_stop", false);
    safety_stop_range = (float)atof(envGet("safety_stop_m", "0.25"));
    // The simulated cone when anything is simulated, the real sensor only on a
    // robot that is entirely real. rangePresent() is already false in fake mode
    // -- initRange() dropped the pins -- so this cannot drive hardware either
    // way; it decides what, if anything, /sonar carries.
    range_fake = fake_lidar_on && envFlag("fake_sonar", true);
    publish_range = rangePresent() || range_fake;
    Serial.printf("[range] /sonar %s\n",
                  !publish_range ? "off (no sonar pins, no emulator)"
                  : range_fake ? "simulated (raycast from the fake LiDAR room)"
                               : "from the HC-SR04");

    battery_msg = getBattery();
    prev_voltage = battery_msg.voltage;

    // One call for both transports. Which one is installed comes from the env
    // partition (`transport=serial|udp4`), not from how this was compiled --
    // see firmware/common/lib/uros_transport. For udp4 this must run after
    // initWifis(), which it does: the radio is brought up earlier in setup().
    initUrosTransport();

    // Output pins that had to wait for the stack -- a motor-driver enable line
    // that should stay low while the PWM pins were still being decided.
    initBoardLate();
#ifdef USE_ESP32_DUAL_CORE
    if (dual_core)
      xTaskCreatePinnedToCore(
        controlTask,
        "controlTask",
        4096,
        NULL,
        configMAX_PRIORITIES - 2,
        &controlTaskHandle,
        0
    );
#endif
    syslog(LOG_INFO, "%s Ready %lu", __FUNCTION__, millis());

    // Last thing in setup(): sensor probing and Wi-Fi association above are
    // allowed to take as long as they take, and a watchdog armed before them
    // would turn a slow boot into a boot loop.
    wdtBegin();
}

// Simulated wall contact indicator.
//
// LED_PIN may be -1 on boards with no addressable status LED, or LED_BUILTIN,
// which is a non-macro identifier the preprocessor evaluates as 0 -- so the
// guard is "defined and >= 0" and LED_BUILTIN boards stay enabled.
//
// The flash is timed rather than delayed: this runs inside the 50 Hz control
// path, and a delay() here would stall the whole loop.
#if defined(LED_PIN) && (LED_PIN) >= 0
#define FAKE_WALL_LED
#endif

static unsigned long fake_wall_led_off_at = 0;

static inline void fakeWallLedOn()
{
#ifdef FAKE_WALL_LED
    digitalWrite(LED_PIN, HIGH);
    fake_wall_led_off_at = millis() + 120;
#endif
}

static inline void fakeWallLedService()
{
#ifdef FAKE_WALL_LED
    if (fake_wall_led_off_at != 0 && (long)(millis() - fake_wall_led_off_at) >= 0)
    {
        digitalWrite(LED_PIN, LOW);
        fake_wall_led_off_at = 0;
    }
#endif
}

void loop() {
    if (app_mode != APP_BASE) {
        // The radio is the dispatcher's: a tool that serviced it as well would
        // run OTA twice per iteration.
        runWifis();
        runOta();
        toolLoop(app_mode);
        wdtFeed();
        return;
    }
    fakeWallLedService();
    diagCount(DIAG_LOOP);
    diagState((int)state);
    switch (state) 
    {
        case WAITING_AGENT:
            EXECUTE_EVERY_N_MS(500, {
                const bool ok = (RMW_RET_OK == rmw_uros_ping_agent(100, 1));
                diagCount(ok ? DIAG_PING_OK : DIAG_PING_FAIL);
                state = ok ? AGENT_AVAILABLE : WAITING_AGENT;
            });
            break;
        case AGENT_AVAILABLE:
            syslog(LOG_INFO, "%s agent available %lu", __FUNCTION__, millis());
            state = (true == createEntities()) ? AGENT_CONNECTED : WAITING_AGENT;
            if (state == WAITING_AGENT) 
            {
                destroyEntities();
            }
            break;
        case AGENT_CONNECTED:
            // Pinging is a property of the transport, so it is decided at run
            // time along with the transport itself. On udp4 the agent is
            // reached over the LAN and a 200 ms ping is both unnecessary and a
            // way to declare a working link dead on one lost datagram; on a
            // serial link the ping is how a disappeared agent is noticed at all.
            if (!urosTransportIsUdp())
            {
                EXECUTE_EVERY_N_MS(200, {
                    const bool ok = (RMW_RET_OK == rmw_uros_ping_agent(100, 1));
                    diagCount(ok ? DIAG_PING_OK : DIAG_PING_FAIL);
                    state = ok ? AGENT_CONNECTED : AGENT_DISCONNECTED;
                });
            }
            if (state == AGENT_CONNECTED) 
            {
                // 100 ms, which is what this was for most of its life. It was
                // cut to 10 recently; put it back. The timeout is how long the
                // executor may spend servicing the session per loop(), and
                // starving it does not give the rest of loop() more useful work
                // -- it gives the micro-ROS transport less time to drain, on a
                // board where the radio path is already the scarce resource.
                const uint32_t spin_t0 = micros();
                const rcl_ret_t spin_rc = rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
                diagSpin((int)spin_rc, micros() - spin_t0);
            }
            break;
        case AGENT_DISCONNECTED:
            syslog(LOG_INFO, "%s agent disconnected %lu", __FUNCTION__, millis());
            fullStop();
            destroyEntities();
            state = WAITING_AGENT;
            break;
        default:
            break;
    }
    runWifis();
    runOta();
#ifdef WDT_TIMEOUT
    esp_task_wdt_reset();
#endif
    wdtFeed();
#ifdef BOARD_LOOP // board specific loop
    BOARD_LOOP
#endif
    fake_ld19.step();
    flushRawScan();
    // The emulator's own account of itself every 5 s, over syslog (verified to
    // arrive with the agent up). Cumulative counters; read the deltas. Not
    // UDP-only: a serial robot's scan goes out of a UART pin, and "the
    // emulator is running" and "bytes are leaving that pin" are different
    // claims -- the bench saw no /scan with both sinks and could not say which.
    // A serial-transport robot with the radio off has no channel for this; the
    // UART1 diagnostic is the instrument there (`diag_tx`), and it cannot be
    // the same UART the emulator streams out of.
    if (fake_lidar_on)
    {
        EXECUTE_EVERY_N_MS(5000, {
            char stats[192];
            fake_ld19.statsLine(stats, sizeof(stats));
            syslog(LOG_INFO, "fake_ld19 %s state=%d", stats, (int)state);
        });
    }
    diagTick();
}

void controlCallback(rcl_timer_t * timer, int64_t last_call_time) 
{
    RCLC_UNUSED(last_call_time);
    if (timer != NULL) 
    {
       diagCount(DIAG_TIMER);
       const uint32_t mv_t0 = micros();
#ifdef USE_ESP32_DUAL_CORE
       if (!dual_core)
#endif
       moveBase();
       diagTime(DIAGT_MOVE, micros() - mv_t0);
       publishData();
       diagCount(DIAG_PUBLISH);
    }
}

#ifdef USE_STAMPED_CMD_VEL
void twistStampedCallback(const void * msgin) 
{
    (void)msgin;
    ledWrite(!ledRead());

    prev_cmd_time = millis();
#ifdef USE_ESP32_DUAL_CORE
    if (dual_core) portENTER_CRITICAL_ISR(&controlMux);
#endif
    twist_msg = twist_stamped_msg.twist;
#ifdef USE_ESP32_DUAL_CORE
    if (dual_core) portEXIT_CRITICAL_ISR(&controlMux);
#endif
}
#endif

void twistCallback(const void * msgin) 
{
    (void)msgin;
    ledWrite(!ledRead());

    prev_cmd_time = millis();
}

bool createEntities()
{
    syslog(LOG_INFO, "%s %lu", __FUNCTION__, millis());
    allocator = rcl_get_default_allocator();
    //create init_options
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    // create node
    RCCHECK(rclc_node_init_default(&node, envGet("node", NODE_NAME), "", &support));
    // The 50 Hz topics take the env's QoS (see best_effort above).
    rcl_ret_t (*init_fast)(rcl_publisher_t *, const rcl_node_t *,
                           const rosidl_message_type_support_t *, const char *) =
        best_effort ? rclc_publisher_init_best_effort : rclc_publisher_init_default;
    // create odometry publisher
    RCCHECK(init_fast(
        &odom_publisher, 
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
        TOPIC_PREFIX "odom/unfiltered"
    ));
    // create IMU publisher: raw sensor data for madgwick filter
    RCCHECK(init_fast(
        &imu_publisher, 
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        TOPIC_PREFIX "imu/data_raw"
    ));
    if (publish_mag)
        RCCHECK(init_fast(
            &mag_publisher,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, MagneticField),
            TOPIC_PREFIX "imu/mag"
        ));
    // create battery publisher, if this robot can measure a voltage at all
    if (publish_battery)
        RCCHECK(rclc_publisher_init_default(
        &battery_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, BatteryState),
        TOPIC_PREFIX "battery"
        ));
    if (safety_stop_on)
    {
        // Tells ROS the robot stopped itself. The stop is a firmware reflex --
        // it has to keep working when the ROS side is busy, wedged or
        // disconnected -- so this publisher only reports the state, it never
        // decides it.
        RCCHECK(rclc_publisher_init_default(
        &safety_stop_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
        TOPIC_PREFIX "safety_stop"
        ));
    }
    if (publish_range)
    {
        RCCHECK(rclc_publisher_init_default(
        &range_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),
        TOPIC_PREFIX "sonar"
        ));
    }
    if (publish_env)
    {
        RCCHECK(rclc_publisher_init_default(
            &pressure_publisher, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, FluidPressure),
            TOPIC_PREFIX "pressure"));
        RCCHECK(rclc_publisher_init_default(
            &temperature_publisher, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Temperature),
            TOPIC_PREFIX "temperature"));
        if (envHasHumidity())
            RCCHECK(rclc_publisher_init_default(
                &humidity_publisher, &node,
                ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, RelativeHumidity),
                TOPIC_PREFIX "humidity"));
    }
    // create raw_scan publisher for fake LiDAR -- only in topic mode. The
    // publisher is compiled into every image now, so `fake_lidar_on` alone
    // would put an unread raw_scan on the wire for every serial and udp robot,
    // and spend one of RMW_UXRCE_MAX_PUBLISHERS doing it.
    if (fake_lidar_on && fake_lidar_comm == FakeLD19::COMM_TOPIC)
    {
        RCCHECK(rclc_publisher_init_default(
            &raw_scan_publisher,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8MultiArray),
            TOPIC_PREFIX "raw_scan"
        ));
        std_msgs__msg__UInt8MultiArray__init(&raw_scan_msg);
        raw_scan_pub_ready = true;
    }
#ifdef USE_STAMPED_CMD_VEL
    // create stamped twist subscriber for Nav2 on /cmd_vel
    RCCHECK(rclc_subscription_init_default( 
        &twist_stamped_subscriber, 
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, TwistStamped),
        TOPIC_PREFIX "cmd_vel"
    ));
    geometry_msgs__msg__TwistStamped__init(&twist_stamped_msg);
    twist_stamped_msg.header.frame_id.data = twist_stamped_frame_id;
    twist_stamped_msg.header.frame_id.size = 0;
    twist_stamped_msg.header.frame_id.capacity = sizeof(twist_stamped_frame_id);

    // create unstamped fallback subscriber on /cmd_vel_unstamped for legacy teleop tools
    RCCHECK(rclc_subscription_init_default( 
        &twist_subscriber, 
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        TOPIC_PREFIX "cmd_vel_unstamped"
    ));
    const size_t executor_handles = 3;
#else
    // create standard unstamped twist command subscriber on /cmd_vel
    RCCHECK(rclc_subscription_init_default( 
        &twist_subscriber, 
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        TOPIC_PREFIX "cmd_vel"
    ));
    const size_t executor_handles = 2;
#endif
    // create timer for actuating the motors at 50 Hz (1000/20)
    const unsigned int control_timeout = 20;
    RCCHECK(rclc_timer_init_default2( 
        &control_timer, 
        &support,
        RCL_MS_TO_NS(control_timeout),
        (rcl_timer_callback_t) controlCallback,
        true
    ));
    executor = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor, &support.context, executor_handles, & allocator));
#ifdef USE_STAMPED_CMD_VEL
    RCCHECK(rclc_executor_add_subscription(
        &executor, 
        &twist_stamped_subscriber, 
        &twist_stamped_msg, 
        &twistStampedCallback, 
        ON_NEW_DATA
    ));
#endif
    RCCHECK(rclc_executor_add_subscription(
        &executor, 
        &twist_subscriber, 
        &twist_msg, 
        &twistCallback, 
        ON_NEW_DATA
    ));
    RCCHECK(rclc_executor_add_timer(&executor, &control_timer));

    // synchronize time with the agent
    syncTime();
    ledWrite(HIGH);

    if (fake_wheels) {
        // A simulated robot has no way to be picked up and put back at the start,
        // and its pose is board state: it survives the host container, the agent
        // and the whole ROS stack being torn down and rebuilt. So a second test run
        // silently begins wherever the first one parked the robot -- and once that
        // is against a simulated wall, the safety stop zeroes forward velocity and
        // navigation fails as "goal outside map" or "failed to make progress",
        // neither of which points at inherited state. A new agent session means a
        // new run, so start it from the origin.
        //
        // Real robots deliberately do not do this: odometry must stay continuous
        // across a reconnect, or the transform tree jumps under whatever is
        // localising against it.
        odometry.reset();
        fake_ld19.updatePose(0.0f, 0.0f, 0.0f);
        syslog(LOG_INFO, "%s simulated pose reset to origin %lu", __FUNCTION__, millis());
    }

    return true;
}

bool destroyEntities()
{
    syslog(LOG_INFO, "%s %lu", __FUNCTION__, millis());
    rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
    (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    RCSOFTCHECK(rcl_publisher_fini(&odom_publisher, &node));
    RCSOFTCHECK(rcl_publisher_fini(&imu_publisher, &node));
    if (publish_mag)
        RCSOFTCHECK(rcl_publisher_fini(&mag_publisher, &node));
    if (publish_battery)
        RCSOFTCHECK(rcl_publisher_fini(&battery_publisher, &node));
    if (safety_stop_on)
        RCSOFTCHECK(rcl_publisher_fini(&safety_stop_publisher, &node));
    if (publish_range)
        RCSOFTCHECK(rcl_publisher_fini(&range_publisher, &node));
    if (publish_env)
    {
        RCSOFTCHECK(rcl_publisher_fini(&pressure_publisher, &node));
        RCSOFTCHECK(rcl_publisher_fini(&temperature_publisher, &node));
        if (envHasHumidity())
            RCSOFTCHECK(rcl_publisher_fini(&humidity_publisher, &node));
    }
    if (raw_scan_pub_ready)
    {
        raw_scan_pub_ready = false;
        std_msgs__msg__UInt8MultiArray__fini(&raw_scan_msg);
        RCSOFTCHECK(rcl_publisher_fini(&raw_scan_publisher, &node));
    }
#ifdef USE_STAMPED_CMD_VEL
    RCSOFTCHECK(rcl_subscription_fini(&twist_stamped_subscriber, &node));
#endif
    RCSOFTCHECK(rcl_subscription_fini(&twist_subscriber, &node));
    RCSOFTCHECK(rcl_timer_fini(&control_timer));
    RCSOFTCHECK(rclc_executor_fini(&executor));
    RCSOFTCHECK(rcl_node_fini(&node))
    RCSOFTCHECK(rclc_support_fini(&support));

    ledWrite(HIGH);

    return true;
}

void fullStop()
{
#ifdef USE_ESP32_DUAL_CORE
    if (dual_core) portENTER_CRITICAL(&controlMux);
#endif
    twist_msg.linear.x = 0.0;
    twist_msg.linear.y = 0.0;
    twist_msg.angular.z = 0.0;

    motor1_controller.brake();
    motor2_controller.brake();
    motor3_controller.brake();
    motor4_controller.brake();
#ifdef USE_ESP32_DUAL_CORE
    if (dual_core) portEXIT_CRITICAL(&controlMux);
#endif
}

#ifdef USE_ESP32_DUAL_CORE
void controlTask(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(CONTROL_TIMER);
    for (;;)
    {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        if (state == AGENT_CONNECTED)
        {
            portENTER_CRITICAL(&controlMux);
            moveBase();
            portEXIT_CRITICAL(&controlMux);
        }
    }
}
#endif

// Forward range from whichever sensor is live, or -1 when there is none to
// consult -- in which case nothing is blocked, because a missing sensor must
// not brake the robot.
static inline float rangeAheadOrNegative()
{
    if (!publish_range)
        return -1.0f;
    if (range_fake)
        return fake_lidar_on ? fake_ld19.rangeAheadM() : -1.0f;
    const float r = getRange().range;
    return isfinite(r) ? r : -1.0f;
}

void moveBase()
{
    // brake if there's no command received, or when it's only the first command sent
    if(((millis() - prev_cmd_time) >= 200)) 
    {
        twist_msg.linear.x = 0.0;
        twist_msg.linear.y = 0.0;
        twist_msg.angular.z = 0.0;

        ledWrite(HIGH);
    }

    // Forward hazard stop, decided here rather than in ROS. A stop that has to
    // travel out on a topic, be reasoned about, and come back as cmd_vel is one
    // network round trip too slow, and does nothing at all if the ROS side is
    // wedged or the link drops. This runs every control cycle regardless.
    //
    // Only forward motion is blocked: reverse and rotation stay available, or
    // the robot would be stuck against the obstacle with no way to back off.
    if (safety_stop_on)
    {
        const float range = rangeAheadOrNegative();
        const bool blocked = (range >= 0.0f) && (range < safety_stop_range);
        if (blocked && twist_msg.linear.x > 0.0)
        {
            twist_msg.linear.x = 0.0;
            twist_msg.linear.y = 0.0;
        }
        if (blocked != safety_stopped)
        {
            safety_stopped = blocked;
            syslog(LOG_INFO, "%s safety stop %s at %.2f m %lu", __FUNCTION__,
                   blocked ? "engaged" : "cleared", range, millis());
        }
    }

    // get the required rpm for each motor based on required velocities, and base used
    Kinematics::rpm req_rpm = kinematics->getRPM(
        twist_msg.linear.x, 
        twist_msg.linear.y, 
        twist_msg.angular.z
    );

    // get the current speed of each motor
    float current_rpm1 = motor1_encoder.getRPM();
    float current_rpm2 = motor2_encoder.getRPM();
    float current_rpm3 = motor3_encoder.getRPM();
    float current_rpm4 = motor4_encoder.getRPM();

    // the required rpm is capped at -/+ MAX_RPM to prevent the PID from having too much error
    // the PWM value sent to the motor driver is the calculated PID based on required RPM vs measured RPM
    int pwm1 = motor1_pid.compute(req_rpm.motor1, current_rpm1);
    int pwm2 = motor2_pid.compute(req_rpm.motor2, current_rpm2);
    int pwm3 = motor3_pid.compute(req_rpm.motor3, current_rpm3);
    int pwm4 = motor4_pid.compute(req_rpm.motor4, current_rpm4);
    motor1_controller.spin(pwm1);
    motor2_controller.spin(pwm2);
    motor3_controller.spin(pwm3);
    motor4_controller.spin(pwm4);
    // Close the loop in software when the wheels are simulated. No #ifdef: a
    // real encoder's feed() does nothing, so the same binary runs both and the
    // choice is the env's, not the compiler's.
    motor1_encoder.feed(pwm1);
    motor2_encoder.feed(pwm2);
    motor3_encoder.feed(pwm3);
    motor4_encoder.feed(pwm4);

    Kinematics::velocities current_vel = kinematics->getVelocities(
        current_rpm1, 
        current_rpm2, 
        current_rpm3, 
        current_rpm4
    );

    unsigned long now = millis();
    float vel_dt = (now - prev_odom_update) / 1000.0;
    prev_odom_update = now;
    odometry.update(
        vel_dt, 
        current_vel.linear_x, 
        current_vel.linear_y, 
        current_vel.angular_z
    );
    // Stop the simulated robot at the simulated walls, and correct the
    // odometry to match, so /odom and /scan never disagree about where it is.
    // fake_lidar_on is the only gate: on a real robot the emulator is off and
    // clampToRoom() is never consulted, so the walls do not exist.
    float fake_x = odometry.getX();
    float fake_y = odometry.getY();
    const bool hit_wall = fake_lidar_on && fake_ld19.clampToRoom(fake_x, fake_y);
    if (hit_wall)
        odometry.setPosition(fake_x, fake_y);
    if (fake_lidar_on)
        fake_ld19.updatePose(fake_x, fake_y, odometry.getHeading());
    if (fake_wheels) {
        // The IMU rides on how the body actually moved, which is not what the
        // wheels claim once the robot is against a wall. Real hardware behaves
        // the same way: the wheels slip and keep reporting speed, while the IMU
        // feels no acceleration and the robot goes nowhere. Keeping that
        // disagreement is the only feedback there is that something was hit --
        // there is no bump sensor, and odometry velocity alone never reveals it.
        // Rotation survives, since a robot pinned against a wall can still turn.
        fake_imu.update(
            hit_wall ? 0.0f : current_vel.linear_x,
            hit_wall ? 0.0f : current_vel.linear_y,
            current_vel.angular_z,
            vel_dt
        );
        fake_imu.setHeading(odometry.getHeading());
    }
    // Announce the contact once, on the way in. Driving into a wall holds the
    // clamp active for as long as the command lasts, so logging every 20 ms
    // cycle would bury the syslog in identical lines.
    static bool was_clamped = false;
    if (hit_wall && !was_clamped)
    {
        syslog(LOG_INFO, "%s fake wall contact at x %.2f y %.2f %lu",
               __FUNCTION__, fake_x, fake_y, millis());
        fakeWallLedOn();
    }
    was_clamped = hit_wall;
}

void publishData()
{
    static unsigned skip_dip = 0;
#ifdef USE_ESP32_DUAL_CORE
    if (dual_core) portENTER_CRITICAL(&controlMux);
#endif
    odom_msg = odometry.getData();
#ifdef USE_ESP32_DUAL_CORE
    if (dual_core) portEXIT_CRITICAL(&controlMux);
#endif
    const uint32_t sens_t0 = micros();
    if (fake_wheels) {
        // Every field these would return is overwritten just below, and on a bare
        // module the reads are two failing I2C transactions per publish, each one
        // stalling the loop for the bus timeout. Skip them.
        fake_imu.apply(imu_msg);
        // Simulated wheels mean a simulated heading, so the magnetometer has to
        // follow it: a real one left in the loop here would fight the fused yaw.
        fake_imu.applyMag(mag_msg);
    } else {
        imu_msg = imu->getData();
        if (imu_is_fake)
            imu_msg.angular_velocity.z = odom_msg.twist.twist.angular.z;
        mag_msg = mag->getData();
    }
    // Hard-iron offsets, from the env like everything else about this robot.
    // Read once -- this runs at the publish rate -- and applied only when the
    // robot has actually been calibrated, because subtracting a bias nobody
    // measured is worse than subtracting none.
    {
        static bool mag_bias_read = false;
        static bool mag_bias_set = false;
        static float mag_bias[3] = {0.0f, 0.0f, 0.0f};
        if (!mag_bias_read)
        {
            mag_bias_read = true;
#ifdef MAG_BIAS
            const float compiled[3] = MAG_BIAS;
            mag_bias[0] = compiled[0];
            mag_bias[1] = compiled[1];
            mag_bias[2] = compiled[2];
#endif
            envFloatVec("mag_bias", mag_bias, 3);
            mag_bias_set = (mag_bias[0] != 0.0f || mag_bias[1] != 0.0f
                            || mag_bias[2] != 0.0f);
        }
        if (mag_bias_set)
        {
            mag_msg.magnetic_field.x -= mag_bias[0];
            mag_msg.magnetic_field.y -= mag_bias[1];
            mag_msg.magnetic_field.z -= mag_bias[2];
        }
    }

    diagTime(DIAGT_SENSORS, micros() - sens_t0);
    const uint32_t pub_t0 = micros();
    struct timespec time_stamp = getTime();

    odom_msg.header.stamp.sec = time_stamp.tv_sec;
    odom_msg.header.stamp.nanosec = time_stamp.tv_nsec;

    imu_msg.header.stamp.sec = time_stamp.tv_sec;
    imu_msg.header.stamp.nanosec = time_stamp.tv_nsec;

    if (publish_mag)
    {
        mag_msg.header.stamp.sec = time_stamp.tv_sec;
        mag_msg.header.stamp.nanosec = time_stamp.tv_nsec;
    }

    RCSOFTCHECK(rcl_publish(&imu_publisher, &imu_msg, NULL));
    if (publish_mag)
        RCSOFTCHECK(rcl_publish(&mag_publisher, &mag_msg, NULL));
    RCSOFTCHECK(rcl_publish(&odom_publisher, &odom_msg, NULL));
    if (publish_battery) {
#ifdef BATTERY_DIP
    battery_msg = getBattery();
    battery_msg.header.stamp.sec = time_stamp.tv_sec;
    battery_msg.header.stamp.nanosec = time_stamp.tv_nsec;
    if (!skip_dip && battery_msg.voltage > 1.0 && battery_msg.voltage < prev_voltage * BATTERY_DIP) {
        RCSOFTCHECK(rcl_publish(&battery_publisher, &battery_msg, NULL));
        syslog(LOG_WARNING, "%s voltage dip %.2f", __FUNCTION__, battery_msg.voltage);
        skip_dip = 5;
    }
    if (skip_dip) skip_dip--;
    battery_msg.voltage = prev_voltage = battery_msg.voltage * 0.01 + prev_voltage * 0.99;
    // PHASE 40 ms — keep /battery off the cycle /sonar and /pressure ride on.
    EXECUTE_EVERY_N_MS_PHASED(BATTERY_TIMER, 40, {
        getBatteryPercentage(&battery_msg);
        RCSOFTCHECK(rcl_publish(&battery_publisher, &battery_msg, NULL));
    });
#else
    // Low sampling rate fallback: poll battery strictly within phased timer when BATTERY_DIP is disabled
    EXECUTE_EVERY_N_MS_PHASED(BATTERY_TIMER, 40, {
        battery_msg = getBattery();
        battery_msg.header.stamp.sec = time_stamp.tv_sec;
        battery_msg.header.stamp.nanosec = time_stamp.tv_nsec;
        getBatteryPercentage(&battery_msg);
        RCSOFTCHECK(rcl_publish(&battery_publisher, &battery_msg, NULL));
    });
#endif
    }
    if (safety_stop_on)
    {
        safety_stop_msg.data = safety_stopped;
        RCSOFTCHECK(rcl_publish(&safety_stop_publisher, &safety_stop_msg, NULL));
    }
    if (publish_range)
    {
        // Either a real HC-SR04 on the pins the env named, or a simulated
        // ultrasonic cone raycast from the same room the simulated LiDAR uses.
        // The simulated one is the robot's own feedback that something is
        // ahead -- wheel odometry cannot provide it, because the wheels keep
        // turning when the robot is stopped against something.
        EXECUTE_EVERY_N_MS(RANGE_TIMER, {
            if (range_fake)
            {
                range_msg.range = fake_ld19.rangeAheadM();
                range_msg.field_of_view = (float)FAKE_SONAR_CONE_DEG * (float)DEG_TO_RAD;
                range_msg.min_range = 0.02;
                range_msg.max_range = 4.0;
                range_msg.radiation_type = sensor_msgs__msg__Range__ULTRASOUND;
            }
            else
            {
                range_msg = getRange();
            }
            range_msg.header.stamp.sec = time_stamp.tv_sec;
            range_msg.header.stamp.nanosec = time_stamp.tv_nsec;
            RCSOFTCHECK(rcl_publish(&range_publisher, &range_msg, NULL)) });
    }
    // PHASE 60 ms — the barometer's 1 Hz burst lands between the /sonar (≈20 ms)
    // and /battery (40 ms) cycles, not on top of them.
    if (publish_env)
        EXECUTE_EVERY_N_MS_PHASED(ENV_TIMER, 60, {
            EnvData e = readEnv();
            if (e.valid)
            {
                pressure_msg.header.stamp.sec = time_stamp.tv_sec;
                pressure_msg.header.stamp.nanosec = time_stamp.tv_nsec;
                pressure_msg.fluid_pressure = e.pressure;
                temperature_msg.header.stamp = pressure_msg.header.stamp;
                temperature_msg.temperature = e.temperature;
                RCSOFTCHECK(rcl_publish(&pressure_publisher, &pressure_msg, NULL));
                RCSOFTCHECK(rcl_publish(&temperature_publisher, &temperature_msg, NULL));
                if (envHasHumidity())
                {
                    humidity_msg.header.stamp = pressure_msg.header.stamp;
                    humidity_msg.relative_humidity = e.humidity;
                    RCSOFTCHECK(rcl_publish(&humidity_publisher, &humidity_msg, NULL));
                }
            }
        });
    diagTime(DIAGT_PUB, micros() - pub_t0);
}

bool syncTime()
{
    const int timeout_ms = 1000;
    if (rmw_uros_epoch_synchronized()) return true; // synchronized previously
    // get the current time from the agent
    RCSOFTCHECK(rmw_uros_sync_session(timeout_ms));
    if (rmw_uros_epoch_synchronized()) {
#if (_POSIX_TIMERS > 0)
        // Get time in milliseconds or nanoseconds
        int64_t time_ns = rmw_uros_epoch_nanos();
    timespec tp;
    tp.tv_sec = time_ns / 1000000000;
    tp.tv_nsec = time_ns % 1000000000;
    clock_settime(CLOCK_REALTIME, &tp);
#else
    unsigned long long ros_time_ms = rmw_uros_epoch_millis();
    // now we can find the difference between ROS time and uC time
    time_offset = ros_time_ms - millis();
#endif
    return true;
    }
    return false;
}

struct timespec getTime()
{
    struct timespec tp = {0};
#if (_POSIX_TIMERS > 0)
    clock_gettime(CLOCK_REALTIME, &tp);
#else
    // add time difference between uC time and ROS time to
    // synchronize time with ROS
    unsigned long long now = millis() + time_offset;
    tp.tv_sec = now / 1000;
    tp.tv_nsec = (now % 1000) * 1000000;
#endif
    return tp;
}

void rclErrorLoop() 
{
    // Deliberately does not feed the watchdog. On ESP32 this loop is a recovery
    // path -- runOta() can still take a new image over the air -- so it must be
    // allowed to run forever. On RP2 there is no OTA to wait for and no watchdog
    // was armed before this change, which made a failed createEntities() a
    // permanent blinking board; now it reboots and retries.
    while(true)
    {
        flashLED(2); // flash 2 times
        runOta();
#if defined(ESP32) || defined(ARDUINO_ARCH_ESP32)
        wdtFeed();
#endif
    }
}

void flashLED(int n_times)
{
    for(int i=0; i<n_times; i++)
    {
        ledWrite(HIGH);
        delay(150);
        ledWrite(LOW);
        delay(150);
    }
    delay(1000);
}
