// linorobot2 base controller for the Arduino UNO Q's STM32U585 (Zephyr + micro-ROS).
//
// The same contract as the Arduino firmware (firmware/src/main.cpp), so the stack
// cannot tell the boards apart: node `linorobot_base_node`, `odom/unfiltered` at
// 50 Hz (frames odom -> base_footprint), `cmd_vel` with a 200 ms dead-man, the env
// block's keys and defaults. The link is lpuart1 <-> the QRB2210's /dev/ttyHS1.
//
// Phase 3 of cockpit/docs/unoq-zephyr.md: 2WD only, no IMU or battery yet.
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/printk.h>

#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/twist_stamped.h>
#include <nav_msgs/msg/odometry.h>
extern "C" {
#include <microros_transports.h>
}

#include "drive.h"
#include "env.h"
#include "kinematics.h"
#include "pid.h"
#include "test_sensors.h"

#define NODE_NAME      "linorobot_base_node"
#define CONTROL_MS     20      // 50 Hz, as every other board
#define CMD_TIMEOUT_MS 200     // the Arduino firmware's dead-man

#define RCOK(fn) ((fn) == RCL_RET_OK)

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);

static rcl_allocator_t allocator;
static rclc_support_t support;
static rcl_node_t node;
static rcl_publisher_t odom_pub;
static rcl_subscription_t twist_sub, twist_stamped_sub;
static rcl_timer_t control_timer;
static rclc_executor_t executor;

static nav_msgs__msg__Odometry odom_msg;
static geometry_msgs__msg__Twist twist_msg;
static geometry_msgs__msg__TwistStamped twist_stamped_msg;
static char frame_odom[] = "odom", frame_base[] = "base_footprint";
static char twist_stamped_frame[32];

static bool stamped_cmd_vel;
static bool drive_enabled;
static int64_t last_cmd_ms = -1000000;
static int64_t last_control_ms;
static Kinematics *kinematics;
static PID *pid[DRIVE_MOTORS];
static double x_pos, y_pos, heading;
static float pose_cov[6] = {0.0001f, 0.0001f, 0, 0, 0, 0.0001f};
static float twist_cov[6] = {0.00001f, 0.00001f, 0, 0, 0, 0.00001f};

static void twistCallback(const void *msgin)
{
    (void)msgin;
    last_cmd_ms = k_uptime_get();
    gpio_pin_toggle_dt(&led);
}

static void twistStampedCallback(const void *msgin)
{
    (void)msgin;
    twist_msg = twist_stamped_msg.twist;
    last_cmd_ms = k_uptime_get();
    gpio_pin_toggle_dt(&led);
}

static void stamp(builtin_interfaces__msg__Time *t)
{
    const int64_t ns = rmw_uros_epoch_nanos();
    t->sec = (int32_t)(ns / 1000000000LL);
    t->nanosec = (uint32_t)(ns % 1000000000LL);
}

static void controlCallback(rcl_timer_t *timer, int64_t last_call_time)
{
    (void)last_call_time;
    if (timer == NULL)
        return;
    const int64_t now = k_uptime_get();
    const float dt = (float)(now - last_control_ms) / 1000.0f;
    last_control_ms = now;

    if (now - last_cmd_ms >= CMD_TIMEOUT_MS) {
        twist_msg.linear.x = twist_msg.linear.y = twist_msg.angular.z = 0.0;
        gpio_pin_set_dt(&led, 0);
    }

    float rpm[DRIVE_MOTORS];
    driveMeasure(dt, rpm);
    if (drive_enabled) {
        const Kinematics::rpm req = kinematics->getRPM((float)twist_msg.linear.x, 0.0f,
                                                       (float)twist_msg.angular.z);
        const float target[DRIVE_MOTORS] = {req.motor1, req.motor2};
        for (int m = 0; m < DRIVE_MOTORS; m++)
            driveSpin(m, (int)pid[m]->compute(target[m], rpm[m]));
    }

    const Kinematics::velocities v = kinematics->getVelocities(rpm[0], rpm[1], 0.0f, 0.0f);
    heading += v.angular_z * dt;
    x_pos += v.linear_x * cos(heading) * dt;
    y_pos += v.linear_x * sin(heading) * dt;

    stamp(&odom_msg.header.stamp);
    odom_msg.pose.pose.position.x = x_pos;
    odom_msg.pose.pose.position.y = y_pos;
    odom_msg.pose.pose.orientation.z = sin(heading / 2.0);
    odom_msg.pose.pose.orientation.w = cos(heading / 2.0);
    odom_msg.twist.twist.linear.x = v.linear_x;
    odom_msg.twist.twist.linear.y = 0.0;
    odom_msg.twist.twist.angular.z = v.angular_z;
    (void)rcl_publish(&odom_pub, &odom_msg, NULL);
}

static void initOdomMsg(void)
{
    nav_msgs__msg__Odometry__init(&odom_msg);
    odom_msg.header.frame_id.data = frame_odom;
    odom_msg.header.frame_id.size = strlen(frame_odom);
    odom_msg.header.frame_id.capacity = sizeof(frame_odom);
    odom_msg.child_frame_id.data = frame_base;
    odom_msg.child_frame_id.size = strlen(frame_base);
    odom_msg.child_frame_id.capacity = sizeof(frame_base);
    for (int i = 0; i < 6; i++) {
        odom_msg.pose.covariance[i * 7] = pose_cov[i];
        odom_msg.twist.covariance[i * 7] = twist_cov[i];
    }
}

static bool createEntities(void)
{
    allocator = rcl_get_default_allocator();
    rcl_init_options_t opts = rcl_get_zero_initialized_init_options();
    if (!RCOK(rcl_init_options_init(&opts, allocator)))
        return false;
    rcl_init_options_set_domain_id(&opts, (size_t)envInt("domain_id", 0));
    const rcl_ret_t rc = rclc_support_init_with_options(&support, 0, NULL, &opts, &allocator);
    (void)rcl_init_options_fini(&opts);
    if (!RCOK(rc))
        return false;
    if (!RCOK(rclc_node_init_default(&node, envGet("node", NODE_NAME), "", &support)))
        return false;
    // The 50 Hz topic takes the env's QoS, best effort unless told otherwise.
    const bool best_effort = envFlag("best_effort", true);
    if (!RCOK((best_effort ? rclc_publisher_init_best_effort : rclc_publisher_init_default)(
            &odom_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), "odom/unfiltered")))
        return false;
    size_t handles = 2;
    if (stamped_cmd_vel) {
        if (!RCOK(rclc_subscription_init_default(&twist_stamped_sub, &node,
                ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, TwistStamped), "cmd_vel")))
            return false;
        if (!RCOK(rclc_subscription_init_default(&twist_sub, &node,
                ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel_unstamped")))
            return false;
        handles = 3;
    } else if (!RCOK(rclc_subscription_init_default(&twist_sub, &node,
                   ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel"))) {
        return false;
    }
    if (!RCOK(rclc_timer_init_default2(&control_timer, &support, RCL_MS_TO_NS(CONTROL_MS),
                                       controlCallback, true)))
        return false;
    executor = rclc_executor_get_zero_initialized_executor();
    if (!RCOK(rclc_executor_init(&executor, &support.context, handles, &allocator)))
        return false;
    if (stamped_cmd_vel &&
        !RCOK(rclc_executor_add_subscription(&executor, &twist_stamped_sub, &twist_stamped_msg,
                                             twistStampedCallback, ON_NEW_DATA)))
        return false;
    if (!RCOK(rclc_executor_add_subscription(&executor, &twist_sub, &twist_msg, twistCallback, ON_NEW_DATA)))
        return false;
    if (!RCOK(rclc_executor_add_timer(&executor, &control_timer)))
        return false;
    rmw_uros_sync_session(1000);
    last_control_ms = k_uptime_get();
    return true;
}

static void destroyEntities(void)
{
    rmw_context_t *ctx = rcl_context_get_rmw_context(&support.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(ctx, 0);
    (void)rcl_publisher_fini(&odom_pub, &node);
    (void)rcl_subscription_fini(&twist_sub, &node);
    if (stamped_cmd_vel)
        (void)rcl_subscription_fini(&twist_stamped_sub, &node);
    (void)rcl_timer_fini(&control_timer);
    (void)rclc_executor_fini(&executor);
    (void)rcl_node_fini(&node);
    (void)rclc_support_fini(&support);
}

static void stopWheels(void)
{
    for (int m = 0; m < DRIVE_MOTORS; m++)
        driveSpin(m, 0);
}

int main(void)
{
    gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE);
    envInit();
    // The env's `app` picks what this image runs, as on the other boards.
    const char *app = envGet("app", "base");
    if (strcmp(app, "test_sensors") == 0)
        testSensors();
    if (strcmp(app, "base") != 0)
        printk("[base] app=%s is not in this image; running base\n", app);

    const char *base = envGet("base", "2wd");
    drive_enabled = strcmp(base, "2wd") == 0;
    if (!drive_enabled)
        printk("[base] env base=%s: this board drives two wheels only; holding the motors\n", base);
    stamped_cmd_vel = envFlag("stamped_cmd_vel", false);   // jazzy: unstamped, as the Arduino firmware
    twist_stamped_msg.header.frame_id.data = twist_stamped_frame;
    twist_stamped_msg.header.frame_id.capacity = sizeof(twist_stamped_frame);

    kinematics = new Kinematics(Kinematics::DIFFERENTIAL_DRIVE,
                                envInt("max_rpm", 140), envFloat("rpm_ratio", 0.80f),
                                envFloat("motor_v", 24.0f), envFloat("power_v", 12.0f),
                                envFloat("wheel_d", 0.1f), envFloat("lr_dist", 0.271f),
                                0.0f, envFloat("angular_scale", 1.0f));
    if (!driveInit())
        printk("[base] some drive outputs failed to initialise\n");
    const float pmax = (float)drivePwmMax();
    for (int m = 0; m < DRIVE_MOTORS; m++)
        pid[m] = new PID(-pmax, pmax, envFloat("kp", 0.6f), envFloat("ki", 0.8f), envFloat("kd", 0.5f));
    initOdomMsg();
    printk("[base] linorobot2 UNO Q base: %s, %s cmd_vel\n", base, stamped_cmd_vel ? "stamped" : "unstamped");

    // The link's speed is the env's `baud`, 921600 by default as the Arduino serial
    // boards. At the board file's 115200 one Odometry message (~730 bytes) takes 63 ms
    // to send against a 20 ms control period: the polled transport write then owns the
    // CPU and odometry never gets out.
    const struct device *link = DEVICE_DT_GET(DT_NODELABEL(lpuart1));
    struct uart_config link_cfg;
    const int baud = envInt("baud", 921600);
    if (uart_config_get(link, &link_cfg) == 0) {
        link_cfg.baudrate = (uint32_t)baud;
        const int rc = uart_configure(link, &link_cfg);
        printk("[base] lpuart1 -> /dev/ttyHS1 at %d baud%s\n", baud, rc ? " FAILED" : "");
    }

    rmw_uros_set_custom_transport(MICRO_ROS_FRAMING_REQUIRED, (void *)&default_params,
                                  zephyr_transport_open, zephyr_transport_close,
                                  zephyr_transport_write, zephyr_transport_read);

    enum { WAITING, CONNECTED } state = WAITING;
    int64_t last_ping = 0;
    for (;;) {
        if (state == WAITING) {
            stopWheels();
            if (rmw_uros_ping_agent(100, 1) == RMW_RET_OK) {
                if (createEntities()) {
                    printk("[base] agent connected\n");
                    state = CONNECTED;
                    last_ping = k_uptime_get();
                } else {
                    destroyEntities();
                }
            }
            k_msleep(200);
            continue;
        }
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));
        const int64_t now = k_uptime_get();
        if (now - last_ping > 1000) {
            last_ping = now;
            if (rmw_uros_ping_agent(50, 3) != RMW_RET_OK) {
                printk("[base] agent lost\n");
                stopWheels();
                destroyEntities();
                state = WAITING;
                continue;
            }
            if (!rmw_uros_epoch_synchronized())
                rmw_uros_sync_session(100);
        }
        k_yield();
    }
}
