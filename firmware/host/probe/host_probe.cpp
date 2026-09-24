// The robot computer as a micro-ROS client, brought up the way a board is.
//
// WHY THIS EXISTS. micro_ros_setup's `host` platform builds the demo SOURCES
// against the host's ordinary ROS 2 stack, so its int32_publisher links
// librmw_implementation from /opt/ros and publishes over Fast DDS. It happily
// produced a topic in `ros2 topic list` WITH NO AGENT RUNNING -- a green that
// says nothing about micro-ROS. This links rmw_microxrcedds explicitly, so if it
// publishes, it published through the agent.
//
// The proof is the CONTROL: run it with no agent and it must fail to init. A
// transport test that passes without the far end is not a transport test.
//
// WHAT IS UNDER TEST. Not this file. This file is the sequence a board's setup()
// runs, and the code it calls is the firmware's own, compiled unmodified:
//
//     initMcuEnv()        firmware/common/lib/mcu_env/mcu_env.cpp
//                         reads the SAME 4096-byte image scripts/mcu_env.py
//                         flashes, mmap'd from a file instead of from a
//                         partition (LINO_ENV_BIN, default ./env.bin)
//     initUrosTransport() firmware/common/lib/uros_transport/uros_transport.cpp
//                         picks udp4 from the env's `transport` key and installs
//                         the firmware's four transport functions
//
// So a `transport=udp4` regression in either file fails here, on a host, in
// seconds -- which is the whole point of the target. See firmware/host/README.md.
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <unistd.h>

#include <Arduino.h>
#include "mcu_env.h"
#include "uros_transport.h"

extern "C" {
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/int32.h>
}

// The shim declares it extern; every host binary defines it once, as the Arduino
// core does for a board.
_LinoHostSerial Serial;

#define RCCHECK(fn) { \
    rcl_ret_t rc = fn; \
    if (rc != RCL_RET_OK) { printf("FAILED line %d: rc=%d\n", __LINE__, (int)rc); return 1; } }

static rcl_publisher_t publisher;
static std_msgs__msg__Int32 msg;

static void on_timer(rcl_timer_t *timer, int64_t last)
{
    (void)last;
    if (!timer) return;
    rcl_ret_t rc = rcl_publish(&publisher, &msg, NULL);
    printf("publish rc=%d value=%d\n", (int)rc, (int)msg.data);
    fflush(stdout);
    msg.data++;
}

int main(int argc, const char *argv[])
{
    // Like a board: the env image decides, and it is the ONLY thing that decides.
    // argv names which image to read -- it does not override any key in it. The
    // CONTROL half of the proof therefore points at a dead agent the same way a
    // real deployment points at a live one, by building an image that says so
    // (`mcu_env.py build --set agent_port=...`), so both halves exercise the env
    // path. An --agent-ip flag here would be a second source of truth for a value
    // the env already carries, and the first thing the pair would do is disagree.
    if (argc > 1) {
        printf("[probe] env image: %s\n", argv[1]);
        setenv("LINO_ENV_BIN", argv[1], 1);   // read by mcu_env.cpp's host loader
    }

    initMcuEnv();
    if (!mcuEnvValid())
        printf("[probe] no valid env image — using the compiled-in host defaults\n");

    // The firmware's own transport selection, reading the firmware's own env.
    const bool udp = initUrosTransport();
    if (!udp) {
        printf("[probe] transport is not udp4 — the host target cannot run serial. "
               "Set transport=udp4 in the env image.\n");
        return 2;
    }

    rcl_allocator_t allocator = rcl_get_default_allocator();
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_options, allocator));

    // NOTE: no rmw_uros_options_set_udp_address() here, and that absence is the
    // point. That call configures the rmw's OWN built-in UDP socket, which would
    // have bypassed initUrosTransport() entirely -- the firmware's four functions
    // would have been installed and never called. With the rmw built
    // RMW_UXRCE_TRANSPORT=custom there is no built-in socket to configure: the
    // address came from the env, and every byte goes through uros_transport.cpp.
    rclc_support_t support;
    RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));
    printf("SESSION ESTABLISHED with the agent\n");
    fflush(stdout);

    rcl_node_t node;
    RCCHECK(rclc_node_init_default(&node, "lino_host_probe", "", &support));
    RCCHECK(rclc_publisher_init_default(
        &publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "lino_host_probe"));

    rcl_timer_t timer;
    // ..._default2 with autostart: _default is deprecated in this rclc.
    RCCHECK(rclc_timer_init_default2(&timer, &support, RCL_MS_TO_NS(500), on_timer, true));

    rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));

    msg.data = 0;
    for (int i = 0; i < 40; i++) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        usleep(100000);
    }
    printf("PROBE_OK published %d messages\n", (int)msg.data);
    return 0;
}
