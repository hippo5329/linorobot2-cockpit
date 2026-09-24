// A genuine micro-ROS client on Linux: rclc + rmw_microxrcedds over UDP4.
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
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/int32.h>

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
    const char *agent_ip = (argc > 1) ? argv[1] : "127.0.0.1";
    const char *agent_port = (argc > 2) ? argv[2] : "8888";
    printf("micro-ROS client -> udp4 %s:%s\n", agent_ip, agent_port);
    fflush(stdout);

    rcl_allocator_t allocator = rcl_get_default_allocator();

    // The agent address is set on the INIT OPTIONS, before the session is opened.
    // This is the host equivalent of the firmware reading agent_ip/agent_port out
    // of its env block.
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_options, allocator));
    rmw_init_options_t *rmw_options = rcl_init_options_get_rmw_init_options(&init_options);
    RCCHECK(rmw_uros_options_set_udp_address(agent_ip, agent_port, rmw_options));

    // This is where a missing agent must stop us: session establishment.
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
    RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(500), on_timer));

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
