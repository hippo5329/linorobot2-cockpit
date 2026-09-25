// micro_ros_platformio.h for the HOST target.
//
// On a board this is micro_ros_platformio's umbrella header, and what the
// firmware uses from it is rmw_microros's API (rmw_uros_set_custom_transport,
// rmw_uros_ping_agent, the session and epoch calls). On a colcon build that API
// is rmw_microros itself, from the same rmw_microxrcedds -- so this forwards
// there and main.cpp compiles as it is.
#ifndef LINO_HOST_MICRO_ROS_PLATFORMIO_H
#define LINO_HOST_MICRO_ROS_PLATFORMIO_H
#include <rmw_microros/rmw_microros.h>
#endif
