cmake_minimum_required(VERSION 3.16)
project(lino_host_probe C CXX)

find_package(ament_cmake REQUIRED)
find_package(rcl REQUIRED)
find_package(rclc REQUIRED)
find_package(rmw_microxrcedds REQUIRED)
find_package(microxrcedds_client REQUIRED)
find_package(std_msgs REQUIRED)

# The firmware tree, reached as it sits in the repo. FIRMWARE_ROOT is passed by
# the build script because this package is COPIED into a colcon workspace (the
# ROS 2 build cannot reach outside it), so a relative path to ../../.. would
# resolve to the workspace and silently compile nothing.
if(NOT DEFINED FIRMWARE_ROOT)
  message(FATAL_ERROR "FIRMWARE_ROOT is not set: pass -DFIRMWARE_ROOT=<repo>/firmware")
endif()
if(NOT EXISTS ${FIRMWARE_ROOT}/common/lib/mcu_env/mcu_env.cpp)
  message(FATAL_ERROR "FIRMWARE_ROOT=${FIRMWARE_ROOT} does not look like the firmware tree")
endif()

# THE SOURCES UNDER TEST. Compiled from the firmware tree unmodified -- not
# copied, not adapted. A second copy of either would make this instrument lie,
# which is the fault tests/test_sim_base_node.py exists to prevent for the
# Python simulated base and that this target must not reintroduce.
add_executable(lino_host_probe
  host_probe.cpp
  ${FIRMWARE_ROOT}/common/lib/mcu_env/mcu_env.cpp
  ${FIRMWARE_ROOT}/common/lib/uros_transport/uros_transport.cpp
  ${FIRMWARE_ROOT}/common/lib/diag/diag.cpp
)

# host/shim FIRST: <Arduino.h> and "config.h" must resolve to the shim's, and it
# is the shim's Arduino.h that defines LINO_HOST for every one of these TUs.
target_include_directories(lino_host_probe PRIVATE
  ${FIRMWARE_ROOT}/host/shim
  ${FIRMWARE_ROOT}/common/lib/mcu_env
  ${FIRMWARE_ROOT}/common/lib/uros_transport
  ${FIRMWARE_ROOT}/common/lib/diag
)

# Not ament_target_dependencies(): Lyrical removed it. Same rule as ../app.
foreach(pkg rcl rclc rmw_microxrcedds std_msgs)
  if(TARGET ${pkg}::${pkg})
    target_link_libraries(lino_host_probe ${pkg}::${pkg})
  elseif(DEFINED ${pkg}_TARGETS)
    target_link_libraries(lino_host_probe ${${pkg}_TARGETS})
  else()
    target_include_directories(lino_host_probe PRIVATE ${${pkg}_INCLUDE_DIRS})
    target_link_libraries(lino_host_probe ${${pkg}_LIBRARIES})
  endif()
endforeach()

# uros_transport.cpp calls uxr_millis() directly. microxrcedds_client is a plain
# CMake package rather than an ament one, so ament_target_dependencies above finds
# no microxrcedds_client_LIBRARIES to add and the library reaches the link line
# only transitively through the rmw -- where --as-needed drops it, because at that
# point nothing has asked for the symbol yet. Its exported IMPORTED target is a
# bare `microxrcedds_client`; naming it puts the library after our objects, where
# the reference is live.
target_link_libraries(lino_host_probe microxrcedds_client)
install(TARGETS lino_host_probe DESTINATION lib/${PROJECT_NAME})
ament_package()
