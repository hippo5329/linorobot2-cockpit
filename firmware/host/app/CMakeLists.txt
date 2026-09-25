cmake_minimum_required(VERSION 3.16)
project(lino_host_fw C CXX)

# The firmware -- src/main.cpp and every library it links on a board -- built
# for the robot computer as a micro-ROS UDP4 client: the Sim MCU. Same rules as
# ../probe: the sources are compiled from the firmware tree UNMODIFIED, the rmw
# must be RMW_UXRCE_TRANSPORT=custom so uros_transport.cpp's own functions carry
# every byte, and anything the firmware needs that a host lacks is emulated in
# ../shim rather than branched around in the firmware.
find_package(ament_cmake REQUIRED)
find_package(rcl REQUIRED)
find_package(rclc REQUIRED)
find_package(rmw_microxrcedds REQUIRED)
find_package(microxrcedds_client REQUIRED)
find_package(micro_ros_utilities REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

if(NOT DEFINED FIRMWARE_ROOT)
  message(FATAL_ERROR "FIRMWARE_ROOT is not set: pass -DFIRMWARE_ROOT=<repo>/firmware")
endif()
if(NOT EXISTS ${FIRMWARE_ROOT}/src/main.cpp)
  message(FATAL_ERROR "FIRMWARE_ROOT=${FIRMWARE_ROOT} does not look like the firmware tree")
endif()
# The header generated for the `host` silicon (gen_firmware_header.py --mcu
# host), exactly as a released image's is generated for its silicon.
if(NOT DEFINED LINO_HOST_HEADER OR NOT EXISTS ${LINO_HOST_HEADER})
  message(FATAL_ERROR "LINO_HOST_HEADER must name the header from gen_firmware_header.py --mcu host")
endif()

file(GLOB LIB_SOURCES ${FIRMWARE_ROOT}/common/lib/*/*.cpp)
file(GLOB TOOL_SOURCES ${FIRMWARE_ROOT}/src/tools/*.cpp)
file(GLOB LIB_DIRS LIST_DIRECTORIES true ${FIRMWARE_ROOT}/common/lib/*)

# The third-party Arduino libraries a board takes from platformio's lib_deps
# (common/platformio_base.ini): the I2Cdevlib drivers and SparkFun's BNO080. They
# are plain Arduino C++ over Wire, so they compile against the shim's Wire, whose
# bus is empty -- a host is a board with no chip fitted, which is what the Sim
# MCU is. LINO_LIBDEPS is a platformio .pio/libdeps/<env> directory; the
# micro-ROS library and the RP2 encoder library are the two it does not take
# (the host links the colcon micro-ROS client, and has no PIO state machine).
if(NOT DEFINED LINO_LIBDEPS OR NOT EXISTS ${LINO_LIBDEPS}/I2Cdevlib-Core/I2Cdev.cpp)
  message(FATAL_ERROR "LINO_LIBDEPS must be a platformio .pio/libdeps/<env> directory")
endif()
file(GLOB DEP_DIRS LIST_DIRECTORIES true ${LINO_LIBDEPS}/I2Cdevlib-* "${LINO_LIBDEPS}/SparkFun BNO080*")
set(DEP_SOURCES "")
set(DEP_INCLUDES "")
foreach(d ${DEP_DIRS})
  if(IS_DIRECTORY ${d})
    file(GLOB s ${d}/*.cpp ${d}/src/*.cpp)
    list(APPEND DEP_SOURCES ${s})
    list(APPEND DEP_INCLUDES ${d})
    if(IS_DIRECTORY ${d}/src)
      list(APPEND DEP_INCLUDES ${d}/src)
    endif()
  endif()
endforeach()

add_executable(lino_host_fw
  host_main.cpp
  ${FIRMWARE_ROOT}/src/main.cpp
  ${LIB_SOURCES}
  ${TOOL_SOURCES}
  ${DEP_SOURCES}
)
# The stamps a board build takes from common/build_stamp.py: the caller passes
# the same git revision (the builder may have no .git) and the date; the distro
# is the ROS the client library was built for. main.cpp says "unknown" for any
# that is missing rather than guessing.
set(FW_GIT_REV "unknown" CACHE STRING "7-character git revision of the firmware tree")
string(TIMESTAMP FW_BUILD_DATE_NOW "%Y-%m-%d" UTC)
set(FW_BUILD_DATE "${FW_BUILD_DATE_NOW}" CACHE STRING "build date")
# ARDUINO is what every Arduino toolchain defines (the IDE's version number), and
# what third-party libraries test to pick Arduino.h over WProgram.h.
target_compile_definitions(lino_host_fw PRIVATE
  ARDUINO=10819
  CONFIG_PATH="${LINO_HOST_HEADER}"
  FW_GIT_REV="${FW_GIT_REV}"
  FW_BUILD_DATE="${FW_BUILD_DATE}"
  FW_ROS_DISTRO="$ENV{ROS_DISTRO}")

# firmware/include FIRST so "config.h" is the firmware's own (the probe's shim
# config.h is for the probe alone); then the shim, for <Arduino.h> and friends.
target_include_directories(lino_host_fw PRIVATE
  ${FIRMWARE_ROOT}/include
  ${FIRMWARE_ROOT}/src
  ${FIRMWARE_ROOT}/host/shim
  ${LIB_DIRS}
  ${DEP_INCLUDES}
)

# Not ament_target_dependencies(): Lyrical removed that command ("Unknown CMake
# command"). A package's exported <pkg>::<pkg> target where it has one, else
# its <pkg>_LIBRARIES / _INCLUDE_DIRS -- rmw_microxrcedds exports no target.
# Messages through their <pkg>_TARGETS lists. Works on every distro since Humble.
function(lino_link tgt)
  foreach(pkg ${ARGN})
    if(TARGET ${pkg}::${pkg})
      target_link_libraries(${tgt} ${pkg}::${pkg})
    elseif(DEFINED ${pkg}_TARGETS)
      target_link_libraries(${tgt} ${${pkg}_TARGETS})
    else()
      target_include_directories(${tgt} PRIVATE ${${pkg}_INCLUDE_DIRS})
      target_link_libraries(${tgt} ${${pkg}_LIBRARIES})
    endif()
  endforeach()
endfunction()
lino_link(lino_host_fw rcl rclc rmw_microxrcedds micro_ros_utilities
  std_msgs geometry_msgs nav_msgs sensor_msgs)
# See ../probe/CMakeLists.txt: the transport calls uxr_millis() directly.
target_link_libraries(lino_host_fw microxrcedds_client pthread)
install(TARGETS lino_host_fw DESTINATION lib/${PROJECT_NAME})
ament_package()
