cmake_minimum_required(VERSION 3.20.0)
list(APPEND ZEPHYR_EXTRA_MODULES /work/micro_ros_zephyr_module/modules/libmicroros)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(unoq_fw)
# The linorobot2 firmware itself -- src/main.cpp, common/lib, src/tools -- compiled
# unmodified for the UNO Q's STM32U585, against shim/ (the Arduino surface it uses),
# as firmware/host/app does for the robot computer. build.sh passes:
#   FIRMWARE_ROOT  the firmware tree
#   LINO_HEADER    gen_firmware_header.py --mcu unoq
#   LINO_LIBDEPS   fetch_libdeps.py's directory (I2Cdevlib, SparkFun BNO080)
foreach(v FIRMWARE_ROOT LINO_HEADER LINO_LIBDEPS)
  if(NOT DEFINED ${v})
    message(FATAL_ERROR "${v} is not set (see build.sh)")
  endif()
endforeach()
file(GLOB LIB_SOURCES ${FIRMWARE_ROOT}/common/lib/*/*.cpp)
file(GLOB TOOL_SOURCES ${FIRMWARE_ROOT}/src/tools/*.cpp)
file(GLOB LIB_DIRS LIST_DIRECTORIES true ${FIRMWARE_ROOT}/common/lib/*)
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
# Every source in src/, as PlatformIO compiles it -- not main.cpp by name (base_ident.cpp).
file(GLOB APP_SOURCES ${FIRMWARE_ROOT}/src/*.cpp)
target_sources(app PRIVATE
  src/zephyr_core.cpp src/uart_link.cpp src/pins.cpp src/lidar_uart.cpp
  ${APP_SOURCES} ${LIB_SOURCES} ${TOOL_SOURCES} ${DEP_SOURCES})
if(NOT DEFINED FW_GIT_REV)
  set(FW_GIT_REV "unknown")
endif()
string(TIMESTAMP FW_BUILD_DATE "%Y-%m-%d" UTC)
# The micro-ROS distro this image links (build.sh MICROROS_DISTRO). It sets the banner and
# the default cmd_vel type (Twist through jazzy, TwistStamped after), so it is never
# assumed: a lyrical image that said "jazzy" here listened for Twist while lyrical's Nav2
# sent TwistStamped, and the robot never moved (2026-10-05).
if(NOT DEFINED FW_ROS_DISTRO)
  message(FATAL_ERROR "FW_ROS_DISTRO is not set (build.sh passes MICROROS_DISTRO)")
endif()
target_compile_definitions(app PRIVATE
  LINO_ZEPHYR=1
  ARDUINO=10819
  CONFIG_PATH="${LINO_HEADER}"
  FW_GIT_REV="${FW_GIT_REV}"
  FW_BUILD_DATE="${FW_BUILD_DATE}"
  FW_ROS_DISTRO="${FW_ROS_DISTRO}"
  # rcutils headers ask for C11 Annex K; picolibc + Zephyr 4.4 cannot provide it.
  __STDC_WANT_LIB_EXT1__=0)
# firmware/include first, so its config.h (which includes CONFIG_PATH) is the one found.
target_include_directories(app PRIVATE
  ${FIRMWARE_ROOT}/include ${FIRMWARE_ROOT}/src shim src ${LIB_DIRS} ${DEP_INCLUDES})
# Third-party Arduino libraries and the shared firmware are not written for
# -Wdouble-promotion and friends; their warnings are not this target's to fix.
target_compile_options(app PRIVATE -Wno-double-promotion -Wno-unused-parameter)
