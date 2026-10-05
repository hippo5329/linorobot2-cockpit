cmake_minimum_required(VERSION 3.20.0)
list(APPEND ZEPHYR_EXTRA_MODULES /work/micro_ros_zephyr_module/modules/libmicroros)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(unoq_base)
# The 2WD base controller for the UNO Q's STM32U585. Kinematics and PID are the
# Arduino firmware's own files (firmware/common/lib), compiled against a shim
# Arduino.h; build.sh copies them to common/ beside this app.
target_sources(app PRIVATE
    src/main.cpp src/env.cpp src/drive.cpp src/test_sensors.cpp src/uart_link.cpp
    common/kinematics/kinematics.cpp common/pid/pid.cpp)
target_include_directories(app PRIVATE src src/shim common/kinematics common/pid)
# rcutils headers ask for C11 Annex K; picolibc + Zephyr 4.4 cannot provide it (see the module patch).
target_compile_definitions(app PRIVATE __STDC_WANT_LIB_EXT1__=0)
