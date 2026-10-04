cmake_minimum_required(VERSION 3.20.0)
list(APPEND ZEPHYR_EXTRA_MODULES /work/micro_ros_zephyr_module/modules/libmicroros)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(unoq_uros)
target_sources(app PRIVATE src/main.c)
# rcutils headers ask for C11 Annex K; picolibc + Zephyr 4.4 cannot provide it (see the module patch).
target_compile_definitions(app PRIVATE __STDC_WANT_LIB_EXT1__=0)
