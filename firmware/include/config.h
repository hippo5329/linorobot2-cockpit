#ifndef CONFIG_H
#define CONFIG_H

#ifdef CONFIG_PATH
    #include CONFIG_PATH
#else
    #include "custom/lino_base_config.h"
#endif

// ---------------------------------------------------------------------------
// Defaults the generated header is NOT allowed to supply.
//
// A released image is built for a silicon and describes no robot, so its
// header names no LiDAR, no host and no wiring. Anything the code still needs
// a compile-time answer for has to have one HERE, where it can be justified
// per silicon rather than borrowed from whichever robot built the image.
//
// Every one of these is overridden by an env key at flash time; they are only
// the answer for a board whose env has never been written.

// Which sink the synthetic scan takes by default. Bandwidth, so silicon:
//
// An RP2 carries the scan over micro-ROS and still holds the control loop --
// measured on the bench 2026-09-20, /odom and /imu at 50.0 Hz with /raw_scan
// at 85-100 Hz alongside -- so a bare Pico needs no wiring at all to produce
// one. An ESP32 cannot: the same configuration drops every topic to 40-45 Hz
// because 921600 baud carries the scan and the 50 Hz loop together (33 Hz on
// a GenDrv that also reads four I2C sensors). Its scan has to leave by a UART
// or the radio, and a bare ESP32 has neither wired, so `serial` is the honest
// default rather than one that quietly halves the control rate.
#ifndef LIDAR_COMM_DEFAULT
#if defined(ARDUINO_ARCH_RP2040)
#define LIDAR_COMM_DEFAULT "topic"
#else
#define LIDAR_COMM_DEFAULT "serial"
#endif
#endif

#endif
