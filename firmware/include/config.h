#ifndef CONFIG_H
#define CONFIG_H

#ifdef CONFIG_PATH
    #include CONFIG_PATH
#else
    #include "custom/lino_base_config.h"
#endif

// ---------------------------------------------------------------------------
// What the SILICON can do. Never a choice about a robot: a released image is
// built per MCU, and everything a robot chooses is an env key read at boot.
//
// HAS_WIFI: a radio to talk over. Every ESP32 has one; on RP2 only the W
// boards, whose PlatformIO envs pass -D HAS_WIFI (a plain pico/pico2 build has
// no WiFi.h to compile against).
#if defined(ESP32) && !defined(HAS_WIFI)
#define HAS_WIFI 1
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

// The wheelbase, front axle to rear axle. Zero is the honest answer for a
// board that describes no robot: a 2wd base has no second axle, and it is the
// value that leaves the rotation radius at lr/2 -- the behaviour every image
// had before the wheelbase was read at all. A 4-wheel robot's env supplies the
// real number.
#ifndef FR_WHEELS_DISTANCE
#define FR_WHEELS_DISTANCE 0.0
#endif

// Skid-steer scrub. 1.0 is the ideal differential model, which is what a 2wd
// and a mecanum base use regardless. It is a measurement of tyres and floor,
// not a constant, so an image that describes no robot must not ship a guess:
// see Kinematics::rotationRadius for how to measure it.
#ifndef ANGULAR_SCALE
#define ANGULAR_SCALE 1.0
#endif

#endif
