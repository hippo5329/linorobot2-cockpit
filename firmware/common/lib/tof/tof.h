// The 8x8 multizone time-of-flight sensor on the I2C bus: DFRobot SEN0628 (ST VL53L7CX
// behind an RP2040 that runs the sensor and answers commands).
//
// The board speaks DFRobot's command protocol (DFRobot_MatrixLidar, MIT): a request is
// 0x55, the argument count (two bytes, big end first, counting the command), the command
// and its arguments; the reply, read a byte at a time until it is ready, is a status
// (0x53 ok, 0x63 failed; 0xFF "not yet"), the command, the payload length (two bytes,
// little end first) and the payload. All 64 zones (CMD_ALLData) are 128 bytes: 64 x
// uint16 millimetres, row-major, 4000 = no valid reading.
//
// DFRobot's library waits with delay(): 17 ms per poll, 5 s after a mode change. This
// driver never does -- tofPoll() takes one small step and returns, so the micro-ROS loop
// keeps its timing. The address comes from the env (tof_addr, mcu_env.py), never from a
// probe guess: 0x30 is also an MMC5603's.
#pragma once
#include <stdint.h>
#include <stddef.h>

#define TOF_ZONES 64
#define TOF_FRAME_BYTES (TOF_ZONES * 2)

// Is a board answering at `addr` (0 = none configured)? Sends the 8x8 mode command when it
// is. Returns false when nothing is configured or nothing ACKs; tofPresent() says the same.
bool initTof(uint8_t addr);
bool tofPresent();
// One step of the read cycle; call every loop. Cheap when there is nothing to do.
void tofPoll();
// Copies the newest complete frame (128 bytes) into `out` and clears it: true once per frame.
bool tofTakeFrame(uint8_t *out);
// Frames read since boot, and failed exchanges (timeouts, error replies), for the banner.
uint32_t tofFrames();
uint32_t tofErrors();
