// A second UART as a diagnostic channel while micro-ROS owns the first.
//
// A board that connects, advertises its topics and then publishes nothing looks
// exactly like a healthy one from the agent's side, and the console it would
// explain itself on is the micro-ROS link. The GenDrv's LIDAR_SERIAL is a
// second UART whose TX can go out the LiDAR pin into a USB bridge, and with no
// LiDAR on that UART it is free -- so the firmware can say, once a second, what
// its loop is actually doing: how often loop() ran, how often the control timer
// fired, how many publishes went out and failed, how many bytes the transport
// wrote and read, and what the executor returned.
//
// Off unless the env says otherwise: `diag_tx=<gpio>` opens the UART at
// `diag_baud` (230400). Costs nothing when off -- the counters are increments
// and the tick is one comparison.
#ifndef DIAG_H
#define DIAG_H

#include <Arduino.h>

enum DiagCounter {
    DIAG_LOOP = 0,   // loop() iterations
    DIAG_TIMER,      // control timer callbacks
    DIAG_PUBLISH,    // publishData() completions
    DIAG_PUBFAIL,    // rcl calls that returned an error (RCSOFTCHECK)
    DIAG_TX_BYTES,   // transport write, bytes
    DIAG_RX_BYTES,   // transport read, bytes
    DIAG_RX_CALLS,   // transport read calls
    DIAG_RX_EMPTY,   // transport read calls that returned nothing
    DIAG_PING_OK,
    DIAG_PING_FAIL,
    DIAG_N
};

void diagBegin(void);
bool diagEnabled(void);
void diagCount(DiagCounter c, uint32_t n = 1);
// The executor's return code and how long the call took.
void diagSpin(int rc, uint32_t elapsed_us);
// Longest duration per second of a named stretch of the control path.
enum DiagTimer { DIAGT_MOVE = 0, DIAGT_SENSORS, DIAGT_PUB, DIAGT_N };
void diagTime(DiagTimer t, uint32_t elapsed_us);
void diagState(int state);
// Prints one line a second. Call from loop().
void diagTick(void);
// Free-form line, when something worth saying happens once.
void diagPrintf(const char *fmt, ...);

#endif // DIAG_H
