// The two things firmware/common/lib/{kinematics,pid} take from Arduino.h.
#pragma once
#include <math.h>
#include <stdint.h>
#ifndef PI
#define PI 3.14159265358979323846
#endif
template <class T, class L, class H>
static inline T constrain(T x, L lo, H hi) { return x < lo ? (T)lo : (x > hi ? (T)hi : x); }
