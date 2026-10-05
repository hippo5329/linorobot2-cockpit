// The robot's env block, read in place from flash.
//
// The layout is the Arduino firmware's (firmware/common/lib/mcu_env/mcu_env.h): a
// little-endian CRC32 of the 4 KB that follow it, then "key=value\0...\0\0" padded
// with 0xFF -- written by scripts/mcu_env.py, so one tool keys every board. A blank
// or corrupt block is not fatal: every accessor returns its fallback.
#pragma once
#include <stdbool.h>
#include <stdint.h>

void        envInit(void);
bool        envValid(void);
const char *envGet(const char *key, const char *fallback);
int         envInt(const char *key, int fallback);
float       envFloat(const char *key, float fallback);
bool        envFlag(const char *key, bool fallback);
// Diagonal covariances: one value fills every axis, or give all n ("a,b,c"). False when
// the key is absent, so the caller keeps its own default -- as mcu_env.h.
bool        envFloatVec(const char *key, float *out, int n);   // "0"/"false"/"no" are false, as mcu_env.cpp
