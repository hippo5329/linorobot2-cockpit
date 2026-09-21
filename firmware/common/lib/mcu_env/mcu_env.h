// U-Boot-style environment block, read from its own flash partition.
//
// Wi-Fi credentials and the address of the machine running micro_ros_agent are
// properties of the SITE, not of the robot. Compiling them into the application
// image produces a binary that only works on one LAN, which is fatal for the
// prebuilt images in firmware/prebuilt/: a user who has to rebuild to enter
// their own SSID gets no benefit from a prebuilt image at all.
//
// So they live in the `env` partition instead (firmware/common/partitions_lino.csv),
// in exactly the U-Boot on-flash layout:
//
//     uint32  CRC32 of everything after it, little-endian
//     bytes   "key=value\0key=value\0...\0\0", padded with 0xFF
//
// Written by scripts/mcu_env.py and flashed independently of the application,
// so `esptool write_flash 0x10000 firmware.bin` never disturbs the keys and
// re-keying a robot never involves a compiler.
#ifndef MCU_ENV_H
#define MCU_ENV_H

#include <Arduino.h>

// Loads and CRC-checks the env partition once. Safe to call repeatedly, and
// safe to call before WiFi is up. NOT safe to call from a static constructor:
// the flash partition API is not ready that early, which is why syslog takes
// its server address in initSyslog() rather than at construction.
void initMcuEnv(void);

// true when the partition existed and its CRC32 matched. When it is false every
// accessor below returns its fallback, so a board with a blank or corrupt env
// still boots and still says so over serial.
bool mcuEnvValid(void);

// Accessors. Each returns `fallback` when the key is absent or the env is
// invalid, so a caller never has to check mcuEnvValid() first.
const char *envGet(const char *key, const char *fallback = "");
int         envInt(const char *key, int fallback);   // pins: -1 means not wired
bool        envFlag(const char *key, bool fallback); // "0"/"false"/"no" are false
uint16_t    envU16(const char *key, uint16_t fallback);
uint32_t    envU32(const char *key, uint32_t fallback);
float       envFloat(const char *key, float fallback);
// Diagonal covariances: one value expands to every axis, or give the whole
// list. False when the key is absent, so the caller keeps its own default.
bool        envFloatVec(const char *key, float *out, int n);
IPAddress   envIP(const char *key, IPAddress fallback);

// ---------------------------------------------------------------------------
// The robot's namespace, from the env's `topic_prefix`.
//
// It applies to BOTH the topic names the board publishes and the frame_ids it
// stamps inside them, and it has to: a namespaced robot whose messages say
// `frame_id: odom` describes a frame that does not exist in its own TF tree,
// where robot_state_publisher has published `<prefix>/odom`. The EKF then finds
// no transform relating the two, ignores the input, and publishes nothing --
// while looking entirely healthy. Measured on the two-robot bench, 2026-09-21.
//
// Cached by the suffix's ADDRESS: every caller passes a string literal, so the
// pointer is stable and a reconnect reuses the buffer instead of consuming the
// arena again. The arena is heap and is allocated on the first PREFIXED name,
// so an unprefixed board -- the default, and every prebuilt image -- pays no
// DRAM for the feature at all (see docs/firmware.md on dram0_0_seg).
//
// envPrefixInit() supplies the compiled-in fallback for a board with a blank
// env; call it once from setup(), before anything asks for a name. Reading the
// env any earlier is the static-initialisation trap initSyslog() documents.
void        envPrefixInit(const char *compiled_fallback);
const char *envPrefixed(const char *suffix);

#endif // MCU_ENV_H
