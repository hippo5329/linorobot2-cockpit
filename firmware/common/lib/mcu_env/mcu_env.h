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
uint16_t    envU16(const char *key, uint16_t fallback);
uint32_t    envU32(const char *key, uint32_t fallback);
IPAddress   envIP(const char *key, IPAddress fallback);

#endif // MCU_ENV_H
