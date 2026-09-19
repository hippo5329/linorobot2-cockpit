#include <Arduino.h>
#include "config.h"
#include "mcu_env.h"

// ---------------------------------------------------------------------------
// The block itself is MCU-independent: a CRC32 followed by NUL-separated
// "key=value" entries. What differs per MCU is only where that block lives and
// how it is reached, so the loader is per-MCU and everything below it is shared.
//
//   ESP32/S3   the `env` partition (firmware/common/partitions_lino.csv),
//              mmap'd into the DROM window and parsed in place.
//   RP2040/2350 the 4 KB EEPROM sector arduino-pico reserves at the very top of
//              flash. It is memory-mapped, so it is parsed in place.
// ---------------------------------------------------------------------------

#define ENV_SIZE      0x1000
#define ENV_CRC_LEN   4
#define ENV_DATA_LEN  (ENV_SIZE - ENV_CRC_LEN)

// Points at the entry block once a loader has validated it: XIP-mapped flash on
// both backends now. NULL until then.
static const char *env_data = NULL;
static bool env_loaded = false;
static bool env_valid = false;

// Plain CRC-32/ISO-HDLC, the same one U-Boot and Python's zlib.crc32 use. Done
// by hand rather than through esp_rom_crc32_le() because that function's
// pre/post inversion convention has changed between IDF releases, and a CRC that
// disagrees with scripts/mcu_env.py by an inversion looks exactly like a corrupt
// partition.
static uint32_t crc32_iso(const uint8_t *data, size_t len)
{
    uint32_t crc = 0xFFFFFFFFu;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; bit++)
            crc = (crc >> 1) ^ (0xEDB88320u & (uint32_t) - (int32_t)(crc & 1));
    }
    return ~crc;
}

#if defined(ESP32) || defined(ARDUINO_ARCH_ESP32)

#include <esp_partition.h>

// Mirrors the `env` row of firmware/common/partitions_lino.csv. The partition is
// found by name rather than by offset so that changing the CSV cannot silently
// point this at the wrong region -- a lookup that fails is loud, an offset that
// drifts is not.
#define ENV_PART_NAME    "env"
#define ENV_PART_SUBTYPE ((esp_partition_subtype_t)0x99)

// Mapped, not copied -- the same trick the RP2 path below has always used.
//
// This used to be `static char env_buf[ENV_DATA_LEN]`: 4 KB of .bss holding a
// permanent copy of a partition that never changes while the board runs. The
// ESP32's entire static segment is 124580 bytes (memory.ld: 0x2c200 - 0xdb5c),
// and the lyrical Wi-Fi image sits at 98% of it, so 4 KB was the difference
// between a comfortable margin and a board that stops building the next time
// anyone adds a publisher.
//
// esp_partition_mmap() puts the partition in the DROM address space instead,
// so entries are parsed straight out of XIP flash and the copy costs NOTHING
// -- not DRAM, not heap. Reads go through the flash cache, which is ample for
// a handful of envGet() lookups at boot. The mapping is never unmapped: it
// must outlive every caller holding the pointer, which is the whole run.
//
// The handle type and the mmap enum were renamed between IDF 4 and 5, and
// `platform = espressif32` is unpinned in platformio_base.ini and resolves to
// either -- so both spellings are kept. Getting this wrong is a compile error,
// not a silent fault, but only on whichever core the next `pio pkg update`
// happens to pull.
#if ESP_IDF_VERSION_MAJOR >= 5
typedef esp_partition_mmap_handle_t env_mmap_handle_t;
#define ENV_MMAP_DATA ESP_PARTITION_MMAP_DATA
#else
typedef spi_flash_mmap_handle_t env_mmap_handle_t;
#define ENV_MMAP_DATA SPI_FLASH_MMAP_DATA
#endif
static env_mmap_handle_t env_map_handle;

static void loadEnv(void)
{
    const esp_partition_t *part = esp_partition_find_first(
        ESP_PARTITION_TYPE_DATA, ENV_PART_SUBTYPE, ENV_PART_NAME);
    if (!part) {
        Serial.println("[env] no 'env' partition — flash one with scripts/mcu_env.py");
        return;
    }
    if (part->size < ENV_SIZE) {
        Serial.printf("[env] 'env' partition is %u bytes, expected %u\n",
                      (unsigned)part->size, (unsigned)ENV_SIZE);
        return;
    }

    const void *mapped = NULL;
    esp_err_t err = esp_partition_mmap(part, 0, ENV_SIZE, ENV_MMAP_DATA,
                                       &mapped, &env_map_handle);
    if (err != ESP_OK || !mapped) {
        Serial.printf("[env] could not map the 'env' partition (%d) — "
                      "using built-in defaults\n", (int)err);
        return;
    }

    // The CRC covers the data only; the stored value is the first word.
    uint32_t stored = 0;
    memcpy(&stored, mapped, ENV_CRC_LEN);
    const char *data = (const char *)mapped + ENV_CRC_LEN;

    uint32_t actual = crc32_iso((const uint8_t *)data, ENV_DATA_LEN);
    if (stored != actual) {
        Serial.printf("[env] CRC32 mismatch (flash %08x, computed %08x) — "
                      "the env partition is blank or corrupt\n",
                      (unsigned)stored, (unsigned)actual);
        return;
    }
    env_data = data;
    env_valid = true;
}

#elif defined(ARDUINO_ARCH_RP2040)

// The RP2 boards have no partition table, but arduino-pico already reserves a
// region that suits this exactly: the last 4 KB of flash, the sector its EEPROM
// emulation uses. Its builder computes
//
//     eeprom_start        = 0x10000000 + flash_size - 4096
//     maximum_sketch_size = flash_size - 4096 - filesystem_size
//
// so the sector sits above both the sketch and the filesystem on EVERY RP2
// board, whatever the flash size and whether or not a filesystem is configured.
// That placement is what makes the keys survive an update:
//
//   * `picotool load firmware.uf2` writes only the blocks the UF2 contains, and
//     a sketch UF2 contains none above maximum_sketch_size.
//   * arduino-pico's OTA stages the new image in the filesystem area and the
//     bootloader copies it down over the sketch region. Neither step reaches
//     the last sector.
//
// So flashing an application never disturbs the keys, which is the same
// guarantee the ESP32 `env` partition gives -- and re-keying a board still
// never involves a compiler. A full-chip erase (`picotool erase` with no range,
// or a flash_nuke UF2) does wipe it; nothing short of that does.
//
// The region is XIP-mapped, so unlike the ESP32 path this parses the entries
// straight out of flash and spends no RAM on a copy.
extern "C" uint8_t _EEPROM_start;

static void loadEnv(void)
{
    const uint8_t *base = (const uint8_t *)&_EEPROM_start;
    uint32_t stored;
    memcpy(&stored, base, ENV_CRC_LEN);

    uint32_t actual = crc32_iso(base + ENV_CRC_LEN, ENV_DATA_LEN);
    if (stored != actual) {
        Serial.printf("[env] CRC32 mismatch (flash %08x, computed %08x) — the env "
                      "sector at %p is blank or corrupt\n",
                      (unsigned)stored, (unsigned)actual, (const void *)base);
        return;
    }
    env_data = (const char *)(base + ENV_CRC_LEN);
    env_valid = true;
}

#else

static void loadEnv(void)
{
    Serial.println("[env] no env backend for this MCU — using compiled-in defaults");
}

#endif

void initMcuEnv(void)
{
    if (env_loaded)
        return;
    env_loaded = true;
    loadEnv();
}

bool mcuEnvValid(void)
{
    initMcuEnv();
    return env_valid;
}

const char *envGet(const char *key, const char *fallback)
{
    initMcuEnv();
    if (!env_valid || !env_data || !key)
        return fallback;

    const size_t klen = strlen(key);
    // Entries are NUL-separated; an empty entry ends the list, exactly as in
    // U-Boot. Walking rather than indexing keeps this independent of ordering.
    for (size_t pos = 0; pos < ENV_DATA_LEN && env_data[pos]; ) {
        const char *entry = &env_data[pos];
        size_t elen = strnlen(entry, ENV_DATA_LEN - pos);
        if (elen > klen && entry[klen] == '=' && strncmp(entry, key, klen) == 0)
            return entry + klen + 1;
        pos += elen + 1;
    }
    return fallback;
}

uint16_t envU16(const char *key, uint16_t fallback)
{
    const char *value = envGet(key, NULL);
    if (!value || !*value)
        return fallback;
    long parsed = strtol(value, NULL, 10);
    if (parsed < 0 || parsed > 65535)
        return fallback;
    return (uint16_t)parsed;
}

// 32 bits, because two of the things the env carries do not fit in 16: the I2C
// clock (400000) and the micro-ROS serial rate (up to 1500000). This used to be
// a static helper inside board_init.cpp, which meant main.cpp could not read a
// baud rate from the env at all.
uint32_t envU32(const char *key, uint32_t fallback)
{
    const char *value = envGet(key, NULL);
    if (!value || !*value)
        return fallback;
    char *end = NULL;
    unsigned long parsed = strtoul(value, &end, 10);
    if (end == value || parsed > 0xFFFFFFFFUL)
        return fallback;
    return (uint32_t)parsed;
}

IPAddress envIP(const char *key, IPAddress fallback)
{
    const char *value = envGet(key, NULL);
    IPAddress parsed;
    if (!value || !*value || !parsed.fromString(value))
        return fallback;
    return parsed;
}
