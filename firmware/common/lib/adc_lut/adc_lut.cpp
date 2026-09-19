#include "adc_lut.h"

#if ADC_LUT_SUPPORTED

#include <esp_partition.h>
#include <esp_idf_version.h>
#include <string.h>

// esp_partition_mmap() was respelled in ESP-IDF v5: the handle type gained an
// `esp_partition_` prefix and the memory-type enum moved out of the spi_flash
// namespace. Arduino-ESP32 2.x is IDF 4.4 and 3.x is IDF 5.x, and this file is
// built under both.
#if ESP_IDF_VERSION_MAJOR >= 5
typedef esp_partition_mmap_handle_t lut_mmap_handle_t;
#define LUT_MMAP_DATA ESP_PARTITION_MMAP_DATA
#else
typedef spi_flash_mmap_handle_t lut_mmap_handle_t;
#define LUT_MMAP_DATA SPI_FLASH_MMAP_DATA
// v4 has no esp_partition_munmap(): the mapping is released through the
// spi_flash API that created the handle.
#define esp_partition_munmap spi_flash_munmap
#endif

// Mirrors the `adclut` row of firmware/common/partitions_lino.csv.
#define LUT_PART_NAME     "adclut"
#define LUT_PART_SUBTYPE  ((esp_partition_subtype_t)0x9a)
#define LUT_MAGIC         0x54554c41u          // "ALUT", little-endian
#define LUT_VERSION       1
#define LUT_SECTOR        0x1000
#define LUT_TABLE_OFFSET  LUT_SECTOR           // header occupies sector 0
#define LUT_TABLE_BYTES   (ADC_LUT_ENTRIES * (int)sizeof(int16_t))
#define LUT_PART_SIZE     (LUT_TABLE_OFFSET + LUT_TABLE_BYTES)

struct LutHeader {
    uint32_t magic;
    uint16_t version;
    uint16_t entries;
    uint32_t crc;
};

static const esp_partition_t *lut_part = NULL;
static const int16_t *lut_table = NULL;       // XIP-mapped, not a RAM copy
static lut_mmap_handle_t lut_map = 0;

// Same CRC32 as mcu_env: reflected, polynomial 0xEDB88320, no final xor, so the
// two stores can be verified by one implementation on the host side.
static uint32_t crc32_iso(const uint8_t *data, size_t len, uint32_t crc)
{
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; bit++)
            crc = (crc >> 1) ^ (0xEDB88320u & (uint32_t) - (int32_t)(crc & 1));
    }
    return crc;
}

static const esp_partition_t *findLutPartition(void)
{
    if (lut_part)
        return lut_part;
    lut_part = esp_partition_find_first(
        ESP_PARTITION_TYPE_DATA, LUT_PART_SUBTYPE, LUT_PART_NAME);
    return lut_part;
}

void initAdcLut(void)
{
    const esp_partition_t *part = findLutPartition();
    if (!part) {
        Serial.println("[adclut] no 'adclut' partition - ADC readings stay raw");
        return;
    }
    if (part->size < LUT_PART_SIZE) {
        Serial.printf("[adclut] partition is %u bytes, expected at least %u\n",
                      (unsigned)part->size, (unsigned)LUT_PART_SIZE);
        return;
    }

    LutHeader hdr;
    if (esp_partition_read(part, 0, &hdr, sizeof(hdr)) != ESP_OK) {
        Serial.println("[adclut] could not read the 'adclut' header");
        return;
    }
    if (hdr.magic != LUT_MAGIC) {
        Serial.println("[adclut] no table stored - run the adc_calibrate tool to build one");
        return;
    }
    if (hdr.version != LUT_VERSION || hdr.entries != ADC_LUT_ENTRIES) {
        Serial.printf("[adclut] stored table is version %u with %u entries, "
                      "this firmware wants version %u with %u\n",
                      (unsigned)hdr.version, (unsigned)hdr.entries,
                      (unsigned)LUT_VERSION, (unsigned)ADC_LUT_ENTRIES);
        return;
    }

    const void *mapped = NULL;
    if (esp_partition_mmap(part, LUT_TABLE_OFFSET, LUT_TABLE_BYTES,
                           LUT_MMAP_DATA, &mapped, &lut_map) != ESP_OK) {
        Serial.println("[adclut] could not map the table");
        return;
    }

    uint32_t actual = crc32_iso((const uint8_t *)mapped, LUT_TABLE_BYTES, 0xFFFFFFFFu);
    if (actual != hdr.crc) {
        Serial.printf("[adclut] CRC32 %08x does not match the stored %08x - "
                      "the table is corrupt and will be ignored\n",
                      (unsigned)actual, (unsigned)hdr.crc);
        esp_partition_munmap(lut_map);
        lut_map = 0;
        return;
    }

    lut_table = (const int16_t *)mapped;
    Serial.println("[adclut] ADC linearisation table loaded");
}

bool adcLutValid(void) { return lut_table != NULL; }

uint16_t adcLinearize(uint16_t raw)
{
    if (!lut_table || raw >= ADC_LUT_ENTRIES)
        return raw;
    int16_t value = lut_table[raw];
    if (value < 0)
        return raw;
    return (uint16_t)value;
}

// ---------------------------------------------------------------- writer ----

static uint32_t write_crc = 0;
static size_t write_count = 0;
static bool write_open = false;

bool adcLutBeginWrite(void)
{
    const esp_partition_t *part = findLutPartition();
    if (!part) {
        Serial.println("[adclut] no 'adclut' partition to write to - is this image built "
                       "with firmware/common/partitions_lino.csv?");
        return false;
    }
    if (part->size < LUT_PART_SIZE) {
        Serial.printf("[adclut] partition is %u bytes, need %u\n",
                      (unsigned)part->size, (unsigned)LUT_PART_SIZE);
        return false;
    }
    // Drop the mapping first: the region is about to stop being what it claims.
    if (lut_map) {
        esp_partition_munmap(lut_map);
        lut_map = 0;
    }
    lut_table = NULL;
    if (esp_partition_erase_range(part, 0, LUT_PART_SIZE) != ESP_OK) {
        Serial.println("[adclut] erase failed");
        return false;
    }
    write_crc = 0xFFFFFFFFu;
    write_count = 0;
    write_open = true;
    return true;
}

bool adcLutWriteBlock(const int16_t *values, size_t count)
{
    if (!write_open)
        return false;
    if (write_count + count > ADC_LUT_ENTRIES) {
        Serial.println("[adclut] more entries offered than the table holds");
        write_open = false;
        return false;
    }
    size_t bytes = count * sizeof(int16_t);
    size_t offset = LUT_TABLE_OFFSET + write_count * sizeof(int16_t);
    if (esp_partition_write(lut_part, offset, values, bytes) != ESP_OK) {
        Serial.printf("[adclut] write failed at entry %u\n", (unsigned)write_count);
        write_open = false;
        return false;
    }
    write_crc = crc32_iso((const uint8_t *)values, bytes, write_crc);
    write_count += count;
    return true;
}

bool adcLutEndWrite(void)
{
    if (!write_open)
        return false;
    write_open = false;
    if (write_count != ADC_LUT_ENTRIES) {
        Serial.printf("[adclut] only %u of %u entries were written - not committing\n",
                      (unsigned)write_count, (unsigned)ADC_LUT_ENTRIES);
        return false;
    }
    LutHeader hdr = { LUT_MAGIC, LUT_VERSION, ADC_LUT_ENTRIES, write_crc };
    if (esp_partition_write(lut_part, 0, &hdr, sizeof(hdr)) != ESP_OK) {
        Serial.println("[adclut] header write failed - the table is present but will not be used");
        return false;
    }
    Serial.printf("[adclut] committed %u entries, CRC32 %08x\n",
                  (unsigned)write_count, (unsigned)write_crc);
    return true;
}

#else  // ADC_LUT_SUPPORTED == 0

// ESP32-S3/C3/C6/H2 and the RP2040/RP2350 have no hardware DAC, so they cannot
// run the sweep that builds this table and there is nothing to store. The
// accessors exist so that shared code compiles unchanged.
void initAdcLut(void) {}
bool adcLutValid(void) { return false; }
uint16_t adcLinearize(uint16_t raw) { return raw; }
bool adcLutBeginWrite(void) { return false; }
bool adcLutWriteBlock(const int16_t *values, size_t count) { (void)values; (void)count; return false; }
bool adcLutEndWrite(void) { return false; }

#endif
