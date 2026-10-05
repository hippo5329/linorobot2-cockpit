#include "env.h"

#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <zephyr/devicetree.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/sys/crc.h>
#include <zephyr/sys/printk.h>

#define ENV_SIZE     0x1000
#define ENV_CRC_LEN  4
#define ENV_DATA_LEN (ENV_SIZE - ENV_CRC_LEN)

// Internal flash is memory-mapped: the block is read where it lies.
#define ENV_ADDR (DT_REG_ADDR(DT_CHOSEN(zephyr_flash)) + \
                  DT_REG_ADDR(DT_NODELABEL(lino_env_partition)))

static const uint8_t *env_base;
static bool env_ok;

void envInit(void)
{
    env_base = (const uint8_t *)ENV_ADDR;
    uint32_t stored;
    memcpy(&stored, env_base, sizeof(stored));
    const uint32_t actual = crc32_ieee(env_base + ENV_CRC_LEN, ENV_DATA_LEN);
    env_ok = stored == actual;
    if (env_ok)
        printk("[env] 0x%08x: %d bytes, CRC32 %08x OK\n", (unsigned)ENV_ADDR, ENV_SIZE, actual);
    else
        printk("[env] 0x%08x: CRC32 %08x, stored %08x -- no env, every key takes its default\n",
               (unsigned)ENV_ADDR, actual, stored);
}

bool envValid(void) { return env_ok; }

const char *envGet(const char *key, const char *fallback)
{
    if (!env_ok)
        return fallback;
    const size_t klen = strlen(key);
    const char *p = (const char *)env_base + ENV_CRC_LEN;
    const char *end = p + ENV_DATA_LEN;
    while (p < end && *p) {
        const size_t n = strnlen(p, end - p);
        if (n > klen && p[klen] == '=' && strncmp(p, key, klen) == 0)
            return p + klen + 1;
        p += n + 1;
    }
    return fallback;
}

int envInt(const char *key, int fallback)
{
    const char *v = envGet(key, NULL);
    return (v && *v) ? (int)strtol(v, NULL, 0) : fallback;
}

float envFloat(const char *key, float fallback)
{
    const char *v = envGet(key, NULL);
    return (v && *v) ? strtof(v, NULL) : fallback;
}

bool envFlag(const char *key, bool fallback)
{
    const char *v = envGet(key, NULL);
    if (!v || !*v)
        return fallback;
    return !(strcmp(v, "0") == 0 || strcasecmp(v, "false") == 0 ||
             strcasecmp(v, "no") == 0);
}
