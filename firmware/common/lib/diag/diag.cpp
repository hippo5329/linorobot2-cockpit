#include "diag.h"
#include "config.h"
#include "mcu_env.h"
#include <stdarg.h>
#include <string.h>

#ifndef LIDAR_SERIAL
#define LIDAR_SERIAL 1
#endif

// A board whose config names no LiDAR pin still compiles this file.
#ifndef LIDAR_RXD
#define LIDAR_RXD (-1)
#endif

static Stream *diag_out = NULL;
static volatile uint32_t counters[DIAG_N];
static uint32_t last_counters[DIAG_N];
static volatile int spin_rc = 0;
static volatile uint32_t spin_max_us = 0;
static volatile uint32_t spin_calls = 0;
static volatile int diag_state = -1;
static volatile uint32_t time_max_us[DIAGT_N];
static uint32_t last_tick_ms = 0;

void diagBegin(void)
{
    const char *tx_env = envGet("diag_tx", NULL);
    if (!tx_env || !*tx_env) return;
    const int tx = (int)strtol(tx_env, NULL, 10);
    if (tx < 0) return;
    // The LiDAR emulator streams out of this same UART (both take
    // LIDAR_SERIAL), and on an ESP32 the second begin() simply re-points the
    // peripheral: whichever starts last owns the pin and the other goes
    // silently nowhere. Turning both on has never meant anything, so say so
    // instead of leaving the operator to wonder which instrument is lying.
    const char *sim = envGet("sim_ld19", NULL);
    const bool sim_on = (sim && *sim)
        ? !(strcmp(sim, "0") == 0 || strcasecmp(sim, "false") == 0
            || strcasecmp(sim, "no") == 0)
        : (bool)SIM_LD19_DEFAULT;
    // Same defaulting as main.cpp: the env's pin, else the header's, and a
    // negative pin means the emulator has no UART sink at all.
    const int lidar_rx = envInt("lidar_rx", LIDAR_RXD);
    if (sim_on && lidar_rx >= 0) {
        Serial.printf("[diag] diag_tx=%d ignored: the LiDAR emulator owns UART%d "
                      "(set sim_ld19=0 to use the diagnostic UART)\r\n", tx, LIDAR_SERIAL);
        return;
    }
    const uint32_t baud = envU32("diag_baud", 230400);
#if defined(ESP32)
    HardwareSerial *serial = new HardwareSerial(LIDAR_SERIAL);
    serial->setTxBufferSize(1024);
    serial->begin(baud, SERIAL_8N1, -1, tx);
    diag_out = serial;
#elif defined(ARDUINO_ARCH_RP2040)
    Serial1.setTX(tx);
    Serial1.begin(baud);
    diag_out = &Serial1;
#else
    (void)baud;
    return;
#endif
    last_tick_ms = millis();
    diag_out->printf("\r\n[diag] up on gpio %d @ %lu, %s\r\n", tx, (unsigned long)baud,
                     mcuEnvValid() ? "env ok" : "env INVALID");
}

bool diagEnabled(void) { return diag_out != NULL; }

void diagCount(DiagCounter c, uint32_t n)
{
    if (c < DIAG_N) counters[c] += n;
}

void diagSpin(int rc, uint32_t elapsed_us)
{
    spin_calls++;
    if (rc != 0) spin_rc = rc;
    if (elapsed_us > spin_max_us) spin_max_us = elapsed_us;
}

void diagState(int state) { diag_state = state; }

void diagTime(DiagTimer t, uint32_t elapsed_us)
{
    if (t < DIAGT_N && elapsed_us > time_max_us[t]) time_max_us[t] = elapsed_us;
}

void diagPrintf(const char *fmt, ...)
{
    if (!diag_out) return;
    char buf[192];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    diag_out->print(buf);
}

void diagTick(void)
{
    if (!diag_out) return;
    const uint32_t now = millis();
    if (now - last_tick_ms < 1000) return;
    const uint32_t dt = now - last_tick_ms;
    last_tick_ms = now;

    uint32_t d[DIAG_N];
    for (int i = 0; i < DIAG_N; i++) {
        const uint32_t v = counters[i];
        d[i] = v - last_counters[i];
        last_counters[i] = v;
    }
    const uint32_t sc = spin_calls; spin_calls = 0;
    const uint32_t smax = spin_max_us; spin_max_us = 0;
    const int src = spin_rc; spin_rc = 0;

    diag_out->printf("D t=%lu dt=%lu st=%d loop=%lu tmr=%lu pub=%lu fail=%lu "
                     "tx=%lu rx=%lu/%lu(%lu empty) spin=%lu rc=%d max=%luus "
                     "ping=%lu/%lu",
                     (unsigned long)now, (unsigned long)dt, diag_state,
                     (unsigned long)d[DIAG_LOOP], (unsigned long)d[DIAG_TIMER],
                     (unsigned long)d[DIAG_PUBLISH], (unsigned long)d[DIAG_PUBFAIL],
                     (unsigned long)d[DIAG_TX_BYTES], (unsigned long)d[DIAG_RX_BYTES],
                     (unsigned long)d[DIAG_RX_CALLS], (unsigned long)d[DIAG_RX_EMPTY],
                     (unsigned long)sc, src, (unsigned long)smax,
                     (unsigned long)d[DIAG_PING_OK], (unsigned long)d[DIAG_PING_FAIL]);
    uint32_t tm[DIAGT_N];
    for (int i = 0; i < DIAGT_N; i++) { tm[i] = time_max_us[i]; time_max_us[i] = 0; }
    diag_out->printf(" mv=%lu sens=%lu pubs=%luus", (unsigned long)tm[DIAGT_MOVE],
                     (unsigned long)tm[DIAGT_SENSORS], (unsigned long)tm[DIAGT_PUB]);
#if defined(ESP32)
    diag_out->printf(" heap=%lu/%lu", (unsigned long)ESP.getFreeHeap(),
                     (unsigned long)ESP.getMinFreeHeap());
#endif
    diag_out->print("\r\n");
}
