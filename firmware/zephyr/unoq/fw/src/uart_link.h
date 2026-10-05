// The lpuart1 link (-> the QRB2210's /dev/ttyHS1) as an interrupt-driven byte stream:
// write() queues and returns, the UART interrupt drains the queue into the 8-byte
// hardware FIFO; received bytes land in a queue and wake readers. The shim's Serial
// sits on this, and the firmware's own micro-ROS serial transport on Serial.
#pragma once
#include <stddef.h>
#include <stdint.h>

struct UartLinkStats {
    uint32_t tx_bytes, rx_bytes;
    uint32_t tx_full_waits;   // a write found the queue full and had to wait for it
    uint32_t tx_dropped;      // bytes a write gave up on (queue still full at its deadline)
    uint32_t rx_dropped;      // bytes received with the RX queue full
    uint32_t uart_errors;     // overrun / framing / parity / noise, from uart_err_check()
};

bool   linkBegin(uint32_t baud);
size_t linkWrite(const uint8_t *buf, size_t len);
size_t linkRead(uint8_t *buf, size_t len, int timeout_ms);
int    linkAvailable(void);
void   linkFlush(int timeout_ms);
void   linkStats(struct UartLinkStats *out);
