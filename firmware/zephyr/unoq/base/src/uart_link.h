// The micro-ROS custom transport on lpuart1 (-> the QRB2210's /dev/ttyHS1), interrupt
// driven both ways. The module's own transport polls each TX byte out (uart_poll_out),
// so a ~730-byte Odometry held the CPU for the whole frame. Here write() only queues the
// frame; the UART interrupt drains the queue into the 8-byte hardware FIFO.
#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif
struct uxrCustomTransport;
bool   uartLinkOpen(struct uxrCustomTransport *transport);
bool   uartLinkClose(struct uxrCustomTransport *transport);
size_t uartLinkWrite(struct uxrCustomTransport *transport, const uint8_t *buf, size_t len, uint8_t *err);
size_t uartLinkRead(struct uxrCustomTransport *transport, uint8_t *buf, size_t len, int timeout, uint8_t *err);

struct UartLinkStats {
    uint32_t tx_bytes, rx_bytes;
    uint32_t tx_full_waits;   // write() found the queue full and had to wait for it
    uint32_t tx_dropped;      // bytes write() gave up on (queue still full at its deadline)
    uint32_t rx_dropped;      // bytes received with the RX queue full
    uint32_t uart_errors;     // overrun / framing / parity / noise, from uart_err_check()
};
void uartLinkStats(struct UartLinkStats *out);
#ifdef __cplusplus
}
#endif
