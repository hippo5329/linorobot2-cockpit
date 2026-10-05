#include "uart_link.h"

#include <string.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/ring_buffer.h>

#include <uxr/client/profile/transport/custom/custom_transport.h>

static const struct device *const link = DEVICE_DT_GET(DT_NODELABEL(lpuart1));

// TX holds a few frames: micro-ROS writes a whole serialized frame per call (MTU 1024
// plus framing), and the 50 Hz odom plus the session's own traffic must not stall on it.
RING_BUF_DECLARE(tx_ring, 4096);
RING_BUF_DECLARE(rx_ring, 4096);
static K_SEM_DEFINE(rx_sem, 0, 1);
static struct UartLinkStats stats;

static void linkIsr(const struct device *dev, void *user)
{
    ARG_UNUSED(user);
    while (uart_irq_update(dev) && uart_irq_is_pending(dev)) {
        if (uart_irq_rx_ready(dev)) {
            uint8_t buf[32];
            const int n = uart_fifo_read(dev, buf, sizeof(buf));
            if (n > 0) {
                const uint32_t put = ring_buf_put(&rx_ring, buf, (uint32_t)n);
                stats.rx_bytes += put;
                stats.rx_dropped += (uint32_t)n - put;
                k_sem_give(&rx_sem);
            }
        }
        if (uart_irq_tx_ready(dev)) {
            uint8_t *chunk;
            const uint32_t avail = ring_buf_get_claim(&tx_ring, &chunk, 16);
            if (avail == 0) {
                uart_irq_tx_disable(dev);
            } else {
                const int sent = uart_fifo_fill(dev, chunk, (int)avail);
                ring_buf_get_finish(&tx_ring, sent > 0 ? (uint32_t)sent : 0);
                if (sent > 0)
                    stats.tx_bytes += (uint32_t)sent;
            }
        }
        // uart_err_check() reads and clears the error flags; an uncleared overrun would
        // keep the interrupt pending forever (the 3 Mbaud hang before the FIFOs were on).
        if (uart_err_check(dev) > 0)
            stats.uart_errors++;
    }
}

bool uartLinkOpen(struct uxrCustomTransport *transport)
{
    ARG_UNUSED(transport);
    if (!device_is_ready(link))
        return false;
    // Called again on every reconnect and agent ping: start from empty queues.
    uart_irq_rx_disable(link);
    uart_irq_tx_disable(link);
    ring_buf_reset(&tx_ring);
    ring_buf_reset(&rx_ring);
    k_sem_reset(&rx_sem);
    uart_irq_callback_user_data_set(link, linkIsr, NULL);
    uart_irq_rx_enable(link);
    return true;
}

bool uartLinkClose(struct uxrCustomTransport *transport)
{
    ARG_UNUSED(transport);
    return true;   // leave the interrupts on: the next open resets the queues
}

size_t uartLinkWrite(struct uxrCustomTransport *transport, const uint8_t *buf, size_t len, uint8_t *err)
{
    ARG_UNUSED(transport);
    size_t done = 0;
    const int64_t deadline = k_uptime_get() + 50;   // ~20 KB at 4 Mbaud: a full queue drains in 10 ms
    bool waited = false;
    while (done < len) {
        // The ISR is the only consumer and cannot run while the lock is held, so the
        // producer side needs nothing more than this.
        const unsigned int key = irq_lock();
        const uint32_t put = ring_buf_put(&tx_ring, buf + done, (uint32_t)(len - done));
        irq_unlock(key);
        done += put;
        uart_irq_tx_enable(link);
        if (done < len) {
            if (k_uptime_get() > deadline) {
                stats.tx_dropped += (uint32_t)(len - done);
                *err = 1;
                break;
            }
            waited = true;
            k_usleep(100);
        }
    }
    if (waited)
        stats.tx_full_waits++;
    return done;
}

size_t uartLinkRead(struct uxrCustomTransport *transport, uint8_t *buf, size_t len, int timeout, uint8_t *err)
{
    ARG_UNUSED(transport);
    ARG_UNUSED(err);
    if (ring_buf_is_empty(&rx_ring))
        (void)k_sem_take(&rx_sem, K_MSEC(timeout > 0 ? timeout : 0));
    const unsigned int key = irq_lock();
    const uint32_t got = ring_buf_get(&rx_ring, buf, (uint32_t)len);
    irq_unlock(key);
    return got;
}

void uartLinkStats(struct UartLinkStats *out)
{
    const unsigned int key = irq_lock();
    *out = stats;
    irq_unlock(key);
}
