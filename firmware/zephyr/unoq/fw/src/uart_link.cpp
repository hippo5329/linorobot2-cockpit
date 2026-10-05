#include "uart_link.h"

#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/ring_buffer.h>

static const struct device *const link = DEVICE_DT_GET(DT_NODELABEL(lpuart1));

// TX holds a few frames: micro-ROS writes a whole serialized frame per call (MTU 1024
// plus framing), and the 50 Hz topics plus the session's own traffic must not stall.
RING_BUF_DECLARE(tx_ring, 4096);
RING_BUF_DECLARE(rx_ring, 4096);
static K_SEM_DEFINE(rx_sem, 0, 1);
static struct UartLinkStats stats;
static bool started;

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
        // uart_err_check() reads and clears the error flags; an uncleared overrun
        // would keep the interrupt pending for ever.
        if (uart_err_check(dev) > 0)
            stats.uart_errors++;
    }
}

bool linkBegin(uint32_t baud)
{
    if (!device_is_ready(link))
        return false;
    struct uart_config cfg;
    if (uart_config_get(link, &cfg) == 0 && baud > 0) {
        cfg.baudrate = baud;
        const int rc = uart_configure(link, &cfg);
        printk("[link] lpuart1 -> /dev/ttyHS1 at %u baud%s\n", (unsigned)baud, rc ? " FAILED" : "");
    }
    if (!started) {
        uart_irq_callback_user_data_set(link, linkIsr, NULL);
        uart_irq_rx_enable(link);
        started = true;
    }
    return true;
}

size_t linkWrite(const uint8_t *buf, size_t len)
{
    if (!started)
        return 0;
    size_t done = 0;
    const int64_t deadline = k_uptime_get() + 50;   // a full 4 KB queue drains in 10 ms at 4 Mbaud
    bool waited = false;
    while (done < len) {
        // The ISR is the only consumer and cannot run under the lock.
        const unsigned int key = irq_lock();
        const uint32_t put = ring_buf_put(&tx_ring, buf + done, (uint32_t)(len - done));
        irq_unlock(key);
        done += put;
        uart_irq_tx_enable(link);
        if (done < len) {
            if (k_uptime_get() > deadline) {
                stats.tx_dropped += (uint32_t)(len - done);
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

size_t linkRead(uint8_t *buf, size_t len, int timeout_ms)
{
    if (!started)
        return 0;
    const int64_t deadline = k_uptime_get() + (timeout_ms > 0 ? timeout_ms : 0);
    size_t got = 0;
    for (;;) {
        const unsigned int key = irq_lock();
        got += ring_buf_get(&rx_ring, buf + got, (uint32_t)(len - got));
        irq_unlock(key);
        const int64_t left = deadline - k_uptime_get();
        if (got == len || left <= 0)
            return got;
        (void)k_sem_take(&rx_sem, K_MSEC(left));
    }
}

int linkAvailable(void)
{
    const unsigned int key = irq_lock();
    const int n = (int)ring_buf_size_get(&rx_ring);
    irq_unlock(key);
    return n;
}

void linkFlush(int timeout_ms)
{
    const int64_t deadline = k_uptime_get() + timeout_ms;
    while (!ring_buf_is_empty(&tx_ring) && k_uptime_get() < deadline)
        k_usleep(100);
}

void linkStats(struct UartLinkStats *out)
{
    const unsigned int key = irq_lock();
    *out = stats;
    irq_unlock(key);
}
