// The firmware's lidarUartOpen() hook on the UNO Q: a real LD19's TX on D0 (PB7), which
// is USART1's RX -- the board file's `arduino_serial`. Interrupt-driven into a ring, read
// by the firmware's raw_scan forwarder (src/main.cpp, pumpRealLidar) from loop(). D1 (the
// UART's TX) is not used: the LD19 needs nothing sent to it.
#include <Arduino.h>

#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/ring_buffer.h>

static const struct device *const usart1 = DEVICE_DT_GET(DT_NODELABEL(usart1));
RING_BUF_DECLARE(lidar_ring, 2048);     // ~90 ms of LD19 at 230400
static uint32_t lidar_dropped, lidar_errors;

static void lidarIsr(const struct device *dev, void *)
{
    while (uart_irq_update(dev) && uart_irq_is_pending(dev)) {
        if (uart_irq_rx_ready(dev)) {
            uint8_t buf[32];
            const int n = uart_fifo_read(dev, buf, sizeof(buf));
            if (n > 0)
                lidar_dropped += (uint32_t)n - ring_buf_put(&lidar_ring, buf, (uint32_t)n);
        }
        if (uart_err_check(dev) > 0)
            lidar_errors++;
    }
}

class LidarUart : public Stream
{
public:
    size_t write(uint8_t) override { return 0; }
    using Print::write;
    int available() override
    {
        const unsigned int key = irq_lock();
        const int n = (int)ring_buf_size_get(&lidar_ring);
        irq_unlock(key);
        return n;
    }
    int read() override
    {
        uint8_t c;
        const unsigned int key = irq_lock();
        const uint32_t got = ring_buf_get(&lidar_ring, &c, 1);
        irq_unlock(key);
        return got ? c : -1;
    }
};
static LidarUart lidar_uart;

Stream *lidarUartOpen(int rx_pin, uint32_t baud)
{
    if (rx_pin != 0) {           // D0 is the only header pin with a free UART RX here
        printk("[lidar] lidar_rx %d: only D0 (USART1 RX) can take a LiDAR on the UNO Q\n", rx_pin);
        return nullptr;
    }
    if (!device_is_ready(usart1))
        return nullptr;
    struct uart_config cfg;
    if (uart_config_get(usart1, &cfg) == 0) {
        cfg.baudrate = baud;
        cfg.flow_ctrl = UART_CFG_FLOW_CTRL_NONE;
        if (uart_configure(usart1, &cfg) != 0)
            return nullptr;
    }
    uart_irq_callback_user_data_set(usart1, lidarIsr, NULL);
    uart_irq_rx_enable(usart1);
    return &lidar_uart;
}
