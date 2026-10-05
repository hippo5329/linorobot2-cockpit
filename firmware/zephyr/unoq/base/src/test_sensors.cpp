#include "test_sensors.h"

#include <zephyr/device.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/hwinfo.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include "env.h"

#define USER_NODE DT_PATH(zephyr_user)

static const struct adc_dt_spec batt = ADC_DT_SPEC_GET_BY_IDX(USER_NODE, 0);
static const struct gpio_dt_spec enc[4] = {
    GPIO_DT_SPEC_GET_BY_IDX(USER_NODE, enc_gpios, 0), GPIO_DT_SPEC_GET_BY_IDX(USER_NODE, enc_gpios, 1),
    GPIO_DT_SPEC_GET_BY_IDX(USER_NODE, enc_gpios, 2), GPIO_DT_SPEC_GET_BY_IDX(USER_NODE, enc_gpios, 3),
};

static void scanI2c(const struct device *bus)
{
    if (!device_is_ready(bus)) {
        printk("[i2c] %s not ready\n", bus->name);
        return;
    }
    int found = 0;
    printk("[i2c] %s:", bus->name);
    for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
        uint8_t b;
        if (i2c_read(bus, &b, 1, addr) == 0) {
            printk(" 0x%02x", addr);
            found++;
        }
    }
    printk(found ? "  (%d found)\n" : " nothing answered\n", found);
}

static int readBatteryMv(void)
{
    if (!adc_is_ready_dt(&batt) || adc_channel_setup_dt(&batt))
        return -1;
    int16_t raw = 0;
    struct adc_sequence seq = {};
    seq.buffer = &raw;
    seq.buffer_size = sizeof(raw);
    if (adc_sequence_init_dt(&batt, &seq) || adc_read_dt(&batt, &seq))
        return -1;
    int32_t mv = raw;
    return adc_raw_to_millivolts_dt(&batt, &mv) ? -1 : (int)mv;
}

void testSensors(void)
{
    uint8_t id[12] = {};
    const ssize_t n = hwinfo_get_device_id(id, sizeof(id));
    printk("linorobot2 unoq app=test_sensors uid=");
    for (ssize_t i = 0; i < n; i++)
        printk("%02X", id[i]);
    printk("\n[env] %s\n", envValid() ? "valid" : "BLANK or corrupt -- every key at its default");
    for (int i = 0; i < 4; i++)
        gpio_pin_configure_dt(&enc[i], GPIO_INPUT);

    const float r1 = envFloat("bat_r1", 0.0f), r2 = envFloat("bat_r2", 0.0f);
    for (int pass = 1;; pass++) {
        printk("--- pass %d, %lld ms\n", pass, k_uptime_get());
        scanI2c(DEVICE_DT_GET(DT_NODELABEL(i2c4)));
        const int mv = readBatteryMv();
        if (mv < 0)
            printk("[battery] A0: ADC read failed\n");
        else if (r1 > 0.0f && r2 > 0.0f)
            printk("[battery] A0 %d mV -> pack %.2f V (bat_r1 %.0f, bat_r2 %.0f)\n", mv,
                   (double)(mv / 1000.0f * (r1 + r2) / r2), (double)r1, (double)r2);
        else
            printk("[battery] A0 %d mV (no bat_r1/bat_r2: no divider configured, nothing published)\n", mv);
        printk("[encoders] m1 A=%d B=%d  m2 A=%d B=%d (pulled up: 1 = open)\n",
               gpio_pin_get_dt(&enc[0]), gpio_pin_get_dt(&enc[1]),
               gpio_pin_get_dt(&enc[2]), gpio_pin_get_dt(&enc[3]));
        k_msleep(2000);
    }
}
