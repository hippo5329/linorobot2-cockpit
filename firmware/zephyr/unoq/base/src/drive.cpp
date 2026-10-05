#include "drive.h"

#include <math.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/irq.h>
#include <zephyr/sys/printk.h>

#include "env.h"

#define USER_NODE DT_PATH(zephyr_user)

// in1/in2 per motor, in the overlay's order
static const struct pwm_dt_spec pwm_out[2 * DRIVE_MOTORS] = {
    PWM_DT_SPEC_GET_BY_IDX(USER_NODE, 0), PWM_DT_SPEC_GET_BY_IDX(USER_NODE, 1),
    PWM_DT_SPEC_GET_BY_IDX(USER_NODE, 2), PWM_DT_SPEC_GET_BY_IDX(USER_NODE, 3),
};
// A/B per motor
static const struct gpio_dt_spec enc_in[2 * DRIVE_MOTORS] = {
    GPIO_DT_SPEC_GET_BY_IDX(USER_NODE, enc_gpios, 0), GPIO_DT_SPEC_GET_BY_IDX(USER_NODE, enc_gpios, 1),
    GPIO_DT_SPEC_GET_BY_IDX(USER_NODE, enc_gpios, 2), GPIO_DT_SPEC_GET_BY_IDX(USER_NODE, enc_gpios, 3),
};

static uint32_t period_ns;
static int pwm_max;
static bool sim;
static float max_rpm, volt_ratio;
static int cpr[DRIVE_MOTORS];
static bool motor_inv[DRIVE_MOTORS], enc_inv[DRIVE_MOTORS];

static volatile int32_t counts[DRIVE_MOTORS];
static int32_t last_counts[DRIVE_MOTORS];
static uint8_t enc_state[DRIVE_MOTORS];
static struct gpio_callback enc_cb[2 * DRIVE_MOTORS];

// The command each wheel last got, and the model's speed, for sim_wheel.
static int last_pwm[DRIVE_MOTORS];
static float sim_rpm[DRIVE_MOTORS];
static double sim_counts[DRIVE_MOTORS];

// x4 quadrature: index (previous AB << 2) | current AB. Invalid jumps (both
// lines changed) count nothing rather than guess a direction.
static const int8_t QUAD[16] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};

static void encIsr(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    ARG_UNUSED(dev); ARG_UNUSED(pins);
    const int m = (int)((cb - enc_cb) / 2);
    const uint8_t ab = (uint8_t)((gpio_pin_get_dt(&enc_in[2 * m]) << 1) | gpio_pin_get_dt(&enc_in[2 * m + 1]));
    counts[m] += QUAD[(enc_state[m] << 2) | ab];
    enc_state[m] = ab;
}

bool driveInit(void)
{
    const int freq = envInt("pwm_freq", 20000);
    const int bits = envInt("pwm_bits", 10);
    period_ns = (uint32_t)(1000000000ULL / (uint32_t)(freq > 0 ? freq : 20000));
    pwm_max = (1 << (bits > 0 && bits < 16 ? bits : 10)) - 1;
    sim = envFlag("sim_wheel", false);
    max_rpm = envFloat("max_rpm", 140.0f);
    const float op = envFloat("motor_v", 24.0f), pw = envFloat("power_v", 12.0f);
    volt_ratio = op > 0.0f ? fminf(fmaxf(pw / op, 0.0f), 1.0f) : 1.0f;

    static const char *const CPR[]  = {"m1_cpr", "m2_cpr"};
    static const char *const MINV[] = {"m1_inv", "m2_inv"};
    static const char *const EINV[] = {"m1_enc_inv", "m2_enc_inv"};
    bool ok = true;
    for (int m = 0; m < DRIVE_MOTORS; m++) {
        cpr[m] = envInt(CPR[m], 144000);
        motor_inv[m] = envFlag(MINV[m], false);
        enc_inv[m] = envFlag(EINV[m], false);
    }
    for (int i = 0; i < 2 * DRIVE_MOTORS; i++) {
        if (!pwm_is_ready_dt(&pwm_out[i])) { printk("[drive] pwm %d not ready\n", i); ok = false; continue; }
        pwm_set_dt(&pwm_out[i], period_ns, 0);
    }
    if (!sim) {
        for (int i = 0; i < 2 * DRIVE_MOTORS; i++) {
            if (!gpio_is_ready_dt(&enc_in[i]) || gpio_pin_configure_dt(&enc_in[i], GPIO_INPUT)) {
                printk("[drive] encoder pin %d not ready\n", i); ok = false; continue;
            }
            gpio_init_callback(&enc_cb[i], encIsr, BIT(enc_in[i].pin));
            gpio_add_callback(enc_in[i].port, &enc_cb[i]);
            gpio_pin_interrupt_configure_dt(&enc_in[i], GPIO_INT_EDGE_BOTH);
        }
        for (int m = 0; m < DRIVE_MOTORS; m++)
            enc_state[m] = (uint8_t)((gpio_pin_get_dt(&enc_in[2 * m]) << 1) | gpio_pin_get_dt(&enc_in[2 * m + 1]));
    }
    printk("[drive] %s wheels, pwm %d Hz / %d bits, cpr %d/%d, max_rpm %.0f\n",
           sim ? "SIMULATED" : "real", (int)(1000000000ULL / period_ns), bits, cpr[0], cpr[1], (double)max_rpm);
    return ok;
}

int drivePwmMax(void) { return pwm_max; }
bool driveSimulated(void) { return sim; }

void driveSpin(int motor, int pwm)
{
    if (motor < 0 || motor >= DRIVE_MOTORS)
        return;
    if (pwm > pwm_max) pwm = pwm_max;
    if (pwm < -pwm_max) pwm = -pwm_max;
    last_pwm[motor] = pwm;
    const int signed_pwm = motor_inv[motor] ? -pwm : pwm;
    const uint32_t pulse = (uint32_t)((uint64_t)period_ns * (uint32_t)abs(signed_pwm) / (uint32_t)pwm_max);
    pwm_set_dt(&pwm_out[2 * motor],     period_ns, signed_pwm > 0 ? pulse : 0);
    pwm_set_dt(&pwm_out[2 * motor + 1], period_ns, signed_pwm < 0 ? pulse : 0);
}

// First-order wheel: the commanded duty sets a target speed, reached with a
// 150 ms time constant (sim_wheel.h's SIM_WHEEL_TAU_MS). The full model -- mass,
// friction, stall duty, battery sag -- is phase 4.
static void simAdvance(float dt_s)
{
    const float tau = 0.150f;
    const float k = 1.0f - expf(-dt_s / tau);
    for (int m = 0; m < DRIVE_MOTORS; m++) {
        const float target = (float)last_pwm[m] / (float)pwm_max * max_rpm * volt_ratio;
        sim_rpm[m] += (target - sim_rpm[m]) * k;
        sim_counts[m] += (double)sim_rpm[m] / 60.0 * cpr[m] * dt_s;
    }
}

void driveMeasure(float dt_s, float rpm_out[DRIVE_MOTORS])
{
    if (sim)
        simAdvance(dt_s);
    for (int m = 0; m < DRIVE_MOTORS; m++) {
        int32_t now;
        if (sim) {
            now = (int32_t)llround(sim_counts[m]);
        } else {
            const unsigned int key = irq_lock();
            now = counts[m];
            irq_unlock(key);
        }
        const int32_t delta = now - last_counts[m];
        last_counts[m] = now;
        float rpm = (dt_s > 0.0f && cpr[m] > 0) ? (float)delta / (float)cpr[m] / dt_s * 60.0f : 0.0f;
        rpm_out[m] = (enc_inv[m] && !sim) ? -rpm : rpm;
    }
}
