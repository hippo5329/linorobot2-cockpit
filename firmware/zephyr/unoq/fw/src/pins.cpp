// The Arduino pin API on the UNO Q's STM32U585: digital I/O, PWM, ADC, pin interrupts
// and the x4 quadrature counters the firmware's Encoder uses. Pin numbers are the
// board's Arduino header numbers (shim/Arduino.h); the devicetree list in app.overlay
// is in the same order.
#include <Arduino.h>

#include <zephyr/device.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#define USER_NODE DT_PATH(zephyr_user)
#define GPIO_SPEC(node, prop, idx) GPIO_DT_SPEC_GET_BY_IDX(node, prop, idx),
#define PWM_SPEC(node, prop, idx) PWM_DT_SPEC_GET_BY_IDX(node, idx),
#define ADC_SPEC(node, prop, idx) ADC_DT_SPEC_GET_BY_IDX(node, idx),

static const struct gpio_dt_spec pins[] = {DT_FOREACH_PROP_ELEM(USER_NODE, lino_gpios, GPIO_SPEC)};
static const struct pwm_dt_spec pwms[] = {DT_FOREACH_PROP_ELEM(USER_NODE, pwms, PWM_SPEC)};
static const struct adc_dt_spec adcs[] = {DT_FOREACH_PROP_ELEM(USER_NODE, io_channels, ADC_SPEC)};
#define NPINS ((int)ARRAY_SIZE(pins))

// Which header pins carry a timer channel, and which entry of pwms[] it is.
static int pwmIndex(int pin)
{
    switch (pin) {
    case 3: return 0;    // D3  PB0 TIM3 CH3
    case 6: return 1;    // D6  PB1 TIM3 CH4
    case 9: return 2;    // D9  PB8 TIM4 CH3
    case 10: return 3;   // D10 PB9 TIM4 CH4
    case 16: return 4;   // A2  PA6 TIM3 CH1
    case 17: return 5;   // A3  PA7 TIM3 CH2
    case 12: return 6;   // D12 PB14 TIM15 CH1
    case 11: return 7;   // D11 PB15 TIM15 CH2
    default: return -1;
    }
}

static int pwm_bits = 8;                        // Arduino's default resolution
#define NPWM 8
BUILD_ASSERT(ARRAY_SIZE(pwms) == NPWM, "app.overlay pwms and pwmIndex() must list the same channels");
static uint32_t pwm_period_ns[NPWM] = {50000, 50000, 50000, 50000,
                                       50000, 50000, 50000, 50000};   // 20 kHz until told
static bool pwm_ready[NPWM];

static bool validPin(int pin) { return pin >= 0 && pin < NPINS && gpio_is_ready_dt(&pins[pin]); }

void pinMode(int pin, int mode)
{
    if (!validPin(pin)) return;
    // A timer pin set to OUTPUT stays the timer's: the motor drivers call pinMode on
    // the pins they then analogWrite, and reclaiming it as a GPIO would stop the PWM.
    if (mode == OUTPUT && pwmIndex(pin) >= 0) return;
    gpio_flags_t f = mode == OUTPUT ? GPIO_OUTPUT_INACTIVE
                   : mode == INPUT_PULLUP ? (GPIO_INPUT | GPIO_PULL_UP)
                   : mode == INPUT_PULLDOWN ? (GPIO_INPUT | GPIO_PULL_DOWN) : GPIO_INPUT;
    gpio_pin_configure_dt(&pins[pin], f);
}

void analogWriteResolution(int bits) { if (bits >= 1 && bits <= 16) pwm_bits = bits; }
void analogWriteRange(uint32_t range)
{
    int b = 1;
    while (b < 16 && ((1u << b) - 1) < range) b++;
    pwm_bits = b;
}

void analogWriteFrequency(int pin, uint32_t hz)
{
    if (hz == 0) return;
    const int i = pwmIndex(pin);
    for (int k = 0; k < NPWM; k++)
        if (i < 0 ? pin < 0 : k == i) pwm_period_ns[k] = 1000000000u / hz;
}

static void pwmSet(int i, uint32_t value)
{
    if (!pwm_ready[i]) {
        if (!pwm_is_ready_dt(&pwms[i])) return;
        pwm_ready[i] = true;
    }
    const uint32_t max = (1u << pwm_bits) - 1;
    if (value > max) value = max;
    const uint32_t pulse = (uint32_t)((uint64_t)pwm_period_ns[i] * value / max);
    pwm_set_dt(&pwms[i], pwm_period_ns[i], pulse);
}

void analogWrite(int pin, int value)
{
    const int i = pwmIndex(pin);
    if (i >= 0) {
        pwmSet(i, value < 0 ? 0 : (uint32_t)value);
        return;
    }
    // Arduino's analogWrite on a pin with no timer: on at half or more, off below.
    if (validPin(pin)) {
        gpio_pin_configure_dt(&pins[pin], GPIO_OUTPUT);
        gpio_pin_set_dt(&pins[pin], value >= (1 << (pwm_bits - 1)));
    }
}

void digitalWrite(int pin, int v)
{
    const int i = pwmIndex(pin);
    if (i >= 0 && pwm_ready[i]) {            // a PWM pin stays PWM: 0 % or 100 %
        pwmSet(i, v ? (1u << pwm_bits) - 1 : 0);
        return;
    }
    if (validPin(pin)) gpio_pin_set_dt(&pins[pin], v ? 1 : 0);
}

int digitalRead(int pin) { return validPin(pin) && gpio_pin_get_dt(&pins[pin]) > 0 ? HIGH : LOW; }

static int adc_bits = 10;                       // Arduino's default
void analogReadResolution(int bits) { if (bits >= 1 && bits <= 16) adc_bits = bits; }

int analogRead(int pin)
{
    const int i = pin - LINO_UNOQ_A0;
    if (i < 0 || i >= (int)ARRAY_SIZE(adcs) || !adc_is_ready_dt(&adcs[i])) return 0;
    if (adc_channel_setup_dt(&adcs[i])) return 0;
    int16_t raw = 0;
    struct adc_sequence seq = {};
    seq.buffer = &raw;
    seq.buffer_size = sizeof(raw);
    if (adc_sequence_init_dt(&adcs[i], &seq) || adc_read_dt(&adcs[i], &seq)) return 0;
    const int res = adcs[i].resolution;
    int v = raw < 0 ? 0 : raw;
    return res > adc_bits ? v >> (res - adc_bits) : v << (adc_bits - res);
}

// --- pin interrupts ------------------------------------------------------------------
struct PinIsr {
    struct gpio_callback cb;
    void (*fn)(void);
    void (*fn_arg)(void *);
    void *arg;
};
static PinIsr pin_isr[32];

static void pinIsrTrampoline(const struct device *, struct gpio_callback *cb, uint32_t)
{
    PinIsr *p = CONTAINER_OF(cb, PinIsr, cb);
    if (p->fn) p->fn();
    else if (p->fn_arg) p->fn_arg(p->arg);
}

static void attach(int pin, void (*fn)(void), void (*fn_arg)(void *), void *arg, int mode)
{
    if (!validPin(pin) || pin >= (int)ARRAY_SIZE(pin_isr)) return;
    PinIsr &p = pin_isr[pin];
    p.fn = fn; p.fn_arg = fn_arg; p.arg = arg;
    gpio_pin_configure_dt(&pins[pin], GPIO_INPUT | GPIO_PULL_UP);
    gpio_init_callback(&p.cb, pinIsrTrampoline, BIT(pins[pin].pin));
    gpio_add_callback(pins[pin].port, &p.cb);
    gpio_pin_interrupt_configure_dt(&pins[pin], mode == RISING ? GPIO_INT_EDGE_RISING
                                               : mode == FALLING ? GPIO_INT_EDGE_FALLING
                                               : GPIO_INT_EDGE_BOTH);
}

void attachInterrupt(int pin, void (*isr)(void), int mode) { attach(pin, isr, nullptr, nullptr, mode); }
void attachInterrupt(int pin, void (*isr)(void *), int mode, void *arg) { attach(pin, nullptr, isr, arg, mode); }
void detachInterrupt(int pin)
{
    if (!validPin(pin)) return;
    gpio_pin_interrupt_configure_dt(&pins[pin], GPIO_INT_DISABLE);
    gpio_remove_callback(pins[pin].port, &pin_isr[pin].cb);
}

// --- x4 quadrature, for the firmware's Encoder (common/lib/encoder/encoder.h) --------
// Index (previous AB << 2) | current AB; a jump of both lines counts nothing.
static const int8_t QUAD[16] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};
struct Quad { int a, b; uint8_t state; volatile int32_t count; };
static Quad quads[4];
static int nquads;

static void quadEdge(void *arg)
{
    Quad *q = (Quad *)arg;
    const uint8_t ab = (uint8_t)((gpio_pin_get_dt(&pins[q->a]) << 1) | gpio_pin_get_dt(&pins[q->b]));
    q->count += QUAD[(q->state << 2) | ab];
    q->state = ab;
}

extern "C" int linoZephyrQuadAttach(int pin_a, int pin_b)
{
    if (!validPin(pin_a) || !validPin(pin_b) || nquads >= (int)ARRAY_SIZE(quads)) {
        printk("[encoder] pins %d/%d: not usable on this board\n", pin_a, pin_b);
        return -1;
    }
    Quad &q = quads[nquads];
    q.a = pin_a; q.b = pin_b; q.count = 0;
    attach(pin_a, nullptr, quadEdge, &q, CHANGE);
    attach(pin_b, nullptr, quadEdge, &q, CHANGE);
    q.state = (uint8_t)((gpio_pin_get_dt(&pins[pin_a]) << 1) | gpio_pin_get_dt(&pins[pin_b]));
    return nquads++;
}

extern "C" int32_t linoZephyrQuadRead(int h)
{
    if (h < 0 || h >= nquads) return 0;
    const unsigned int key = irq_lock();
    const int32_t c = quads[h].count;
    irq_unlock(key);
    return c;
}

extern "C" void linoZephyrQuadWrite(int h, int32_t count)
{
    if (h < 0 || h >= nquads) return;
    const unsigned int key = irq_lock();
    quads[h].count = count;
    irq_unlock(key);
}
