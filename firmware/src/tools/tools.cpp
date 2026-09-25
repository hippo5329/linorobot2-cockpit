#include <Arduino.h>
#include "config.h"
#include "tools.h"
#include "mcu_env.h"
#include "adc_lut.h"

// Each tool's entry points. Declared here rather than in tools.h so that a tool
// adding a function cannot accidentally become part of the shared interface.
extern "C" {
void test_sensors_setup(void);   void test_sensors_loop(void);
void test_motors_setup(void);    void test_motors_loop(void);
void test_acc_setup(void);       void test_acc_loop(void);
void i2c_detect_setup(void);     void i2c_detect_loop(void);
#if ADC_LUT_SUPPORTED
void adc_calibrate_setup(void);  void adc_calibrate_loop(void);
#endif
}

struct ToolEntry {
    AppMode      mode;
    const char  *name;
    void       (*setup)(void);
    void       (*loop)(void);
};

static const ToolEntry TOOLS[] = {
    { APP_BASE,          "base",          NULL,                NULL               },
    { APP_TEST_SENSORS,  "test_sensors",  test_sensors_setup,  test_sensors_loop  },
    { APP_TEST_MOTORS,   "test_motors",   test_motors_setup,   test_motors_loop   },
    { APP_TEST_ACC,      "test_acc",      test_acc_setup,      test_acc_loop      },
    { APP_I2C_DETECT,    "i2c_detect",    i2c_detect_setup,    i2c_detect_loop    },
#if ADC_LUT_SUPPORTED
    // Only on the parts with a hardware DAC. Elsewhere the name is not in the
    // table at all, so `app=adc_calibrate` falls through to base with a message
    // rather than booting a tool that cannot do anything.
    { APP_ADC_CALIBRATE, "adc_calibrate", adc_calibrate_setup, adc_calibrate_loop },
#endif
};

static const size_t TOOL_COUNT = sizeof(TOOLS) / sizeof(TOOLS[0]);

static const ToolEntry *findByMode(AppMode mode)
{
    for (size_t i = 0; i < TOOL_COUNT; i++)
        if (TOOLS[i].mode == mode)
            return &TOOLS[i];
    return &TOOLS[0];
}

AppMode toolSelect(void)
{
    initMcuEnv();
    const char *want = envGet("app", "base");
    if (!want || !*want)
        return APP_BASE;

    for (size_t i = 0; i < TOOL_COUNT; i++)
        if (strcmp(TOOLS[i].name, want) == 0)
            return TOOLS[i].mode;

    Serial.printf("[app] '%s' is not an application this image carries - "
                  "starting the robot firmware instead.\n", want);
    return APP_BASE;
}

const char *toolName(AppMode mode) { return findByMode(mode)->name; }

void toolSetup(AppMode mode)
{
    const ToolEntry *tool = findByMode(mode);
    if (tool->setup)
        tool->setup();
}

void toolLoop(AppMode mode)
{
    const ToolEntry *tool = findByMode(mode);
    if (tool->loop)
        tool->loop();
}
