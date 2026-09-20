#include <Arduino.h>
#include <sensor_msgs/msg/battery_state.h>
#include "config.h"

// The INA219 is compiled in unconditionally and detected at boot, exactly like
// the IMU and the magnetometer. A released image is built for an MCU, not for a
// robot, so "does this robot meter its own power" cannot be a macro -- it is
// whether the chip answered 0x42 on the bus. ina219.init() returns false when
// it did not, and batteryPresent() is then the only gate.
#include <INA219_WE.h>
#include <stdlib.h>
#include "mcu_env.h"
// ESP32/S2 only in substance: adc_lut.h compiles to stubs everywhere else, so
// this include and the call below cost nothing on an S3 or an RP2.
#include "adc_lut.h"

// The ADC battery monitor, from the env with the generated header as the
// fallback. -1 means no ADC pin: the INA219 (if any) is then the only source.
// The divider is R1 to the pack, R2 to ground; a config without one (r1 == 0)
// reads the pin as the pack voltage itself.
#ifndef BATTERY_PIN
#define BATTERY_PIN -1
#endif
#ifndef BATTERY_R1
#define BATTERY_R1 0.0f
#endif
#ifndef BATTERY_R2
#define BATTERY_R2 1.0f
#endif
static int   bat_pin = -1;
static float bat_r1 = 0.0f, bat_r2 = 1.0f;
static float bat_min = 0.0f, bat_max = 0.0f, bat_cap = 0.0f;

#define INA219_ADDRESS 0x42
INA219_WE ina219 = INA219_WE(INA219_ADDRESS);

float shuntVoltage_mV = 0.0;
float loadVoltage_V = 0.0;
float busVoltage_V = 0.0;
float current_mA = 0.0;
float power_mW = 0.0;
bool ina219_overflow = false;
static bool ina219_present = false;

void initBattery(){
  ina219_present = ina219.init();
  if (ina219_present) {
    ina219.setADCMode(INA219_BIT_MODE_9);
    ina219.setPGain(INA219_PG_320);
    ina219.setBusRange(INA219_BRNG_16);
    ina219.setShuntSizeInOhms(0.01); // used in INA219.
  }
  initMcuEnv();
  const char *pin_env = envGet("battery_pin", NULL);
  bat_pin = (pin_env && *pin_env) ? (int)strtol(pin_env, NULL, 10) : (int)BATTERY_PIN;
  bat_r1 = envFloat("bat_r1", BATTERY_R1);
  bat_r2 = envFloat("bat_r2", BATTERY_R2);
#if defined(BATTERY_MIN) && defined(BATTERY_MAX)
  bat_min = envFloat("bat_min", BATTERY_MIN);
  bat_max = envFloat("bat_max", BATTERY_MAX);
#else
  bat_min = envFloat("bat_min", 0.0f);
  bat_max = envFloat("bat_max", 0.0f);
#endif
#ifdef BATTERY_CAP
  bat_cap = envFloat("bat_cap", BATTERY_CAP);
#else
  bat_cap = envFloat("bat_cap", 0.0f);
#endif
  if (bat_pin >= 0) {
    pinMode(bat_pin, INPUT);
    analogReadResolution(12);
    // Map the calibration table, if this board has one. Safe to call when no
    // table has ever been written -- adcLutValid() then stays false and
    // readVoltage() keeps using the core's own conversion.
    initAdcLut();
    if (adcLutValid())
      Serial.println("[battery] ADC linearisation table found — using the "
                     "measured curve for this chip");
  }
}

// True when there is any way to report a voltage: the INA219 answered, or this
// board reads the pack through a divider on an ADC pin.
bool batteryPresent(void)
{
  return bat_pin >= 0 || ina219_present;
}

void InaDataUpdate(){
  shuntVoltage_mV = ina219.getShuntVoltage_mV();
  busVoltage_V = ina219.getBusVoltage_V();
  current_mA = ina219.getCurrent_mA();
  power_mW = ina219.getBusPower();
  loadVoltage_V  = busVoltage_V + (shuntVoltage_mV/1000);
  ina219_overflow = ina219.getOverflow();
}

// Pack voltage through the divider. The ESP32 core converts to millivolts
// with the chip's own calibration; the RP2 ADC is 12-bit against 3.3 V.
static double readVoltage(int pin) {
  long reading = 0;
  int i;
  double pin_v;
#if defined(ESP32) && ADC_LUT_SUPPORTED
  // ESP32 (and S2) only: the one family with a DAC to sweep, so the only one
  // that can have a table. When this board has been calibrated, its OWN
  // measured curve beats the factory eFuse calibration that
  // analogReadMilliVolts applies -- measuring THIS chip is the entire point of
  // firmware/adc_calibrate, and until now the table it wrote was read by
  // nobody: adcLinearize() had no callers anywhere in the firmware.
  if (adcLutValid()) {
    for (i = 0; i < 4; i++) // smoothing, on the linearised reading
      reading += adcLinearize((uint16_t)analogRead(pin));
    reading /= i;
    // The table maps a raw count to the count a LINEAR converter would have
    // produced, so full scale is the top of the ADC range: 3.3 V at the 11 dB
    // attenuation the Arduino core defaults to.
    pin_v = reading * 3.3 / (double)(ADC_LUT_ENTRIES - 1);
  } else
#endif
  {
    for (i = 0; i < 4; i++) // smoothing
#ifdef ESP32
      reading += analogReadMilliVolts(pin);
#else
      reading += analogRead(pin);
#endif
    reading /= i;
#ifdef ESP32
    pin_v = reading / 1000.0;
#else
    pin_v = reading * 3.3 / 4095.0;
#endif
  }
  const double ratio = (bat_r2 > 0.0f) ? (bat_r1 + bat_r2) / bat_r2 : 1.0;
  return pin_v * ratio;
}

sensor_msgs__msg__BatteryState battery_msg_ = { .current = NAN, .charge = NAN,
    .capacity = NAN, .design_capacity = NAN, .percentage = NAN, .present =  true };
sensor_msgs__msg__BatteryState getBattery()
{
    if (bat_pin >= 0)
        battery_msg_.voltage = readVoltage(bat_pin);
    if (ina219_present) {
        // read voltage
        InaDataUpdate();
        battery_msg_.voltage = loadVoltage_V;
        battery_msg_.current = -current_mA / 1000; // Amp, minu when discharge
    }
    return battery_msg_;
}

/** https://github.com/rlogiacco/BatterySense/blob/master/Battery.h
 *
 * Symmetric sigmoidal approximation
 * https://www.desmos.com/calculator/7m9lu26vpy
 *
 * c - c / (1 + k*x/v)^3
 */
static inline float sigmoidal(float voltage, float minVoltage, float maxVoltage) {
    // slow
    // float result = 110 - (110 / (1 + pow(1.468 * (voltage - minVoltage)/(maxVoltage - minVoltage), 6)));

    // steep
    // float result = 102 - (102 / (1 + pow(1.621 * (voltage - minVoltage)/(maxVoltage - minVoltage), 8.1)));

    // normal
    float result = 105 - (105 / (1 + pow(1.724 * (voltage - minVoltage)/(maxVoltage - minVoltage), 5.5)));
    return result >= 100 ? 100 : result;
}

/**
 * Asymmetric sigmoidal approximation
 * https://www.desmos.com/calculator/oyhpsu8jnw
 *
 * c - c / [1 + (k*x/v)^4.5]^3
 */
static inline float asigmoidal(float voltage, float minVoltage, float maxVoltage) {
    float result = 101 - (101 / pow(1 + pow(1.33 * (voltage - minVoltage)/(maxVoltage - minVoltage) ,4.5), 3));
    return result >= 100 ? 100 : result;
}

void getBatteryPercentage(sensor_msgs__msg__BatteryState *msg)
{
    if (bat_max <= bat_min)
        return;   // no pack described: the message keeps NaN, which is honest
    msg->percentage = sigmoidal(msg->voltage, bat_min, bat_max) / 100;
    if (bat_cap > 0.0f) {
        msg->design_capacity = bat_cap;
        msg->capacity = bat_cap * msg->percentage;
    }
}
