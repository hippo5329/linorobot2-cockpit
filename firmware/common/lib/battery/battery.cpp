#include <Arduino.h>
#include <sensor_msgs/msg/battery_state.h>
#include "config.h"

// The INA219 is compiled in unconditionally and detected at boot, exactly like
// the IMU and the magnetometer. A released image is built for an MCU, not for a
// robot, so "does this robot meter its own power" cannot be a macro -- it is
// whether the chip answered 0x42 on the bus. The config write fails when
// it did not, and batteryPresent() is then the only gate.
#include <Wire.h>
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

// The INA219, register by register -- our own, replacing INA219_WE: a config
// write and three reads is all this ever asked of that library. Programmed as
// before: 16 V bus range, +/-320 mV shunt gain, 9-bit conversions, continuous
// shunt + bus (config 0x1807), and a 0.01 ohm shunt. Current is the shunt
// voltage over the shunt resistance, so the calibration register is not used.
#define INA219_ADDRESS 0x42
#define INA219_CONFIG  0x1807
#define INA219_SHUNT_OHMS 0.01f
static bool ina219Write(uint8_t reg, uint16_t v)
{
  Wire.beginTransmission(INA219_ADDRESS);
  Wire.write(reg);
  Wire.write((uint8_t)(v >> 8));
  Wire.write((uint8_t)(v & 0xFF));
  return Wire.endTransmission() == 0;
}
static bool ina219Read(uint8_t reg, uint16_t *v)
{
  Wire.beginTransmission(INA219_ADDRESS);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)INA219_ADDRESS, 2) != 2) return false;
  *v = ((uint16_t)Wire.read() << 8) | Wire.read();
  return true;
}

float shuntVoltage_mV = 0.0;
float loadVoltage_V = 0.0;
float busVoltage_V = 0.0;
float current_mA = 0.0;
float power_mW = 0.0;
bool ina219_overflow = false;
static bool ina219_present = false;

// --- the simulated pack ------------------------------------------------------
// A voltage sensor for a board with none wired (env `sim_battery`, default off;
// every bare config turns it on). Open-circuit voltage falls linearly from
// bat_max to bat_min as the charge drains; the wheel model's sag divides it
// under load, as it divides the voltage the simulated motors see; a fixed idle
// draw stands for the robot computer and the electronics. An empty pack is
// swapped for a full one rather than left at 0 %, so a soak runs forever.
#ifndef SIM_BATTERY_DEFAULT
#define SIM_BATTERY_DEFAULT false
#endif
#define SIM_BATT_IDLE_A 0.3f
static bool  s_sim = false;
static float s_sim_div = 1.0f, s_sim_amps = 0.0f;
static float s_charge_ah = 0.0f;
static unsigned long s_sim_ms = 0;

bool batteryIsSim() { return s_sim; }
void setSimPackLoad(float sag_divisor, float amps)
{
  s_sim_div = sag_divisor >= 1.0f ? sag_divisor : 1.0f;
  s_sim_amps = amps > 0.0f ? amps : 0.0f;
}

void initBattery(){
  // Present when it acknowledges the config write, as INA219_WE's init() was.
  ina219_present = ina219Write(0x00, INA219_CONFIG);
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
  s_sim = envFlag("sim_battery", SIM_BATTERY_DEFAULT);
  if (s_sim) {
    // A config with no pack described gets a typical 3S Li-ion one.
    if (bat_max <= bat_min || bat_max <= 0.0f) { bat_min = 9.9f; bat_max = 12.6f; }
    if (bat_cap <= 0.0f) bat_cap = 5.0f;
    s_charge_ah = bat_cap;
    Serial.printf("[battery] sim_battery=1: simulated %.1f-%.1f V, %.1f Ah pack\n",
                  bat_min, bat_max, bat_cap);
  }
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
  return s_sim || bat_pin >= 0 || ina219_present;
}

void InaDataUpdate(){
  uint16_t shunt = 0, bus = 0;
  if (!ina219Read(0x01, &shunt) || !ina219Read(0x02, &bus))
    return;                                     // keep the last reading
  shuntVoltage_mV = (int16_t)shunt * 0.01f;     // LSB 10 uV
  busVoltage_V = (bus >> 3) * 0.004f;           // bits 15:3, LSB 4 mV
  ina219_overflow = bus & 0x0001;               // OVF
  current_mA = shuntVoltage_mV / INA219_SHUNT_OHMS;
  power_mW = busVoltage_V * current_mA;
  loadVoltage_V  = busVoltage_V + (shuntVoltage_mV/1000);
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
    if (s_sim) {
        const unsigned long now = millis();
        const float dt_h = s_sim_ms ? (now - s_sim_ms) / 3600000.0f : 0.0f;
        s_sim_ms = now;
        const float amps = s_sim_amps + SIM_BATT_IDLE_A;
        s_charge_ah -= amps * dt_h;
        if (s_charge_ah <= 0.0f) {
            s_charge_ah = bat_cap;
            Serial.println("[battery] simulated pack empty: swapped for a full one");
        }
        const float soc = s_charge_ah / bat_cap;
        const float v_oc = bat_min + (bat_max - bat_min) * soc;
        // +/-10 mV of reading noise, the order of a 12-bit divider's
        battery_msg_.voltage = v_oc / s_sim_div + ((float)random(-100, 101)) * 0.0001f;
        battery_msg_.current = -amps;           // negative: discharging
        battery_msg_.charge = s_charge_ah;
        battery_msg_.capacity = bat_cap;
        battery_msg_.design_capacity = bat_cap;
        battery_msg_.present = true;
        return battery_msg_;
    }
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
