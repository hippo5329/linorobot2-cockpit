#include <Arduino.h>
#include <string.h>
#include <stdio.h>
#include "config.h"
#include "hw_factory.h"
#include "mcu_env.h"
#include "default_motor.h"
#include "encoder.h"
#include "fake_wheel.h"

// Per-motor env keys are built rather than listed: m1_pwm, m2_pwm, ... Four
// motors times six settings is twenty-four names, and spelling each one out
// invites exactly the sort of copy-paste slip where motor 3 quietly reads
// motor 2's pin.
static int envPin(int index, const char *suffix, int fallback)
{
    char key[16];
    snprintf(key, sizeof(key), "m%d_%s", index, suffix);
    const char *value = envGet(key, NULL);
    if (!value || !*value)
        return fallback;
    return (int)strtol(value, NULL, 10);
}

static bool envFlag(const char *key, bool fallback)
{
    const char *value = envGet(key, NULL);
    if (!value || !*value)
        return fallback;
    return !(strcmp(value, "0") == 0 || strcasecmp(value, "false") == 0
             || strcasecmp(value, "no") == 0);
}

static float envFloat(const char *key, float fallback)
{
    const char *value = envGet(key, NULL);
    if (!value || !*value)
        return fallback;
    return strtof(value, NULL);
}

// The compile-time matrix, indexed so the factory can loop. These are the
// values the config was generated for and they remain the fallback for every
// key above -- a board with a blank env is wired exactly as its YAML says.
static const int PIN_PWM[4]     = {MOTOR1_PWM, MOTOR2_PWM, MOTOR3_PWM, MOTOR4_PWM};
static const int PIN_IN_A[4]    = {MOTOR1_IN_A, MOTOR2_IN_A, MOTOR3_IN_A, MOTOR4_IN_A};
static const int PIN_IN_B[4]    = {MOTOR1_IN_B, MOTOR2_IN_B, MOTOR3_IN_B, MOTOR4_IN_B};
static const bool MOTOR_INV[4]  = {MOTOR1_INV, MOTOR2_INV, MOTOR3_INV, MOTOR4_INV};
static const int ENC_A[4]       = {MOTOR1_ENCODER_A, MOTOR2_ENCODER_A, MOTOR3_ENCODER_A, MOTOR4_ENCODER_A};
static const int ENC_B[4]       = {MOTOR1_ENCODER_B, MOTOR2_ENCODER_B, MOTOR3_ENCODER_B, MOTOR4_ENCODER_B};
static const int ENC_CPR[4]     = {COUNTS_PER_REV1, COUNTS_PER_REV2, COUNTS_PER_REV3, COUNTS_PER_REV4};
static const bool ENC_INV[4]    = {MOTOR1_ENCODER_INV, MOTOR2_ENCODER_INV,
                                   MOTOR3_ENCODER_INV, MOTOR4_ENCODER_INV};

// A real encoder behind the shared interface. `Encoder` is vendored upstream
// code in three platform variants, so it is wrapped rather than modified.
class RealEncoder : public EncoderInterface
{
    public:
        RealEncoder(int pin_a, int pin_b, int counts_per_rev, bool invert)
            : enc_(pin_a, pin_b, counts_per_rev, invert) {}
        float getRPM() override { return enc_.getRPM(); }
    private:
        Encoder enc_;
};

bool wheelsAreFake(void)
{
    initMcuEnv();
#ifdef USE_FAKE_WHEEL
    return envFlag("fake_wheel", true);
#else
    return envFlag("fake_wheel", false);
#endif
}

EncoderInterface *createEncoder(int index)
{
    const int i = index - 1;
    const int pin_a = envPin(index, "enc_a", ENC_A[i]);
    const int pin_b = envPin(index, "enc_b", ENC_B[i]);
    const int cpr   = envPin(index, "cpr",   ENC_CPR[i]);

    char key[16];
    snprintf(key, sizeof(key), "m%d_enc_inv", index);
    const bool invert = envFlag(key, ENC_INV[i]);

    if (wheelsAreFake())
        return new FakeEncoder(pin_a, pin_b, cpr, invert);
    return new RealEncoder(pin_a, pin_b, cpr, invert);
}

MotorInterface *createMotor(int index)
{
    const int i = index - 1;
    const float freq = envFloat("pwm_freq", PWM_FREQUENCY);
    const int bits   = (int)envU16("pwm_bits", PWM_BITS);
    const int pwm    = envPin(index, "pwm",  PIN_PWM[i]);
    const int in_a   = envPin(index, "in_a", PIN_IN_A[i]);
    const int in_b   = envPin(index, "in_b", PIN_IN_B[i]);

    char key[16];
    snprintf(key, sizeof(key), "m%d_inv", index);
    const bool invert = envFlag(key, MOTOR_INV[i]);

    // The driver type is one string for all four motors: a robot with two
    // different H-bridges on one axle is not a case worth carrying an extra
    // twenty bytes of env for, and nothing in the tree has ever done it.
    const char *driver = envGet("motor_driver", MOTOR_DRIVER_DEFAULT);
    if (strcasecmp(driver, "generic1") == 0)
        return new Generic1(freq, bits, invert, pwm, in_a, in_b);
    if (strcasecmp(driver, "bts7960") == 0)
        return new BTS7960(freq, bits, invert, pwm, in_a, in_b);
    if (strcasecmp(driver, "esc") == 0)
        return new ESC(freq, bits, invert, pwm, in_a, in_b);
    if (strcasecmp(driver, "generic2") != 0 && index == 1)
        Serial.printf("[hw] unknown motor_driver '%s' — using generic2\n", driver);
    return new Generic2(freq, bits, invert, pwm, in_a, in_b);
}

PID *createPID(void)
{
    // The controller's output range is the motor's PWM range, so it is
    // derived from the same `pwm_bits` the motors read -- not from separate
    // pwm_min / pwm_max keys, which nothing ever wrote: an image compiled for
    // 10-bit PWM clamped the PID at +-1023 on a robot whose env said 8 bits,
    // and the motor was told 1023 on a 255-wide channel.
    const int bits = (int)envU16("pwm_bits", PWM_BITS);
    const float pwm_max = (float)((1L << bits) - 1);
    return new PID(-pwm_max, pwm_max,
                   envFloat("kp", K_P), envFloat("ki", K_I), envFloat("kd", K_D));
}

Kinematics *createKinematics(void)
{
    // Kinematics already took its base type as a constructor argument, so the
    // only thing standing between it and the env was the LINO_BASE macro.
    // Two vocabularies reach this string and BOTH must be understood. The robot
    // config spells the base `2wd` / `4wd` / `mecanum` and mcu_env.py writes that
    // straight into the env block; the generated header spells the same fact
    // `differential_drive` / `skid_steer` / `mecanum` (KINEMATICS_BASE_DEFAULT).
    // Only `mecanum` overlapped, so every `base=2wd` in an env partition matched
    // nothing here, fell through to the compiled-in macro and printed
    // "[hw] unknown base '2wd'" at every boot -- which made the `base` env key
    // DEAD for the two most common bases. It read as harmless because the
    // fallback is generated from the same config and therefore usually agrees;
    // it stops being harmless the moment the env is meant to change the base
    // without a rebuild, which is the entire point of the env block (AGENTS.md
    // §10) -- a `4wd` written there would silently keep running differential
    // kinematics, and the robot would simply drive wrong.
    const char *base = envGet("base", KINEMATICS_BASE_DEFAULT);
    Kinematics::base robot_base = Kinematics::LINO_BASE;
    if (strcasecmp(base, "2wd") == 0 || strcasecmp(base, "differential") == 0 ||
        strcasecmp(base, "differential_drive") == 0)
        robot_base = Kinematics::DIFFERENTIAL_DRIVE;
    else if (strcasecmp(base, "4wd") == 0 || strcasecmp(base, "skid") == 0 ||
             strcasecmp(base, "skid_steer") == 0)
        robot_base = Kinematics::SKID_STEER;
    else if (strcasecmp(base, "mecanum") == 0)
        robot_base = Kinematics::MECANUM;
    else
        Serial.printf("[hw] unknown base '%s' — using the compiled-in default\n", base);

    return new Kinematics(robot_base,
                          (int)envU16("max_rpm", MOTOR_MAX_RPM),
                          envFloat("rpm_ratio", MAX_RPM_RATIO),
                          envFloat("motor_v", MOTOR_OPERATING_VOLTAGE),
                          envFloat("power_v", MOTOR_POWER_MAX_VOLTAGE),
                          envFloat("wheel_d", WHEEL_DIAMETER),
                          envFloat("lr_dist", LR_WHEELS_DISTANCE));
}
