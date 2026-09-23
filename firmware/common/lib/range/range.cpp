#include <Arduino.h>
#include <micro_ros_utilities/string_utilities.h>
#include <sensor_msgs/msg/range.h>
#include <stdlib.h>
#include "config.h"
#include "mcu_env.h"

// define sound speed in m/uS
#define SOUND_SPEED 0.00034
#define TIMEOUT_US 30000 // 30ms (~5.1m max range)
#define MIN_DURATION_US 115 // ~2cm min range
#define FOV (15 * 0.0174533) // field of view rad
#define MIN_RANGE 0.02 // 2cm
#define MAX_RANGE (SOUND_SPEED * TIMEOUT_US / 2.0 * 0.95)

// The HC-SR04 is compiled in unconditionally and wired at boot, exactly like the
// INA219 in battery.cpp and the BMP280 in env.cpp. It used to be gated on
// `#ifdef TRIG_PIN`, which made "does this robot have a sonar" a property of the
// IMAGE: no reference config carried the pins, so range.cpp compiled to nothing
// in every released build and the interrupt-driven driver had never run on any
// board. A released image is built for an MCU, not for a robot -- the pins come
// from the env partition, and -1 means "not wired", checked at run time.
#ifndef TRIG_PIN
#define TRIG_PIN -1
#endif
#ifndef ECHO_PIN
#define ECHO_PIN -1
#endif

static int trig_pin = -1;
static int echo_pin = -1;

enum SonarState {
    SONAR_IDLE,
    SONAR_TRIGGERED,
    SONAR_WAIT_ECHO_FALL
};

static volatile SonarState sonar_state = SONAR_IDLE;
static volatile uint32_t echo_start_us = 0;
static volatile uint32_t echo_duration_us = 0;
static volatile bool new_reading_available = false;
static uint32_t last_trigger_us = 0;

#if defined(ESP32) || defined(ESP8266)
void IRAM_ATTR echoPinISR()
#else
void echoPinISR()
#endif
{
    uint32_t now = micros();
    // echo_pin is read, not a constant, so the ISR is the same code on a board
    // with the sensor on GP28 and one with it on GP15. It is only ever attached
    // when echo_pin >= 0, so there is no guard here.
    if (digitalRead(echo_pin) == HIGH) {
        if (sonar_state == SONAR_TRIGGERED || sonar_state == SONAR_IDLE) {
            echo_start_us = now;
            sonar_state = SONAR_WAIT_ECHO_FALL;
        }
    } else {
        if (sonar_state == SONAR_WAIT_ECHO_FALL) {
            uint32_t duration = now - echo_start_us;
            if (duration >= MIN_DURATION_US && duration <= TIMEOUT_US) {
                echo_duration_us = duration;
                new_reading_available = true;
            } else {
                echo_duration_us = 0; // out of bounds
                new_reading_available = true;
            }
            sonar_state = SONAR_IDLE;
        }
    }
}

sensor_msgs__msg__Range range_msg_;

// "Nothing within reach" is MAX_RANGE, never +INF and never 0.
//
// sensor_msgs/Range says a value outside [min_range, max_range] should be
// discarded, and +INF was the honest way to say "no echo came back". But
// nav2_collision_monitor does not merely discard it: Range::getSourceData()
// returns false for an out-of-span reading, and a source that returns false is
// an INVALID SOURCE, which makes the monitor stop the robot --
//
//     [collision_monitor]: Robot to stop due to invalid source.
//
// -- and log the reason at DEBUG, where nobody sees it. So a sonar facing an
// open room, which is the safest possible situation, reads to Nav2 exactly
// like a sonar that has died, and the robot never moves again. Measured on the
// bench 2026-09-23: every leg stalled mid-route, the monitor latched on the
// first reading with nothing in the cone, and the drive suite scored 8/8 on
// the same board seconds later.
//
// MAX_RANGE says the true thing -- clear as far as I can see -- inside the
// span, so the fail-safe still fires for a sensor that has genuinely stopped
// publishing while an open room does not brake the robot.
static inline float rangeClamped(float metres)
{
    if (!isfinite(metres) || metres > (float)MAX_RANGE) return (float)MAX_RANGE;
    if (metres < (float)MIN_RANGE) return (float)MIN_RANGE;
    return metres;
}

bool rangePresent() { return trig_pin >= 0 && echo_pin >= 0; }

sensor_msgs__msg__Range getRange()
{
    if (!rangePresent())
        return range_msg_;

    uint32_t now = micros();

    // 1. Check for timeout if waiting for echo from far-away object
    if (sonar_state == SONAR_WAIT_ECHO_FALL || sonar_state == SONAR_TRIGGERED) {
        if ((now - echo_start_us) > TIMEOUT_US || (now - last_trigger_us) > TIMEOUT_US) {
            range_msg_.range = (float)MAX_RANGE;   // no echo: clear, not broken
            sonar_state = SONAR_IDLE;
            new_reading_available = false;
        }
    }

    // 2. If a valid reading arrived from ISR, update range
    if (new_reading_available) {
        if (echo_duration_us > 0) {
            range_msg_.range = rangeClamped((float)(echo_duration_us * SOUND_SPEED / 2.0));
        } else {
            range_msg_.range = (float)MAX_RANGE;   // out of bounds: clear, not broken
        }
        new_reading_available = false;
    }

    range_msg_.field_of_view = FOV;
    range_msg_.min_range = MIN_RANGE;
    range_msg_.max_range = MAX_RANGE;

    // 3. Trigger next measurement only if previous echo finished (non-blocking 10us)
    if (sonar_state == SONAR_IDLE && (now - last_trigger_us >= 25000)) {
        sonar_state = SONAR_TRIGGERED;
        last_trigger_us = now;
        echo_start_us = now;
        digitalWrite(trig_pin, HIGH);
        delayMicroseconds(10);
        digitalWrite(trig_pin, LOW);
    }
    return range_msg_;
}

void initRange(bool allow_hardware)
{
    initMcuEnv();
    trig_pin = envInt("sonar_trig", TRIG_PIN);
    echo_pin = envInt("sonar_echo", ECHO_PIN);

    // Fake mode masks a real sonar. A simulated robot must not drive real pins:
    // its range has to come from the same simulated room its scan does, or
    // /sonar and /scan describe two different worlds. The caller decides --
    // main.cpp knows whether the wheels and the LiDAR are simulated -- and here
    // it simply means the pins are dropped before anything is configured.
    if (!allow_hardware)
    {
        trig_pin = -1;
        echo_pin = -1;
    }

    // initRange() runs from setup(), so the env is readable here and the
    // namespace can go straight on.
    range_msg_.header.frame_id = micro_ros_string_utilities_set(range_msg_.header.frame_id, envPrefixed("sonar_link"));
    range_msg_.field_of_view = FOV;
    range_msg_.min_range = MIN_RANGE;
    range_msg_.max_range = MAX_RANGE;
    // Seeded clear rather than +INF: the first messages go out before any echo
    // has come back, and an out-of-span reading is a stopped robot (above).
    range_msg_.range = (float)MAX_RANGE;

    if (!rangePresent()) {
        // Say so once. A /range that reads clear for ever looks identical to a
        // sensor pointing at open air, which is the failure this print exists
        // to tell apart. (Nothing is published in that case -- main.cpp only
        // creates the publisher when the pins resolved or the emulator runs --
        // but the print is what makes a missing sonar visible at boot.)
        Serial.printf("[range] no sonar: trigger=%d echo=%d (set sonar_trig / sonar_echo)\n",
                      trig_pin, echo_pin);
        return;
    }

    pinMode(trig_pin, OUTPUT);
    digitalWrite(trig_pin, LOW);
    pinMode(echo_pin, INPUT);
    sonar_state = SONAR_IDLE;
    new_reading_available = false;
    attachInterrupt(digitalPinToInterrupt(echo_pin), echoPinISR, CHANGE);
    Serial.printf("[range] HC-SR04 trigger=%d echo=%d (interrupt driven)\n", trig_pin, echo_pin);
}
