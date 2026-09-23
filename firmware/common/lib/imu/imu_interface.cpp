// The IMU data-ready ISRs, out of line -- see the declaration for why.
//
// One trampoline per slot. attachInterrupt() takes a bare function pointer on
// the AVR and RP2 cores (only the ESP32 has attachInterruptArg), so the slot
// index cannot be passed as an argument and has to be baked into the function.
#include "imu_interface.h"

IMUInterface *IMUInterface::sources_[IMU_INT_MAX_SOURCES] = { nullptr };

void IMU_ISR_ATTR IMUInterface::dataReadyISR(int slot)
{
    IMUInterface *s = sources_[slot];
    if (s) {
        // WHEN the sample exists, captured before anything else can delay us.
        // This is the whole point of the line for timing purposes: the chip
        // raises DATA_RDY the instant the conversion is done, so micros() here
        // is the sample's own time, not the time the MCU got round to reading
        // it. micros() is IRAM-safe on both cores (esp_timer_get_time on the
        // ESP32, time_us_32 on the RP2).
        //
        // It must stay FIRST, and it must be per-source: the slot lookup above
        // is two loads, but anything between the edge and this read is jitter
        // attributed to the sensor.
        s->data_ready_us_ = micros();
        s->data_ready_ = true;
        s->int_edges_++;
    }
}

void IMU_ISR_ATTR IMUInterface::isrSlot0() { dataReadyISR(0); }
void IMU_ISR_ATTR IMUInterface::isrSlot1() { dataReadyISR(1); }
void IMU_ISR_ATTR IMUInterface::isrSlot2() { dataReadyISR(2); }
void IMU_ISR_ATTR IMUInterface::isrSlot3() { dataReadyISR(3); }

// A table rather than a switch so that raising IMU_INT_MAX_SOURCES without
// adding a trampoline fails to compile instead of attaching nothing.
void (*IMUInterface::slotISR(int slot))()
{
    static void (*const table[4])() = { isrSlot0, isrSlot1, isrSlot2, isrSlot3 };
    static_assert(IMU_INT_MAX_SOURCES <= 4,
                  "add a trampoline per slot: attachInterrupt takes no argument on AVR/RP2");
    return table[slot];
}
