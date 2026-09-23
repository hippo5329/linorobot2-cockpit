// The IMU data-ready ISR, out of line -- see the declaration for why.
#include "imu_interface.h"

IMUInterface *IMUInterface::instance_ = nullptr;

void IMU_ISR_ATTR IMUInterface::dataReadyISR()
{
    if (instance_) {
        // WHEN the sample exists, captured before anything else can delay us.
        // This is the whole point of the line for timing purposes: the chip
        // raises DATA_RDY the instant the conversion is done, so micros() here
        // is the sample's own time, not the time the MCU got round to reading
        // it. micros() is IRAM-safe on both cores (esp_timer_get_time on the
        // ESP32, time_us_32 on the RP2).
        instance_->data_ready_us_ = micros();
        instance_->data_ready_ = true;
        instance_->int_edges_++;
    }
}
