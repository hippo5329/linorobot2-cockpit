// The IMU data-ready ISR, out of line -- see the declaration for why.
#include "imu_interface.h"

IMUInterface *IMUInterface::instance_ = nullptr;

void IMU_ISR_ATTR IMUInterface::dataReadyISR()
{
    if (instance_)
        instance_->data_ready_ = true;
}
