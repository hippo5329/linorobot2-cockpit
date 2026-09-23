// Nothing left to define out of line.
//
// This file held the data-ready ISR and its slot table. The whole interrupt
// path was removed on 2026-09-24: an ISR that cannot read the bus -- and it
// cannot, because the ESP32 Arduino I2C driver takes a FreeRTOS mutex with
// portMAX_DELAY -- buys only a timestamp whose pairing with the eventually-read
// sample is uncertain, plus one avoided transaction. The chip's own FIFO and
// timestamp counter give the same information with the sample attached to it,
// and need no pin, no ISR and no masking.
//
// Kept as an empty translation unit rather than deleted so that every
// platformio env's source list stays valid without a change per board.
#include "imu_interface.h"
