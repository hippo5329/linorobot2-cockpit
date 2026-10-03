/*
    Initialize the Pico W WiFi driver

    Copyright (c) 2022 Earle F. Philhower, III <earlephilhower@yahoo.com>

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include <cyw43_wrappers.h>

// linorobot2-cockpit: the CYW43's pins are the board's, not the chip's. A Pico 2 W
// wires them to GP23/24/29/25 (the driver's defaults); the SparkFun XRP Controller's
// RM2 sits on GP26 (REG_ON), 29 (DATA), 28 (CLOCK), 27 (CS). This image serves
// both, so the firmware sets them from the env (cyw43_pins) before the driver
// starts -- board_init.cpp defines the hook; an image built without it keeps the
// defaults. Possible because the board is built with CYW43_PIN_WL_DYNAMIC=1, as
// arduino-pico's own XRP variant is.
extern "C" void lino_cyw43_pins(void) __attribute__((weak));

extern "C" void initVariant() {
    if (lino_cyw43_pins) {
        lino_cyw43_pins();
    }
    init_cyw43_wifi();
}
