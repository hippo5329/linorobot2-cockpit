// Board bring-up: the I2C bus and whatever GPIOs a board must drive at boot.
//
// This replaces the BOARD_INIT / BOARD_INIT_LATE macro hooks inherited from
// linorobot2_hardware. Those were a block of C statements pasted into a header
// and expanded inside setup(), which meant every board that needed its I2C bus
// on non-default pins -- or an enable line pulled high before its motor driver
// would answer -- carried a fragment of setup() in its config. A board is then
// a build, and the same binary cannot serve two of them.
//
// Here the same two jobs are data:
//
//     i2c_sda, i2c_scl    bus pins            (fallback SDA_PIN / SCL_PIN)
//     i2c_clock           bus frequency, Hz   (fallback I2C_CLOCK, 400000)
//     gpio_out            "5=1,13=0"          driven in initBoard()
//     gpio_out_late       "12=1"              driven in initBoardLate()
//     boot_delay          ms to wait in initBoard()
//
// Each is an env key (scripts/mcu_env.py) falling back to the macro the config
// generator emitted, so a board with a blank env comes up exactly as its YAML
// describes and a board with an env can be rewired without a compiler.
#ifndef BOARD_INIT_H
#define BOARD_INIT_H

// I2C bus, boot-time output pins, and the optional settling delay. Call it
// early in setup(), before anything touches a sensor: the drivers call
// Wire.begin() themselves and a parameterless begin() keeps the pins this set.
void initBoard(void);

// Output pins that must not be driven until the stack is up -- a motor-driver
// enable line, typically, which should stay low while the board is still
// deciding what its PWM pins are. Call it at the end of setup().
void initBoardLate(void);

#endif // BOARD_INIT_H
