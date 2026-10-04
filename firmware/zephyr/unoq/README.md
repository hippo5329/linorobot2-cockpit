# Arduino UNO Q MCU on Zephyr (experiment, `devel` only)

The UNO Q's STM32U585 as a micro-ROS base controller, running Zephyr instead of the
PlatformIO/Arduino firmware in `firmware/src` (PlatformIO cannot build for this chip).
Status: **builds** (Zephyr 4.4.2, micro-ROS jazzy, 92 KB flash / 52 KB RAM); not yet run on
a board.

- `app/` -- the micro-ROS Zephyr module's sample app (`src/main.c` is upstream's, Apache-2.0),
  with the serial transport on **lpuart1**, the UNO Q's internal link to Linux
  (`/dev/ttyHS1`), and a `prj.conf` updated for Zephyr 4.4.
- `patches/` -- what the module needs on Zephyr 4.4: the serial transport on lpuart1 (it
  hardcodes usart1, which is the D0/D1 console here), and compile flags for picolibc:
  decline C11 Annex K (`__STDC_WANT_LIB_EXT1__=0`), `_DEFAULT_SOURCE`, and `compat/`.
- `compat/` -- `zephyr/posix/time.h` (removed in Zephyr 4.x, still included by rcutils) and
  an `isatty` declaration.
- `build.sh [workdir]` -- reproduces the build in `zephyrprojectrtos/ci:v0.29.4`.

Flashing (from the board's own Linux; the SWD pins are not on any header): stop
`arduino-router`, `adb forward tcp:3333 tcp:3333`, `adb shell arduino-debug`, then
`west flash -r openocd`. Restore Arduino's image with
`adb shell arduino-cli burn-bootloader -b arduino:zephyr:unoq -P jlink`.
