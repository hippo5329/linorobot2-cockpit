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

## The base controller (`base/`, phase 3)

`build.sh <workdir>` builds it (`build.sh <workdir> app` builds the int32 link test).
Same contract as the Arduino firmware: node `linorobot_base_node`, `odom/unfiltered` at
50 Hz (best effort, `odom` -> `base_footprint`), `cmd_vel` (`Twist`, or `TwistStamped` with
`stamped_cmd_vel=1`) with the 200 ms dead-man, and the env block `scripts/mcu_env.py` writes,
read from the last flash page (0x081FE000). Keys read: `app` (`base` | `test_sensors`),
`base` (2wd only), `max_rpm`, `rpm_ratio`, `wheel_d`, `lr_dist`, `angular_scale`, `motor_v`,
`power_v`, `kp`/`ki`/`kd`, `pwm_freq`, `pwm_bits`, `m1_cpr`/`m2_cpr`, `m1_inv`/`m2_inv`,
`m1_enc_inv`/`m2_enc_inv`, `sim_wheel`, `best_effort`, `baud` (default 921600),
`domain_id`, `node`. Pins are fixed by `base/app.overlay`, not the env.

`sim_wheel=1` is a first-order wheel (150 ms) fed the commanded PWM; the full model is phase 4.
The console is a RAM buffer (`ram_console_buf`), read over SWD; the UART console does not reach Linux.

Not yet: IMU, battery publishing, `topic_prefix`, the boot banner on the link, an
interrupt-driven transport write (the polled write takes ~8 ms per Odometry at 921600).
