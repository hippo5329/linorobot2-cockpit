# Prebuilt fake-mode firmware

Ready-to-flash images so a board can be brought up without installing
PlatformIO, a cross-compiler, or anything else. They are **release assets**,
not tracked files: this directory is empty in git and filled by

```bash
python3 scripts/fetch_prebuilt.py pico2-jazzy    # -> firmware/prebuilt/pico2-jazzy/
```

| Profile | MCU | Transport |
|---|---|---|
| `pico2-jazzy`, `pico2-lyrical` | RP2350 | USB serial |
| `pico-jazzy`, `pico-lyrical` | RP2040 | USB serial |
| `esp32-jazzy`, `esp32-lyrical` | ESP32 | serial or `udp4` — whichever the env partition asks for |
| `esp32s3-jazzy`, `esp32s3-lyrical` | ESP32-S3 | native USB CDC, serial or `udp4` |

Every profile names its ROS 2 distro. The two halves of a row are **not**
interchangeable: `board_microros_distro` picks the precompiled micro-ROS library
the image links against, and a jazzy image will not talk to a lyrical agent. A
board flashed with the wrong one enumerates over USB and then does nothing
useful, so the distro belongs in the name rather than being implied by its
absence.

You can also name the PlatformIO env and let it be mapped — `fetch_prebuilt.py
pico2` fetches `pico2-jazzy`, because `[env:pico2]` is pinned to jazzy in
`platformio.ini`. The envs are asymmetric for that reason; the profiles are not.

**One image per MCU per ROS 2 distro — not one per robot.** A Waveshare General Driver
board and a bare ESP32 DevKit run the same `esp32-jazzy` image. What differs between
them — the pin matrix, the I2C bus, the LiDAR wiring, which IMU is fitted, the
transport, the credentials — is in the `env` flash partition, not in the binary.
A board is a configuration, not a build.

All of them are **fake mode**: the firmware simulates the IMU, magnetometer and
wheel encoders, so a board with nothing wired to it still publishes odometry and
a scan. That is what lets one image be useful without knowing anything about
your hardware — it is for bringing a board up and proving the pipeline, not for
driving a real robot.

## Flashing

```bash
python3 scripts/flash_mcu.py --prebuilt pico2-jazzy --port /dev/ttyACM0
```

For an ESP32, name the robot so the env block describes the board in front of
you rather than whichever robot the image happened to be compiled for:

```bash
python3 scripts/flash_mcu.py --prebuilt esp32-jazzy --port /dev/ttyUSB0 \
    --params ~/linorobot2-config/gendrv_config.yaml
```

Every artifact is checksummed against `manifest.json` before anything is
written. Nothing is compiled and PlatformIO is never invoked.

## Nothing about your site or your wiring is in these images

The credentials, the agent address, the syslog address and the LiDAR UDP address
are facts about **your network**; the pin matrix is a fact about **your board**.
An image with either compiled in works on exactly one bench, which would make a
prebuilt image pointless.

They live in a separate 4 KB `env` flash partition at `0x290000` instead
(`firmware/common/partitions_lino.csv`), in the same layout U-Boot uses: a
CRC32 followed by `key=value\0…\0\0`. Every key falls back to what the image was
compiled with, so a board with a blank env still boots and still says so.

Put your credentials in `~/linorobot2-config/secrets.yaml` and
flash as above — the env block is generated from that and your robot config and
written for you. The three addresses default to the IP of the machine you run it
on, which is the machine running `micro_ros_agent`, so usually there is nothing
to fill in.

To re-key or rewire a robot later, without touching the application:

```bash
python3 scripts/mcu_env.py build --params ~/linorobot2-config/gendrv_config.yaml --out env.bin
python3 scripts/mcu_env.py set env.bin wifi_ssid=other-ap agent_ip=192.168.1.10
esptool write_flash 0x290000 env.bin
```

Writing the application at `0x10000` never touches `0x290000`, so reflashing
firmware keeps the configuration, and rewriting the configuration keeps the
firmware.

### What the firmware reads

```
site        wifi_ssid  wifi_psk  wifi
            agent_ip   agent_port   transport
            syslog_ip  syslog_port
            lidar_ip   lidar_port

board       i2c_sda  i2c_scl  i2c_clock
            gpio_out  gpio_out_late  boot_delay
            imu  mag

drivetrain  m<N>_pwm  m<N>_in_a  m<N>_in_b  m<N>_inv
            m<N>_enc_a  m<N>_enc_b  m<N>_cpr  m<N>_enc_inv    (N = 1..4)
            motor_driver  pwm_freq  pwm_bits  pwm_min  pwm_max
            kp  ki  kd  fake_wheel

kinematics  base  max_rpm  rpm_ratio  wheel_d  lr_dist
            motor_v  power_v
```

`gpio_out` and `gpio_out_late` are `"pin=level,pin=level"` — pins driven at the
start and at the end of `setup()` respectively. Anything else you put in the
block is preserved and ignored, as in U-Boot.

Read a board's configuration back with `python3 scripts/mcu_env.py print env.bin`
(the PSK is redacted unless you pass `--show-secrets`).

## Rebuilding these images

```bash
python3 scripts/build_prebuilt.py                        # all eight, needs PlatformIO
python3 scripts/build_prebuilt.py esp32-jazzy            # just one
docker compose run --rm pio python3 scripts/build_prebuilt.py   # in the build image
```

Releases do this in `.github/workflows/release.yml` and attach the archives.
