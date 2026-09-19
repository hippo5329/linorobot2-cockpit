# Linorobot2 Cockpit 🚀

> **From a bare microcontroller to SLAM, Nav2 and a saved map — in one click, from a browser.**

[![ROS 2](https://img.shields.io/badge/ROS%202-Jazzy%20%7C%20Lyrical-blue.svg)](https://docs.ros.org/)
[![micro-ROS](https://img.shields.io/badge/micro--ROS-Jazzy%20%7C%20Lyrical-green.svg)](https://micro.ros.org/)
[![Boards](https://img.shields.io/badge/Boards-Pico%202%20%7C%20Pico%20%7C%20ESP32%20%7C%20ESP32--S3-orange.svg)](#supported-boards)
[![License](https://img.shields.io/badge/License-Apache%202.0-lightgrey.svg)](LICENSE)

Linorobot2 Cockpit is a ROS 2 package, a web supervisor and the unified micro-ROS firmware for
[linorobot2](https://github.com/linorobot/linorobot2)-style mobile robots. It runs on the
**robot computer** — the Raspberry Pi 5, Jetson or Linux PC the microcontroller is plugged
into — and you drive it from any browser on the network.

- **Zero-wiring first run.** A bare Raspberry Pi Pico 2 or ESP32 with nothing soldered to it
  publishes simulated odometry, IMU and a virtual room's LiDAR scan, so the whole pipeline
  (micro-ROS, EKF, SLAM Toolbox, Nav2, map saver) runs before you build anything.
- **Prebuilt firmware, prebuilt images.** Releases ship flashable images for every supported
  board and Docker images for the ROS 2 stack. Nothing is compiled on your side unless you
  want to.
- **One config file per robot**, kept in your own git repository outside this one.
- **The board says what it runs.** Every boot prints its application, distro, build date and
  git revision; the cockpit only reflashes a board that actually differs.

---

## Quick start

You need: a Linux computer with USB (the **robot computer**) and a supported board on a USB
cable. Everything below happens on that computer, and the commands bring their own tools —
a fresh install has neither `git` nor a container engine.

**Plug the board in first**, then:

```bash
sudo apt install -y git
git clone https://github.com/hippo5329/linorobot2-cockpit.git
cd linorobot2-cockpit
bash scripts/install_docker.sh          # only if you have neither docker nor podman
docker compose up -d
docker compose logs | grep token     # prints http://<robot-computer>:8000/?token=... -- open that once
```

**Which Linux does not matter** — ROS 2 and every tool run inside the container, so Raspberry
Pi OS, Debian, Ubuntu, Fedora or Arch are all fine; the machine provides a 64-bit kernel, the
USB bus and a container engine. `install_docker.sh` installs Docker Engine (rootless) only when
the machine has neither Docker nor Podman, and never replaces one you have. Plugging the board
in first means the first page already names your MCU. The details — rootless vs rootful, why
Ubuntu's `docker.io` cannot run rootless, file ownership — are in [docs/docker.md](docs/docker.md).

Open the address the cockpit prints — `docker compose logs | grep token` shows it as
**`http://<robot-computer>:8000/?token=…`**. Open it once; the browser remembers the token
and every later visit is plain `http://<robot-computer>:8000`. The token is what stops
another machine, or another web page open on your LAN, from telling the cockpit to flash
and run things; it lives in `~/linorobot2-config/.cockpit_token` if you need it again, and
`COCKPIT_AUTH=off` in the environment switches the check off on a network you trust.

On first start the cockpit creates `~/linorobot2-config/` with the reference robot configs
and turns it into a git repository — that directory is yours to edit and commit.

The images come from Docker Hub; the first start downloads about 1.2 GB, which is a
minute or so on a normal connection.

Then, in the browser:

1. Check the **Base** panel. It should already name your board — "Raspberry Pi Pico
   (RP2040)", the VID:PID and the port it is on. If it says nothing is connected, press
   **Re-scan Hardware**.
2. Pick your robot in the **Robot** selector at the top left. This is the one that
   matters: 1-Click runs the base controller named in that robot's config file, so it
   has to match the board you plugged in — `pico` for a Raspberry Pi Pico, `pico2` for a
   Pico 2, `esp32` for an ESP32. The **Reference Build** dropdown beside it is a
   different control: it fills in the Config Studio fields for a known wiring, and
   changes nothing until you press **Save & Regenerate Header**.
3. Press **Start 1-Click**.

If the robot and the board disagree, the run stops before writing anything and says so
(`MCU MISMATCH`) — it will not flash an RP2350 image onto an RP2040. Pick the matching
robot, or edit `base_controller.name` in the config, and press Start again.

About a minute later the map appears in the **Map Viewer** tab and is saved under `maps/`.
What happened: the cockpit fetched the release firmware for your board, asked the board what
it was running, flashed it only if needed, wrote the 4 KB configuration block, started
`micro_ros_agent`, bringup, SLAM Toolbox and Nav2, verified the topics, sent a navigation
goal, and saved the map.

### Without Docker

```bash
bash scripts/install_deps.sh            # python3-yaml python3-fastapi python3-uvicorn python3-serial
python3 web/backend/main.py             # supervisor on :8000
```

ROS 2 (jazzy or lyrical), `micro_ros_agent`, `slam_toolbox`, `nav2_bringup`,
`robot_localization`, `imu_tools`, `rosbridge_server`, `robot_state_publisher`,
`joint_state_publisher` and the `ldlidar_stl_ros2` package must be installed and sourced; `esptool` and `picotool` are needed
to flash. The Docker image carries all of that, which is why it is the recommended path.

### From the command line

```bash
python3 scripts/one_click_pipeline.py --controller pico2 --distro jazzy
python3 scripts/one_click_pipeline.py --controller esp32_wifi --firmware prebuilt --no-nav2
```

---

## Words the cockpit uses

| word | meaning |
|---|---|
| **robot computer** | the Linux machine the board is plugged into; everything runs there |
| **base controller** | the microcontroller board (Pico 2, ESP32, …) running the firmware that drives the wheels |
| **MCU** | the chip on that board — RP2350, RP2040, ESP32, ESP32-S3 |
| **robot config** | one YAML file per robot in `~/linorobot2-config/`; the single source of truth |
| **reference build** | a shipped example config for a known board; loading one fills the fields, it does not flash anything |
| **env block** | the 4 KB block written to the board with your config's facts (pins, sensors, baud); rewritten without reflashing |
| **header** | the C file generated from your config as the fallback for a board with a blank env; "Save & Regenerate Header" means "save the config" |
| **flash** | write the firmware image to the board |
| **agent** | `micro_ros_agent`, the bridge between the board's serial port and ROS 2 |
| **bringup** | starting the ROS 2 nodes that turn the board's data into `/odom`, TF and `/scan` |
| **fake mode** | the firmware simulating wheels, IMU and a LiDAR room, so a bare board runs the whole pipeline |
| **description** | the robot's URDF — body, wheels, where the LiDAR and IMU sit — generated from your config at every bringup into `generated/` next to it; `robot_state_publisher` broadcasts it as the TF tree |

## How it is put together

```
   any browser ──http://robot:8000──▶ ROBOT COMPUTER
                                        ├─ web supervisor (FastAPI, web/)
                                        ├─ micro_ros_agent · bringup · SLAM · Nav2 · rosbridge :9090
                                        ├─ esptool / picotool ──USB──▶ base controller (Pico 2, ESP32, …)
                                        ├─ ~/linorobot2-config/      your robots (own git repo)
                                        └─ linorobot2-cockpit/       this repo
```

**The firmware** (`firmware/`) is the low-level motor controller and sensor interface. It
subscribes to `/cmd_vel`, drives the wheels with PID, and publishes `/odom/unfiltered`,
`/imu/data` and, in fake mode, a LiDAR scan. One image per board carries the robot firmware
*and* every diagnostic application (`test_sensors`, `test_motors`, `test_acc`, `i2c_detect`,
`bno085_cal`, `adc_calibrate`); which one boots is a key in a 4 KB `env` flash block, not a
build.

**The robot computer** runs the ROS 2 side: `robot_localization` fuses odometry and IMU into
`/odom`, `robot_state_publisher` broadcasts the TF tree from the URDF generated out of your
config, SLAM Toolbox builds the map, Nav2 navigates. The supervisor wraps all of it in a web UI and a one-click pipeline, and flashes
the board natively with `esptool` / `picotool` — build and flash are always separate steps,
and a browser never flashes.

---

## Your robot's configuration

Each robot is one YAML file in `~/linorobot2-config/` (override with `COCKPIT_CONFIG_DIR`):
base controller, pins, sensors, kinematics, EKF, SLAM and Nav2 parameters, all in one place.
The cockpit's **Config Studio** edits it; you commit it.

```yaml
robot:
  name: rover_pico2
base_controller:
  name: pico2            # also the PlatformIO env
  transport: serial      # or udp4 on an ESP32
  serial_port: /dev/ttyACM0
  baudrate: 921600
  sensors: {imu: FAKE, use_fake_imu: true, use_fake_mag: true, use_fake_wheel: true, use_fake_ld19: true}
  pins: {motor1: {pwm: -1, in_a: -1, in_b: -1}, ...}   # -1 = not connected
kinematics: {base_type: 2wd, wheel_diameter: 0.152, lr_wheels_distance: 0.271, max_rpm: 140, ...}
geometry: {body: {...}, wheel: {...}, laser: {x: 0.12, z: 0.10, frame: laser}, imu: {...}}   # the URDF
ekf: {...}
slam: {...}
nav2: {...}
```

Reference robots ship in `config/reference/` and are copied into your directory on first
start. `secrets.yaml` beside them holds Wi-Fi credentials and addresses and is gitignored, and
so is `generated/`, where the URDF built from each config lands. A config from an older
cockpit is brought up to date by `python3 scripts/migrate_config_schema.py` (it adds a
`geometry:` block derived from the kinematics and removes keys nothing reads).

**A board is a configuration, not a build.** Pins, I2C bus, LiDAR wiring, the battery
divider and pack, whether the LiDAR emulator runs, transport, credentials and addresses all
reach the firmware through the `env` flash block (`scripts/mcu_env.py`), written at flash
time from your config. Every key the config side writes is one the firmware reads, and a test
(`tests/test_env_contract.py`) fails when that stops being true. Editing the config and pressing
Start again rewrites 4 KB; the application image is untouched. The IMU and magnetometer are
detected on the I2C bus at boot, so swapping a sensor needs no edit at all.

To switch fake mode off and describe real hardware, use Config Studio or edit
`base_controller.sensors` and `base_controller.pins`, then press Start again.

### The URDF is generated from your config

`kinematics` drives the firmware: the wheel diameter, the track width and the encoder counts
reach the board in the `env` block, so **odometry is computed from your numbers**. The same
numbers build the robot *description*: at every bringup, `scripts/gen_robot_description.py`
writes `generated/<robot>.urdf` next to your config (gitignored there) and
`robot_state_publisher` broadcasts that tree. Wheel radius and axle positions come from
`kinematics`; everything kinematics does not say is the `geometry:` block:

```yaml
geometry:
  body: {length: 0.228, width: 0.214, height: 0.046, mass: 1.0}   # the base_link box
  wheel: {width: 0.038, mass: 0.3, z: 0.0}                          # tyre width, axle height
  casters: {front: true, rear: true}                                # 2wd only
  laser: {x: 0.12, y: 0.0, z: 0.10, roll: 0, pitch: 0, yaw: 0, frame: laser}
  imu:   {x: 0.0, y: 0.0, z: 0.0, roll: 0, pitch: 0, yaw: 0}       # frame is imu_link
  mesh:  {base: '', wheel: ''}                                      # optional meshes
```

Config Studio edits these on the Base tab ("Body & Sensor Placement"). A config from
before this block gets one derived from its kinematics by `scripts/migrate_config_schema.py`,
written into the file so the numbers are yours to correct. The LiDAR driver stamps `/scan`
with `geometry.laser.frame`, and in fake mode the emulator raycasts from `geometry.laser.x`,
so the scan and the transform always agree. Warnings you get for free: a LiDAR inside the
body box, four mecanum wheels on one axle, a Nav2 `robot_radius` smaller than the body.

---

## Supported boards

| Reference config | MCU | PlatformIO env | Transport | Notes |
|---|---|---|---|---|
| `rover_pico2` | RP2350 | `pico2` | USB serial | **Default.** Bare module runs the whole pipeline; UF2 flashing |
| `pico2w` | RP2350 + CYW43 | `pico2w` | USB serial + Wi-Fi syslog | serial micro-ROS, Wi-Fi for telemetry only |
| `pico`, `picow` | RP2040 | `pico`, `picow` | USB serial | as above, RP2040 |
| `esp32`, `esp32_wifi` | ESP32 | `esp32` | serial or `udp4` | one binary; the transport is an env key. A DevKit's 921 600 baud UART cannot carry a scan, so fake-mode LiDAR needs `udp4` |
| `gendrv`, `gendrv_real` | ESP32 | `esp32` | serial: 921 600 fake, **1.5 Mbaud** with the real sensors | Waveshare General Driver Board: fixed pinout, QMI8658 + AK09918 + INA219 + BMP280, UART LD19 |
| `esp32s3` | ESP32-S3 | `esp32s3` | native USB CDC | |
| `linorobot2` | RP2350 | `pico2` | USB serial | a wired 2WD robot |
| `pico2_mecanum` | RP2350 | `pico2` | USB serial | a wired mecanum 4WD: four two-PWM bridges, four encoders, MPU6050, battery ADC through a divider; builds, **not yet run on hardware** |

Release firmware profiles: `pico2 pico esp32 esp32s3`, each also as `<profile>-lyrical`
**except the classic ESP32**, whose lyrical build does not fit: lyrical's micro-ROS overruns
the ESP32's data RAM by about 14 KB, unchanged across every transport MTU tried. The ROS 2
distro is the one thing that cannot be a run-time setting — the micro-ROS library is linked
in — so each board ships twice where it fits.

The lyrical half of the matrix **has not been run on a robot**. It builds and the images are
published, but every hardware run on record is jazzy. Nav2 itself is no longer the gap: lyrical
publishes the `nav2_*` components but neither the `navigation2` metapackage nor `nav2_bringup`,
so the image installs every published component and builds those two from source. Treat lyrical
as a build target rather than a supported robot until someone drives one.

---

## Firmware

**You normally never build it.** The 1-Click pipeline fetches the release image that matches
your board and the cockpit's version:

```bash
python3 scripts/fetch_prebuilt.py pico2                       # -> firmware/prebuilt/pico2/
python3 scripts/flash_mcu.py --prebuilt pico2 --port /dev/ttyACM0 \
        --params ~/linorobot2-config/rover_pico2_config.yaml
```

Every file is checked against the manifest's sha256 before anything is written. To build
instead — after changing the firmware, or for a board profile that is not released:

```bash
docker compose run --rm pio pio run -d firmware -e pico2   # the PlatformIO build image
# or, with PlatformIO installed locally:
python3 scripts/gen_firmware_header.py --params ~/linorobot2-config/rover_pico2_config.yaml --distro jazzy
pio run -d firmware -e pico2
python3 scripts/flash_mcu.py --env pico2 --port /dev/ttyACM0 --params ~/linorobot2-config/rover_pico2_config.yaml
```

`pio run -t upload` is never used. `pio run` compiles; `flash_mcu.py` writes, with
`micro_ros_agent` stopped and the port verified free, and with the recovery paths a bare
`picotool` or `esptool` call does not have (BOOTSEL entry through the 1200-baud touch, UF2
mass-storage fallback, chip auto-detection).

**Switching application** is an env write, not a flash:

```bash
python3 scripts/flash_mcu.py --env pico2 --port /dev/ttyACM0 --app i2c_detect --env-only
```

**What the board tells you.** Every boot starts with

```text
[fw] linorobot2_hardware app=base distro=jazzy built=2026-09-18 git=1a2b3c4
[i2c] 0x68  imu      MPU6050
```

`scripts/mcu_probe.py` reads that line and compares it with what the cockpit would flash:

| verdict | what a run does |
|---|---|
| `up_to_date` | nothing is written |
| `env_stale` | the 4 KB env block only |
| `stale` / `unknown` | reflash (auto-update on, the default) — or say so and continue (off) |
| `no_firmware` | flash image and env; a board in BOOTSEL has nothing to protect |

Turn **Auto-update firmware when stale** off in the Cockpit for an assembled robot you do not
want touched; **Force firmware update** rewrites even a matching image.

---

## Fake mode

Fake mode is what makes a bare board useful. Under `base_controller.sensors`:

| key | emulates | how |
|---|---|---|
| `use_fake_wheel` | motors and encoders | software kinematics with motor inertia, responds to `/cmd_vel`, resets pose on every agent connection |
| `use_fake_imu` | 6-DoF IMU | synthetic acceleration, angular rate from the simulated heading; no I2C traffic |
| `use_fake_mag` | magnetometer | field vector tracking the simulated heading |
| `use_fake_ld19` | 360° LD19 LiDAR | raycast of a 10 m × 6 m room with an interior wall, on the MCU (`raw_scan`, a UART, or UDP) or on the robot computer (`scripts/fake_laser_node.py`); both raycast from `geometry.laser.x`. On the MCU it is also the env key `fake_ld19`, so a prebuilt image built with the emulator in is silent on a real robot |
| `use_fake_env` | barometer | sea-level pressure and 25 °C |
| `use_fake_sonar` | ultrasonic range | raycast ahead from the same room, drives the firmware's safety stop |

The Nav2 goal test sends the robot behind that interior wall, so a green run proves planning,
not just motion.

---

## Topics

Published by the firmware (rates at the 50 Hz control loop unless noted; the 50 Hz topics are
best-effort, like `SensorDataQoS` — subscribe best-effort, or set `qos: reliable` in the config):

| topic | type | when |
|---|---|---|
| `odom/unfiltered` | `nav_msgs/Odometry` | always |
| `imu/data` or `imu/data_raw` + `imu/mag` | `sensor_msgs/Imu`, `MagneticField` | `imu/mag` only with a magnetometer (`PUBLISH_MAG`) |
| `raw_scan` | `std_msgs/UInt8MultiArray` | fake LD19 on the MCU |
| `battery` (0.5 Hz), `pressure`, `temperature`, `humidity` (1 Hz), `sonar` (10 Hz), `safety_stop` | | when the sensor is fitted or faked; `battery` reads an INA219 or an ADC divider (`pins.battery: {pin, r1, r2, min_v, max_v, capacity_ah}`), percentage only when the pack is described |

Subscribed: `cmd_vel` as `geometry_msgs/Twist` on jazzy and `TwistStamped` on lyrical
(Nav2's own default per distro, compiled in from `kinematics.stamped_cmd_vel: auto`), plus
`cmd_vel_unstamped` for keyboard teleop in stamped mode. The pipeline passes the same
`--distro` to the header generator, the launchers and the goal test so build and run agree.

---

## The web cockpit

- **Dashboard**: 1-Click pipeline with live log, topic-rate gate (`/odom`, `/imu/data`,
  `/scan`), syslog receiver for Wi-Fi boards (UDP 5140), saved-map gallery.
- **Config Studio**: base controller, pins, kinematics, body and sensor placement (the
  URDF), EKF, SLAM and Nav2 editors, YAML import/export, secrets. Pin errors from the
  per-MCU catalogue and geometry warnings show next to the fields.
- **Hardware Tests**: flash any diagnostic application, WebSerial monitor (Chromium browsers;
  monitor only, never flashes).
- **Map Viewer**: `/map`, `/scan` and the robot pose drawn in the browser over rosbridge on
  port 9090, with 2D pose estimate and Nav goal tools — no RViz, nothing to install.
- **Teleop**: virtual gamepad publishing `/cmd_vel`.

---

## Repository layout

```text
config/reference/   shipped robot configs (copied to your config dir, never edited in place)
docs/               firmware.md · flashing.md · ros2-stack.md
firmware/           one PlatformIO project: src/, common/lib/, include/, prebuilt/ (fetched)
launchers/          bringup.launch.py · slam.launch.py · nav2.launch.py
scripts/            one_click_pipeline.py · flash_mcu.py · mcu_probe.py · mcu_env.py
                    gen_firmware_header.py · gen_robot_description.py · pin_catalog.py
                    migrate_config_schema.py · fetch_prebuilt.py · build_prebuilt.py · …
tests/              pytest: env block, header, URDF, access policy, pin catalogue, key contract
web/                backend/ (FastAPI supervisor) · frontend/ (static HTML/CSS/JS)
docker/             Dockerfile (robot runtime) · Dockerfile.pio (build image) · entrypoint.sh
docker-compose.yml  the robot runtime, plus the optional `pio` build service
```

`docs/` holds the design notes and the reasons behind the rules: `firmware.md` for the
board side, `flashing.md` for how images get written, `ros2-stack.md` for the launch trees.

---

## Releases

Datestamped tags (`20260918`; `rc-20260918` for a two-week candidate) publish the firmware
archives as release assets and three images on Docker Hub:
`linorobot2-cockpit-robot:{jazzy,lyrical}` and `linorobot2-cockpit-pio`.
`docker-compose.yml` defaults to those images; `docker compose build` builds them from your
checkout instead, and `COCKPIT_IMAGE` / `COCKPIT_PIO_IMAGE` point it at any other registry.

## Credits and license

Built on [linorobot2](https://github.com/linorobot/linorobot2),
[linorobot2_hardware](https://github.com/linorobot/linorobot2_hardware),
[micro-ROS](https://micro.ros.org/), [Nav2](https://nav2.org/) and
[SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox). Apache License 2.0 — see
[LICENSE](LICENSE).

Patches are welcome. [CONTRIBUTING.md](CONTRIBUTING.md) has the checks to run and the seven
invariants that keep a change from bricking a board; it is one page.
