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
- **Nothing about a robot is compiled in.** Pins, sensors, transport, baud, the simulated-sensor
  flags, the LiDAR sink, the sonar, the motor brake mode — all of it comes from a 4 KB `env`
  flash partition and can be changed on a flashed board without a compiler. The firmware's
  remaining `#if`s are about the silicon or the ROS 2 distro's message ABI, nothing else.

---

## Documentation

The **[wiki](https://github.com/hippo5329/linorobot2-cockpit/wiki)** is the place to read
first — installation, the one-click flow tab by tab, wiring, and what to do when something
breaks:

| | |
|---|---|
| [Installation & Docker](https://github.com/hippo5329/linorobot2-cockpit/wiki/Installation-and-Docker) | get it running |
| [The One-Click Pipeline](https://github.com/hippo5329/linorobot2-cockpit/wiki/The-One-Click-Pipeline) | what the button actually does |
| [Sim Mode & the Bare Module](https://github.com/hippo5329/linorobot2-cockpit/wiki/Sim-Mode-and-the-Bare-Module) | the whole stack on an unwired board |
| [Web UI Guide](https://github.com/hippo5329/linorobot2-cockpit/wiki/Web-UI-Guide) | every tab |
| [Pin Matrix & Wiring](https://github.com/hippo5329/linorobot2-cockpit/wiki/Pin-Matrix-and-Wiring) | wire a real board |
| [The Env Partition & Prebuilt Images](https://github.com/hippo5329/linorobot2-cockpit/wiki/The-Env-Partition-and-Prebuilt-Images) | how config reaches the board |
| [Heading & Magnetometer Calibration](https://github.com/hippo5329/linorobot2-cockpit/wiki/Heading-and-Magnetometer-Calibration) | stop the heading drifting |
| [Troubleshooting](https://github.com/hippo5329/linorobot2-cockpit/wiki/Troubleshooting) | when it goes wrong |

`docs/` in this repository is the other half: the design notes and the reasons behind the
rules, written for someone changing the code rather than using it.

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

**No board yet?** Pick **Sim MCU** as the base controller and the same run works with nothing
plugged in: the firmware itself runs on the robot computer, as a micro-ROS client of the agent
over UDP, with its simulated LiDAR going through the LD19 driver, and nothing is built or
flashed. When no board is detected at all, the cockpit warns (*No MCU board detected*) and
switches to the Sim MCU itself; a choice you make yourself is never overridden. Boards such as
the Waveshare GenDrv and the Yahboom YB-EET01 are Reference Designs listed under their MCU.

A few minutes later (about five on a bare board) the map appears in the **Map Viewer** tab
and is saved under `maps/`. What happened: the cockpit stopped anything a previous run left
running, fetched the release firmware for your board, asked the board what it was running,
flashed it only if needed, wrote a fresh 4 KB configuration block, started `micro_ros_agent`,
bringup, SLAM Toolbox and Nav2, verified the topics, drove the eight-manoeuvre drive suite,
drove four round trips to a goal behind the obstacle wall and back, and saved the map. The
stack stays up afterwards so you can keep driving; the next run stops it first.

### Without Docker

```bash
bash scripts/install_deps.sh            # python3-yaml python3-fastapi python3-uvicorn python3-serial
python3 web/backend/main.py             # supervisor on :8000
```

ROS 2 (jazzy or lyrical), `micro_ros_agent`, `slam_toolbox`, `nav2_bringup`,
`robot_localization`, `rosbridge_server`, `robot_state_publisher`,
`joint_state_publisher` and the `ldlidar_stl_ros2` package must be installed and sourced; `esptool` and `picotool` are needed
to flash. **Take `ldlidar_stl_ros2` from [our fork](https://github.com/hippo5329/ldlidar_stl_ros2),
not from ldrobot** — see [Forks we maintain](#forks-we-maintain); with the upstream driver a
non-serial LiDAR publishes no `/scan` at all. The Docker image carries all of that, which is
why it is the recommended path.

### From the command line

```bash
python3 scripts/one_click_pipeline.py --controller pico2 --distro jazzy
python3 scripts/one_click_pipeline.py --controller gendrv --firmware prebuilt --no-nav2
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
| **simulation mode** | the firmware simulating wheels, IMU and a LiDAR room, so a bare board runs the whole pipeline |
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
`/imu/data` and, in simulation mode, a LiDAR scan. One image per board carries the robot firmware
*and* every diagnostic application (`test_sensors`, `test_motors`, `test_acc`, `i2c_detect`,
`adc_calibrate`); which one boots is a key in a 4 KB `env` flash block, not a
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
  name: pico2_mecanum
base_controller:
  name: pico2            # the controller name; the PlatformIO env is picked per board
  transport: serial      # or udp4 on an ESP32
  serial_port: /dev/ttyACM0
  baudrate: 921600
  topic_prefix: ""       # set it, and this robot's topics and frames move under /<prefix>/
  sensors: {imu: SIM, use_sim_imu: true, use_sim_mag: true, use_sim_wheel: true, use_sim_ld19: true}
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
`geometry:` block derived from the kinematics and removes keys nothing reads). Values that would
fail a run silently are not migrated: the 1-Click run refuses the config and lists them (the
pre-rename `use_fake_*` keys or `FAKE` sensors, an EKF on a frame other than `base_link`, gravity
removed twice). Replace such a config: select a `bare_<board>` robot, or copy one from
`config/reference/` over it and re-apply your own pins and kinematics.

**A board is a configuration, not a build.** Pins, I2C bus, LiDAR wiring, the battery
divider and pack, whether the LiDAR emulator runs, transport, credentials and addresses all
reach the firmware through the `env` flash block (`scripts/mcu_env.py`), written at flash
time from your config. Every key the config side writes is one the firmware reads, and a test
(`tests/test_env_contract.py`) fails when that stops being true. Editing the config and pressing
Start again rewrites 4 KB; the application image is untouched. The IMU and magnetometer are
detected on the I2C bus at boot, so swapping a sensor needs no edit at all.

### One wire worth adding

It is optional and cheap, and it removes an error the stack cannot otherwise see — the kind
that never prints and reaches you as a map that will not sit still.

**Fit and calibrate a magnetometer.** A wheeled robot's EKF fuses *velocities*, and
velocities integrate: without an absolute reference the heading error only grows, and it
grows into `map -> odom`, a transform nothing displays. The magnetometer is the only
sensor here that knows which way is north. Calibration is not optional either — an
uncorrected hard iron is worth several degrees on its own (7.4° for the offset the
simulated robot carries). See `docs/ros2-stack.md` and the wiki's
[Heading & Magnetometer Calibration](https://github.com/hippo5329/linorobot2-cockpit/wiki/Heading-and-Magnetometer-Calibration).

The IMU is read by polling on every board, at the control loop's rate. Where a chip has a
FIFO with its own timestamp counter, that carries the sample time with the sample and needs
no extra wiring.

To switch simulation mode off and describe real hardware, use Config Studio or edit
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
with `geometry.laser.frame`, and in simulation mode the emulator raycasts from `geometry.laser.x`,
so the scan and the transform always agree. Warnings you get for free: a LiDAR inside the
body box, four mecanum wheels on one axle, a Nav2 `robot_radius` smaller than the body.

---

## Supported boards

Four microcontrollers. **One image per MCU per ROS 2 distro — not one per robot.**

| MCU | PlatformIO env | Firmware profiles | Transport |
|---|---|---|---|
| **RP2350** | `pico2w` | `pico2-jazzy`, `pico2-lyrical` | serial |
| **RP2040** | `picow` | `pico-jazzy`, `pico-lyrical` | serial |
| **ESP32** | `esp32` | `esp32-jazzy`, `esp32-lyrical` | serial or Wi-Fi |
| **ESP32-S3** | `esp32s3` | `esp32s3-jazzy`, `esp32s3-lyrical` | serial or Wi-Fi |

A Waveshare General Driver board and a bare ESP32 DevKit run the same `esp32-jazzy` image:
the pin matrix, I2C bus, LiDAR pin and baud, micro-ROS transport and credentials all live in
the `env` flash partition, not the binary. A robot is a configuration, not a build: the pin
matrix, `comm_mode`, the sonar pins, the simulated-sensor flags, the motor brake mode and the
forward safety stop are all env keys. The only conditionals in the firmware are about the
silicon (ESP32 vs RP2) or the ROS 2 distro's message ABI.

**The RP2 images are built from the W envs and run on both.** A Pico W is an RP2040 and a
Pico 2 W is an RP2350, so one image serves the W and non-W board alike. The radio is compiled in but never initialised until you enter a Wi-Fi list: with no
SSID in the env and none compiled in, nothing touches the CYW43, so a board that has no radio
at all is unaffected and pays only flash for the capability. Entering the list is what turns
Wi-Fi, syslog and OTA on.

**Flash the row half that matches your ROS 2 distro.** The micro-ROS library and the
`/cmd_vel` type are both fixed at link time — lyrical takes `TwistStamped`, jazzy plain
`Twist` — so the wrong half gives a board that enumerates, publishes odometry, and never
moves.

Notes: an ESP32 at 921 600 baud cannot carry a full LiDAR scan alongside the 50 Hz control
loop. With the scan on `lidar_comm: topic` every topic drops to **33 Hz** and jitter triples;
with the scan off the same board holds **50.0 Hz**. At
1.5 Mbaud, which the GenDrv's CP2102N does, the scan runs at 78 Hz and the control topics
stay at 50 Hz. So use a faster bridge, or `udp4`, or leave the scan to the robot computer.

The ESP32-S3's serial is native USB CDC, so its port comes and goes with the firmware rather
than the cable.

The onboard LED is on by default wherever a board has one — GP25 on the Picos, GPIO 2 on the
ESP32s, GPIO 48 on the S3 — because the blink pattern is the only thing a board tells you
before micro-ROS is up, and a bench board in simulation mode needs it as much as a wired one. A
Pico W's or Pico 2 W's LED is on the CYW43 radio chip, not a GPIO, so their bare robots set
`led: 64`, arduino-pico's number for it; a non-W Pico running the same image maps 64 to GP25.
GPIO 2 on an ESP32 is also a strapping pin, so the
pin checker warns about it; that is correct and harmless here, since an LED to ground pulls
the pin the way the bootloader already wants.

Reference robots ship in `config/reference/` and are copied into your config directory on
first start — see [Your robot's configuration](#your-robots-configuration).

---

## Firmware

**You normally never build it.** The 1-Click pipeline fetches the release image that matches
your board and the cockpit's version:

```bash
python3 scripts/fetch_prebuilt.py pico2-jazzy                 # -> firmware/prebuilt/pico2-jazzy/
python3 scripts/flash_mcu.py --prebuilt pico2-jazzy --port /dev/ttyACM0 \
        --params ~/linorobot2-config/pico2_mecanum_config.yaml
```

Every file is checked against the manifest's sha256 before anything is written. To build
instead — after changing the firmware, or for a board profile that is not released:

```bash
docker compose run --rm pio pio run -d firmware -e pico2w  # the PlatformIO build image
# or, with PlatformIO installed locally:
python3 scripts/gen_firmware_header.py --params ~/linorobot2-config/pico2_mecanum_config.yaml --distro jazzy
pio run -d firmware -e pico2
python3 scripts/flash_mcu.py --env pico2w --port /dev/ttyACM0 --params ~/linorobot2-config/pico2_mecanum_config.yaml
```

`pio run -t upload` is never used. `pio run` compiles; `flash_mcu.py` writes, with
`micro_ros_agent` stopped and the port verified free, and with the recovery paths a bare
`picotool` or `esptool` call does not have (BOOTSEL entry through the 1200-baud touch, UF2
mass-storage fallback, chip auto-detection).

**Switching application** is an env write, not a flash:

```bash
python3 scripts/flash_mcu.py --env pico2w --port /dev/ttyACM0 --app i2c_detect --env-only
```

**What the board tells you.** Every boot starts with

```text
[fw] linorobot2_hardware app=base distro=jazzy built=2026-09-18 git=1a2b3c4 uid=C3AF89DC55525350
[i2c] 0x68  imu      MPU6050
```

The last field names the **board**, and the key says what was identified: `uid=` is the silicon's own
id (RP2350 chip info, ESP32 eFuse MAC), `flashid=` the external flash chip's — all an RP2040 has, and
it moves if the flash is replaced. They are never merged, so two identical boards on one bench are
told apart without either claiming to be something it is not. See `docs/firmware.md`.

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

## Simulation mode

Simulation mode is what makes a bare board useful. Under `base_controller.sensors`:

| key | emulates | how |
|---|---|---|
| `use_sim_wheel` | motors and encoders | software kinematics with motor inertia, responds to `/cmd_vel`, resets pose on every agent connection |
| `use_sim_imu` | 6-DoF IMU | synthetic acceleration, angular rate from the simulated heading; no I2C traffic. Bias, drift, scale error and noise are those of a typical real chip, and the covariance it publishes is derived from that noise rather than declared separately |
| `use_sim_mag` | magnetometer | field vector tracking the simulated heading, sized and noised like a typical real part, with a hard-iron offset so a calibration has something to find. Removed by default, so the simulated robot starts where a real one does after `robot_calibration`; `mag_bias 0,0,0` in the env puts it back to uncalibrated |
| `use_sim_ld19` | 360° LD19 LiDAR | raycast of a 10 m × 6 m room with an interior wall, on the MCU (`raw_scan`, a UART, or UDP) or on the robot computer (`scripts/sim_laser_node.py`); both raycast from `geometry.laser.x`. On the MCU it is also the env key `sim_ld19`, so a prebuilt image built with the emulator in is silent on a real robot |
| `use_sim_env` | barometer | sea-level pressure and 25 °C |
| `use_sim_sonar` | ultrasonic range | raycast ahead from the same room, drives the firmware's safety stop |

**Every one of these is an env key, not a build switch.** The config value is only the
default a board falls back to with a blank env: `sim_wheel`, `sim_ld19`, `sim_env`,
`sim_sonar` and the IMU/mag names can all be changed on a flashed board without a compiler.
The same binary is a bench simulator or a real robot depending on four bytes in the env
partition.

**Where the scan is raycast follows the wiring, not the config.** A board whose emulated
LD19 goes out a real serial bridge is read by the real LiDAR driver, because that is the path
worth testing; a bare module has no such tty, so the room runs on the robot computer instead
(`scripts/sim_laser_node.py`) and `/scan` arrives anyway. Both raycast from
`geometry.laser.x`, so the scan and the transform agree either way — and a module with nothing
soldered to it still reaches SLAM and Nav2.

The Nav2 goal test sends the robot behind that interior wall, so a green run proves planning,
not just motion.

### The simulated drivetrain is a real motor, and the whole stack can run without a board

`use_sim_wheel` is not a ramp toward the commanded speed. It is a brushed DC gear motor:
torque falling linearly from stall to no-load, a gearbox that returns 70–80% of it, constant
gear drag, viscous friction, one battery shared by four wheels whose sag *lags*, the bridge's
fixed and current-proportional losses, and an optional driver current limiter in amps. So a
12 kg robot does not accelerate like a 3.5 kg one, four driven wheels cost more than two, and
a motor rated above its pack never reaches its rated speed.

Every term is a `base_controller.simulation` key and therefore an env key, so sweeping one
costs a 4 KB write rather than a firmware build — and the generated configs write the block
out in full, at the firmware's own defaults, so a config you read describes the whole
simulated robot.

```bash
python3 scripts/drivetrain_report.py --params config/reference/gendrv_config.yaml
```

tells you what those motors can deliver — top speed, acceleration from rest and once the pack
has sagged, the rotation radius with the rule that produced it — and whether the Nav2 limits
in that same config are asking for more than the robot has. It parses the model's constants
out of `sim_wheel.h` rather than restating them, and it prints what `test_acc` *would*
measure by running the model on that tool's own 20 ms sampling. Which means **you no longer
flash a board to find those numbers**; on a simulated-wheel board `test_acc` says so and points
here.

The Nav2 velocity limits and `max_rpm_ratio` are derived from it on save
(`kinematics.auto_nav2_limits: false` turns that off). That matters: a 2.5% rise in one
yaw ceiling took a drivetrain from ten green legs to seven, twice, because the shipped
tuning had no margin.

The **Sim MCU** is the firmware compiled for the robot computer (`firmware/host/`, built into
the robot image): the same `main.cpp`, wheel model and LD19 emulator, booting from the env block
a flash would write, talking micro-ROS over UDP to `micro_ros_agent` and streaming its scan to
the LD19 driver's UDP server. So a run with no board still exercises micro-ROS, the agent and
the LiDAR driver. It does not cover the board's loop timing, the real flash or the serial link,
so it is not a substitute for hardware.

`scripts/sim_base_node.py` is a lighter instrument: the same wheel model as an rclpy node,
publishing straight into DDS. `ros2 launch linorobot2_cockpit bringup.launch.py sim_base:=true`
brings up EKF, SLAM, Nav2 and the goal test with it -- for config questions, sweeps and CI --
and it is what the Sim MCU falls back to on a checkout without the host build.

---

## Topics

Published by the firmware (rates at the 50 Hz control loop unless noted; the 50 Hz topics are
best-effort, like `SensorDataQoS` — subscribe best-effort, or set `qos: reliable` in the config):

| topic | type | when |
|---|---|---|
| `odom/unfiltered` | `nav_msgs/Odometry` | always |
| `imu/data` + `imu/mag` | `sensor_msgs/Imu`, `MagneticField` | `imu/data` always, orientation included -- the board fuses gyro, accel and field itself. `imu/mag` only with a magnetometer (`PUBLISH_MAG`), for calibration; nothing pairs against it |
| `raw_scan` | `std_msgs/UInt8MultiArray` | simulated LD19 on the MCU |
| `battery` (1 Hz), `pressure`, `temperature`, `humidity` (1 Hz), `sonar` (10 Hz), `safety_stop` | | when the sensor is fitted or simd. `sonar` takes its HC-SR04 pins from the env (`sonar_trig`, `sonar_echo`). `safety_stop` brakes the robot, so it is armed only where a real HC-SR04 is wired (`pico2_mecanum` does; add `safety_stop: {enabled: true, range_m: 0.25}` to any config with real sonar pins). It runs in the firmware every control cycle, below ROS, so it still acts when the ROS side is wedged or the link has dropped -- the case nav2_collision_monitor cannot cover because it is the ROS side. Only FORWARD motion is blocked, so the robot can still reverse and turn off the obstacle. A simd range never arms it: the simulated cone is raycast from the emulated room, and a hazard stop must not fire at an imaginary obstacle.; `battery` reads an INA219 or an ADC divider (`pins.battery: {pin, r1, r2, min_v, max_v, capacity_ah}`), percentage only when the pack is described |

**Two robots on one network.** Set `base_controller.topic_prefix: lino1` and every name
above moves under `/lino1/` — on the board, which builds both its topic names *and* the
frame_ids it stamps at run time from the env key, and on the robot computer, where bringup,
SLAM and Nav2 run in the matching namespace with their TF frames prefixed to suit. TF itself
stays on the global `/tf`, so one RViz still sees every robot. Unset is the default and
changes nothing.

Run on two boards — a Pico W and a Pico 2 W on one DDS domain: both robots' topics side by
side at 50 Hz, frames `lino1/…` and `lino2/…` on one `/tf`, and driving one moved it 0.42 m
while the other moved 0.0007 m.

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
  per-MCU catalogue and geometry warnings show next to the fields, and the **wiring chart**
  reads the saved pins back as function → GPIO and GPIO → functions — the sheet you take to
  the bench. It is written beside the URDF at every bringup too, so it exists as a file and
  not only behind a button.
- **Hardware Tests**: switch to any diagnostic application (an env write, not a reflash),
  WebSerial monitor (Chromium browsers; monitor only, never flashes), and the ESP32 ADC
  calibration drawn as a curve so a bad channel is visible rather than inferred.
- **Map Viewer**: `/map`, `/scan` and the robot pose drawn in the browser over rosbridge on
  port 9090, with 2D pose estimate and Nav goal tools — no RViz, nothing to install.
- **Teleop**: virtual gamepad publishing `/cmd_vel`.

---

## Repository layout

```text
config/reference/   shipped robot configs (copied to your config dir, never edited in place)
docs/               firmware.md · flashing.md · ros2-stack.md · docker.md
firmware/           one PlatformIO project: src/, common/lib/, include/, prebuilt/ (fetched)
launchers/          bringup.launch.py · slam.launch.py · nav2.launch.py
scripts/            one_click_pipeline.py · flash_mcu.py · mcu_probe.py · mcu_env.py
                    gen_firmware_header.py · gen_robot_description.py · pin_catalog.py
                    migrate_config_schema.py · fetch_prebuilt.py · build_prebuilt.py · …
tests/              pytest: env block, header, URDF, access policy, pin catalogue, key contract
tests/api/          the cockpit's HTTP API, against a running instance (see below)
web/backend/        FastAPI supervisor: core.py (the app, config and helpers) +
                    routes_*.py, one module per area, registered by importing them
web/frontend/       static HTML/CSS/JS: index.html loads app-*.js in order, no bundler
docker/             Dockerfile (robot runtime) · Dockerfile.pio (build image) · entrypoint.sh
docker-compose.yml  the robot runtime, plus the optional `pio` build service
```

`docs/` holds the design notes and the reasons behind the rules: `firmware.md` for the
board side, `flashing.md` for how images get written and what to do when a board stops
accepting them, `ros2-stack.md` for the launch trees, `docker.md` for the images. The
[wiki](https://github.com/hippo5329/linorobot2-cockpit/wiki) is the user-facing half.

### Running the tests

```bash
python3 -m pytest -q tests          # no hardware, no ROS, no network
```

The API suite is separate because it needs a cockpit to talk to. It skips unless you point it
at one, and it works against anything — a robot on your bench, or a copy started just for the
test:

```bash
# against a running robot
COCKPIT_URL=http://<robot>:8000 COCKPIT_TOKEN=$(cat ~/linorobot2-config/.cockpit_token) \
  python3 -m pytest tests/api -v

# or headless, with no robot at all: the backend needs only these three packages
pip install fastapi uvicorn pyyaml
COCKPIT_CONFIG_DIR=/tmp/cockpit-cfg COCKPIT_PORT=18099 python3 web/backend/main.py &
COCKPIT_URL=http://127.0.0.1:18099 \
  COCKPIT_TOKEN=$(cat /tmp/cockpit-cfg/.cockpit_token) python3 -m pytest tests/api -v
```

It is worth running on its own: the web UI and the one-click pipeline are different code paths,
and the UI is the one most people use.

---

## Releases

Datestamped tags (`20260918`; `rc-20260918` for a two-week candidate) publish **eight firmware
archives** — four MCUs x two ROS 2 distros, `linorobot2-firmware-<board>-<distro>.tar.gz` —
and three images on Docker Hub:
`linorobot2-cockpit-robot:{jazzy,lyrical}` and `linorobot2-cockpit-pio`.
`docker-compose.yml` defaults to those images; `docker compose build` builds them from your
checkout instead, and `COCKPIT_IMAGE` / `COCKPIT_PIO_IMAGE` point it at any other registry.

## Forks we maintain

Three dependencies are our own forks. None is a cosmetic patch — each one is load-bearing, and
building against upstream instead gives you a robot that fails in a way the logs do not
explain.

| fork | why |
|---|---|
| **[hippo5329/micro_ros_platformio](https://github.com/hippo5329/micro_ros_platformio)** | Builds the micro-ROS library the firmware links against. Its per-distro recipe lists the repositories to clone, **by branch**, at build time. Upstream's `lyrical` entry pins four of them to `rolling`, which keeps moving and drifts until the build overflows the ESP32's `dram0_0_seg`. Here, a repository with a branch for our distro is pinned to it, and only one that genuinely has none stays on `rolling`. Upstream is not developing this package. |
| **[hippo5329/ldlidar_stl_ros2](https://github.com/hippo5329/ldlidar_stl_ros2)** | The LD19/LD06 driver. ldrobot's node declares only the serial parameters; every non-serial path here needs the ones the fork adds — `comm_mode`, `server_ip`/`server_port` for a board streaming its scan over UDP, and `raw_scan_topic`/`bins`. With the upstream driver, `comm_mode: udp_server` reaches a node that has never heard of it, falls through to the serial path, dies on `input serial param error` with an empty port, and no `/scan` is ever published. |
| **[hippo5329/arduino-pico](https://github.com/hippo5329/arduino-pico)** (branch `fix/rp2350-bootsel-touch-hang`) | The RP2 core. Fixes an RP2350 hang on the 1200-baud BOOTSEL touch — the mechanism the cockpit uses to put a board into the bootloader without anyone reaching for the button. See [docs/flashing.md](docs/flashing.md). |

The reasons behind each are in [docs/firmware.md](docs/firmware.md) and
[docs/flashing.md](docs/flashing.md); patches are upstreamed where upstream is still taking
them.

## Credits and license

Built on [linorobot2](https://github.com/linorobot/linorobot2),
[linorobot2_hardware](https://github.com/linorobot/linorobot2_hardware),
[micro-ROS](https://micro.ros.org/), [Nav2](https://nav2.org/),
[SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox) and
[ldlidar_stl_ros2](https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2). The LiDAR driver,
the micro-ROS PlatformIO builder and the RP2 core each reach us through a fork — see
[Forks we maintain](#forks-we-maintain). Apache License 2.0 — see [LICENSE](LICENSE).

Patches are welcome. [CONTRIBUTING.md](CONTRIBUTING.md) has the checks to run and the seven
invariants that keep a change from bricking a board; it is one page.
