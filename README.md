# Linorobot2 Cockpit 🚀

> **From a bare microcontroller to SLAM, Nav2 and a saved map — in one click, from a browser.**

[![ROS 2](https://img.shields.io/badge/ROS%202-Jazzy%20%7C%20Lyrical-blue.svg)](https://docs.ros.org/)
[![micro-ROS](https://img.shields.io/badge/micro--ROS-Jazzy%20%7C%20Lyrical-green.svg)](https://micro.ros.org/)
[![Boards](https://img.shields.io/badge/Boards-RP2350%20%7C%20RP2040%20%7C%20ESP32%20%7C%20ESP32--S3-orange.svg)](#supported-boards)
[![License](https://img.shields.io/badge/License-Apache%202.0-lightgrey.svg)](LICENSE)

Linorobot2 Cockpit is a web supervisor, prebuilt micro-ROS firmware, and a complete ROS 2 navigation stack for [linorobot2](https://github.com/linorobot/linorobot2)-style mobile robots. It runs on the **robot computer** (Raspberry Pi 5, Jetson, or Linux PC) connected to your microcontroller, operated from any browser on your network.

- **Zero-wiring first run.** A bare microcontroller with nothing soldered to it publishes simulated odometry, IMU, and a virtual room's LiDAR scan so the whole navigation pipeline runs before you build hardware.
- **Prebuilt firmware & images.** Releases ship flashable binaries for every supported board and prebuilt Docker images for the ROS 2 stack. Nothing needs to be compiled locally.
- **Single configuration file per robot.** Pins, sensors, kinematics, and navigation limits live in one YAML file in `~/linorobot2-config/`, kept in your own git repository.
- **Full web interface.** Live SLAM map viewer, virtual joystick teleoperation, configuration editor with real-time pin conflict validation, and hardware diagnostic monitors in the browser.

---

## Documentation

* 📖 **[User Wiki (Getting Started & User Guides)](https://github.com/hippo5329/linorobot2-cockpit/wiki)**  
  Installation, the 1-Click pipeline, Web UI guide, pin wiring charts, magnetometer calibration, ESP32 ADC tuning, multi-robot setups, and troubleshooting.

* 🛠️ **[Technical Details & Developer Reference](https://github.com/hippo5329/linorobot2-cockpit/wiki/Technical-Details)**  
  System architecture, advanced CLI pipelines, 4 KB `env` partition specification, firmware compilation, DC motor simulation physics, micro-ROS topics catalog, and developer test suites.

---

## Quick Start

You need: a Linux computer with USB (the **robot computer**) and a supported microcontroller board on a USB cable.

**Plug the board in first**, then run:

```bash
sudo apt install -y git
git clone https://github.com/hippo5329/linorobot2-cockpit.git
cd linorobot2-cockpit
bash scripts/install_docker.sh          # installs rootless Docker Engine if needed
docker compose up -d
docker compose logs | grep token       # prints your authentication URL
```

1. Open the printed URL (**`http://<robot-computer>:8000/?token=…`**) in any browser. The browser remembers the token for future visits.
2. Click **Start 1-Click**.

The cockpit automatically detects your connected board (or uses **Sim MCU** if no board is plugged in), flashes matching release firmware if needed, writes your configuration, starts `micro_ros_agent`, boots EKF, SLAM Toolbox, and Nav2, performs qualification drive manoeuvres, and displays the generated map in the **Map Viewer** tab.

---

## Supported Boards

| Board | MCU Chip | Released Profiles | Default Transport |
|---|---|---|---|
| **Raspberry Pi Pico 2 / Pico 2 W** | RP2350 | `pico2-jazzy`, `pico2-lyrical` | Serial (USB CDC) |
| **Raspberry Pi Pico / Pico W** | RP2040 | `pico-jazzy`, `pico-lyrical` | Serial (USB CDC) |
| **ESP32 DevKit** | ESP32 | `esp32-jazzy`, `esp32-lyrical` | Serial or Wi-Fi (UDP) |
| **ESP32-S3** | ESP32-S3 | `esp32s3-jazzy`, `esp32s3-lyrical` | Serial (native USB CDC) |
| **Waveshare General Driver** | ESP32 | `esp32-jazzy`, `esp32-lyrical` | Serial (1.5 Mbaud) or Wi-Fi |
| **Sim MCU** | Host CPU | Built into robot image | UDP4 micro-ROS client |

Both **ROS 2 Jazzy** and **ROS 2 Lyrical** distributions are fully supported.

---

## Essential CLI Workflows

For automation, CI, or headless operation, all functions can be run from the command line:

```bash
# Run headless 1-Click pipeline on RP2350 with ROS 2 Jazzy
python3 scripts/one_click_pipeline.py --controller pico2 --distro jazzy

# Autonomous frontier exploration in multi-room virtual environments
python3 scripts/one_click_pipeline.py --controller pico2 --explore

# Navigate an existing pre-saved map
python3 scripts/one_click_pipeline.py --controller pico2 --map ~/maps/room.yaml

# Run unit tests (no hardware or ROS 2 dependencies needed)
python3 -m pytest -q tests
```

See the [[Technical Details Wiki|https://github.com/hippo5329/linorobot2-cockpit/wiki/Technical-Details]] for detailed flag documentation, firmware flashing commands, and REST API test runners.

---

## Repository Structure

```text
config/reference/   shipped robot reference configurations
docker/             Dockerfile definitions for robot runtime and build services
firmware/           PlatformIO firmware: src/, common/lib/, host/ (Sim MCU)
launchers/          ROS 2 launch files: bringup.launch.py, slam.launch.py, nav2.launch.py
scripts/            one_click_pipeline.py, flash_mcu.py, mcu_probe.py, mcu_env.py
tests/              pytest unit test suite and API integration tests
web/backend/        FastAPI web supervisor: REST endpoints and background runners
web/frontend/       Dashboard, Config Studio, Map Viewer, and Teleop web UI
docker-compose.yml  Production multi-container orchestration
```

---

## License & Contributing

Linorobot2 Cockpit is licensed under the [Apache License 2.0](LICENSE).  
For contribution guidelines and hardware safety invariants, see [CONTRIBUTING.md](CONTRIBUTING.md).
