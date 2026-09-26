# Linorobot2 Cockpit 🚀

> **From a bare microcontroller to SLAM, Nav2 and a saved map — in one click, from a browser.**

[![ROS 2](https://img.shields.io/badge/ROS%202-Jazzy%20%7C%20Lyrical-blue.svg)](https://docs.ros.org/)
[![micro-ROS](https://img.shields.io/badge/micro--ROS-Jazzy%20%7C%20Lyrical-green.svg)](https://micro.ros.org/)
[![Boards](https://img.shields.io/badge/Boards-RP2350%20%7C%20RP2040%20%7C%20ESP32%20%7C%20ESP32--S3-orange.svg)](#supported-boards)
[![License](https://img.shields.io/badge/License-Apache%202.0-lightgrey.svg)](LICENSE)

Linorobot2 Cockpit is the first robotics platform to combine an **end-to-end 1-click web workflow** with a **realistic built-in firmware physics simulator** for [linorobot2](https://github.com/linorobot/linorobot2)-style mobile robots:

- **First 1-Click End-to-End Workflow**: Takes a mobile robot from bare hardware to a saved SLAM map in one click. It automatically detects MCU silicon, flashes prebuilt micro-ROS firmware, writes configuration into a 4 KB flash runtime `env` partition, boots `micro_ros_agent`, validates topic rates, executes an 8-manoeuvre drive qualification suite, brings up SLAM Toolbox and Nav2, and saves the resulting map—completely from a browser without terminal commands or RViz setup.
- **First Realistic Built-In Firmware Physics Simulator**: Traditional robotics requires heavy 3D simulators (Gazebo, Isaac Sim) on dedicated desktop workstations. Cockpit embeds true electro-mechanical DC motor physics (torque-speed curves, gearbox efficiency, gear drag, battery voltage sag with lag, driver resistance, stall current limits, robot mass) and sensor models (calibrated noise, magnetometer hard-iron offset, ultrasonic safety cone, virtual room LiDAR raycasting) directly into the microcontroller firmware (and built-in Sim MCU). A bare $5 development board runs real micro-ROS and qualifies the entire navigation stack before you solder a single wire.
- **Zero-wiring first run.** A bare microcontroller with nothing soldered to it publishes simulated odometry, IMU, and a virtual room's LiDAR scan so the whole navigation pipeline runs before you build hardware.
- **Prebuilt firmware & images.** Releases ship flashable binaries for every supported board and prebuilt Docker images for the ROS 2 stack. Nothing needs to be compiled locally.
- **Single configuration file per robot.** Pins, sensors, kinematics, and navigation limits live in one YAML file in `~/linorobot2-config/`, kept in your own git repository.
- **Full web interface.** Live SLAM map viewer, virtual joystick teleoperation, configuration editor with real-time pin conflict validation, and hardware diagnostic monitors in the browser.

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

For a step-by-step beginner walkthrough and tour of the web interface (Dashboard, Config Studio, Map Viewer, Teleop, and Navigation), follow the **[Web UI Quick Start Walkthrough](https://github.com/hippo5329/linorobot2-cockpit/wiki/Web-UI-Guide#quick-start-web-ui-walkthrough-beginners)** in the wiki.

---

## Supported Boards

| Board | MCU Chip | Released Profiles | Default Transport |
|---|---|---|---|
| **Raspberry Pi Pico 2 / Pico 2 W** | RP2350 | `pico2-jazzy`, `pico2-lyrical` | Serial (USB CDC) |
| **Raspberry Pi Pico / Pico W** | RP2040 | `pico-jazzy`, `pico-lyrical` | Serial (USB CDC) |
| **ESP32 DevKit** | ESP32 | `esp32-jazzy`, `esp32-lyrical` | Serial or Wi-Fi (UDP) |
| **ESP32-S3** | ESP32-S3 | `esp32s3-jazzy`, `esp32s3-lyrical` | Serial (native USB CDC) |
| **Waveshare General Driver** | ESP32 | `esp32-jazzy`, `esp32-lyrical` | Serial (1.5 Mbaud) or Wi-Fi |
| **Yahboom ESP32-S3 (YB-EET01)** | ESP32-S3 | `esp32s3-jazzy`, `esp32s3-lyrical` | Serial or Wi-Fi |
| **Sim MCU** | Host CPU | Built into robot image | UDP4 micro-ROS client |

---

## Supported ROS 2 Distributions

Both **ROS 2 Jazzy** and **ROS 2 Lyrical** distributions are fully supported across all firmware profiles and Docker runtime images.

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

## Documentation

* 📖 **[User Wiki (Getting Started & User Guides)](https://github.com/hippo5329/linorobot2-cockpit/wiki)**  
  Installation, the 1-Click pipeline, Web UI guide, pin wiring charts, magnetometer calibration, ESP32 ADC tuning, multi-robot setups, and troubleshooting.

* 🛠️ **[Technical Details & Developer Reference](https://github.com/hippo5329/linorobot2-cockpit/wiki/Technical-Details)**  
  System architecture, advanced CLI pipelines, 4 KB `env` partition specification, firmware compilation, DC motor simulation physics, micro-ROS topics catalog, and developer test suites.

---

## License & Contributing

Linorobot2 Cockpit is licensed under the [Apache License 2.0](LICENSE).  
For contribution guidelines and hardware safety invariants, see [CONTRIBUTING.md](CONTRIBUTING.md).
