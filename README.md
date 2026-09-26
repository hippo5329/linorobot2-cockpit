# Linorobot2 Cockpit 🚀

> **From a bare microcontroller to SLAM, Nav2 and a saved map — in one click, from a browser.**

[![ROS 2](https://img.shields.io/badge/ROS%202-Jazzy%20%7C%20Lyrical-blue.svg)](https://docs.ros.org/)
[![micro-ROS](https://img.shields.io/badge/micro--ROS-Jazzy%20%7C%20Lyrical-green.svg)](https://micro.ros.org/)
[![Microcontrollers](https://img.shields.io/badge/MCUs-RP2350%20%7C%20RP2040%20%7C%20ESP32%20%7C%20ESP32--S3-orange.svg)](#supported-microcontrollers)
[![License](https://img.shields.io/badge/License-Apache%202.0-lightgrey.svg)](LICENSE)

Linorobot2 Cockpit is the first robotics platform to combine an **end-to-end 1-click web workflow** with a **realistic built-in firmware physics simulator** for [linorobot2](https://github.com/linorobot/linorobot2)-style mobile robots:

- **1-Click Autonomy Workflow**: Takes a mobile robot from bare hardware to a saved SLAM map in one click. It automatically detects MCU silicon, flashes prebuilt micro-ROS firmware, writes configuration into a 4 KB flash runtime `env` partition, boots `micro_ros_agent`, validates topic rates, executes a drive qualification suite, brings up SLAM Toolbox and Nav2, and saves the resulting map—completely from a browser without terminal commands or desktop RViz.
- **Realistic Firmware Physics Simulator**: Embeds true electro-mechanical DC motor physics (torque-speed curves, gearbox efficiency, gear drag, battery voltage sag with lag, driver resistance, stall current limits, robot mass) and sensor models (calibrated noise, magnetometer hard-iron offset, ultrasonic safety cone, virtual room LiDAR raycasting) directly into microcontroller firmware and the host-based Sim MCU.
- **Zero-Wiring First Run**: Run and qualify your entire autonomy pipeline before soldering a single wire. A bare board publishes simulated odometry, IMU, and virtual room LiDAR scans over real micro-ROS.
- **Prebuilt Firmware & Images**: Releases ship flashable binaries for supported microcontroller families and multi-arch Docker images for the ROS 2 stack. Nothing needs to be compiled locally.
- **Single Configuration File**: Pins, sensors, kinematics, and navigation limits live in one YAML file in `~/linorobot2-config/`, kept in your own git repository.
- **Full Web Supervisor**: Live SLAM map viewer, virtual joystick teleoperation, configuration editor with real-time pin conflict validation, and hardware diagnostic monitors in the browser.

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

## Supported Microcontrollers

| Microcontroller | Architecture | Default Transport | Recommended For |
|---|---|---|---|
| **Raspberry Pi Pico 2 / Pico 2 W** | RP2350 (ARM Cortex-M33) | Serial (USB CDC) | High performance dual-core, hardware FPU, ultra-low jitter |
| **Raspberry Pi Pico / Pico W** | RP2040 (ARM Cortex-M0+) | Serial (USB CDC) | Solid, cost-effective dual-core micro-ROS controller |
| **ESP32 DevKit** | ESP32 (Xtensa Dual-Core) | Serial / Wi-Fi (UDP) | Native Wi-Fi micro-ROS transport; battery ADC monitoring |
| **ESP32-S3** | ESP32-S3 (Xtensa Dual-Core) | Native USB CDC / Wi-Fi | High-speed native USB CDC, vector extensions |
| **Sim MCU (Virtual)** | Host x86_64 / ARM64 | UDP (`udp4`) | Pure software simulation without hardware or physical wiring |

*For GPIO allocation guidelines, board-specific pinout diagrams, and wiring restrictions, see the [Pin Matrix & Wiring Guide](https://github.com/hippo5329/linorobot2-cockpit/wiki/Pin-Matrix-and-Wiring).*

---

## Reference Designs

Cockpit includes pre-tuned, production-tested reference configurations for popular commercial integrated robot boards and custom DIY reference chassis (differential drive, skid steer, and omnidirectional 4WD mecanum):

- **Integrated Commercial Boards**: Plug-and-play presets for all-in-one controller boards with built-in motor drivers, IMUs, and serial LiDAR headers.
- **Reference Custom Builds**: Production-tested reference platforms combining microcontrollers with standalone dual H-bridge motor drivers, high-rate 9-DOF IMUs, sonar safety stops, and battery dividers.

*For complete bills of materials, schematics, motor driver scheme comparisons, and wiring pinouts, see the [Reference Designs Guide](https://github.com/hippo5329/linorobot2-cockpit/wiki/Reference-Designs).*

---

## Virtual Simulation (Built-In Simulator)

Test your entire autonomy stack (odometry, EKF, SLAM, Nav2) with zero physical hardware:

- **Host Sim MCU**: A pure-software virtual microcontroller running directly in the container. Communicates over micro-ROS via UDP (`udp4`) and publishes odometry, IMU, sonar, and virtual room LiDAR raycasts.
- **Bare-Module Firmware Simulation**: Real microcontroller boards can be flashed with simulation mode enabled to validate physical MCU communication before wiring motors or sensors.
- **Realistic Drivetrain Physics**: Built-in electro-mechanical simulation includes motor torque curves, battery voltage sag, Coulomb friction, and encoder quantization noise.

*For detailed setup, world models, and simulation parameters, see the [Built-In Simulator Guide](https://github.com/hippo5329/linorobot2-cockpit/wiki/Built-in-Simulator).*

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

Explore the full documentation on the **[Cockpit Wiki](https://github.com/hippo5329/linorobot2-cockpit/wiki)**:

* 📖 **[Wiki Home](https://github.com/hippo5329/linorobot2-cockpit/wiki)** — Overview, system concept, and getting started roadmap
* 🚀 **[Installation & Docker](https://github.com/hippo5329/linorobot2-cockpit/wiki/Installation-and-Docker)** — Host prerequisites, container deployment, and rootless setup
* 🖥️ **[Web UI Guide](https://github.com/hippo5329/linorobot2-cockpit/wiki/Web-UI-Guide)** — Tour of Dashboard, MCU & Sim configuration, Base settings, Map Viewer, and Teleop
* 🔄 **[The 1-Click Pipeline](https://github.com/hippo5329/linorobot2-cockpit/wiki/The-One-Click-Pipeline)** — Automated multi-stage bringup, qualification drive suite, and CLI options
* 🕹️ **[Built-in Simulator](https://github.com/hippo5329/linorobot2-cockpit/wiki/Built-in-Simulator)** — Sim MCU, bare-module simulation mode, and virtual room environments
* 🏎️ **[Reference Designs](https://github.com/hippo5329/linorobot2-cockpit/wiki/Reference-Designs)** — Pre-tuned builds, bills of materials, and motor driver scheme comparisons
* 🔌 **[Pin Matrix & Wiring](https://github.com/hippo5329/linorobot2-cockpit/wiki/Pin-Matrix-and-Wiring)** — MCU pin assignments, hardware conflicts, and safety invariants
* 🗺️ **[Sensors, LiDAR & Maps](https://github.com/hippo5329/linorobot2-cockpit/wiki/Sensors-and-Maps)** — LiDAR configuration, footprint masking, depth cameras, and Nav2 navigation
* 🧮 **[Calibration Guides](https://github.com/hippo5329/linorobot2-cockpit/wiki/Heading-and-Magnetometer-Calibration)** — Magnetometer hard-iron offset tuning and ESP32 battery ADC calibration
* 🛠️ **[Technical Details](https://github.com/hippo5329/linorobot2-cockpit/wiki/Technical-Details)** — Architecture, 4 KB flash env specification, micro-ROS topics, and test suites
* 🔍 **[Troubleshooting & Error Index](https://github.com/hippo5329/linorobot2-cockpit/wiki/Troubleshooting)** — Searchable catalog of common hardware and software error resolutions

---

## License & Contributing

Linorobot2 Cockpit is licensed under the [Apache License 2.0](LICENSE).  
For contribution guidelines and hardware safety invariants, see [CONTRIBUTING.md](CONTRIBUTING.md).
