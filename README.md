# Linorobot2 Cockpit

> **From a bare microcontroller to SLAM, Nav2 and a saved map — in one click, from a browser.**

[![ROS 2](https://img.shields.io/badge/ROS%202-Jazzy%20%7C%20Lyrical-blue.svg)](https://docs.ros.org/)
[![License](https://img.shields.io/badge/License-Apache%202.0-lightgrey.svg)](LICENSE)

Linorobot2 Cockpit is a web supervisor, prebuilt micro-ROS firmware and a ROS 2 navigation stack for
[linorobot2](https://github.com/linorobot/linorobot2)-style mobile robots. It runs in Docker on the
**robot computer**, the Linux machine the microcontroller is plugged into, and is driven from any
browser on the network.

- **One click** detects and flashes the board, writes the robot's settings, starts micro-ROS, and runs
  SLAM and Nav2 through to a saved map.
- **Simulation first:** a bare board with nothing wired, or no board at all, runs the real firmware with
  simulated motors, IMU and LiDAR, so the whole stack works before you build anything.
- **No compiling:** releases ship firmware for each supported microcontroller and multi-arch Docker images.
- **One YAML file per robot**, kept in your own git repository in `~/linorobot2-config/`.

## Quick start

Plug the board in, then on the robot computer:

```bash
git clone https://github.com/hippo5329/linorobot2-cockpit.git
cd linorobot2-cockpit
bash scripts/install_docker.sh     # only if neither Docker nor Podman is installed
docker compose up -d
docker compose logs | grep token   # open the printed http://<robot-computer>:8000/?token=... link
```

Then click **Start 1-Click**. With no board plugged in, the cockpit uses the built-in **Sim MCU**.

## Documentation

Everything else (supported boards, reference designs, wiring, calibration, the pipeline and
troubleshooting) is in the **[wiki](https://github.com/hippo5329/linorobot2-cockpit/wiki)**. Start with:

- [Installation & Docker](https://github.com/hippo5329/linorobot2-cockpit/wiki/Installation-and-Docker)
- [Web UI walkthrough](https://github.com/hippo5329/linorobot2-cockpit/wiki/Web-UI-Guide#quick-start-web-ui-walkthrough-beginners)
- [Troubleshooting](https://github.com/hippo5329/linorobot2-cockpit/wiki/Troubleshooting)

Developer documentation lives in [`docs/`](docs/).

## License & contributing

Apache License 2.0, see [LICENSE](LICENSE). Contribution guidelines and hardware safety invariants are in
[CONTRIBUTING.md](CONTRIBUTING.md).
