"""The cockpit's containers set a sane open-file limit. Docker on Debian 13 (the UNO Q) hands a
container 1073741816, and LTTng-UST, which every ROS 2 jazzy process loads through tracetools,
sizes its fd tracker by it: 128 MB in each process, and the MC200's Nav2 could not start in what
was left (2026-10-16). At 65536 it is 1 MB."""
import os

import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def test_every_ros_service_caps_open_files():
    services = yaml.safe_load(open(os.path.join(ROOT, "docker-compose.yml")))["services"]
    for name in ("cockpit", "rviz"):
        nofile = services[name]["ulimits"]["nofile"]
        assert nofile["soft"] <= 65536 and nofile["hard"] <= 524288, name
