"""The page's micro-ROS agent follows the robot's transport (web/frontend/app-agent-bringup.js).

The hidden cfg-agent-* fields are filled only from the serial port, so every agent the page
started was a serial one: on a Wi-Fi robot (base_controller.transport udp4) the Bringup and
the bring-up LiDAR check waited for a board that was calling a UDP agent nobody ran.
"""
import json
import os
import re
import shutil
import subprocess

import pytest

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
JS = os.path.join(REPO_ROOT, "web", "frontend", "app-agent-bringup.js")


def _helpers() -> str:
    src = open(JS).read()
    out = []
    for name in ("agentTransport", "agentUdpPort"):
        m = re.search(r"^function %s\(\) \{.*?^\}" % name, src, re.S | re.M)
        assert m, name
        out.append(m.group(0))
    return "\n".join(out)


def _run(config, field_transport=""):
    script = (
        "const fields = {'cfg-agent-transport': {value: %s}, 'cfg-agent-port': {value: ''}};\n"
        "const document = {getElementById: (id) => fields[id] || null};\n"
        "const state = {config: %s};\n%s\n"
        "console.log(JSON.stringify([agentTransport(), String(agentUdpPort())]));\n"
    ) % (json.dumps(field_transport), json.dumps(config), _helpers())
    r = subprocess.run(["node", "-e", script], capture_output=True, text=True, timeout=20)
    assert r.returncode == 0, r.stderr
    return json.loads(r.stdout)


@pytest.mark.skipif(shutil.which("node") is None, reason="node is not installed")
def test_a_wifi_robot_gets_a_udp4_agent_on_its_port():
    assert _run({"base_controller": {"transport": "udp4", "agent_port": 8890}}, "serial") == ["udp4", "8890"]
    assert _run({"base_controller": {"transport": "wifi"}}) == ["udp4", "8888"]


@pytest.mark.skipif(shutil.which("node") is None, reason="node is not installed")
def test_a_serial_robot_keeps_a_serial_agent():
    assert _run({"base_controller": {"transport": "serial"}}, "udp4")[0] == "serial"


@pytest.mark.skipif(shutil.which("node") is None, reason="node is not installed")
def test_no_transport_in_the_config_falls_back_to_the_field():
    assert _run({"base_controller": {}}, "udp4")[0] == "udp4"
    assert _run({}, "")[0] == "serial"
