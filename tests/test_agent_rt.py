"""The micro-ROS agent can run real-time on a small robot computer (scripts/agent_rt.py).

Under SLAM + Nav2 an agent at normal priority on an Arduino UNO Q could not drain the 2 Mbaud
UART in time and the STM32 dropped the session (2026-10-15). bringup.launch.py puts
agent_rt.py in front of the agent when LINO_AGENT_RT_PRIO / LINO_AGENT_CPU are set
(docker-compose.unoq.yml); every step is best-effort, and the agent always starts.
"""
import os
import sys

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "scripts"))
import agent_rt  # noqa: E402


def test_cpu_specs():
    assert agent_rt.parse_cpus("3") == {3}
    assert agent_rt.parse_cpus("2,3") == {2, 3}
    assert agent_rt.parse_cpus("0-2") == {0, 1, 2}
    assert agent_rt.parse_cpus("") == set()


def test_the_serial_device_is_found_in_the_agent_args():
    argv = ["ros2", "run", "micro_ros_agent", "micro_ros_agent", "serial", "--dev", "/dev/ttyHS1", "-b", "2000000"]
    assert agent_rt.serial_dev(argv) == "/dev/ttyHS1"
    assert agent_rt.serial_dev(["ros2", "run", "micro_ros_agent", "micro_ros_agent", "udp4", "--port", "8888"]) == ""


def test_nothing_asked_nothing_done():
    assert agent_rt.apply({}, ["true"]) == []


@pytest.mark.skipif(not hasattr(os, "sched_setaffinity"), reason="no CPU affinity here")
def test_pinning_and_a_refused_priority_are_reported_never_fatal(monkeypatch):
    have = sorted(os.sched_getaffinity(0))
    monkeypatch.setattr(os, "sched_setaffinity", lambda pid, cpus: None)
    def refuse(*a, **k):
        raise PermissionError(1, "Operation not permitted")
    monkeypatch.setattr(os, "sched_setscheduler", refuse)
    notes = agent_rt.apply({"LINO_AGENT_CPU": str(have[-1]), "LINO_AGENT_RT_PRIO": "50"}, ["true"])
    assert any(n.startswith("pinned to CPU") for n in notes)
    assert any("SCHED_FIFO 50 not set" in n and "CAP_SYS_NICE" in n for n in notes)


def test_a_missing_serial_port_is_reported_not_fatal():
    note = agent_rt.low_latency("/dev/does-not-exist-lino")
    assert note.startswith("low latency:") and "not opened" in note


def test_bringup_puts_it_in_front_of_the_agent_only_when_asked():
    src = open(os.path.join(ROOT, "launchers", "bringup.launch.py")).read()
    assert 'cmd=agent_rt_prefix() + ["ros2", "run", "micro_ros_agent", "micro_ros_agent"]' in src
    body = src[src.index("def agent_rt_prefix()"):]
    assert 'os.environ.get("LINO_AGENT_RT_PRIO") or os.environ.get("LINO_AGENT_CPU")' in body


def test_the_unoq_compose_asks_for_it_and_allows_it():
    import yaml
    c = yaml.safe_load(open(os.path.join(ROOT, "docker-compose.unoq.yml")))["services"]["cockpit"]
    assert "SYS_NICE" in c["cap_add"] and c["ulimits"]["rtprio"] >= 50
    env = dict(e.split("=", 1) for e in c["environment"])
    assert env["LINO_AGENT_RT_PRIO"] == "50" and env["LINO_AGENT_CPU"] == "3"
