"""A defunct agent is not a running agent.

The backend is the container's PID 1 and reaps nothing, so a stopped bringup
could leave micro_ros_agent <defunct>. pgrep matched it by name, the status said
an agent was running, and the next Bringup launched with micro_ros:=false: no
agent, no odometry, a dead teleop (browser, jazzy, 2026-10-04). The probe now
reads `ps` with the state column and drops state Z; compose runs an init.
"""
import os
import re
import shutil
import subprocess
import sys
import time

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _live_filter():
    src = open(os.path.join(ROOT, "web", "backend", "core.py")).read()
    return eval(re.search(r'    live = (".*")\n', src).group(1))   # the probe's own text


@pytest.mark.skipif(not shutil.which("ps") or sys.platform != "linux", reason="needs Linux ps")
def test_the_probe_keeps_a_live_process_and_drops_a_zombie(tmp_path):
    zombie, parent = tmp_path / "zzprobe", tmp_path / "zzalive"
    real = os.path.realpath(sys.executable)
    shutil.copy(real, zombie)
    shutil.copy(real, parent)
    os.chmod(zombie, 0o755)
    os.chmod(parent, 0o755)
    # zzprobe exits at once; its parent has exec'd into zzalive and never waits.
    proc = subprocess.Popen(["bash", "-c", f"{zombie} -c pass & exec {parent} -c 'import time; time.sleep(5)'"])
    try:
        pat = "'[z]zprobe|[z]zalive'"
        for _ in range(30):
            raw = subprocess.run(f"ps -eo stat=,args= | grep -E {pat}", shell=True,
                                 capture_output=True, text=True).stdout
            if "<defunct>" in raw:
                break
            time.sleep(0.1)
        assert "<defunct>" in raw, "no zombie to test against"
        seen = subprocess.run(f"{_live_filter()} | grep -E {pat}", shell=True,
                              capture_output=True, text=True).stdout
        assert "zzalive" in seen and "zzprobe" not in seen
    finally:
        proc.kill()
        proc.wait()


def test_both_probes_use_the_filter():
    src = open(os.path.join(ROOT, "web", "backend", "core.py")).read()
    a = src.index("def _refresh_liveness")
    body = src[a:src.index("agent = bringup = False", a)]
    assert "pgrep" not in body.split("probe = (", 1)[1]
    assert body.count("{live} | grep -E") == 2


def test_compose_runs_an_init():
    compose = open(os.path.join(ROOT, "docker-compose.yml")).read()
    svc = compose[compose.index("\n  cockpit:"):]
    svc = svc[:svc.index("\n  ", svc.index("container_name")) + 400]
    assert re.search(r"\n    init: true\n", svc)
