#!/usr/bin/env python3
# ==============================================================================
# Linorobot2 Cockpit — Process Runners, Gamepad & Port Safety Manager
#
# Adheres strictly to AGENTS.md:
# 1. Targeted PID signaling only (kill / killpg). NEVER use pkill or killall.
# 2. Port immunity for 8000, 5173, and 9090.
# ==============================================================================

import collections
import glob
import json
import os
import queue
import re
import shlex
import shutil
import signal
import subprocess
import sys
import threading
import time
from typing import Any, Callable, Dict, List, Optional, Tuple

IMMUNE_PORTS = {8000, 5173, 9090}
BRINGUP_HEALTH_TOPICS = [
    ("odom", "/odom", "EKF fused odometry", 10.0),
    ("odom_raw", "/odom/unfiltered", "micro-ROS raw odometry", 10.0),
    ("imu", "/imu/data", "IMU orientation & angular velocity", 20.0),
    ("scan", "/scan", "Laser scan", 5.0),
]
BRINGUP_TF_CHAIN = [
    ("odom", "base_link"),
    ("base_link", "laser"),
]


def ros_setup_shell(distro: str = "auto") -> str:
    """Shell prefix that sources whichever ROS 2 is installed here.

    Explicit distro first, then whatever /opt/ros holds, then the workspace
    built from this repo. Every ROS command the supervisor runs goes through
    this, so there is exactly one place that knows where ROS lives.
    """
    repo_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    ros = (f"if [ -n \"{distro}\" ] && [ \"{distro}\" != \"auto\" ] && [ -f /opt/ros/{distro}/setup.bash ]; "
           f"then source /opt/ros/{distro}/setup.bash; "
           "elif [ -f /opt/ros/lyrical/setup.bash ]; then source /opt/ros/lyrical/setup.bash; "
           "elif [ -f /opt/ros/jazzy/setup.bash ]; then source /opt/ros/jazzy/setup.bash; "
           "elif [ -f /opt/ros/rolling/setup.bash ]; then source /opt/ros/rolling/setup.bash; fi")
    uros = ("if [ -f /uros_ws/install/setup.bash ]; then source /uros_ws/install/setup.bash; "
            "elif [ -f /opt/uros_ws/install/setup.bash ]; then source /opt/uros_ws/install/setup.bash; "
            "elif [ -f $HOME/uros_ws/install/setup.bash ]; then source $HOME/uros_ws/install/setup.bash; fi")
    # See one_click_pipeline._ros_env: Nav2 from source on distros with no
    # nav2_bringup binary.
    nav2 = ("if [ -f /opt/nav2_ws/install/setup.bash ]; then "
            "source /opt/nav2_ws/install/setup.bash; fi")
    ws = (f"if [ -f /opt/lino_ws/setup.bash ]; then source /opt/lino_ws/setup.bash; "
          f"elif [ -f {repo_root}/install/setup.bash ]; then source {repo_root}/install/setup.bash; "
          f"elif [ -f {repo_root}/../../install/setup.bash ]; then source {repo_root}/../../install/setup.bash; "
          "elif [ -f $HOME/cockpit_ws/install/setup.bash ]; then source $HOME/cockpit_ws/install/setup.bash; fi")
    return f"{ros}; {uros}; {nav2}; {ws}; true"


class ProcessRunner:
    """Manages long-running streaming shell processes for a named slot."""

    def __init__(self, name: str):
        self.name = name
        self.process: Optional[subprocess.Popen] = None
        self.lock: threading.Lock = threading.Lock()
        self.history: collections.deque = collections.deque(maxlen=1000)
        self.subscribers: List[queue.Queue] = []
        self.command_str: str = ""

    def subscribe(self, q: queue.Queue):
        with self.lock:
            self.subscribers.append(q)

    def unsubscribe(self, q: queue.Queue):
        with self.lock:
            if q in self.subscribers:
                self.subscribers.remove(q)

    def _broadcast(self, event_type: str, payload: Dict[str, Any]):
        with self.lock:
            if event_type == "output":
                self.history.append(payload.get("line", ""))
            subs = list(self.subscribers)
        for q in subs:
            try:
                q.put_nowait((event_type, payload))
            except Exception:
                pass

    def is_busy(self) -> bool:
        with self.lock:
            return self.process is not None and self.process.poll() is None

    def get_history(self) -> List[str]:
        with self.lock:
            return list(self.history)

    def start_streaming(
        self,
        command: str,
        cwd: str,
        send_event: Optional[Callable[[str, Dict[str, Any]], None]] = None,
    ) -> bool:
        with self.lock:
            if self.process is not None and self.process.poll() is None:
                return False
            self.history.clear()
            self.command_str = command
            env = os.environ.copy()
            env["PYTHONUNBUFFERED"] = "1"

            full_cmd = ["bash", "-lc", command]

            try:
                self.process = subprocess.Popen(
                    full_cmd,
                    cwd=cwd,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    text=True,
                    env=env,
                    preexec_fn=os.setsid,
                )
            except Exception as e:
                if send_event:
                    send_event("output", {"line": f"[runner error] failed to start: {e}"})
                return False

        proc = self.process
        try:
            for line in iter(proc.stdout.readline, ""):
                if not line:
                    break
                stripped = line.rstrip("\n")
                if send_event:
                    send_event("output", {"line": stripped})
                self._broadcast("output", {"line": stripped})
        finally:
            proc.wait()
            exit_code = proc.returncode
            with self.lock:
                if self.process is proc:
                    self.process = None
            if send_event:
                send_event("done", {"exit_code": exit_code})
            self._broadcast("done", {"exit_code": exit_code})
        return True

    def kill(self) -> bool:
        with self.lock:
            proc = self.process
            if proc is None or proc.poll() is not None:
                return False
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
                proc.wait(timeout=3)
            except Exception:
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                except Exception:
                    pass
            self.process = None
            return True


class GamepadRunner:
    """Virtual gamepad cmd_vel publisher."""

    def __init__(self, repo_root: str):
        self.repo_root = repo_root
        self.process: Optional[subprocess.Popen] = None
        self.lock: threading.Lock = threading.Lock()
        self.target: Tuple[float, float, float] = (0.0, 0.0, 0.0)

    def is_running(self) -> bool:
        with self.lock:
            return self.process is not None and self.process.poll() is None

    def start(self, topic: str = "/cmd_vel") -> bool:
        with self.lock:
            if self.process is not None and self.process.poll() is None:
                return True
            script = os.path.join(self.repo_root, "scripts", "gamepad_publisher.py")
            if not os.path.isfile(script):
                return False
            cmd = f"python3 {shlex.quote(script)} --topic {shlex.quote(topic)}"
            env = os.environ.copy()
            env["PYTHONUNBUFFERED"] = "1"
            try:
                full_cmd = ["bash", "-lc", f"{ros_setup_shell()} && {cmd}"]

                self.process = subprocess.Popen(
                    full_cmd,
                    stdin=subprocess.PIPE,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                    text=True,
                    env=env,
                    preexec_fn=os.setsid,
                )
                return True
            except Exception:
                return False

    def send(self, linear_x: float, linear_y: float, angular_z: float) -> bool:
        with self.lock:
            proc = self.process
            if proc is None or proc.poll() is not None:
                return False
            try:
                proc.stdin.write(f"{linear_x} {linear_y} {angular_z}\n")
                proc.stdin.flush()
                self.target = (linear_x, linear_y, angular_z)
                return True
            except (BrokenPipeError, ValueError):
                self.process = None
                return False

    def kill(self) -> bool:
        with self.lock:
            proc = self.process
            self.process = None
        if proc is None or proc.poll() is not None:
            return False
        try:
            try:
                proc.stdin.close()
            except Exception:
                pass
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
            proc.wait(timeout=2)
        except Exception:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            except Exception:
                pass
        return True


def check_agent_port_status(
    port: str = "/dev/ttyUSB0",
    mode: str = "serial",
    udp_port: int = 8888,
    host: Optional[str] = None,
    user: Optional[str] = None,
) -> Dict[str, Any]:
    res: Dict[str, Any] = {
        "status": "ok",
        "in_use": False,
        "mode": mode,
        "target": port if mode in ["serial", "multiserial"] else f"UDP:{udp_port}",
        "holder_type": "none",
        "pids": [],
        "process_names": [],
        "container_id": "",
        "container_name": "",
        "is_microros": False,
        "details": "",
        "summary": "Port is available",
    }

    if host:
        ssh_cmd = [
            "ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=3",
            "-o", "StrictHostKeyChecking=no", "-o", "UserKnownHostsFile=/dev/null",
            f"{user}@{host}" if user else host
        ]
        if mode in ["serial", "multiserial"]:
            script = (
                f"echo '---FUSER---'; fuser '{port}' 2>/dev/null || true; "
                f"echo '---CONTAINERS---'; "
                f"docker ps --no-trunc --format '{{{{.ID}}}}|{{{{.Names}}}}|{{{{.Image}}}}|{{{{.Command}}}}' 2>/dev/null || true; "
                f"podman ps --no-trunc --format '{{{{.ID}}}}|{{{{.Names}}}}|{{{{.Image}}}}|{{{{.Command}}}}' 2>/dev/null || true; "
                f"echo '---PROCESSES---'; "
                f"pgrep -fa 'micro_ros_agent' 2>/dev/null || true"
            )
        else:
            script = (
                f"echo '---FUSER---'; fuser '{udp_port}/udp' 2>/dev/null || true; "
                f"echo '---CONTAINERS---'; "
                f"docker ps --no-trunc --format '{{{{.ID}}}}|{{{{.Names}}}}|{{{{.Image}}}}|{{{{.Command}}}}' 2>/dev/null || true; "
                f"podman ps --no-trunc --format '{{{{.ID}}}}|{{{{.Names}}}}|{{{{.Image}}}}|{{{{.Command}}}}' 2>/dev/null || true; "
                f"echo '---PROCESSES---'; "
                f"ss -ulnp 'sport = :{udp_port}' 2>/dev/null || true"
            )
        try:
            r = subprocess.run(ssh_cmd + [script], capture_output=True, text=True, timeout=5)
            output = r.stdout
        except Exception as e:
            res["status"] = "error"
            res["details"] = f"SSH error: {e}"
            return res
        return _parse_port_check_output(output, port, mode, udp_port, res)

    # Local port checks
    output_parts = ["---FUSER---"]
    if mode in ["serial", "multiserial"]:
        if os.path.exists(port):
            try:
                f = subprocess.run(["fuser", port], capture_output=True, text=True, timeout=2)
                output_parts.append(f.stdout)
            except Exception:
                pass
        output_parts.append("---CONTAINERS---")
        for engine in ["docker", "podman"]:
            try:
                d = subprocess.run([engine, "ps", "--no-trunc", "--format", "{{.ID}}|{{.Names}}|{{.Image}}|{{.Command}}"], capture_output=True, text=True, timeout=2)
                output_parts.append(d.stdout)
            except Exception:
                pass
        output_parts.append("---PROCESSES---")
        try:
            pr = subprocess.run(["pgrep", "-fa", "micro_ros_agent"], capture_output=True, text=True, timeout=2)
            output_parts.append(pr.stdout)
        except Exception:
            pass
    else:
        try:
            f = subprocess.run(["fuser", f"{udp_port}/udp"], capture_output=True, text=True, timeout=2)
            output_parts.append(f.stdout)
        except Exception:
            pass
        output_parts.append("---CONTAINERS---")
        for engine in ["docker", "podman"]:
            try:
                d = subprocess.run([engine, "ps", "--no-trunc", "--format", "{{.ID}}|{{.Names}}|{{.Image}}|{{.Command}}"], capture_output=True, text=True, timeout=2)
                output_parts.append(d.stdout)
            except Exception:
                pass
        output_parts.append("---PROCESSES---")
        try:
            pr = subprocess.run(["ss", "-ulnp", f"sport = :{udp_port}"], capture_output=True, text=True, timeout=2)
            output_parts.append(pr.stdout)
        except Exception:
            pass

    return _parse_port_check_output("\n".join(output_parts), port, mode, udp_port, res)


def _parse_port_check_output(output: str, port: str, mode: str, udp_port: int, res: Dict[str, Any]) -> Dict[str, Any]:
    fuser_pids = []
    containers = []
    processes = []
    current_sec = None

    for line in output.splitlines():
        line_s = line.strip()
        if line_s == "---FUSER---":
            current_sec = "fuser"
            continue
        elif line_s == "---CONTAINERS---":
            current_sec = "containers"
            continue
        elif line_s == "---PROCESSES---":
            current_sec = "processes"
            continue

        if current_sec == "fuser" and line_s:
            for p in line_s.split():
                clean_p = p.rstrip("m").rstrip("e")
                if clean_p.isdigit():
                    fuser_pids.append(int(clean_p))
        elif current_sec == "containers" and line_s:
            containers.append(line_s)
        elif current_sec == "processes" and line_s:
            processes.append(line_s)

    # Check matching containers
    target_match = os.path.basename(port) if mode in ["serial", "multiserial"] else str(udp_port)
    for c_line in containers:
        parts = c_line.split("|")
        cid = parts[0]
        cname = parts[1] if len(parts) > 1 else ""
        cimg = parts[2] if len(parts) > 2 else ""
        ccmd = parts[3] if len(parts) > 3 else ""
        if target_match in ccmd or "micro-ros-agent" in cimg or "micro_ros_agent" in ccmd:
            res["in_use"] = True
            res["holder_type"] = "container"
            res["container_id"] = cid[:12]
            res["container_name"] = cname
            res["is_microros"] = "micro-ros-agent" in cimg or "micro_ros_agent" in ccmd
            res["summary"] = f"Held by container '{cname}' ({cid[:12]})"
            return res

    # Check matching host pids
    if fuser_pids:
        res["in_use"] = True
        res["holder_type"] = "process"
        res["pids"] = fuser_pids
        for pid in fuser_pids:
            try:
                cmdline = subprocess.check_output(["ps", "-p", str(pid), "-o", "args="], text=True).strip()
                res["process_names"].append(cmdline)
                if "micro_ros_agent" in cmdline:
                    res["is_microros"] = True
            except Exception:
                pass
        p_names = ", ".join(res["process_names"]) or f"PID {fuser_pids}"
        res["summary"] = f"Held by process: {p_names}"
        return res

    # Check background micro_ros_agent processes
    for pr_line in processes:
        if "micro_ros_agent" in pr_line:
            parts = pr_line.split(None, 1)
            if parts and parts[0].isdigit():
                pid = int(parts[0])
                res["in_use"] = True
                res["holder_type"] = "process"
                res["pids"].append(pid)
                res["process_names"].append(parts[1] if len(parts) > 1 else "micro_ros_agent")
                res["is_microros"] = True
                res["summary"] = f"Held by micro_ros_agent (PID {pid})"
                return res

    return res


def release_agent_port(
    port: str = "/dev/ttyUSB0",
    mode: str = "serial",
    udp_port: int = 8888,
    host: Optional[str] = None,
    user: Optional[str] = None,
) -> Dict[str, Any]:
    """
    Safely releases the port by terminating the holding processes using targeted PID kill.
    NEVER uses pkill or killall. Enforces port immunity for 8000, 5173, and 9090.
    """
    status = check_agent_port_status(port, mode, udp_port, host=host, user=user)
    res = {
        "status": "ok",
        "released": True,
        "actions": [],
        "errors": [],
    }

    if not status["in_use"]:
        res["actions"].append("Port was already free.")
        return res

    # Container release
    if status.get("container_id"):
        cid = status["container_id"]
        for eng in ["docker", "podman"]:
            try:
                subprocess.run([eng, "stop", cid], capture_output=True, timeout=5)
                res["actions"].append(f"Stopped container {cid} via {eng}")
            except Exception:
                pass

    # Process release by targeted PID
    for pid in status.get("pids", []):
        try:
            # Check if PID is holding an immune port
            holders = subprocess.check_output(["lsof", "-i", "-P", "-n"], text=True).splitlines()
            is_immune = False
            for h in holders:
                if str(pid) in h:
                    for immune_p in IMMUNE_PORTS:
                        if f":{immune_p}" in h:
                            is_immune = True
                            break
            if is_immune:
                res["errors"].append(f"Refusing to terminate PID {pid}: bound to protected port.")
                continue

            os.kill(pid, signal.SIGINT)
            time.sleep(0.5)
            # If still alive, SIGTERM
            try:
                os.kill(pid, 0)
                os.kill(pid, signal.SIGTERM)
            except OSError:
                pass
            res["actions"].append(f"Sent targeted signal to PID {pid}")
        except Exception as e:
            res["errors"].append(f"Error terminating PID {pid}: {e}")

    time.sleep(0.5)
    # Re-verify
    recheck = check_agent_port_status(port, mode, udp_port, host=host, user=user)
    res["released"] = not recheck["in_use"]
    return res


def check_bringup_health(timeout: float = 4.0) -> Dict[str, Any]:
    """Queries the active ROS 2 graph on this machine."""
    res: Dict[str, Any] = {
        "ready": False,
        "ros_available": False,
        "status": "ok",
        "topics": {},
        "tf": [],
        "advertised_but_silent": [],
        "summary": "Checking bringup health...",
    }

    check_cmd = ["bash", "-lc", f"{ros_setup_shell()} && ros2 topic list"]
    try:
        listed = subprocess.run(check_cmd, capture_output=True, text=True, timeout=timeout + 2)
        present = {t.strip() for t in listed.stdout.splitlines() if t.strip()}
        res["ros_available"] = listed.returncode == 0 and bool(present)
    except Exception as e:
        res["status"] = "error"
        res["summary"] = f"Could not query ROS 2 graph: {e}"
        return res

    if not res["ros_available"]:
        res["status"] = "no_graph"
        res["summary"] = "No ROS 2 graph reachable. Ensure micro-ROS agent and bringup are running."
        return res

    for key, topic, what, min_hz in BRINGUP_HEALTH_TOPICS:
        entry = {
            "topic": topic,
            "what": what,
            "min_hz": min_hz,
            "advertised": topic in present,
            "hz": None,
            "ok": False,
        }
        if entry["advertised"]:
            hz_cmd = ["bash", "-lc",
                      f"{ros_setup_shell()} && timeout 3 ros2 topic hz {shlex.quote(topic)}"]
            try:
                hz_out = subprocess.run(hz_cmd, capture_output=True, text=True, timeout=4)
                m = re.search(r"average rate:\s*([0-9.]+)", hz_out.stdout)
                if m:
                    entry["hz"] = float(m.group(1))
                    entry["ok"] = entry["hz"] >= min_hz
            except Exception:
                pass
        res["topics"][key] = entry

    odom_ok = res["topics"].get("odom", {}).get("ok") or res["topics"].get("odom_raw", {}).get("ok")
    res["ready"] = bool(odom_ok)
    res["summary"] = f"Bringup healthy: odom fused={res['topics'].get('odom', {}).get('ok')}, raw={res['topics'].get('odom_raw', {}).get('ok')}"
    return res
