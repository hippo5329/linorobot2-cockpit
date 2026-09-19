#!/usr/bin/env python3
# ==============================================================================
# Linorobot2 Cockpit — UDP Syslog Manager & Real-Time Telemetry Broadcaster
#
# Listens on UDP (default port 514 with auto-fallback to 5140 in rootless mode).
# Appends timestamped logs to logs/syslog_YYYYMMDD.log and broadcasts events
# in real-time to Web UI SSE subscribers.
# ==============================================================================

import collections
import datetime
import json
import os
import queue
import socket
import threading
import time
from typing import Any, Dict, List, Optional, Set

SYSLOG_MAX_BYTES = 10 * 1024 * 1024  # 10 MB per log file before rotation


class SyslogManager:
    """
    Lightweight, multi-threaded UDP Syslog server & real-time telemetry broadcaster.
    Listens on UDP (default port 514 with auto-fallback to 5140 for non-root/rootless),
    appends timestamped telemetry logs to repo logs/syslog_YYYYMMDD.log (rotated
    at SYSLOG_MAX_BYTES with one .1 backup), and broadcasts events in real-time
    to Web UI SSE subscribers.
    """

    def __init__(self, repo_root: str):
        self.repo_root = os.path.abspath(repo_root)
        self.logs_dir = os.path.join(self.repo_root, "logs")
        os.makedirs(self.logs_dir, exist_ok=True)
        self.sock: Optional[socket.socket] = None
        self.thread: Optional[threading.Thread] = None
        self.is_running: bool = False
        self.is_rootless: bool = hasattr(os, "geteuid") and os.geteuid() != 0
        self.port: int = 5140 if self.is_rootless else 514
        self.bound_port: int = self.port
        self.packets_received: int = 0
        self.last_sender: str = ""
        self.last_message: str = ""
        self.last_timestamp: str = ""
        self.recent_logs: collections.deque = collections.deque(maxlen=200)
        self.subscribers: Set[queue.Queue] = set()
        self.subscribers_lock: threading.Lock = threading.Lock()
        self.lock: threading.Lock = threading.Lock()

        # Persistent append handle for the current log file
        self._log_fh = None
        self._log_path = None
        self._log_bytes = 0

    def get_current_log_filepath(self) -> str:
        date_str = datetime.datetime.now().strftime("%Y%m%d")
        return os.path.join(self.logs_dir, f"syslog_{date_str}.log")

    def _close_log(self):
        if self._log_fh:
            try:
                self._log_fh.close()
            except Exception:
                pass
        self._log_fh = None
        self._log_path = None
        self._log_bytes = 0

    def _write_log_line(self, line: str):
        """Append one line, opening/rotating the day's log file as needed."""
        path = self.get_current_log_filepath()
        try:
            if self._log_fh is None or self._log_path != path:
                self._close_log()
                self._log_path = path
                self._log_fh = open(path, "a", encoding="utf-8")
                try:
                    self._log_bytes = os.path.getsize(path)
                except OSError:
                    self._log_bytes = 0

            encoded_len = len(line.encode("utf-8"))
            if self._log_bytes + encoded_len > SYSLOG_MAX_BYTES:
                self._log_fh.close()
                try:
                    os.replace(path, path + ".1")
                except OSError:
                    pass
                self._log_fh = open(path, "a", encoding="utf-8")
                self._log_bytes = 0

            self._log_fh.write(line)
            self._log_fh.flush()
            self._log_bytes += encoded_len
        except Exception:
            self._close_log()

    @staticmethod
    def _host_ip() -> str:
        """Best-effort primary LAN IP of this host."""
        for probe in ("10.255.255.255", "192.168.255.255", "8.8.8.8"):
            try:
                k = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                k.connect((probe, 1))
                ip = k.getsockname()[0]
                k.close()
                if ip and not ip.startswith("127."):
                    return ip
            except Exception:
                pass
        try:
            return socket.gethostbyname(socket.gethostname())
        except Exception:
            return "127.0.0.1"

    def start(self, requested_port: Optional[int] = None) -> Dict[str, Any]:
        with self.lock:
            if self.is_running:
                return {
                    "status": "already_running",
                    "running": True,
                    "host_ip": self._host_ip(),
                    "port": self.bound_port,
                    "requested_port": self.port,
                    "is_rootless": self.is_rootless,
                    "logfile": os.path.relpath(self.get_current_log_filepath(), self.repo_root),
                    "packets": self.packets_received,
                    "fallback": self.bound_port != self.port,
                }

            if requested_port is not None:
                try:
                    self.port = int(requested_port)
                except Exception:
                    self.port = 5140 if self.is_rootless else 514
            else:
                self.port = 5140 if self.is_rootless else 514

            ports_to_try = [self.port]
            if self.port != 5140 and 5140 not in ports_to_try:
                ports_to_try.append(5140)
            if 5514 not in ports_to_try:
                ports_to_try.append(5514)

            bound = False
            last_err = None
            for p in ports_to_try:
                try:
                    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                    s.bind(("0.0.0.0", p))
                    self.sock = s
                    self.bound_port = p
                    bound = True
                    break
                except Exception as e:
                    last_err = e
                    continue

            if not bound:
                raise RuntimeError(f"Could not bind UDP syslog port (tried {ports_to_try}): {last_err}")

            self.is_running = True
            self.thread = threading.Thread(target=self._listen_loop, daemon=True)
            self.thread.start()

            fallback_triggered = (self.bound_port != self.port)

            return {
                "status": "running",
                "running": True,
                "host_ip": self._host_ip(),
                "port": self.bound_port,
                "requested_port": self.port,
                "is_rootless": self.is_rootless,
                "fallback": fallback_triggered,
                "logfile": os.path.relpath(self.get_current_log_filepath(), self.repo_root),
                "packets": self.packets_received,
            }

    def stop(self) -> Dict[str, Any]:
        with self.lock:
            if not self.is_running:
                return {
                    "status": "stopped",
                    "running": False,
                    "port": self.bound_port,
                    "packets": self.packets_received,
                }
            self.is_running = False
            if self.sock:
                try:
                    dummy = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                    dummy.sendto(b"__SHUTDOWN__", ("127.0.0.1", self.bound_port))
                    dummy.close()
                    self.sock.close()
                except Exception:
                    pass
                self.sock = None
            self._close_log()

            return {
                "status": "stopped",
                "running": False,
                "port": self.bound_port,
                "packets": self.packets_received,
            }

    def _listen_loop(self):
        while self.is_running:
            try:
                if not self.sock:
                    break
                data, addr = self.sock.recvfrom(4096)
                if not self.is_running:
                    break
                if data == b"__SHUTDOWN__":
                    continue

                raw_text = data.decode("utf-8", errors="replace").strip()
                now = datetime.datetime.now()
                time_str = now.strftime("%Y-%m-%d %H:%M:%S")
                client_str = f"{addr[0]}:{addr[1]}"

                pri = None
                clean_text = raw_text
                if raw_text.startswith("<") and ">" in raw_text[:6]:
                    pri_end = raw_text.index(">")
                    try:
                        pri = int(raw_text[1:pri_end])
                        clean_text = raw_text[pri_end + 1:].strip()
                    except ValueError:
                        pass

                self.packets_received += 1
                log_entry = {
                    "time": time_str,
                    "sender": client_str,
                    "ip": addr[0],
                    "port": addr[1],
                    "client_ip": addr[0],
                    "client_port": addr[1],
                    "pri": pri,
                    "message": clean_text,
                    "raw": raw_text,
                    "packets_received": self.packets_received,
                }

                formatted_line = f"[{time_str}] [{client_str}] {clean_text}\n"
                self._write_log_line(formatted_line)
                self.last_sender = client_str
                self.last_message = clean_text
                self.last_timestamp = time_str
                self.recent_logs.append(log_entry)

                self._broadcast(log_entry)
            except Exception:
                if not self.is_running:
                    break
                time.sleep(0.05)

    def subscribe(self, q: queue.Queue):
        with self.subscribers_lock:
            self.subscribers.add(q)

    def unsubscribe(self, q: queue.Queue):
        with self.subscribers_lock:
            self.subscribers.discard(q)

    def _broadcast(self, log_entry: Dict[str, Any]):
        with self.subscribers_lock:
            for q in list(self.subscribers):
                try:
                    q.put_nowait(log_entry)
                except Exception:
                    pass

    def get_status(self) -> Dict[str, Any]:
        cur_file = self.get_current_log_filepath()
        file_size = os.path.getsize(cur_file) if os.path.exists(cur_file) else 0
        return {
            "status": "ok",
            "running": self.is_running,
            "host_ip": self._host_ip(),
            "port": self.bound_port,
            "requested_port": self.port,
            "is_rootless": self.is_rootless,
            "fallback": self.bound_port != self.port,
            "logfile": os.path.relpath(cur_file, self.repo_root),
            "filesize": file_size,
            "packets": self.packets_received,
            "last_sender": self.last_sender,
            "last_message": self.last_message,
            "last_timestamp": self.last_timestamp,
            "recent_count": len(self.recent_logs),
        }

    def get_logs_data(self, tail_count: int = 50) -> Dict[str, Any]:
        files = []
        if os.path.exists(self.logs_dir):
            for f in sorted(os.listdir(self.logs_dir), reverse=True):
                if f.endswith(".log"):
                    fp = os.path.join(self.logs_dir, f)
                    files.append({
                        "name": f,
                        "path": os.path.relpath(fp, self.repo_root),
                        "size": os.path.getsize(fp),
                        "mtime": os.path.getmtime(fp),
                    })

        current_log = self.get_current_log_filepath()
        lines = []
        if os.path.exists(current_log):
            try:
                with open(current_log, "r", encoding="utf-8", errors="replace") as f:
                    all_lines = f.readlines()
                    lines = [l.strip() for l in all_lines[-tail_count:]]
            except Exception:
                pass

        return {
            "status": "ok",
            "files": files,
            "current_file": os.path.relpath(current_log, self.repo_root),
            "lines": lines,
        }
