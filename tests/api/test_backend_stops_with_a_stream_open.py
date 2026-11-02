"""One SIGTERM must stop the backend even while a page is open.

Every page holds /api/syslog/stream, an SSE that never ends. uvicorn's graceful
shutdown waited for every connection to close, so a SIGTERM left a backend that
had closed its listener but never exited: Docker saw it running and never
restarted it, and every request failed ("Failed to fetch") until someone
restarted it by hand. Found 2026-11-02 when a bench script's stray SIGTERM hit
the backend mid-walkthrough; any `kill` of the backend did the same.

Self-contained: starts its own backend on a spare port and a scratch config
directory, so it runs wherever fastapi and uvicorn are installed (the API job).
"""
import inspect
import os
import signal
import socket
import subprocess
import sys
import tempfile
import time
import types

import pytest

pytest.importorskip("fastapi", reason="the cockpit backend needs fastapi/uvicorn")
uvicorn = pytest.importorskip("uvicorn", reason="the cockpit backend needs fastapi/uvicorn")

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
BACKEND = os.path.join(ROOT, "web", "backend")
sys.path.insert(0, BACKEND)

import server_options  # noqa: E402  (no side effects; main.py's import would seed a config dir)


def _free_port():
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def _get(port, path, timeout=2.0):
    with socket.create_connection(("127.0.0.1", port), timeout=timeout) as s:
        s.sendall(f"GET {path} HTTP/1.1\r\nHost: x\r\nConnection: close\r\n\r\n".encode())
        return s.recv(64)


def test_a_sigterm_stops_the_backend_while_the_syslog_stream_is_open():
    if not server_options.graceful_shutdown_kwargs(uvicorn):
        pytest.skip(f"uvicorn {uvicorn.__version__} has no timeout_graceful_shutdown")
    port = _free_port()
    with tempfile.TemporaryDirectory() as tmp:
        env = dict(os.environ, COCKPIT_AUTH="off", COCKPIT_CONFIG_DIR=os.path.join(tmp, "cfg"))
        log = open(os.path.join(tmp, "backend.log"), "w+")
        proc = subprocess.Popen([sys.executable, os.path.join(BACKEND, "main.py"),
                                 "--host", "127.0.0.1", "--port", str(port)],
                                cwd=ROOT, env=env, stdout=log, stderr=subprocess.STDOUT)
        stream = None
        try:
            deadline = time.monotonic() + 60
            while True:
                try:
                    if _get(port, "/").startswith(b"HTTP/1.1"):
                        break
                except OSError:
                    pass
                assert proc.poll() is None and time.monotonic() < deadline, "the backend never came up"
                time.sleep(0.3)

            # What the page holds: the syslog stream, open and already flowing.
            stream = socket.create_connection(("127.0.0.1", port), timeout=10)
            stream.sendall(b"GET /api/syslog/stream HTTP/1.1\r\nHost: x\r\n\r\n")
            assert b"200" in stream.recv(256)

            proc.send_signal(signal.SIGTERM)
            bound = server_options.GRACEFUL_SHUTDOWN_S + 10
            try:
                proc.wait(timeout=bound)
            except subprocess.TimeoutExpired:
                log.seek(0)
                pytest.fail(f"the backend was still running {bound} s after SIGTERM with a stream "
                            f"open (the listener is closed: every request fails):\n{log.read()[-2000:]}")
        finally:
            if stream is not None:
                stream.close()
            if proc.poll() is None:
                proc.kill()
                proc.wait()
            log.close()


def test_an_old_uvicorn_still_starts_unbounded():
    # Debian bookworm's python3-uvicorn (0.17) has no timeout_graceful_shutdown and
    # rejects an unknown keyword: the backend must not pass it there.

    class Config:
        def __init__(self, app, host="127.0.0.1", port=8000):
            pass

    assert server_options.graceful_shutdown_kwargs(types.SimpleNamespace(Config=Config)) == {}
    has_it = "timeout_graceful_shutdown" in inspect.signature(uvicorn.Config.__init__).parameters
    assert server_options.graceful_shutdown_kwargs(uvicorn) == (
        {"timeout_graceful_shutdown": server_options.GRACEFUL_SHUTDOWN_S} if has_it else {})
