"""Linorobot2 Cockpit routes -- Docker/Podman rootless setup, container install, autostart-on-boot.

Part of the main.py split: route handlers registered on the shared `app`
as an import side effect (main.py imports this module). Shared state and
helpers come from core.py. See core.py for the split's contract.
"""
from typing import Optional
from fastapi import Request
from core import (
    app,
    check_container_status,
    disable_autostart,
    enable_autostart,
    get_autostart_logs,
    get_autostart_status,
    get_rootless_info,
    install_container_engine,
    json_body,
    setup_rootless_docker,
)


# ==============================================================================
# Rootless Docker & Container Engine APIs
# ==============================================================================
@app.get("/api/docker/status")
def api_docker_status():
    return check_container_status()


@app.get("/api/docker/rootless_info")
def api_docker_rootless_info():
    return get_rootless_info()


@app.post("/api/docker/setup_rootless")
def api_docker_setup_rootless():
    return setup_rootless_docker()


@app.get("/api/container/install")
@app.post("/api/container/install")
async def api_container_install(request: Request, engine: Optional[str] = None):
    req_engine = engine
    if request.method == "POST":
        try:
            data = await json_body(request)
            if isinstance(data, dict):
                req_engine = data.get("engine") or req_engine
        except Exception:
            pass
    if not req_engine:
        req_engine = request.query_params.get("engine", "docker")
    return install_container_engine(req_engine)


# ==============================================================================
# Autostart APIs (systemd user unit)
# ==============================================================================
@app.get("/api/autostart/status")
def api_autostart_status():
    return get_autostart_status()


@app.post("/api/autostart/enable")
async def api_autostart_enable(request: Request):
    data = {}
    try:
        data = await json_body(request)
    except Exception:
        pass
    return enable_autostart(data)


@app.post("/api/autostart/disable")
def api_autostart_disable():
    return disable_autostart()


@app.get("/api/autostart/logs")
def api_autostart_logs():
    return get_autostart_logs()
