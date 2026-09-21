"""Linorobot2 Cockpit routes -- secrets.yaml (Wi-Fi & network) management.

Part of the main.py split: route handlers registered on the shared `app`
as an import side effect (main.py imports this module). Shared state and
helpers come from core.py. See core.py for the split's contract.
"""
import os
import yaml
from fastapi import HTTPException, Request
from core import (
    SECRETS_EXAMPLE_PATH,
    SECRETS_PATH,
    app,
    display_path,
    json_body,
    regenerate_firmware_headers,
    syslog_manager,
)


# ==============================================================================
# Secrets Management API (WiFi, Telemetry, micro-ROS quarantine)
# ==============================================================================
@app.get("/api/secrets")
def get_secrets():
    exists = os.path.isfile(SECRETS_PATH)
    path_to_read = SECRETS_PATH if exists else SECRETS_EXAMPLE_PATH
    raw_yaml = ""
    secrets_data = {}
    if os.path.isfile(path_to_read):
        with open(path_to_read, "r") as f:
            raw_yaml = f.read()
        try:
            secrets_data = yaml.safe_load(raw_yaml) or {}
        except Exception:
            secrets_data = {}

    # If syslog_port in secrets is 514 and system is rootless, note the active bound port
    bound_syslog = syslog_manager.bound_port
    return {
        "success": True,
        "exists": exists,
        "path": display_path(SECRETS_PATH) if exists else display_path(SECRETS_EXAMPLE_PATH),
        "secrets": secrets_data,
        "raw_yaml": raw_yaml,
        "is_rootless": syslog_manager.is_rootless,
        "bound_syslog_port": bound_syslog,
    }


@app.post("/api/secrets")
async def save_secrets(request: Request):
    content_type = request.headers.get("content-type", "")
    raw_yaml = ""
    secrets_data = {}
    if "application/json" in content_type:
        data = await json_body(request)
        if "raw_yaml" in data and isinstance(data["raw_yaml"], str) and data["raw_yaml"].strip():
            raw_yaml = data["raw_yaml"]
            secrets_data = yaml.safe_load(raw_yaml) or {}
        elif "secrets" in data and isinstance(data["secrets"], dict):
            secrets_data = data["secrets"]
            raw_yaml = yaml.dump(secrets_data, sort_keys=False, default_flow_style=False)
        else:
            secrets_data = data
            raw_yaml = yaml.dump(secrets_data, sort_keys=False, default_flow_style=False)
    else:
        body = await request.body()
        raw_yaml = body.decode("utf-8")
        secrets_data = yaml.safe_load(raw_yaml) or {}

    if not isinstance(secrets_data, dict):
        raise HTTPException(status_code=400, detail="Invalid YAML structure")

    os.makedirs(os.path.dirname(SECRETS_PATH), exist_ok=True)
    with open(SECRETS_PATH, "w") as f:
        f.write(raw_yaml)
    try:
        os.chmod(SECRETS_PATH, 0o600)
    except Exception:
        pass

    res = regenerate_firmware_headers()
    return {
        "success": True,
        "status": "saved",
        "path": display_path(SECRETS_PATH),
        "message": "Saved secrets.yaml and regenerated firmware headers",
        "generator_stdout": res.stdout,
    }
