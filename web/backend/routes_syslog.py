"""Linorobot2 Cockpit routes -- UDP syslog server control.

Part of the main.py split: route handlers registered on the shared `app`
as an import side effect (main.py imports this module). Shared state and
helpers come from core.py. See core.py for the split's contract.
"""
import json
import queue
from fastapi import HTTPException, Request
from fastapi.responses import StreamingResponse
from core import (
    app,
    json_body,
    syslog_manager,
)


# ==============================================================================
# UDP Syslog Telemetry Endpoints (with Rootless Fallback)
# ==============================================================================
@app.get("/api/syslog/status")
def api_syslog_status():
    return syslog_manager.get_status()


@app.post("/api/syslog/start")
async def api_syslog_start(request: Request):
    data = {}
    try:
        data = await json_body(request)
    except Exception:
        pass
    req_port = data.get("port")
    try:
        res = syslog_manager.start(req_port)
        return res
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@app.post("/api/syslog/stop")
def api_syslog_stop():
    return syslog_manager.stop()


@app.get("/api/syslog/logs")
def api_syslog_logs(tail: int = 50):
    return syslog_manager.get_logs_data(tail_count=tail)


@app.get("/api/syslog/stream")
def api_syslog_stream():
    q = queue.Queue(maxsize=100)
    syslog_manager.subscribe(q)

    def event_generator():
        try:
            init_status = syslog_manager.get_status()
            yield f"event: status\ndata: {json.dumps(init_status)}\n\n"
            while True:
                try:
                    entry = q.get(timeout=2.0)
                    yield f"event: syslog\ndata: {json.dumps(entry)}\n\n"
                except queue.Empty:
                    yield ": ping\n\n"
        finally:
            syslog_manager.unsubscribe(q)

    return StreamingResponse(event_generator(), media_type="text/event-stream")
