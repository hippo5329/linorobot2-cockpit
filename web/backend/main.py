#!/usr/bin/env python3
"""Linorobot2 Cockpit supervisor entry point.

The app, middleware and shared helpers live in core.py; each routes_* module
registers its handlers on core.app as an import side effect. This file wires
them together, mounts the static frontend, and runs uvicorn. Kept runnable as
`python3 web/backend/main.py` and importable as `main:app`.
"""
import os

from fastapi.staticfiles import StaticFiles

from core import app, FRONTEND_DIR

# Importing these registers every @app route on `app` (side effect). Order is
# irrelevant; noqa: F401 -- imported for the registration, not for a name.
import routes_config  # noqa: F401
import routes_config_editors  # noqa: F401
import routes_exec  # noqa: F401
import routes_hardware  # noqa: F401
import routes_secrets  # noqa: F401
import routes_status  # noqa: F401
import routes_syslog  # noqa: F401
import routes_system  # noqa: F401


# Mount static frontend.
#
# No-store on the app's own files. StaticFiles serves them with an ETag and a
# Last-Modified, which lets a browser keep a stale app.js across a cockpit
# update: the backend restarts with new behaviour, the page keeps the old
# script, and the two disagree in ways that look like the feature never
# landed. Measured here on 2026-09-20 -- a fixed Monitor worked through curl
# and did nothing in the browser until a hard reload. These files are a few
# hundred KB served over a LAN; re-fetching them costs nothing next to that.
class _NoStoreStatic(StaticFiles):
    async def get_response(self, path, scope):
        response = await super().get_response(path, scope)
        response.headers["Cache-Control"] = "no-store, must-revalidate"
        return response


if os.path.isdir(FRONTEND_DIR):
    app.mount("/", _NoStoreStatic(directory=FRONTEND_DIR, html=True), name="frontend")


if __name__ == "__main__":
    import argparse
    import uvicorn

    ap = argparse.ArgumentParser(description="Linorobot2 Cockpit supervisor")
    ap.add_argument("--host", default="0.0.0.0")
    # 8000 is the cockpit's own port; a spare one keeps a test instance clear of it.
    ap.add_argument("--port", type=int, default=int(os.environ.get("COCKPIT_PORT", 8000)))
    cli_args = ap.parse_args()
    uvicorn.run(app, host=cli_args.host, port=cli_args.port)
