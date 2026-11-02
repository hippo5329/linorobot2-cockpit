"""Options for the uvicorn server that runs the backend (main.py).

No imports with side effects: tests read these without starting the cockpit."""
import inspect


# How long a stop waits for open connections before closing them. Without a bound,
# uvicorn's graceful shutdown waits for EVERY connection to end -- and the page always
# holds one that never does, the syslog stream (an SSE). One SIGTERM then left a backend
# that had closed its listener but never exited: alive, so Docker never restarted it, and
# deaf, so every request failed ("Failed to fetch") until someone restarted it by hand
# (2026-11-02, a bench script's stray SIGTERM; any `kill` of the backend does the same).
# Bounded, the stop closes the streams and exits, and the container's restart policy
# brings the cockpit back.
GRACEFUL_SHUTDOWN_S = 5


def graceful_shutdown_kwargs(uvicorn_module) -> dict:
    """uvicorn.run's bound on the graceful shutdown, where this uvicorn has one.

    `timeout_graceful_shutdown` came in uvicorn 0.20. Debian bookworm's python3-uvicorn
    (Raspberry Pi OS, the apt fallback in core.py) is 0.17, which rejects an unknown
    keyword -- the cockpit must still start there, unbounded as before."""
    try:
        params = inspect.signature(uvicorn_module.Config.__init__).parameters
    except (TypeError, ValueError, AttributeError):
        return {}
    return {"timeout_graceful_shutdown": GRACEFUL_SHUTDOWN_S} if "timeout_graceful_shutdown" in params else {}
