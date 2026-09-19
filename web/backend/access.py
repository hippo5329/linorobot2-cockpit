"""Who may write to the cockpit, and where it may read.

The supervisor listens on every interface of the robot computer, and its
API is what the browser uses to run things -- flashing, bringup, arbitrary
commands through the runner slots. Until this module existed, any HTTP
client on the LAN could do all of that, and the CORS policy was `*` with
credentials, so any web page open in any browser on the same network could
too: the preflight for a JSON POST passed from every origin.

Two policies, both small:

* A per-install access token. Generated once into `<config dir>/.cockpit_token`
  (mode 0600) -- or taken from `COCKPIT_TOKEN` -- and printed at start-up as a
  URL the operator opens once: `http://<robot-computer>:8000/?token=...`. The
  page keeps it in the browser (web/frontend/app.js) and sends it on every
  request to /api/. Every method other than GET/HEAD/OPTIONS needs it, and so
  do the few GETs that hand out something private. Read-only status and the
  log streams stay open: EventSource cannot set headers, and a log is not a
  capability. `COCKPIT_AUTH=off` disables the check for a network you trust
  completely; the default is on.

* A root allowlist for endpoints that take a filesystem path from the
  browser (the directory picker, the config export). The picker exists to
  choose serial ports, workspaces and config files, so it may look under the
  user's home, the config dir, the checkout, /dev and the mount points -- and
  nowhere else.
"""
import os
import secrets

TOKEN_FILE_NAME = ".cockpit_token"

# GETs that return something a stranger must not have, or that DO something:
# the 1-Click pipeline and the LiDAR viewer are server-sent streams, so the
# browser opens them with GET, and one of them flashes the board and starts
# the whole ROS 2 stack. An EventSource cannot set a header, so the page
# passes ?token= on those URLs; presented_token() reads it.
PROTECTED_GETS = {
    "/api/secrets",
    "/api/list_dir",
    "/api/workflow/one-click/stream",
    "/api/lidar_stream",
}


def auth_enabled() -> bool:
    return os.environ.get("COCKPIT_AUTH", "on").strip().lower() not in ("off", "0", "false", "no")


def token_path(config_dir: str) -> str:
    return os.path.join(config_dir, TOKEN_FILE_NAME)


def load_or_create_token(config_dir: str) -> str:
    """The install's token: the environment wins, then the file, else a new one."""
    env = os.environ.get("COCKPIT_TOKEN", "").strip()
    if env:
        return env
    path = token_path(config_dir)
    try:
        with open(path) as f:
            existing = f.read().strip()
        if existing:
            return existing
    except FileNotFoundError:
        pass
    token = secrets.token_urlsafe(24)
    fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600)
    with os.fdopen(fd, "w") as f:
        f.write(token + "\n")
    return token


def presented_token(request) -> str:
    """The token the request carries, wherever it put it."""
    auth = request.headers.get("authorization", "")
    if auth.lower().startswith("bearer "):
        return auth[7:].strip()
    header = request.headers.get("x-cockpit-token", "")
    if header:
        return header.strip()
    query = request.query_params.get("token", "")
    if query:
        return query.strip()
    return request.cookies.get("cockpit_token", "").strip()


def needs_token(method: str, path: str) -> bool:
    if not path.startswith("/api/"):
        return False
    if method.upper() not in ("GET", "HEAD", "OPTIONS"):
        return True
    return path.rstrip("/") in PROTECTED_GETS


def token_matches(presented: str, expected: str) -> bool:
    return bool(presented) and secrets.compare_digest(presented, expected)


def allowed_roots(config_dir: str, repo_root: str):
    roots = [os.path.expanduser("~"), config_dir, repo_root, "/dev", "/media", "/mnt", "/run/media", "/tmp"]
    return [os.path.realpath(r) for r in roots if r]


def path_allowed(path: str, config_dir: str, repo_root: str) -> bool:
    """True when `path` lies under one of the roots the browser may point at."""
    real = os.path.realpath(os.path.expanduser(path or ""))
    for root in allowed_roots(config_dir, repo_root):
        if real == root or real.startswith(root + os.sep):
            return True
    return False
