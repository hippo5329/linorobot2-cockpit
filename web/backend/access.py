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
import time

TOKEN_FILE_NAME = ".cockpit_token"

# Short-lived tickets for server-sent-event streams. EventSource cannot set an
# Authorization header, so the long-lived token used to ride in the query string
# of every stream URL -- and a query string lands in the backend's access log, in
# any proxy log, and in `ps` while the request is open. A stream ticket is minted
# by a normal token-authed GET (the header carries the real token), is good once
# and for a minute, and is what the EventSource URL carries instead. The secret
# never appears in a URL again.
_STREAM_TICKET_TTL = 60.0
_stream_tickets: dict = {}   # ticket -> expiry epoch


def mint_stream_ticket() -> str:
    _prune_stream_tickets()
    ticket = secrets.token_urlsafe(24)
    _stream_tickets[ticket] = time.monotonic() + _STREAM_TICKET_TTL
    return ticket


def consume_stream_ticket(ticket: str) -> bool:
    """True exactly once for a valid, unexpired ticket; it is spent on use."""
    if not ticket:
        return False
    _prune_stream_tickets()
    expiry = _stream_tickets.pop(ticket, None)
    return expiry is not None and expiry >= time.monotonic()


def _prune_stream_tickets() -> None:
    now = time.monotonic()
    for t, expiry in list(_stream_tickets.items()):
        if expiry < now:
            _stream_tickets.pop(t, None)

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


def is_stream_path(path: str) -> bool:
    """The protected GETs that are EventSource streams -- they may be authorised
    by a one-shot stream ticket in ?ticket= instead of the token, so the token
    stays out of the URL."""
    return path.rstrip("/") in ("/api/workflow/one-click/stream", "/api/lidar_stream")


def token_matches(presented: str, expected: str) -> bool:
    return bool(presented) and secrets.compare_digest(presented, expected)


def allowed_roots(config_dir: str, repo_root: str, for_write: bool = False):
    """Where the browser's directory picker and config export may point.

    This is a NAVIGATION fence, not a confidentiality boundary: it keeps the
    picker out of `/etc`, `/proc`, `/sys`, `/root` and the like, and `list_dir`
    only ever returns entry NAMES. Reads may look where a person legitimately
    keeps ports, ROS workspaces and configs -- the home directory, the config
    dir, the checkout, `/dev` and the removable-media mounts.

    WRITES (config export) are narrower on purpose: the config dir, and a
    removable drive. Not the whole home directory, and never the checkout --
    the first review flagged an export that could drop a file anywhere under
    `~`, and there is no reason a config export needs that reach.
    """
    if for_write:
        roots = [config_dir, "/media", "/mnt", "/run/media"]
    else:
        roots = [os.path.expanduser("~"), config_dir, repo_root, "/dev", "/media", "/mnt", "/run/media"]
    return [os.path.realpath(r) for r in roots if r]


def path_allowed(path: str, config_dir: str, repo_root: str, for_write: bool = False) -> bool:
    """True when `path` lies under one of the roots the browser may point at.
    Pass `for_write=True` for the export destination, which is fenced tighter."""
    real = os.path.realpath(os.path.expanduser(path or ""))
    for root in allowed_roots(config_dir, repo_root, for_write=for_write):
        if real == root or real.startswith(root + os.sep):
            return True
    return False
