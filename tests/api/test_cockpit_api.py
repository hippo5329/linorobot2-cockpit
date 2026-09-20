"""The cockpit's HTTP API, driven without a browser.

Every defect the 2026-09-19 UI pass found was server-side -- the header
generator, the prebuilt cache swap, the flash stamp, the port an action
flashes. A browser was an expensive way to issue those requests. This suite
issues them directly, so the same ground is covered by a test run.

Point it at any cockpit:

    COCKPIT_URL=http://localhost:18001 \
    COCKPIT_TOKEN=$(...) \
    python3 -m pytest tests/api -v

Skipped entirely when COCKPIT_URL is unset, so the hermetic suite stays
hermetic. stdlib only: no new dependency for something CI has to install.
"""
import json
import os
import urllib.error
import urllib.parse
import urllib.request

import pytest

URL = os.environ.get("COCKPIT_URL")
TOKEN = os.environ.get("COCKPIT_TOKEN", "")
TIMEOUT = float(os.environ.get("COCKPIT_TIMEOUT", "30"))

pytestmark = pytest.mark.skipif(not URL, reason="set COCKPIT_URL to run the API suite")


def call(method, path, body=None, token=TOKEN, timeout=None):
    """(status, parsed-or-text). Never raises for an HTTP error status."""
    url = URL.rstrip("/") + path
    data = None
    headers = {"Accept": "application/json"}
    if token:
        headers["X-Cockpit-Token"] = token
    if body is not None:
        data = json.dumps(body).encode()
        headers["Content-Type"] = "application/json"
    req = urllib.request.Request(url, data=data, headers=headers, method=method)
    try:
        with urllib.request.urlopen(req, timeout=timeout or TIMEOUT) as r:
            raw, status = r.read(), r.status
    except urllib.error.HTTPError as e:
        raw, status = e.read(), e.code
    try:
        return status, json.loads(raw.decode() or "null")
    except (ValueError, UnicodeDecodeError):
        return status, raw.decode(errors="replace")


# --------------------------------------------------------------------------
# It is up, and it is guarded.
# --------------------------------------------------------------------------

def test_the_page_is_served():
    status, _ = call("GET", "/")
    assert status == 200


def test_status_answers():
    status, body = call("GET", "/api/status")
    assert status == 200, body
    assert isinstance(body, dict), body


def test_a_write_without_the_token_is_refused():
    """The token guards writes. A cockpit that takes anonymous POSTs is open."""
    status, body = call("POST", "/api/robot/select", {"robot": "pico"}, token="")
    assert status in (401, 403), f"anonymous write returned {status}: {body}"


# --------------------------------------------------------------------------
# What the UI reads before it shows you anything.
# --------------------------------------------------------------------------

def test_robots_are_listed():
    status, body = call("GET", "/api/robots")
    assert status == 200, body
    names = body if isinstance(body, list) else body.get("robots", body)
    assert names, "no robot configs listed"


def test_the_detected_mcu_is_reported():
    status, body = call("GET", "/api/firmware/detect")
    assert status == 200, body
    assert isinstance(body, dict) and body, body


def test_serial_ports_are_reported():
    status, body = call("GET", "/api/serial_ports")
    assert status == 200, body


# --------------------------------------------------------------------------
# The regressions. Each of these was a real failure on 2026-09-19.
# --------------------------------------------------------------------------

def test_saving_hardware_config_regenerates_the_header():
    """The header generator ran as the other user and could not rewrite the file.

    It surfaced as "Hardware config saved, but the header generator failed",
    and Start 1-Click died at step 1/6 on a raw Python traceback. A save that
    reports success while the header did not regenerate is the bug.
    """
    status, body = call("GET", "/api/hardware/config")
    assert status == 200, body

    status, body = call("POST", "/api/hardware/config", body)
    assert status == 200, body
    text = json.dumps(body).lower()
    assert "traceback" not in text, body
    assert not ("saved" in text and "failed" in text), (
        f"saved but the header did not regenerate: {body}"
    )


def test_selecting_a_robot_is_reflected_in_the_config_that_comes_back():
    """The UI kept showing the previous robot's pins after a switch.

    Server-side half of that: whatever /api/robot/select accepts must be what
    /api/config then returns, or the form is being fed a stale robot.
    """
    status, body = call("GET", "/api/robots")
    names = body if isinstance(body, list) else body.get("robots", [])
    names = [n if isinstance(n, str) else n.get("name") for n in names]
    names = [n for n in names if n]
    if len(names) < 2:
        pytest.skip("need two robot configs to prove a switch")

    status, before = call("GET", "/api/config")
    assert status == 200, before
    current = (before or {}).get("robot") or (before or {}).get("name")
    # /api/config answers with the config, where `robot` is a mapping, not the
    # name. Posting that mapping straight back is what turned up the 500 in
    # /api/robot/select; the name is what this test means by `current`.
    if isinstance(current, dict):
        current = current.get("name")
    if not isinstance(current, str):
        current = None
    target = next((n for n in names if n != current), names[0])

    status, body = call("POST", "/api/robot/select", {"robot": target})
    assert status == 200, body

    status, after = call("GET", "/api/config")
    assert status == 200, after
    got = json.dumps(after)
    assert target in got, f"selected {target} but /api/config does not mention it"

    if current:
        call("POST", "/api/robot/select", {"robot": current})


def test_no_endpoint_leaks_a_python_traceback():
    """A traceback in a response body is a 500 wearing a 200's clothes."""
    offenders = []
    for path in ("/api/status", "/api/robots", "/api/config", "/api/sensors",
                 "/api/serial_ports", "/api/firmware/detect", "/api/mcu/ports",
                 "/api/hardware/config", "/api/params", "/api/maps"):
        status, body = call("GET", path)
        text = body if isinstance(body, str) else json.dumps(body)
        if "Traceback (most recent call last)" in text:
            offenders.append(path)
    assert not offenders, f"tracebacks returned by: {offenders}"


def test_a_write_endpoint_answers_junk_with_a_refusal_not_a_crash():
    """A body of the wrong SHAPE is a client error, not a server error.

    `{"robot": {...}}` is valid JSON, and it is what a caller gets by passing
    /api/config's `robot` along. It reached `.strip()` and raised
    AttributeError inside the handler: 500, traceback in the log. Anything
    a client can send must come back as a 4xx.
    """
    bad = []
    for body in ({"robot": {"name": "pico"}}, {"robot": 7}, {"robot": ["pico"]},
                 {"robot": None}, {}, {"name": {"nested": True}}):
        status, answer = call("POST", "/api/robot/select", body)
        if status >= 500:
            bad.append((body, status, answer))
    assert not bad, f"5xx for a malformed body: {bad}"
