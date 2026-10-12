"""COCKPIT_FIRMWARE_MIRROR: a LAN copy of the release assets is tried before GitHub.

A robot on a slow uplink (the UNO Q, a Pi on Wi-Fi) pulled every firmware archive from
GitHub; a bench mirror laid out as <base>/<tag>/<asset> saves that. A mirror that is down
or lacks the tag must cost nothing but a line: the download goes to GitHub as before.
"""
import json
import os
import sys
import urllib.error

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(HERE), "scripts"))
sys.path.insert(0, HERE)

import fetch_prebuilt  # noqa: E402
from test_fetch_prebuilt_cache import _SimResp, _tarball, prebuilt_dir  # noqa: E402,F401

GH = "https://github.com/"


def _commit(out):
    return json.load(open(os.path.join(out, "manifest.json")))["commit"]


def test_the_mirror_is_used_first(prebuilt_dir, monkeypatch):
    monkeypatch.setenv("COCKPIT_FIRMWARE_MIRROR", "http://lan:8099/")
    urls = []

    def opener(url, *a, **k):
        urls.append(url)
        return _SimResp(_tarball("aaaaaaa" if url.startswith("http://lan") else "bbbbbbb"))
    monkeypatch.setattr(fetch_prebuilt.urllib.request, "urlopen", opener)
    out = fetch_prebuilt.fetch("esp32-jazzy", version="rc-20261012.2")
    assert _commit(out) == "aaaaaaa"
    assert urls == ["http://lan:8099/rc-20261012.2/linorobot2-firmware-esp32-jazzy.tar.gz"]


def test_a_mirror_that_fails_falls_through_to_github(prebuilt_dir, monkeypatch, capsys):
    monkeypatch.setenv("COCKPIT_FIRMWARE_MIRROR", "http://down:8099,http://lan:8099")
    urls = []

    def opener(url, *a, **k):
        urls.append(url)
        if url.startswith("http://down"):
            raise urllib.error.URLError(ConnectionRefusedError("refused"))
        if url.startswith("http://lan"):
            raise urllib.error.HTTPError(url, 404, "Not Found", {}, None)
        return _SimResp(_tarball("ccccccc"))
    monkeypatch.setattr(fetch_prebuilt.urllib.request, "urlopen", opener)
    out = fetch_prebuilt.fetch("esp32-jazzy", version="rc-20261012.2")
    assert _commit(out) == "ccccccc"
    assert [u.split("/")[2] for u in urls] == ["down:8099", "lan:8099", "github.com"]
    assert "trying the next source" in capsys.readouterr().out


def test_no_mirror_set_goes_straight_to_github(prebuilt_dir, monkeypatch):
    monkeypatch.delenv("COCKPIT_FIRMWARE_MIRROR", raising=False)
    urls = []
    monkeypatch.setattr(fetch_prebuilt.urllib.request, "urlopen",
                        lambda url, *a, **k: urls.append(url) or _SimResp(_tarball("ddddddd")))
    fetch_prebuilt.fetch("esp32-jazzy", version="rc-20261012.2")
    assert len(urls) == 1 and urls[0].startswith(GH)
