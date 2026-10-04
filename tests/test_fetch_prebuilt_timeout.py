"""A download that times out stops the run and says DOWNLOAD TIMEOUT -- nothing else is used.

On a slow or stalled link the fetch used to treat a timeout like being offline: with a
cached image it flashed that (possibly an older release, with only a WARNING line), and
without one it told the user to build the image -- and the pipeline added "install
PlatformIO or use the pio build image". A timeout is not a missing release; falling back
flashes something other than what was asked for, on exactly the link where nobody is
watching (user, 2026-10-04: "it should not fall-back to build. It should stop and say
download timeout"). Being offline -- no route, DNS, refused -- keeps the cached fallback.
"""
import os
import socket
import sys
import urllib.error

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(REPO_ROOT, "scripts"))
sys.path.insert(0, HERE)

import fetch_prebuilt  # noqa: E402
from test_fetch_prebuilt_cache import _plant_stale, prebuilt_dir  # noqa: E402,F401


@pytest.mark.parametrize("stall", [
    socket.timeout("timed out"),
    urllib.error.URLError(socket.timeout("timed out")),
    TimeoutError("The read operation timed out"),
])
def test_a_stalled_download_stops_and_the_cache_is_not_served(prebuilt_dir, monkeypatch,
                                                              capsys, stall):
    stale = _plant_stale(prebuilt_dir)
    monkeypatch.setattr(fetch_prebuilt, "newest_release_tag", lambda repo: "rc-20260919")

    def slow(*a, **k):
        raise stall
    monkeypatch.setattr(fetch_prebuilt.urllib.request, "urlopen", slow)
    with pytest.raises(SystemExit) as exc:
        fetch_prebuilt.fetch("esp32-jazzy")
    msg = str(exc.value)
    assert "DOWNLOAD TIMEOUT" in msg
    assert "build_prebuilt" not in msg and "PlatformIO" not in msg
    assert "using the cached" not in capsys.readouterr().out
    assert fetch_prebuilt.verify(str(stale))["commit"] == "bb92e60"   # left as it was


def test_a_stalled_release_list_stops_too(prebuilt_dir, monkeypatch):
    _plant_stale(prebuilt_dir)
    monkeypatch.setattr(fetch_prebuilt, "repo_version", lambda: "dev")

    def slow(*a, **k):
        raise urllib.error.URLError(socket.timeout("timed out"))
    monkeypatch.setattr(fetch_prebuilt.urllib.request, "urlopen", slow)
    with pytest.raises(SystemExit) as exc:
        fetch_prebuilt.fetch("esp32-jazzy")
    assert "DOWNLOAD TIMEOUT" in str(exc.value)


def test_offline_still_uses_the_cache(prebuilt_dir, monkeypatch, capsys):
    stale = _plant_stale(prebuilt_dir)
    monkeypatch.setattr(fetch_prebuilt, "newest_release_tag", lambda repo: "rc-20260919")

    def offline(*a, **k):
        raise urllib.error.URLError(OSError("no route to host"))
    monkeypatch.setattr(fetch_prebuilt.urllib.request, "urlopen", offline)
    assert fetch_prebuilt.fetch("esp32-jazzy") == str(stale)
    assert "using the cached" in capsys.readouterr().out


def test_the_pipeline_passes_the_timeout_through_without_build_advice(monkeypatch):
    import one_click_pipeline

    def stalled(profile, *a, **k):
        raise fetch_prebuilt.download_timeout("https://example/x.tar.gz", 120)
    monkeypatch.setattr(one_click_pipeline.fetch_prebuilt, "fetch", stalled)
    with pytest.raises(SystemExit) as exc:
        one_click_pipeline.firmware_source("prebuilt", "esp32", "jazzy")
    assert "DOWNLOAD TIMEOUT" in str(exc.value)
    assert "PlatformIO" not in str(exc.value)
