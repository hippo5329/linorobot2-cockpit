"""A cached prebuilt must never be served in place of the current release.

The first cut of rc-20260919 was re-cut in place: the tag was force-moved and
its assets replaced under the same name. A bench that had fetched during
rc-20260918 went on flashing that firmware afterwards -- fetch_prebuilt's cache
hit needed only `manifest.json` to exist, and verify() compares the cached
files against the cached manifest, so a stale directory is always internally
consistent and always passes. It printed nothing on the hit path either, so the
pipeline reported a clean flash of a week-old image.

Keying the cache on the release NAME does not fix it, precisely because a
candidate tag is re-cut in place. These tests pin the behaviour that does: the
archive is downloaded whenever the network allows, and the cache is only an
offline fallback, which says so.
"""
import json
import os
import sys

import pytest

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO_ROOT, "scripts"))

import fetch_prebuilt  # noqa: E402

def _tarball(commit):
    """A minimal, well-formed profile archive."""
    import hashlib, io, tarfile, time
    payload = f"firmware for {commit}".encode()
    manifest = {
        "profile": "esp32", "ros_distro": "jazzy", "commit": commit,
        "built": "2026-09-19T06:37:46Z", "description": "test image",
        "files": [{"name": "firmware.bin", "offset": "0x10000", "tool": "esptool",
                   "size": len(payload),
                   "sha256": hashlib.sha256(payload).hexdigest()}],
    }
    buf = io.BytesIO()
    with tarfile.open(fileobj=buf, mode="w:gz") as tar:
        for name, blob in (("firmware.bin", payload),
                           ("manifest.json", json.dumps(manifest).encode())):
            info = tarfile.TarInfo(name)
            info.size = len(blob)
            info.mtime = int(time.time())
            tar.addfile(info, io.BytesIO(blob))
    return buf.getvalue()


@pytest.fixture
def prebuilt_dir(tmp_path, monkeypatch):
    d = tmp_path / "prebuilt"
    d.mkdir()
    monkeypatch.setattr(fetch_prebuilt, "PREBUILT_DIR", str(d))
    return d


def _plant_stale(prebuilt_dir, commit="bb92e60", release="rc-20260918"):
    """A cache exactly as a real one looks: self-consistent, from an older release."""
    import hashlib, tarfile, io
    p = prebuilt_dir / "esp32"
    p.mkdir()
    with tarfile.open(fileobj=io.BytesIO(_tarball(commit)), mode="r:gz") as tar:
        for m in tar.getmembers():
            (p / m.name).write_bytes(tar.extractfile(m).read())
    if release:
        (p / fetch_prebuilt.RELEASE_STAMP).write_text(release + "\n")
    # It must pass the integrity check -- that is the whole point.
    assert fetch_prebuilt.verify(str(p))["commit"] == commit
    return p


def test_stale_cache_is_replaced_not_served(prebuilt_dir, monkeypatch, capsys):
    p = _plant_stale(prebuilt_dir)
    monkeypatch.setattr(fetch_prebuilt, "newest_release_tag", lambda repo: "rc-20260919")
    monkeypatch.setattr(fetch_prebuilt.urllib.request, "urlopen",
                        lambda *a, **k: _FakeResp(_tarball("dcfb4b2")))
    out = fetch_prebuilt.fetch("esp32", version="dev")
    assert json.load(open(os.path.join(out, "manifest.json")))["commit"] == "dcfb4b2"
    assert fetch_prebuilt.cached_release(out) == "rc-20260919"


def test_cache_from_the_same_tag_is_still_replaced(prebuilt_dir, monkeypatch):
    """The re-cut case: same tag name, different contents."""
    _plant_stale(prebuilt_dir, commit="46b184a", release="rc-20260919")
    monkeypatch.setattr(fetch_prebuilt, "newest_release_tag", lambda repo: "rc-20260919")
    monkeypatch.setattr(fetch_prebuilt.urllib.request, "urlopen",
                        lambda *a, **k: _FakeResp(_tarball("dcfb4b2")))
    out = fetch_prebuilt.fetch("esp32", version="dev")
    assert json.load(open(os.path.join(out, "manifest.json")))["commit"] == "dcfb4b2", \
        "a tag re-cut in place must not be served from cache"


def test_offline_falls_back_to_the_cache_and_says_so(prebuilt_dir, monkeypatch, capsys):
    _plant_stale(prebuilt_dir)
    def boom(repo):
        raise OSError("no route to host")
    monkeypatch.setattr(fetch_prebuilt, "newest_release_tag", boom)
    out = fetch_prebuilt.fetch("esp32", version="dev")
    msg = capsys.readouterr().out
    assert "WARNING" in msg and "may not be the current release" in msg
    assert json.load(open(os.path.join(out, "manifest.json")))["commit"] == "bb92e60"


def test_offline_without_a_cache_is_an_error(prebuilt_dir, monkeypatch):
    def boom(repo):
        raise OSError("no route to host")
    monkeypatch.setattr(fetch_prebuilt, "newest_release_tag", boom)
    with pytest.raises(SystemExit) as exc:
        fetch_prebuilt.fetch("esp32", version="dev")
    assert "not cached" in str(exc.value)


class _FakeResp:
    def __init__(self, blob):
        self._blob = blob
    def read(self):
        return self._blob
    def __enter__(self):
        return self
    def __exit__(self, *a):
        return False
