"""Replacing a cached profile must not depend on who wrote the last one.

The one-click pipeline runs as container-root and the cockpit's backend as the
container user. Deleting entries inside a directory needs write permission on
THAT directory, so whichever user cached the profile last leaves it
undeletable by the other -- rmtree(ignore_errors=True) then does nothing,
quietly, and os.rename() dies on a non-empty destination. The web UI showed
that to the user as a traceback out of fetch_prebuilt, halting Start 1-Click
at step 2/6.
"""
import os
import shutil
import stat
import sys

import pytest

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO_ROOT, "scripts"))


def _swap(profile_dir, tmp_dir):
    """The replace step of fetch(), lifted so it can be exercised directly."""
    stale = None
    if os.path.exists(profile_dir):
        candidate = f"{profile_dir}.stale.{os.getpid()}"
        shutil.rmtree(candidate, ignore_errors=True)
        try:
            os.rename(profile_dir, candidate)
            stale = candidate
        except OSError:
            shutil.rmtree(profile_dir, ignore_errors=True)
    os.rename(tmp_dir, profile_dir)
    if stale:
        shutil.rmtree(stale, ignore_errors=True)


def test_replaces_a_cache_this_user_cannot_delete_into(tmp_path):
    if os.geteuid() == 0:
        pytest.skip("root ignores the mode bits this test depends on")

    profile = tmp_path / "pico-jazzy"
    profile.mkdir()
    (profile / "firmware.uf2").write_bytes(b"old")
    # Read+execute only: the directory can be renamed (parent is writable) but
    # its entries cannot be removed. This is the foreign-owner case.
    os.chmod(profile, stat.S_IRUSR | stat.S_IXUSR)

    tmp = tmp_path / "pico-jazzy.tmp"
    tmp.mkdir()
    (tmp / "firmware.uf2").write_bytes(b"new")

    _swap(str(profile), str(tmp))

    assert (profile / "firmware.uf2").read_bytes() == b"new"
    assert not tmp.exists()


def test_plain_replacement_still_works(tmp_path):
    profile = tmp_path / "pico-jazzy"
    profile.mkdir()
    (profile / "firmware.uf2").write_bytes(b"old")
    tmp = tmp_path / "pico-jazzy.tmp"
    tmp.mkdir()
    (tmp / "firmware.uf2").write_bytes(b"new")

    _swap(str(profile), str(tmp))

    assert (profile / "firmware.uf2").read_bytes() == b"new"
    leftovers = [p.name for p in tmp_path.iterdir() if p.name != "pico-jazzy"]
    assert leftovers == [], leftovers


def test_first_fetch_with_no_cache(tmp_path):
    profile = tmp_path / "pico-jazzy"
    tmp = tmp_path / "pico-jazzy.tmp"
    tmp.mkdir()
    (tmp / "firmware.uf2").write_bytes(b"new")

    _swap(str(profile), str(tmp))

    assert (profile / "firmware.uf2").read_bytes() == b"new"


def test_fetch_prebuilt_uses_the_rename_aside():
    src = open(os.path.join(REPO_ROOT, "scripts", "fetch_prebuilt.py")).read()
    assert "os.rename(profile_dir, candidate)" in src, (
        "fetch_prebuilt went back to deleting the cache in place, which fails "
        "for whichever user did not write it"
    )
