#!/usr/bin/env python3
"""
fetch_prebuilt.py — download a ready-to-flash firmware image from a release.

    python3 scripts/fetch_prebuilt.py pico2-jazzy        # by release profile
    python3 scripts/fetch_prebuilt.py pico2-lyrical      # the other distro
    python3 scripts/fetch_prebuilt.py pico2              # by PlatformIO env; jazzy
    python3 scripts/fetch_prebuilt.py esp32-jazzy --version 20260918

Every profile names its distro. A PlatformIO env still may not -- [env:pico2] is
pinned to jazzy in platformio.ini -- so an env is accepted here and mapped.

Profiles are published as release assets named
`linorobot2-firmware-<profile>.tar.gz`, one per board per ROS 2 distro
(scripts/build_prebuilt.py builds them; .github/workflows/release.yml uploads
them). The archive unpacks into firmware/prebuilt/<profile>/ and every file is
checked against the sha256 in its manifest.json before the directory is used.

Which release: `--version`, else the `VERSION` file at the repo root. A version
ending in `-dev`, and any version with no release of its own, resolve to the
newest published release -- prereleases included, so a checkout during an
`rc-YYYYMMDD` freeze finds the candidate's assets instead of 404ing.
"""
import argparse
import hashlib
import io
import json
import os
import shutil
import sys
import tarfile
import urllib.error
import urllib.request

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PREBUILT_DIR = os.path.join(REPO_ROOT, "firmware", "prebuilt")
DEFAULT_REPO = os.environ.get("COCKPIT_RELEASE_REPO", "hippo5329/linorobot2-cockpit")
DEFAULT_DISTRO = "jazzy"
# Names the release a cached profile came from; absent in caches written before
# version-aware caching, which are therefore always re-downloaded.
RELEASE_STAMP = ".release"


def profile_for_env(pio_env: str) -> str:
    """PlatformIO env -> release profile.

    `pico2` -> `pico2-jazzy`, `pico2_lyrical` -> `pico2-lyrical`. The env names
    are asymmetric because [env:pico2] is pinned to jazzy in platformio.ini; the
    profile names are not, because a release asset that does not say which distro
    it is for is a trap -- the two images are not interchangeable and a board
    flashed with the wrong one enumerates and then does nothing useful.
    """
    env = (pio_env or "").strip()
    # The RP2 releases are built from the W envs (picow, pico2w) but ship under
    # the board names users ask for: one image runs on both, so `pico2` and
    # `pico2w` must land on the same asset rather than 404 on one that was
    # never uploaded.
    env = {"picow": "pico", "pico2w": "pico2",
           "picow_lyrical": "pico_lyrical",
           "pico2w_lyrical": "pico2_lyrical"}.get(env, env)
    if "_" in env:
        board, distro = env.split("_", 1)
        return f"{board}-{distro}"
    return f"{env}-{DEFAULT_DISTRO}" if env else env


def env_for_profile(profile: str) -> str:
    """Release profile -> the PlatformIO env its image was built from.

    Asks the profile table rather than un-munging the name, because the two
    stopped matching when the RP2 releases moved to the W envs: `pico2-jazzy`
    is the asset users ask for, and `pico2w` is what builds it. Falls back to
    the name split when build_prebuilt cannot be imported (it pulls in
    PlatformIO paths, and this module is used on robots that have none).
    """
    try:
        import build_prebuilt
        entry = build_prebuilt.PROFILES.get(profile)
        if entry:
            return entry[1]
    except Exception:
        pass
    if "-" in profile:
        board, distro = profile.split("-", 1)
        return board if distro == DEFAULT_DISTRO else f"{board}_{distro}"
    return profile


def repo_version() -> str:
    try:
        with open(os.path.join(REPO_ROOT, "VERSION")) as fh:
            return fh.read().strip()
    except OSError:
        return "dev"


def asset_name(profile: str) -> str:
    return f"linorobot2-firmware-{profile}.tar.gz"


def asset_url(profile: str, version: str, repo: str) -> str:
    return f"https://github.com/{repo}/releases/download/{version}/{asset_name(profile)}"


def is_floating(version: str) -> bool:
    """A version that does not name a release of its own."""
    return not version or version.endswith("-dev") or version in ("dev", "latest")


def newest_release_tag(repo: str) -> str:
    """
    The newest published release, prereleases included.

    `releases/latest` cannot be used: GitHub excludes prereleases from it, so
    during a candidate freeze -- the only release being `rc-YYYYMMDD` -- it
    answers 404 and every prebuilt fetch fails.
    """
    url = f"https://api.github.com/repos/{repo}/releases?per_page=20"
    req = urllib.request.Request(url, headers={"Accept": "application/vnd.github+json"})
    with urllib.request.urlopen(req, timeout=60) as resp:
        releases = json.load(resp)
    for rel in releases:
        if not rel.get("draft"):
            return rel["tag_name"]
    raise SystemExit(f"fetch_prebuilt: {repo} has published no release yet.\n"
                     f"  Build the image yourself: python3 scripts/build_prebuilt.py <profile>")


def verify(profile_dir: str) -> dict:
    manifest_path = os.path.join(profile_dir, "manifest.json")
    with open(manifest_path) as fh:
        manifest = json.load(fh)
    for entry in manifest["files"]:
        path = os.path.join(profile_dir, entry["name"])
        if not os.path.isfile(path):
            raise SystemExit(f"fetch_prebuilt: {entry['name']} missing from {profile_dir}")
        digest = hashlib.sha256(open(path, "rb").read()).hexdigest()
        if digest != entry["sha256"]:
            raise SystemExit(f"fetch_prebuilt: sha256 mismatch for {entry['name']} "
                             f"(got {digest[:12]}…, manifest says {entry['sha256'][:12]}…)")
    return manifest


def cached_release(profile_dir: str) -> str:
    """Which release the cached profile came from, or '' if it predates the stamp."""
    try:
        with open(os.path.join(profile_dir, RELEASE_STAMP)) as fh:
            return fh.read().strip()
    except OSError:
        return ""


def fetch(profile: str, version: str = None, repo: str = DEFAULT_REPO,
          force: bool = False, quiet: bool = False) -> str:
    """
    Ensure firmware/prebuilt/<profile>/ holds the CURRENT release image, verified.

    The archive is re-downloaded on every call when the network allows; the
    cached copy is an offline fallback, not a fast path. It used to be the
    other way round -- a hit needed only `manifest.json` to exist -- and that
    is unfixable by keying on the release name, because a candidate tag gets
    re-cut in place: `rc-20260919` was force-moved and its assets replaced
    under the same name on 2026-09-19, so name and content had already come
    apart. verify() cannot catch it either: it checks the cached files against
    the cached manifest, so a stale directory is always self-consistent. At
    165-690 KB an unconditional download is cheaper than any of the ways of
    being clever about it, and the failure it removes -- flashing last week's
    firmware and being told nothing -- costs an afternoon to find.
    """
    profile_dir = os.path.join(PREBUILT_DIR, profile)
    have_cache = os.path.isfile(os.path.join(profile_dir, "manifest.json"))

    def fall_back(why: str) -> str:
        if not have_cache:
            raise SystemExit(
                f"fetch_prebuilt: cannot reach {repo} ({why}) and {profile} is not "
                f"cached.\n  Build the image yourself: python3 scripts/build_prebuilt.py {profile}")
        manifest = verify(profile_dir)
        print(f"[fetch_prebuilt] WARNING: cannot reach {repo} ({why}); using the cached "
              f"{profile} from {cached_release(profile_dir) or 'an unrecorded release'} "
              f"(built {manifest.get('built')}, commit {manifest.get('commit')}) -- "
              f"it may not be the current release")
        return profile_dir

    version = version or repo_version()
    if is_floating(version):
        try:
            resolved = newest_release_tag(repo)
        except SystemExit:
            raise
        except Exception as exc:
            return fall_back(str(exc))
        if not quiet:
            print(f"[fetch_prebuilt] {version or 'dev'} -> newest release {resolved}")
        version = resolved

    blob = None
    tried = []
    for candidate in (version, None):
        if candidate is None:
            # VERSION names a release that is not published (typically the tree is
            # on `20260918` while only the candidate `rc-20260918` exists). Fall
            # back to the newest release rather than failing the pipeline.
            candidate = newest_release_tag(repo)
            if candidate in tried:
                break
            if not quiet:
                print(f"[fetch_prebuilt] no release {version}; falling back to {candidate}")
        url = asset_url(profile, candidate, repo)
        tried.append(candidate)
        if not quiet:
            print(f"[fetch_prebuilt] {url}")
        try:
            with urllib.request.urlopen(url, timeout=120) as resp:
                blob = resp.read()
            break
        except urllib.error.HTTPError as exc:
            if exc.code != 404:
                raise SystemExit(f"fetch_prebuilt: could not download {url}: {exc}")
            last = exc
        except Exception as exc:
            return fall_back(f"{url}: {exc}")
    if blob is None:
        raise SystemExit(
            f"fetch_prebuilt: {asset_name(profile)} is in no release of {repo} "
            f"(tried {', '.join(tried)}): {last}\n"
            f"  Check the release name (--version), or build the image yourself:\n"
            f"    python3 scripts/build_prebuilt.py {profile}")

    tmp_dir = profile_dir + ".tmp"
    shutil.rmtree(tmp_dir, ignore_errors=True)
    os.makedirs(tmp_dir, exist_ok=True)
    with tarfile.open(fileobj=io.BytesIO(blob), mode="r:gz") as tar:
        for member in tar.getmembers():
            # Flat archive: refuse anything that would escape the profile directory.
            name = os.path.basename(member.name)
            if not member.isfile() or not name or name.startswith("."):
                continue
            with tar.extractfile(member) as src, open(os.path.join(tmp_dir, name), "wb") as dst:
                shutil.copyfileobj(src, dst)
    if not os.path.isfile(os.path.join(tmp_dir, "manifest.json")):
        shutil.rmtree(tmp_dir, ignore_errors=True)
        raise SystemExit("fetch_prebuilt: the archive carries no manifest.json")
    manifest = verify(tmp_dir)
    # Written inside the profile so it travels with the directory, and after
    # verify() so a half-extracted archive never looks like a good cache.
    with open(os.path.join(tmp_dir, RELEASE_STAMP), "w") as fh:
        fh.write(tried[-1] + "\n")
    # Swapping the directory in must not depend on being the user who wrote the
    # last one. The one-click pipeline runs as container-root and the cockpit's
    # backend as the container user, so a cached profile is routinely owned by
    # the other of the two -- and deleting entries inside it needs write
    # permission on THAT directory, which the other user does not have.
    # rmtree(ignore_errors=True) then does nothing at all, quietly, and the
    # rename below dies on a non-empty destination:
    #
    #     File "/ws/scripts/fetch_prebuilt.py", line 228, in fetch
    #     ❌ Pipeline halted: Execution failed (exit code 1)
    #
    # which is what the web UI showed the user, as a traceback, at step 2/6.
    # Renaming the old directory ASIDE needs write permission on the PARENT,
    # which both users have; clearing it afterwards is best effort.
    stale = None
    if os.path.exists(profile_dir):
        candidate = f"{profile_dir}.stale.{os.getpid()}"
        shutil.rmtree(candidate, ignore_errors=True)
        try:
            os.rename(profile_dir, candidate)
            stale = candidate
        except OSError:
            # Same-user case, or a parent we cannot write: fall back to the
            # straightforward removal and let a real failure surface below.
            shutil.rmtree(profile_dir, ignore_errors=True)
    os.rename(tmp_dir, profile_dir)
    if stale:
        shutil.rmtree(stale, ignore_errors=True)
    if not quiet:
        print(f"[fetch_prebuilt] {profile}: {manifest.get('description', '')} "
              f"(built {manifest.get('built')}, commit {manifest.get('commit')}) -> {profile_dir}")
    return profile_dir


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("profile", help="release profile (pico2-jazzy, esp32-lyrical, …) "
                                    "or PlatformIO env (pico2, pico2_lyrical)")
    ap.add_argument("--version", default=None, help="release tag (default: the VERSION file; '-dev' means latest)")
    ap.add_argument("--repo", default=DEFAULT_REPO, help=f"GitHub owner/name (default: {DEFAULT_REPO})")
    ap.add_argument("--force", action="store_true", help="re-download even if the profile is present")
    a = ap.parse_args()
    # A profile always carries its distro after a hyphen; anything without one is
    # a PlatformIO env and gets mapped. Keying on "_" instead would pass a bare
    # `pico2` straight through as a profile name, which no longer exists.
    profile = a.profile if "-" in a.profile else profile_for_env(a.profile)
    fetch(profile, a.version, a.repo, a.force)


if __name__ == "__main__":
    main()
