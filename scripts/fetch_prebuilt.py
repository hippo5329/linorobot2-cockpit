#!/usr/bin/env python3
"""
fetch_prebuilt.py — download a ready-to-flash firmware image from a release.

    python3 scripts/fetch_prebuilt.py pico2              # jazzy image for the pico2 env
    python3 scripts/fetch_prebuilt.py pico2_lyrical      # the lyrical one
    python3 scripts/fetch_prebuilt.py esp32 --version 20260918

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


def profile_for_env(pio_env: str) -> str:
    """PlatformIO env -> release profile: `pico2` -> `pico2`, `pico2_lyrical` -> `pico2-lyrical`."""
    env = (pio_env or "").strip()
    if "_" in env:
        board, distro = env.split("_", 1)
        return board if distro == DEFAULT_DISTRO else f"{board}-{distro}"
    return env


def env_for_profile(profile: str) -> str:
    if "-" in profile:
        board, distro = profile.split("-", 1)
        return f"{board}_{distro}"
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


def fetch(profile: str, version: str = None, repo: str = DEFAULT_REPO,
          force: bool = False, quiet: bool = False) -> str:
    """Ensure firmware/prebuilt/<profile>/ exists and is verified; return its path."""
    profile_dir = os.path.join(PREBUILT_DIR, profile)
    if not force and os.path.isfile(os.path.join(profile_dir, "manifest.json")):
        verify(profile_dir)
        return profile_dir

    version = version or repo_version()
    if is_floating(version):
        resolved = newest_release_tag(repo)
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
            raise SystemExit(
                f"fetch_prebuilt: could not download {url}: {exc}\n"
                f"  Check the release name (--version), or build the image yourself:\n"
                f"    python3 scripts/build_prebuilt.py {profile}")
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
    shutil.rmtree(profile_dir, ignore_errors=True)
    os.rename(tmp_dir, profile_dir)
    if not quiet:
        print(f"[fetch_prebuilt] {profile}: {manifest.get('description', '')} "
              f"(built {manifest.get('built')}, commit {manifest.get('commit')}) -> {profile_dir}")
    return profile_dir


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("profile", help="release profile (pico2, esp32-lyrical, …) or PlatformIO env (pico2_lyrical)")
    ap.add_argument("--version", default=None, help="release tag (default: the VERSION file; '-dev' means latest)")
    ap.add_argument("--repo", default=DEFAULT_REPO, help=f"GitHub owner/name (default: {DEFAULT_REPO})")
    ap.add_argument("--force", action="store_true", help="re-download even if the profile is present")
    a = ap.parse_args()
    profile = profile_for_env(a.profile) if "_" in a.profile else a.profile
    fetch(profile, a.version, a.repo, a.force)


if __name__ == "__main__":
    main()
