"""The release matrix must build every prebuilt profile, and CI must build every one's env.

scripts/build_prebuilt.py derives PROFILES from BOARDS x DISTROS, and a release
attaches one tarball per profile; scripts/fetch_prebuilt.py then downloads by
profile name and fails hard when the asset is absent. So a profile that exists
in build_prebuilt.py but not in the release matrix is not a missing nicety -- it
is a `--prebuilt esp32-lyrical` that 404s for every user, with nothing upstream
having gone red.

That is not hypothetical. esp32-lyrical was dropped from both matrices while it
did not fit the classic ESP32's DRAM, and stayed listed in build_prebuilt.py's
profile table and firmware/prebuilt/README.md the whole time. The drift was
invisible because nothing compared the two.

The CI check is deliberately one-directional: CI omits picow_wifi and
pico2w_wifi on purpose (they differ from their siblings by build flags, not by
code paths worth a runner), so it must be a superset of what ships, not an
equal.
"""
import os
import re
import sys

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO_ROOT, "scripts"))


def _matrix_list(path, key):
    """The flow-sequence value of `key:` in a workflow matrix, comments stripped."""
    text = open(os.path.join(REPO_ROOT, ".github", "workflows", path)).read()
    text = re.sub(r"#[^\n]*", "", text)
    m = re.search(rf"^\s*{key}:\s*\[(.*?)\]", text, re.M | re.S)
    assert m, f"no `{key}:` flow sequence in {path}"
    return [v.strip() for v in m.group(1).split(",") if v.strip()]


def _profiles():
    import build_prebuilt
    return set(build_prebuilt.PROFILES)


def test_release_matrix_builds_exactly_the_prebuilt_profiles():
    listed = _matrix_list("release.yml", "profile")
    assert len(listed) == len(set(listed)), f"duplicate profile in the release matrix: {listed}"
    missing = _profiles() - set(listed)
    extra = set(listed) - _profiles()
    assert not missing, (
        f"build_prebuilt.py produces {sorted(missing)} but the release matrix does not build "
        f"them, so `fetch_prebuilt.py {sorted(missing)[0]}` will 404 against the release.")
    assert not extra, (
        f"the release matrix builds {sorted(extra)}, which build_prebuilt.py does not know how "
        f"to produce; that job will fail with 'unknown profile'.")


def test_ci_builds_the_pio_env_behind_every_shipped_profile():
    import build_prebuilt
    ci_envs = set(_matrix_list("ci.yml", "env"))
    for profile, (_cfg, env, _distro, _desc) in build_prebuilt.PROFILES.items():
        assert env in ci_envs, (
            f"profile {profile} ships PlatformIO env {env}, which ci.yml never builds. A link "
            f"error in it would first appear during a release rather than on the pull request.")


def test_esp32_lyrical_is_in_both_matrices():
    """The tightest env in the tree: 116308 bytes of a 124580-byte dram0_0_seg. It is
    here by name because it is the one that was removed once already, and because it
    is the only env where an added publisher or global turns a build red rather than
    merely larger."""
    assert "esp32-lyrical" in _matrix_list("release.yml", "profile")
    assert "esp32_lyrical" in _matrix_list("ci.yml", "env")
