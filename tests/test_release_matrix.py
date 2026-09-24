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


def test_every_profile_names_its_distro():
    """`pico2-jazzy`, never a bare `pico2`.

    The two images for one board are not interchangeable: board_microros_distro
    picks the precompiled micro-ROS library the firmware links against, and a
    jazzy image will not talk to a lyrical agent. A release that publishes
    `pico2` beside `pico2-lyrical` invites exactly one reading of the first --
    "the normal one" -- and it is the reading that gets a board flashed with an
    image that enumerates over USB and then does nothing useful.

    The PlatformIO envs stay asymmetric ([env:pico2] is pinned to jazzy in
    platformio.ini), which is a build-system detail; it used to leak into the
    published artifact names, and that is what this pins shut.
    """
    import build_prebuilt
    for profile, (_stem, _env, distro, _desc) in build_prebuilt.PROFILES.items():
        assert profile.endswith(f"-{distro}"), (
            f"profile {profile!r} is built for {distro} but does not say so in its name")
        assert profile.count("-") >= 1, f"profile {profile!r} carries no distro suffix"


def test_an_env_resolves_to_the_profile_that_ships_it():
    """fetch_prebuilt maps both spellings, so `fetch_prebuilt.py pico2` still works
    and lands on pico2-jazzy rather than 404ing on an asset name that no longer
    exists."""
    import build_prebuilt
    import fetch_prebuilt
    for profile, (_stem, env, _distro, _desc) in build_prebuilt.PROFILES.items():
        assert fetch_prebuilt.profile_for_env(env) == profile, (
            f"env {env!r} maps to {fetch_prebuilt.profile_for_env(env)!r}, "
            f"but it is {profile!r} that ships it")
        assert fetch_prebuilt.env_for_profile(profile) == env, (
            f"profile {profile!r} maps back to "
            f"{fetch_prebuilt.env_for_profile(profile)!r}, not {env!r}")


def test_the_prune_step_can_reach_the_repo_and_cannot_fail_silently():
    """The stale-asset prune runs in a job with no checkout, so `gh` needs GH_REPO.

    Without it every `gh release view` in that step exits 1 with "could not
    determine what repo to use". The first version sent that to /dev/null and
    fell through to an empty asset list, which is indistinguishable from a
    release that has nothing stale on it: the step printed nothing, deleted
    nothing, and four cuts of rc-20260919 kept the pre-rename pico.tar.gz,
    pico2.tar.gz, esp32.tar.gz and esp32s3.tar.gz attached alongside the
    renamed ones.

    A prune that cannot prune is worse than none, because the release looks
    tidy. So this pins both halves: the step can name the repo, and a listing
    failure that is not "release not found" stops the job.
    """
    text = open(os.path.join(REPO_ROOT, ".github", "workflows", "release.yml")).read()
    m = re.search(r"^      - name: Drop firmware archives.*?(?=^      - )", text, re.M | re.S)
    assert m, "the prune step is gone from release.yml"
    step = m.group(0)

    assert "GH_REPO:" in step, (
        "the prune step has no GH_REPO; its job does not check the repo out, so "
        "gh cannot tell which repository to list"
    )
    assert "2>/dev/null > old.txt" not in step, (
        "the prune step is swallowing the listing error again -- a failed "
        "`gh release view` then reads as an empty release"
    )
    assert "exit 1" in step, "a listing failure must stop the job, not prune nothing"


def test_each_profile_records_its_own_env_address():
    """The manifest tells a flasher where the env block goes, per board.

    `env_partition` was keyed off `#define USE_MCU_ENV` in the header. That
    macro was removed when the env reader stopped being optional, so the key
    silently vanished from all eight manifests -- and flash_mcu.py checks for
    it before writing one. Making it unconditional then introduced the opposite
    bug: every profile got the ESP32 constant, including the RP2 ones, whose
    env lives in the last page of their own flash.
    """
    import build_prebuilt
    import mcu_env
    for profile, (_stem, env, _distro, _desc) in build_prebuilt.PROFILES.items():
        got = build_prebuilt.env_offset_for(env)
        want = f"0x{mcu_env.env_offset(env):X}"
        assert got == want, f"{profile}: manifest would say {got}, board uses {want}"
        if env.startswith("pico"):
            assert got.startswith("0x10"), (
                f"{profile}: {got} is not an RP2 flash address -- that is the ESP32 constant")
        else:
            assert got == "0x3FF000", f"{profile}: {got}"


# --- arm64 -------------------------------------------------------------------
#
# The October robots are a Pi 5 and a Jetson Orin Nano. An amd64-only image does
# not fail slowly on them, it fails with `exec format error`, so every published
# image has to carry linux/arm64 before that run.

def _workflow():
    """release.yml, parsed. The existing checks above read it as text because
    they predate any need for its structure; these need the job graph."""
    import yaml as _yaml
    with open(os.path.join(REPO_ROOT, ".github", "workflows", "release.yml"),
              encoding="utf-8") as fh:
        return _yaml.safe_load(fh)


def _images_job():
    return _workflow()["jobs"]["images"]


def _manifest_job():
    return _workflow()["jobs"]["manifest"]


def test_every_image_is_built_for_both_architectures():
    include = _images_job()["strategy"]["matrix"]["include"]
    by_name = {}
    for row in include:
        by_name.setdefault(row["name"], set()).add(row["arch"])
    assert by_name, "no image matrix"
    for name, arches in sorted(by_name.items()):
        assert arches == {"amd64", "arm64"}, f"{name} builds {sorted(arches)}"


def test_arm64_builds_on_an_arm_runner_not_under_emulation():
    """`platforms: linux/amd64,linux/arm64` on one x86 runner is the one-line
    version, and it does not survive this image: the robot image compiles
    picotool, the micro-ROS agent and the LD19 driver from source, and emulated
    that is hours against GitHub's six-hour ceiling."""
    for row in _images_job()["strategy"]["matrix"]["include"]:
        runner = row["runner"]
        if row["arch"] == "arm64":
            assert runner.endswith("-arm"), f"{row['name']} arm64 on {runner}"
        else:
            assert not runner.endswith("-arm"), f"{row['name']} amd64 on {runner}"


def test_a_build_job_pushes_by_digest_and_never_writes_a_tag():
    """Two architectures pushing the same tag leaves whichever finished last as
    the whole image, silently. An arm64-only :jazzy is worse than no arm64 at
    all, because the bench would pull it."""
    steps = _images_job()["steps"]
    build = [s for s in steps if str(s.get("uses", "")).startswith("docker/build-push-action")]
    assert len(build) == 1, build
    with_ = build[0]["with"]
    assert "push-by-digest=true" in with_["outputs"]
    assert "tags" not in with_, "a per-arch build must not write a tag"
    assert with_["platforms"] == "linux/${{ matrix.arch }}"


def test_the_cache_is_scoped_per_architecture():
    """Sharing one scope makes the two architectures evict each other's layers,
    so every build is cold and the arm64 job is the one that pays."""
    build = [s for s in _images_job()["steps"]
             if str(s.get("uses", "")).startswith("docker/build-push-action")][0]["with"]
    for key in ("cache-from", "cache-to"):
        assert "${{ matrix.arch }}" in build[key], key


def test_the_manifest_job_waits_for_the_builds_and_writes_the_tags():
    job = _manifest_job()
    assert job["needs"] == "images" or "images" in job["needs"]
    body = yaml_dump_steps(job)
    assert "imagetools create" in body
    for name in ("robot-jazzy", "robot-lyrical", "pio"):
        assert any(row["name"] == name for row in job["strategy"]["matrix"]["include"]), name


def test_the_manifest_refuses_to_publish_one_architecture():
    """A manifest with one arch in it looks like a success, and is exactly the
    failure this job exists to prevent."""
    body = yaml_dump_steps(_manifest_job())
    assert 'expected 2 digests' in body
    assert "exit 1" in body


def yaml_dump_steps(job):
    import json
    return json.dumps(job["steps"])
