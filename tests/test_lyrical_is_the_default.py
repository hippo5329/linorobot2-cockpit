"""Lyrical is the default ROS 2 distro; jazzy stays supported and gated.

Lyrical Luth (May 2026, Ubuntu 26.04) is the current LTS, so a robot set up with no distro
named gets it: docker-compose's ROS_DISTRO default (which also picks the image tag), and
every fallback in the scripts, launchers and web UI for when nothing names one.

Two things are NOT defaults and must stay jazzy: the firmware's env naming, where jazzy is
the unsuffixed env (`pico2`) and lyrical the suffixed one (`pico2_lyrical`) -- renaming it
would rename every release asset -- and the git branch fallback for upstream repos that
have a jazzy branch and no lyrical one yet.
"""
import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "scripts"))

FALLBACK = re.compile(
    r'"ROS_DISTRO",\s*"jazzy"|ROS_DISTRO:-jazzy|distro(:\s*(Optional\[str\]|str))?\s*=\s*"jazzy"'
    r'|\|\|\s*"jazzy"|or\s+"jazzy"|get\("distro",\s*"jazzy"\)|"distro",\s*"jazzy"\)')


def _files():
    for sub in ("scripts", "launchers", "web/backend", "web/frontend"):
        for dirpath, _, names in os.walk(os.path.join(ROOT, sub)):
            for n in names:
                if n.endswith((".py", ".js")):
                    yield os.path.join(dirpath, n)
    yield os.path.join(ROOT, "docker-compose.yml")


def test_compose_defaults_to_lyrical():
    compose = open(os.path.join(ROOT, "docker-compose.yml"), encoding="utf-8").read()
    assert "${ROS_DISTRO:-lyrical}" in compose
    assert "ROS_DISTRO:-jazzy" not in compose


def test_no_fallback_names_jazzy():
    hits = []
    for path in _files():
        if path.endswith("verify_nav2_params.py"):      # checks both distros by name
            continue
        for i, line in enumerate(open(path, encoding="utf-8"), 1):
            if FALLBACK.search(line):
                hits.append(f"{os.path.relpath(path, ROOT)}:{i}: {line.strip()}")
    assert not hits, "jazzy fallbacks left:\n" + "\n".join(hits)


def test_the_ui_offers_lyrical_first():
    html = open(os.path.join(ROOT, "web", "frontend", "index.html"), encoding="utf-8").read()
    for sel in ("hdr-distro-select", "nav2-editor-distro", "cfg-ros-distro"):
        block = html[html.index(f'id="{sel}"'):]
        first = re.search(r'<option value="([a-z]+)"', block).group(1)
        assert first == "lyrical", f"{sel} lists {first} first"


def test_the_firmware_naming_is_unchanged():
    import fetch_prebuilt
    assert fetch_prebuilt.profile_for_env("pico2") == "pico2-jazzy"
    assert fetch_prebuilt.profile_for_env("pico2_lyrical") == "pico2-lyrical"
    assert fetch_prebuilt.env_for_profile("esp32-jazzy") == "esp32"


def test_no_reference_design_pins_a_distro():
    ref = os.path.join(ROOT, "config", "reference")
    pinned = [n for n in os.listdir(ref) if n.endswith(".yaml")
              and re.search(r"^ros_distro:", open(os.path.join(ref, n), encoding="utf-8").read(), re.M)]
    assert not pinned, f"a distro is the computer's, not the design's: {pinned}"
