"""The ESP32 builds must garbage-collect unused sections, and must not lose the flags.

The espressif32 platform sets -ffunction-sections/-fdata-sections/--gc-sections
only for its BARE-METAL framework (builder/frameworks/_bare.py). The arduino
framework gets none of them, so every global in a translation unit shares one
.bss section and an unused one cannot be dropped -- which is how test_acc.cpp
and test_motors.cpp carried 3232 bytes of never-referenced message structs into
every image, on a chip whose entire static segment is 124580 bytes. The
raspberrypi platform sets all three itself, which is why the pico boards never
showed the problem.

The second test is the one that matters most. PlatformIO's `extends` does NOT
merge build_flags: redefining the key in a concrete env REPLACES the base's,
silently and with no warning. [env:esp32] did exactly that, so flags added to
[base_esp32] would have been dropped and the collection would simply not have
happened -- a fix that looks applied, changes nothing, and is only caught by
measuring the binary.
"""
import os
import re

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FIRMWARE = os.path.join(REPO_ROOT, "firmware")
NEEDED = ("-ffunction-sections", "-fdata-sections", "-Wl,--gc-sections")


def _section(text, name):
    """The raw body of one [section] from an ini, without parsing interpolation."""
    m = re.search(rf"^\[{re.escape(name)}\]\s*$(.*?)(?=^\[|\Z)", text, re.M | re.S)
    return m.group(1) if m else None


def _base_ini():
    return open(os.path.join(FIRMWARE, "common", "platformio_base.ini")).read()


def _project_ini():
    return open(os.path.join(FIRMWARE, "platformio.ini")).read()


def test_esp32_bases_request_section_gc():
    text = _base_ini()
    for base in ("base_esp32", "base_esp32s3"):
        body = _section(text, base)
        assert body, f"[{base}] not found"
        for flag in NEEDED:
            assert flag in body, (
                f"[{base}] is missing {flag}. The arduino framework on espressif32 sets "
                f"none of the section flags, and --gc-sections without -fdata-sections "
                f"collects nothing.")


def test_concrete_esp32_envs_do_not_drop_their_base_flags():
    text = _project_ini()
    for env, base in (("env:esp32", "base_esp32"), ("env:esp32s3", "base_esp32s3")):
        body = _section(text, env)
        assert body, f"[{env}] not found"
        if "build_flags" not in body:
            continue          # inherits wholesale; nothing to lose
        assert f"${{{base}.build_flags}}" in body, (
            f"[{env}] redefines build_flags without ${{{base}.build_flags}}. `extends` "
            f"REPLACES the key rather than merging it, so every flag on [{base}] -- the "
            f"section-GC flags included -- would be silently discarded.")


def test_the_lyrical_envs_inherit_from_the_jazzy_ones():
    """esp32_lyrical/esp32s3_lyrical carry only a distro override, so whatever the
    jazzy env gets, they get. If one ever grows its own build_flags, it needs the
    same ${...} reference or it will quietly lose section GC on that board alone."""
    text = _project_ini()
    for env in ("env:esp32_lyrical", "env:esp32s3_lyrical"):
        body = _section(text, env)
        assert body, f"[{env}] not found"
        if "build_flags" in body:
            assert "${" in body, f"[{env}] defines build_flags without inheriting any"
