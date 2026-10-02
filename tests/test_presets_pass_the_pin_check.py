"""Every UI reference preset passes the same pin check the server applies on save.

The "ESP32 DevKit (4WD Mecanum + WiFi UDP + BNO085)" preset put the battery
divider on GPIO 36, which its own encoder2 already used. Applying it saved a
config the pin catalogue refuses ("GPIO 36 is used by encoder2.pin_a,
battery.pin"), so the header was not built and the page jumped to the Pin
Matrix -- found by a press-every-control browser pass, not by any test. The
presets are a JavaScript literal (web/frontend/app-presets.js); node evaluates
just that literal, and each preset is checked exactly as a saved config is.
"""
import json
import os
import shutil
import subprocess
import sys

import pytest

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO, "scripts"))
import pin_catalog  # noqa: E402

NODE = shutil.which("node") or shutil.which("nodejs")
HARNESS = r"""
const src = require("fs").readFileSync(process.env.PRESETS_JS, "utf8");
const a = src.indexOf("const REFERENCE_DESIGNS = ");
const b = src.indexOf("\n};\n", a) + 3;
const REFERENCE_DESIGNS = eval("(" + src.slice(a + "const REFERENCE_DESIGNS = ".length, b - 1) + ")");
process.stdout.write(JSON.stringify(REFERENCE_DESIGNS));
"""


def _presets():
    env = dict(os.environ, PRESETS_JS=os.path.join(REPO, "web", "frontend", "app-presets.js"))
    out = subprocess.run([NODE, "-e", HARNESS], env=env,
                         capture_output=True, text=True, timeout=30, check=True).stdout
    return [d for designs in json.loads(out).values() for d in designs]


def _as_config(d):
    p = d.get("pins") or {}
    pins = {"led": p.get("led", -1), "i2c": p.get("i2c") or {},
            "battery": {"pin": p.get("battery", -1)},
            "sonar": {"trigger": (p.get("sonar") or {}).get("trig", -1),
                      "echo": (p.get("sonar") or {}).get("echo", -1)}}
    for n in range(1, 5):
        pins[f"motor{n}"] = p.get(f"motor{n}") or {}
        e = p.get(f"encoder{n}") or {}
        pins[f"encoder{n}"] = {"pin_a": e.get("a", -1), "pin_b": e.get("b", -1)}
    return {"base_controller": {"name": d["mcu"], "mcu": d["mcu"],
                                "driver_type": d.get("driver", ""), "pins": pins}}


@pytest.mark.skipif(not NODE, reason="node evaluates the preset literal")
def test_every_preset_passes_the_pin_check_it_will_be_saved_through():
    presets = _presets()
    assert len(presets) >= 9, f"read {len(presets)} presets -- the literal moved?"
    bad = {}
    for d in presets:
        errors = [msg for level, msg in pin_catalog.check_config(_as_config(d)) if level == "error"]
        if errors:
            bad[d["id"]] = errors
    assert not bad, f"presets the server would refuse on save: {bad}"
