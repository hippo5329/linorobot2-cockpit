"""Reference designs are real robots (user, 2026-10-06, restating an older rule).

The user selects any reference design whatever MCU is detected, or with none attached.
Only actions that need the board -- Flash, 1-Click, Bringup -- need its MCU, and they are
refused with a warning when it is a different one OR none at all. Nothing swaps a real
robot for the Sim MCU: a 1-Click pressed on a design with no board ran `bare_sim`.
Generated robots (bare modules, `bare_*`) keep the Sim MCU fallback.
"""
import glob
import json
import os
import shutil
import subprocess
import sys

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "scripts"))
NODE = shutil.which("node") or shutil.which("nodejs")

import one_click_pipeline as ocp  # noqa: E402

DESIGNS = sorted(os.path.basename(p)[:-len("_config.yaml")]
                 for p in glob.glob(os.path.join(ROOT, "config", "reference", "*_config.yaml")))


def test_every_reference_design_is_a_real_robot():
    assert DESIGNS, "no reference designs found"
    for d in DESIGNS:
        assert not ocp.is_generated_robot(d), d


def test_bare_robots_are_generated():
    for name in ("bare_sim", "bare_pico2", "bare_esp32s3_cdc", "bare_unoq"):
        assert ocp.is_generated_robot(name), name


def test_the_pipeline_refuses_a_real_robot_with_no_board_rather_than_run_the_sim_mcu():
    src = open(os.path.join(ROOT, "scripts", "one_click_pipeline.py")).read()
    a = src.index("no_board_attached(controller_cfg):\n")
    block = src[a:a + 1500]
    assert "if not is_generated_robot(robot_name):" in block
    assert block.index("raise SystemExit") < block.index("controller, sim_mcu = SIM_MCU, True")


GUARD = r"""
const src = require("fs").readFileSync(process.argv[2], "utf8");
const pick = (start) => { const a = src.indexOf(start); return src.slice(a, src.indexOf("\n}\n", a) + 3); };
const answer = JSON.parse(process.argv[3]);
const state = { robot_name: process.argv[4] };
let loadedControllerName = "";
const banners = [], logged = [];
const BOARD_SILICON = { gendrv: "esp32", yb_eet01: "esp32s3" };
function siliconOf(n) { n = String(n || "").toLowerCase(); return BOARD_SILICON[n] || n; }
async function fetch() { return { json: async () => answer }; }
function logLine(x) { logged.push(x); }
function showActionBanner(t, d) { banners.push(t); }
eval(pick("function isGeneratedRobot(") + pick("async function boardMatchesOrWarn(") + `
boardMatchesOrWarn(process.argv[5], "1-Click").then((ok) => console.log(JSON.stringify({ ok, banners })));`);
"""

NO_BOARD = {"board_on_bus": False, "mcu_detected": False, "mcu_mismatch": None}


def guard(tmp_path, answer, robot, controller):
    h = tmp_path / "guard.js"
    h.write_text(GUARD)
    out = subprocess.run([NODE, str(h), os.path.join(ROOT, "web", "frontend", "app-core.js"),
                          json.dumps(answer), robot, controller], capture_output=True, text=True, timeout=30)
    assert out.returncode == 0, out.stderr
    return json.loads(out.stdout.strip().splitlines()[-1])


@pytest.mark.skipif(NODE is None, reason="node is not installed")
@pytest.mark.parametrize("design", DESIGNS)
def test_an_action_on_a_design_with_no_board_is_refused(design, tmp_path):
    res = guard(tmp_path, NO_BOARD, design, "pico2")
    assert res["ok"] is False and "no board attached" in res["banners"][0], res


@pytest.mark.skipif(NODE is None, reason="node is not installed")
def test_a_bare_robot_with_no_board_may_go_on_to_the_sim_mcu(tmp_path):
    assert guard(tmp_path, NO_BOARD, "bare_pico2", "pico2")["ok"] is True


@pytest.mark.skipif(NODE is None, reason="node is not installed")
def test_a_design_with_its_board_attached_runs(tmp_path):
    ok = {"board_on_bus": True, "mcu_detected": True, "mcu_mismatch": None}
    assert guard(tmp_path, ok, "gendrv", "gendrv")["ok"] is True


def test_bringup_asks_the_same_guard():
    js = open(os.path.join(ROOT, "web", "frontend", "app-agent-bringup.js")).read()
    a = js.index('title: "Bringup",')
    assert 'boardMatchesOrWarn(ctl, "Bringup")' in js[a:a + 800]


@pytest.mark.skipif(NODE is None, reason="node is not installed")
def test_an_empty_controller_is_no_licence_for_a_real_robot(tmp_path):
    """makerspet_mini names its board as its controller; the select went blank, the guard got
    '' and let a no-board Bringup through (2026-10-06)."""
    res = guard(tmp_path, NO_BOARD, "makerspet_mini", "")
    assert res["ok"] is False and "no board attached" in res["banners"][0], res
    assert guard(tmp_path, NO_BOARD, "bare_pico2", "")["ok"] is True


def test_every_design_names_its_silicon():
    """The UI learns a board-named controller's silicon from the config's `mcu`."""
    import yaml
    for d in DESIGNS:
        bc = yaml.safe_load(open(os.path.join(ROOT, "config", "reference", f"{d}_config.yaml")))["base_controller"]
        assert bc.get("mcu"), f"{d}: base_controller has no mcu"


def test_a_reference_design_simulates_nothing():
    """User, 2026-10-06: "a ref design mean real robot, so no sim devices"."""
    import yaml
    for d in DESIGNS:
        bc = yaml.safe_load(open(os.path.join(ROOT, "config", "reference", f"{d}_config.yaml")))["base_controller"]
        on = [k for k, v in (bc.get("sensors") or {}).items() if k.startswith("use_sim_") and v]
        assert not on, f"{d} simulates {on}"
        assert (bc.get("simulation") or {}).get("mode") in (None, "real"), d
