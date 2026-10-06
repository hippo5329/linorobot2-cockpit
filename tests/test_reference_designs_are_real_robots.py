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
    assert "if real_robot:" in block
    assert block.index("raise SystemExit") < block.index("controller, sim_mcu = SIM_MCU, True")


GUARD = r"""
const src = require("fs").readFileSync(process.argv[2], "utf8");
const pick = (start) => { const a = src.indexOf(start); return src.slice(a, src.indexOf("\n}\n", a) + 3); };
const answer = JSON.parse(process.argv[3]);
const robot = process.argv[4];
const kind = robot.startsWith("bare_") ? "generated" : (process.argv[6] || "design");
const state = { robot_name: robot, reference: [],
                robots: [{ name: robot, kind, real: kind === "design" || kind === "real" }] };
let loadedControllerName = "";
const banners = [], logged = [];
const BOARD_SILICON = { gendrv: "esp32", yb_eet01: "esp32s3" };
function siliconOf(n) { n = String(n || "").toLowerCase(); return BOARD_SILICON[n] || n; }
async function fetch() { return { json: async () => answer }; }
function logLine(x) { logged.push(x); }
function showActionBanner(t, d) { banners.push(t); }
eval(pick("function isGeneratedRobot(") + pick("function isReferenceDesign(") + pick("function activeRobot(") +
     pick("function activeRobotIsReal(") + pick("async function boardMatchesOrWarn(") + `
boardMatchesOrWarn(process.argv[5], "1-Click").then((ok) => console.log(JSON.stringify({ ok, banners })));`);
"""

NO_BOARD = {"board_on_bus": False, "mcu_detected": False, "mcu_mismatch": None}


def guard(tmp_path, answer, robot, controller, kind=""):
    h = tmp_path / "guard.js"
    h.write_text(GUARD)
    out = subprocess.run([NODE, str(h), os.path.join(ROOT, "web", "frontend", "app-core.js"),
                          json.dumps(answer), robot, controller, kind], capture_output=True, text=True, timeout=30)
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


def test_the_pipeline_discovers_every_design_and_refuses_sim_on_it():
    assert ocp.reference_design_names() == set(DESIGNS)
    src = open(os.path.join(ROOT, "scripts", "one_click_pipeline.py")).read()
    a = src.index("real_robot = is_real_robot(params, robot_name)")
    assert 'if real_robot and (args.mode == "sim" or controller == SIM_MCU):' in src[a:a + 160]
    assert "[SIM REFUSED]" in src[a:a + 600]
    # before anything is flashed or launched
    assert a < src.index("sim_mcu = controller == SIM_MCU")


SIMSYNC = r"""
const src = require("fs").readFileSync(process.argv[2], "utf8");
const pick = (start) => { const a = src.indexOf(start); return src.slice(a, src.indexOf("\n}\n", a) + 3); };
const mk = (v) => ({ value: v, disabled: false, title: "" });
const sels = {};
for (const id of ["hdr-pipeline-mode", "cockpit-pipeline-mode"]) sels[id] = { value: "sim", options: [mk("sim"), mk("real"), mk("auto")] };
for (const id of ["cfg-mcu", "cockpit-target-select", "hw-flash-env"]) sels[id] = { value: "pico2", options: [mk("pico2"), mk("sim")] };
const document = { getElementById: (id) => sels[id] || null };
const state = { robot_name: process.argv[3], reference: JSON.parse(process.argv[4]), robots: [] };
eval(pick("function isReferenceDesign(") + pick("function activeRobot(") + pick("function activeRobotIsReal(") +
     pick("function syncSimForDesign(") + `
syncSimForDesign();
console.log(JSON.stringify({ mode: sels["hdr-pipeline-mode"].value, simOff: sels["hdr-pipeline-mode"].options[0].disabled,
  mcuSimOff: sels["cfg-mcu"].options[1].disabled }));`);
"""


@pytest.mark.skipif(NODE is None, reason="node is not installed")
@pytest.mark.parametrize("robot, design", [("gendrv", True), ("bare_pico2", False), ("bare_sim", False)])
def test_the_ui_offers_no_sim_for_a_design(robot, design, tmp_path):
    h = tmp_path / "simsync.js"
    h.write_text(SIMSYNC)
    out = subprocess.run([NODE, str(h), os.path.join(ROOT, "web", "frontend", "app-core.js"), robot, json.dumps(DESIGNS)],
                         capture_output=True, text=True, timeout=30)
    assert out.returncode == 0, out.stderr
    res = json.loads(out.stdout.strip().splitlines()[-1])
    assert res == ({"mode": "real", "simOff": True, "mcuSimOff": True} if design
                   else {"mode": "sim", "simOff": False, "mcuSimOff": False}), res


def test_a_robot_is_real_by_where_its_config_came_from():
    """Not by its name: the user's robot built from a design is real, one without is not."""
    assert ocp.robot_kind("my_rover", "gendrv") == "real"
    assert ocp.robot_kind("my_rover", None) == "sim"
    assert ocp.is_real_robot({"robot": {"name": "my_rover", "reference": "xrp"}})
    assert not ocp.is_real_robot({"robot": {"name": "my_rover"}})
    for d in DESIGNS:
        assert ocp.robot_kind(d) == "design"


@pytest.mark.skipif(NODE is None, reason="node is not installed")
def test_the_users_robot_built_from_a_design_needs_its_board(tmp_path):
    assert guard(tmp_path, NO_BOARD, "my_rover", "pico2", "real")["ok"] is False
    assert guard(tmp_path, NO_BOARD, "my_rover", "pico2", "sim")["ok"] is True


def test_the_kits_say_they_are_kits():
    """A kit's chassis comes with the design; the user overrides it afterwards (2026-10-06)."""
    import yaml
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from kit_chassis import KIT_CHASSIS
    kits = {d for d in DESIGNS if yaml.safe_load(open(os.path.join(
        ROOT, "config", "reference", f"{d}_config.yaml")))["robot"].get("kit")}
    assert kits == set(KIT_CHASSIS)


@pytest.mark.skipif(NODE is None, reason="node is not installed")
def test_a_real_robot_is_refused_the_sim_mcu_in_the_ui(tmp_path):
    """The guard waved controller "sim" through; a robot built from a design sent --controller sim
    and only the pipeline stopped it (found by the walkthrough, 2026-10-06)."""
    res = guard(tmp_path, NO_BOARD, "my_rover", "sim", "real")
    assert res["ok"] is False and "Sim MCU" in res["banners"][0], res
    assert guard(tmp_path, NO_BOARD, "bare_sim", "sim")["ok"] is True
    assert guard(tmp_path, NO_BOARD, "my_rover", "sim", "sim")["ok"] is True


def test_applying_a_design_moves_every_controller_select():
    js = open(os.path.join(ROOT, "web", "frontend", "app-presets.js")).read()
    a = js.index("async function applyReferenceDesign(")
    body = js[a:js.index("\n}\n", a)]
    assert '"cockpit-target-select"' in body and "loadedControllerName = bc.name" in body


def test_the_design_picker_is_an_action_not_an_autosaved_edit():
    html = open(os.path.join(ROOT, "web", "frontend", "index.html")).read()
    assert 'id="preset-select" data-no-autosave' in html
