"""What the web UI says about the robot's board, when the board is not on USB.

Found in a web UI walkthrough on 2026-10-16, on an Arduino UNO Q robot (its STM32 on the
board's UART and SWD) and an ESP32 robot on Wi-Fi with no USB:

* the MCU & Sim card told the UNO Q "No board is answering on the USB bus ... expected while
  a board is in BOOTSEL": /api/status found the STM32 (the device tree), but the card only
  knew a USB port or nothing;
* the Wi-Fi robot's header said "No MCU board detected -- switched to the Sim MCU robot":
  the over-the-air case fell through to the switched-to-Sim text (and bare_sim's own udp4
  counted as Wi-Fi, which is why that case had been limited to non-generated robots);
* every page load flashed "pico2 builds for RP2350, but the board on the bus is STM32U585":
  the first /api/status carried the markup's pico2 as the controller hint;
* Hardware Tests said "The robot is the Sim MCU" from the controller select, not the robot;
* the ADC card said "pico2" on bare_sim: bare_sim's `mcu: host` was learned as the silicon of
  "sim", the selects (which list no "host") went blank, and the flash env fell back to pico2.
"""
import json
import os
import shutil
import subprocess
import sys

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "scripts"))
NODE = shutil.which("node") or shutil.which("nodejs")
CORE_JS = os.path.join(ROOT, "web", "frontend", "app-core.js")
ADC_JS = os.path.join(ROOT, "web", "frontend", "app-adc.js")


# --- /api/status ---------------------------------------------------------------------------

def _status(monkeypatch, controller, onboard=(None, None, None)):
    pytest.importorskip("fastapi", reason="the cockpit backend needs fastapi/uvicorn")
    os.environ["COCKPIT_AUTH"] = "off"
    sys.path.insert(0, os.path.join(ROOT, "web", "backend"))
    import routes_status
    params = {"robot": {"name": "rover-unoq"}, "base_controller": controller}
    monkeypatch.setattr(routes_status, "load_params", lambda *a, **k: params)
    monkeypatch.setattr(routes_status, "get_serial_ports_dict", lambda *a, **k: {"local_ports": []})
    monkeypatch.setattr(routes_status, "_bus_devices", lambda: [])
    monkeypatch.setattr(routes_status.mcu_identity, "onboard_controller", lambda *a, **k: onboard)
    monkeypatch.setattr(routes_status, "probe_stack_liveness", lambda: {"agent": False, "bringup": False})
    return routes_status.get_status


def test_the_unoq_stm32_is_reported_on_board_with_its_port(monkeypatch):
    get = _status(monkeypatch, {"name": "unoq", "mcu": "unoq", "transport": "serial"},
                  ("unoq", "STM32U585 (Arduino UNO Q)", "/dev/ttyHS1"))
    s = get()
    assert s["mcu_detected"] and s["mcu_onboard"] and s["detected_port"] == "/dev/ttyHS1"
    assert s["mcu_mismatch"] is None and not s["board_on_wifi"]


def test_a_usb_less_computer_has_nothing_on_board(monkeypatch):
    s = _status(monkeypatch, {"name": "pico2", "mcu": "pico2", "transport": "serial"})()
    assert not s["mcu_detected"] and not s["mcu_onboard"] and s["board_on_bus"] is False


def test_a_wifi_robot_is_on_wifi_and_the_sim_mcu_is_not(monkeypatch):
    s = _status(monkeypatch, {"name": "gendrv", "mcu": "esp32", "transport": "udp4"})()
    assert s["board_on_wifi"] is True
    s = _status(monkeypatch, {"name": "sim", "mcu": "host", "transport": "udp4", "agent_ip": "127.0.0.1"})()
    assert s["board_on_wifi"] is False


# --- the page: the header's no-board banner and switch ---------------------------------------

NO_BOARD = r"""
const src = require("fs").readFileSync(process.argv[2], "utf8");
const pick = (start) => { const a = src.indexOf(start); return src.slice(a, src.indexOf("\n}\n", a) + 3); };
const s = JSON.parse(process.argv[3]);
const robot = process.argv[4];
const kind = robot.startsWith("bare_") ? "generated" : "real";
const state = { robot_name: robot, robots: [{ name: robot, kind, real: kind === "real" }] };
const el = { hidden: true, innerHTML: "" };
const document = { getElementById: (id) => (id === "hdr-no-board" ? el : null) };
const switched = [];
let userChoseController = false, controllerPickedByHand = false, noBoardSwitchedFrom = null;
let loadedControllerName = s.controller, robotBeforeSimSilicon = null;
function escapeHtml(x) { return String(x); }
function siliconOf(n) { return String(n || "").toLowerCase(); }
async function useSimRobot() { switched.push("bare_sim"); state.robot_name = "bare_sim"; }
async function selectRobot(n) { switched.push(n); }
async function useDefaultRobotFor(sil) { switched.push(`lino_${sil}`); }
eval(pick("function isGeneratedRobot(") + pick("function activeRobot(") +
     "let noBoardBusy = false;\n" + pick("async function noBoardSwitch(") + `
noBoardSwitch(s).then(() => console.log(JSON.stringify({ switched, robot: state.robot_name,
  shown: !el.hidden, text: el.innerHTML })));`);
"""

WIFI_NO_USB = {"board_on_bus": False, "mcu_detected": False, "board_on_wifi": True,
               "controller": "gendrv", "detected_mcu": "gendrv"}


def no_board(tmp_path, status, robot):
    h = tmp_path / "noboard.js"
    h.write_text(NO_BOARD)
    out = subprocess.run([NODE, str(h), CORE_JS, json.dumps(status), robot],
                         capture_output=True, text=True, timeout=30)
    assert out.returncode == 0, out.stderr
    return json.loads(out.stdout.strip().splitlines()[-1])


@pytest.mark.skipif(NODE is None, reason="node is not installed")
@pytest.mark.parametrize("robot", ["rover_wifi", "bare_esp32"])
def test_a_wifi_robot_with_no_usb_is_neither_switched_nor_told_it_was(tmp_path, robot):
    r = no_board(tmp_path, WIFI_NO_USB, robot)
    assert r["switched"] == [] and r["robot"] == robot, r
    assert not r["shown"] and "Sim MCU" not in r["text"], r


@pytest.mark.skipif(NODE is None, reason="node is not installed")
def test_a_bare_module_with_no_board_still_goes_to_the_sim_mcu(tmp_path):
    r = no_board(tmp_path, dict(WIFI_NO_USB, board_on_wifi=False, controller="pico2"), "bare_pico2")
    assert r["switched"] == ["bare_sim"] and r["shown"] and "switched to the <b>Sim MCU</b>" in r["text"], r


@pytest.mark.skipif(NODE is None, reason="node is not installed")
def test_a_real_usb_robot_with_no_board_is_warned_not_switched(tmp_path):
    r = no_board(tmp_path, dict(WIFI_NO_USB, board_on_wifi=False, controller="pico2"), "rover_pico2")
    assert r["switched"] == [] and r["shown"] and "is a real robot" in r["text"], r
    assert "switched" not in r["text"]


# --- the page: the MCU & Sim card -------------------------------------------------------------

def _card_branch(js):
    a = js.index('const baseMcuDesc = document.getElementById("base-detected-mcu-desc");')
    return js[a:js.index("baseMcuName.textContent = s.mcu_detected", a)]


def test_the_card_says_where_an_on_board_controller_is_not_bootsel():
    js = open(CORE_JS).read()
    card = _card_branch(js)
    usb, onb = card.index("if (s.mcu_detected && lp)"), card.index("} else if (s.mcu_onboard) {")
    assert usb < onb < card.index("BOOTSEL")
    seg = card[onb:card.index("} else if", onb + 10)]
    assert "s.detected_port" in seg and "SWD" in seg and "BOOTSEL" not in seg
    # a UNO Q robot on a computer that is not its UNO Q, a Wi-Fi robot, the Sim MCU: each its own text
    for branch in ("} else if (controllerIsOnboard(s)) {", '} else if (String(s.controller || "").toLowerCase() === "sim") {',
                   "} else if (s.board_on_wifi) {"):
        assert onb < card.index(branch) < card.index("BOOTSEL"), branch
    pill = js[js.index('const baseMcuPill = document.getElementById("base-mcu-status-pill");'):]
    assert pill.index("} else if (s.mcu_onboard) {") < pill.index('"Offline / Not Connected"')


@pytest.mark.skipif(NODE is None, reason="node is not installed")
def test_a_unoq_robot_is_known_on_board_by_its_mcu():
    prog = r"""
const src = require("fs").readFileSync(process.argv[1], "utf8");
const pick = (start) => { const a = src.indexOf(start); return src.slice(a, src.indexOf("\n}\n", a) + 3); };
const BOARD_SILICON = {};
eval(pick("function siliconOf(") + pick("function controllerIsOnboard("));
console.log(JSON.stringify([
  controllerIsOnboard({ base_controller: { name: "unoq", mcu: "unoq" } }),
  controllerIsOnboard({ base_controller: { name: "my_board", mcu: "unoq" } }),
  controllerIsOnboard({ controller: "unoq" }),
  controllerIsOnboard({ base_controller: { name: "gendrv", mcu: "esp32" } }),
  controllerIsOnboard({ base_controller: { name: "sim", mcu: "host" } })]));
"""
    out = subprocess.run([NODE, "-e", prog, CORE_JS], capture_output=True, text=True, timeout=30)
    assert out.returncode == 0, out.stderr
    assert json.loads(out.stdout) == [True, True, True, False, False]


# --- the page: no mismatch before the robot's controller is known -----------------------------

def test_the_first_status_carries_no_controller_hint():
    js = open(CORE_JS).read()
    fn = js[js.index("async function refreshStatus("):]
    q = fn[:fn.index("await fetch(`/api/status${q}`)")]
    assert "const q = state._ctrlSelSynced && sel && sel.value ?" in q


# --- the page: Hardware Tests and the ADC card on the Sim MCU or a real robot -------------------

SIM_NOTE = r"""
const core = require("fs").readFileSync(process.argv[2], "utf8");
const adc = require("fs").readFileSync(process.argv[3], "utf8");
const pick = (src, start) => { const a = src.indexOf(start); return src.slice(a, src.indexOf("\n}\n", a) + 3); };
const cases = JSON.parse(process.argv[4]);
const out = [];
const sel = { value: "" };
const document = { getElementById: (id) => (id === "cfg-mcu" ? sel : null), querySelectorAll: () => [] };
let state;
eval(pick(core, "function isReferenceDesign(") + pick(core, "function activeRobot(") +
     pick(core, "function activeRobotIsReal(") + pick(adc, "function robotIsSimMcu("));
for (const c of cases) {
  state = { robot_name: c.robot, robots: [{ name: c.robot, real: c.real }], reference: [],
            config: { base_controller: c.bc } };
  sel.value = c.select;
  out.push(robotIsSimMcu());
}
console.log(JSON.stringify(out));
"""


@pytest.mark.skipif(NODE is None, reason="node is not installed")
def test_the_sim_mcu_note_follows_the_robot_not_the_select(tmp_path):
    cases = [
        # the UNO Q robot, while the select still shows the previous robot's "sim"
        {"robot": "rover-unoq", "real": True, "bc": {"name": "unoq", "mcu": "unoq"}, "select": "sim"},
        {"robot": "rover-unoq", "real": True, "bc": {"name": "sim", "mcu": "host"}, "select": "pico2"},
        {"robot": "bare_sim", "real": False, "bc": {"name": "sim", "mcu": "host"}, "select": ""},
        {"robot": "bare_pico2", "real": False, "bc": {"name": "pico2", "mcu": "pico2"}, "select": "sim"},
    ]
    h = tmp_path / "simnote.js"
    h.write_text(SIM_NOTE)
    out = subprocess.run([NODE, str(h), CORE_JS, ADC_JS, json.dumps(cases)],
                         capture_output=True, text=True, timeout=30)
    assert out.returncode == 0, out.stderr
    assert json.loads(out.stdout) == [False, False, True, False]
    adc = open(ADC_JS).read()
    fn = adc[adc.index("function updateBoardOnlyButtons("):]
    assert "const sim = robotIsSimMcu();" in fn[:200]


@pytest.mark.skipif(NODE is None, reason="node is not installed")
def test_the_sim_mcus_host_is_not_learned_as_a_silicon():
    prog = r"""
const src = require("fs").readFileSync(process.argv[1], "utf8");
const pick = (start) => { const a = src.indexOf(start); return src.slice(a, src.indexOf("\n}\n", a) + 3); };
const BOARD_SILICON = {};
eval(pick("function learnBoardSilicon(") + pick("function siliconOf("));
learnBoardSilicon("sim", "host");
learnBoardSilicon("gendrv", "esp32");
console.log(JSON.stringify([siliconOf("sim"), siliconOf("gendrv")]));
"""
    out = subprocess.run([NODE, "-e", prog, CORE_JS], capture_output=True, text=True, timeout=30)
    assert out.returncode == 0, out.stderr
    assert json.loads(out.stdout) == ["sim", "esp32"]


def test_the_dac_note_names_the_sim_mcu_and_follows_a_synced_select():
    adc = open(ADC_JS).read()
    fn = adc[adc.index("function applyDacAvailability("):]
    assert 'if (env === "sim") env = "the Sim MCU (no board)";' in fn[:400]
    js = open(CORE_JS).read()
    a = js.index("(synced to the active robot)")
    assert "applyDacAvailability()" in js[a - 400:a]
