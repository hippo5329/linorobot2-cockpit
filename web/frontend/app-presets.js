// Linorobot2 Cockpit frontend -- reference build presets & hardware design engine, simulation-mode.
// Part of the app.js split: a classic script sharing global scope. See app-core.js.

// ==============================================================================
// Reference Build Presets & Hardware Design Engine
// ==============================================================================

// Reference designs are REAL ROBOTS' configurations, applied to the user's own robot
// (user, 2026-10-06): name your robot, then pick a design here. The design's controller --
// MCU, board, pins, motor driver, sensors, link -- becomes your robot's, your chassis and
// tuning stay (a fresh robot takes the design's chassis too), and your robot records the
// design (robot.reference): it is real from then on, with no simulated devices. Everything
// after that is your tuning, autosaved. The designs come from the cockpit (/api/robots,
// discovered from config/reference/), never from a list kept here -- a list in this file
// drifted from the shipped configs and named the set, and more designs are coming.
// The logic: the lab's cockpit/docs/robots-sim-vs-reference.md.
const BARE_CHOICE = "__bare__";

function normalizeMcuFamily(mcu) {
  if (!mcu) return "pico2";
  const s = String(mcu).toLowerCase();
  if (s.includes("xrp")) return "xrp";   // an RP2350B with its own board, not a Pico 2
  if (s.includes("pico2") || s.includes("rp2350")) return "pico2";
  if (s.includes("pico") || s.includes("rp2040")) return "pico";
  if (s.includes("s3") || s.includes("esp32s3") || s.includes("yb_eet01")) return "esp32s3";
  if (s.includes("esp32") || s.includes("gendrv")) return "esp32";
  if (s.includes("unoq") || s.includes("stm32")) return "unoq";   // the Arduino UNO Q's on-board STM32U585
  return "pico2";
}

// The designs the cockpit ships, grouped by silicon, the current MCU's group first. Every
// design is offered whatever MCU is detected (a design is chosen before its board is on the
// desk); actions that touch a board need the matching one (boardMatchesOrWarn).
function updateReferenceDesigns(mcuHint) {
  const select = document.getElementById("preset-select");
  if (!select) return;
  const designs = (state.robots || []).filter((r) => r.kind === "design");
  const family = normalizeMcuFamily(siliconOf(mcuHint));
  const groups = {};
  for (const d of designs) (groups[normalizeMcuFamily(siliconOf(d.mcu))] ||= []).push(d);
  const order = [family, ...Object.keys(groups).filter((k) => k !== family).sort()];
  const active = (state.robots || []).find((r) => r.name === state.robot_name) || {};
  let html = `<option value="${BARE_CHOICE}">No design: a bare module of the board (every device simulated)</option>`;
  for (const k of order) {
    if (!groups[k]) continue;
    html += `<optgroup label="MCU: ${escapeHtml(k)}">`;
    for (const d of groups[k].sort((a, b) => a.name.localeCompare(b.name))) {
      html += `<option value="${escapeHtml(d.name)}">${escapeHtml(d.description || d.name)}</option>`;
    }
    html += "</optgroup>";
  }
  select.innerHTML = html;
  select.value = active.reference && designs.some((d) => d.name === active.reference) ? active.reference : BARE_CHOICE;
}

// Apply a design (or the bare module of the board) to the user's NAMED robot.
async function applyReferenceDesign(designId) {
  const bare = designId === BARE_CHOICE;
  if (!robotIsNamed()) {           // a design or a generated robot takes nothing applied
    askForRobotName();
    updateReferenceDesigns(document.getElementById("cfg-mcu")?.value);
    return;
  }
  if (typeof flushAutosave === "function") await flushAutosave();
  // The board that is really on the bus; with none, the robot's own controller. (/api/status
  // still names a detected_mcu when nothing is detected: the configured one, as a fallback.)
  const onBus = state.status?.mcu_detected ? state.status.detected_mcu : null;
  const mcu = bare ? (siliconOf(onBus || loadedControllerName) || "pico2") : null;
  let res = null;
  robotEpoch++;
  try {
    res = await fetch(bare ? "/api/robot/apply_bare" : "/api/robot/apply_reference", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(bare ? { mcu } : { design: designId }),
    }).then((r) => r.json());
  } catch (e) { res = { detail: String(e) }; }
  robotEpoch++;
  if (!res || res.status !== "ok") {
    logLine(`❌ ${res?.detail || res?.error || "could not apply the design"}`);
    updateReferenceDesigns(document.getElementById("cfg-mcu")?.value);
    return;
  }
  state.robots = res.robots || state.robots;
  state.config = res.config || state.config;
  // The controller follows the robot's config in EVERY select, as selectRobot does: the
  // Operations tab's select is what 1-Click sends, and it was left on the previous robot's
  // controller ("sim"), so a robot just built from a design started on the Sim MCU.
  const bc = res.config?.base_controller || {};
  learnBoardSilicon(bc.name, bc.mcu);
  if (bc.name) {
    loadedControllerName = bc.name;
    const sil = siliconOf(bc.name);
    for (const id of ["cfg-mcu", "cockpit-target-select", "hw-flash-env"]) {
      const sel = document.getElementById(id);
      if (sel && [...sel.options].some((o) => o.value === sil)) sel.value = sil;
    }
    if (window.__syncControllerSelects) window.__syncControllerSelects(sil, "cfg-mcu");
  }
  if (typeof loadHardwareConfig === "function") await loadHardwareConfig();
  syncSimForDesign();
  updateReferenceDesigns(document.getElementById("cfg-mcu")?.value);
  if (typeof refreshSaveState === "function") refreshSaveState();
  logLine(`✅ ${res.message}`);
  showToast(bare ? `Bare ${mcu} module: every device simulated` : `⚡ ${res.message}`);
}


function initReferenceDesigns() {
  const presetSel = document.getElementById("preset-select");
  if (presetSel) {
    presetSel.addEventListener("change", (e) => {
      applyReferenceDesign(e.target.value);
    });
  }

  // There are TWO base-controller selects: #cfg-mcu on the Base & MCU tab,
  // under a heading that says "Single Source of Truth", and
  // #cockpit-target-select on the Operations tab, which is the one runOneClick
  // actually reads. They were independent, so the visible one was not the one
  // that acted. Observed on a freshly loaded page, nothing touched:
  //
  //   cfg-mcu               = pico    (shown to the user, and what Flash MCU uses)
  //   cockpit-target-select = pico2   (what Start 1-Click sends)
  //
  // -- the Reference Build preset writes cfg-mcu and never the other. So Flash
  // MCU wrote RP2040 while Start 1-Click built RP2350, from one screen, and the
  // only thing that caught it was the MCU guard refusing at flash time.
  //
  // Mirror them. Only adopt a value the other select actually offers (the two
  // lists are not identical -- cockpit-target-select carries esp32_wifi and
  // gendrv, cfg-mcu does not); assigning an unknown value blanks the
  // element, which is worse than leaving it alone.
  const syncControllerSelects = (value, fromId) => {
    for (const id of ["cfg-mcu", "cockpit-target-select", "hw-flash-env"]) {
      if (id === fromId) continue;
      const el = document.getElementById(id);
      if (!el || el.value === value) continue;
      if (![...el.options].some((o) => o.value === value)) continue;
      el.value = value;
    }
  };

  const mcuTarget = document.getElementById("cfg-mcu");
  if (mcuTarget) {
    mcuTarget.addEventListener("change", (e) => {
      updateReferenceDesigns(e.target.value);
      syncControllerSelects(e.target.value, "cfg-mcu");
      if (typeof refreshStatus === "function") refreshStatus();
    });
  }

  const cockpitTarget = document.getElementById("cockpit-target-select");
  if (cockpitTarget) {
    cockpitTarget.addEventListener("change", (e) => {
      updateReferenceDesigns(e.target.value);
      syncControllerSelects(e.target.value, "cockpit-target-select");
      // Re-ask immediately: the board-mismatch banner is about the controller
      // that was just chosen, and waiting for the next poll leaves a stale
      // warning (or none) on screen for several seconds.
      if (typeof refreshStatus === "function") refreshStatus();
    });
  }
  window.__syncControllerSelects = syncControllerSelects;

  initSimModeWorkflow();
}

let currentSimMode = true;

function updateSimModeUI(enabled) {
  currentSimMode = enabled;

  const stateBadge = document.getElementById("simulation-mode-state-badge");
  const descElem = document.getElementById("simulation-mode-desc");
  const toggleBtn = document.getElementById("btn-toggle-sim-mode");
  const odomStatus = document.getElementById("sim-odom-status");
  const imuStatus = document.getElementById("simulated-imu-status");
  const lidarStatus = document.getElementById("simulated-lidar-status");
  const cardTitle = document.getElementById("simulation-mode-card-title");

  if (stateBadge) {
    stateBadge.textContent = enabled ? "Zero-Wiring Default ON" : "Real Hardware Active";
    stateBadge.className = "badge-pill " + (enabled ? "badge-ok" : "badge-accent");
  }
  if (cardTitle) {
    cardTitle.textContent = enabled
      ? "Sim Mode / Zero-Wiring Simulation Preview (Active by Default)"
      : "Real Physical Hardware Mode (Sim Simulation Disabled)";
  }
  if (descElem) {
    if (enabled) {
      descElem.innerHTML = `Linorobot2 defaults to safe <b>Sim Mode</b> simulation. The board simulates its wheel encoders (env <code>sim_wheel</code>), a 6-DOF IMU (<code>sim_imu</code>) and a planar LiDAR (<code>sim_ld19</code>), switched in its env partition with no rebuild. This enables complete end-to-end Map, SLAM, and Nav2 testing on a bare MCU module before physical wheels or motors are wired.`;
    } else {
      descElem.innerHTML = `<b>Real Physical Hardware Mode Active.</b> The simulation env keys (<code>sim_wheel</code>, <code>sim_imu</code>, <code>sim_ld19</code>) are off. The microcontroller interacts with real physical motor drivers, wheel encoders, and real I2C sensors. Proceed to Step 2 (Drive &amp; Motors) and Step 4 (Pin Matrix) to finalize wiring pinouts.`;
    }
  }
  if (toggleBtn) {
    if (enabled) {
      toggleBtn.innerHTML = `⚡ Switch Sim Mode OFF ➔ Start Details Hardware Design`;
      toggleBtn.className = "btn btn-accent";
    } else {
      toggleBtn.innerHTML = `🔄 Re-enable Sim Mode Simulation Preview`;
      toggleBtn.className = "btn btn-secondary";
    }
  }
  if (odomStatus) {
    odomStatus.innerHTML = enabled
      ? `<span style="color:#94a3b8;">Simulated Odometry:</span> <b style="color:#34d399;">Active (50 Hz /odom)</b>`
      : `<span style="color:#94a3b8;">Wheel Encoders:</span> <b style="color:#38bdf8;">Physical Hardware Pinouts</b>`;
  }
  if (imuStatus) {
    imuStatus.innerHTML = enabled
      ? `<span style="color:#94a3b8;">Simulated IMU:</span> <b style="color:#34d399;">Active (50 Hz /imu/data)</b>`
      : `<span style="color:#94a3b8;">Physical IMU:</span> <b style="color:#38bdf8;">Real I2C Bus Driver</b>`;
  }
  if (lidarStatus) {
    lidarStatus.innerHTML = enabled
      ? `<span style="color:#94a3b8;">Simulated LiDAR:</span> <b style="color:#34d399;">Active (10 Hz /scan)</b>`
      : `<span style="color:#94a3b8;">Laser Scanner:</span> <b style="color:#38bdf8;">Serial / UDP LiDAR Driver</b>`;
  }

  const slamBadge = document.getElementById("slam-sim-mode-badge");
  if (slamBadge) {
    slamBadge.textContent = enabled ? "Zero-Wiring Simulation Active" : "Real Hardware Mode";
    slamBadge.className = "badge-pill " + (enabled ? "badge-ok" : "badge-accent");
  }
  const cockpitBadge = document.getElementById("cockpit-sim-mode-badge");
  if (cockpitBadge) {
    cockpitBadge.textContent = enabled ? "Sim Mode Active" : "Real Hardware Active";
    cockpitBadge.className = "badge-pill " + (enabled ? "badge-ok" : "badge-accent");
  }

  document.querySelectorAll(".btn-switch-real-hw").forEach((btn) => {
    btn.textContent = enabled ? "⚡ Switch Sim Mode OFF ➔ Start Details Hardware Design ➔" : "⚙️ Proceed to Step 2: Drive & Motors ➔";
  });

  // The Sim MCU is simulation by definition -- no board, no real sensor to
  // switch to -- so "Sim Mode OFF" is not offered on it. (A browser walk
  // pressed it and bare_sim was left with every simulated device off.) Real
  // hardware starts by picking the board on the Base & MCU tab. Re-enabling
  // stays available, so a robot left in that state can be repaired.
  const simMcu = state.robot_name === "bare_sim";
  [toggleBtn, ...document.querySelectorAll(".btn-switch-real-hw")].forEach((b) => {
    if (!b) return;
    const lock = simMcu && enabled;
    if (lock && b.dataset.simMcuTitle === undefined) {
      b.dataset.simMcuTitle = b.title || "";
      b.title = "The Sim MCU is simulation only. To design real hardware, pick your board on the Base & MCU tab.";
    } else if (!lock && b.dataset.simMcuTitle !== undefined) {
      b.title = b.dataset.simMcuTitle;
      delete b.dataset.simMcuTitle;
    }
    b.disabled = lock;
  });
}

async function setSimMode(enabled, transitionToDetails = false) {
  if (!enabled && state.robot_name === "bare_sim") return;   // see updateSimModeUI
  updateSimModeUI(enabled);
  const activeController = state.status?.controller || "pico2";

  try {
    const res = await fetch("/api/hardware/sim_mode", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ enabled: enabled, controller: activeController }),
    });
    const result = await res.json();
    if (result.success) {
      if (!enabled) {
        showToast("⚡ Switched to Real Hardware Mode! You can now configure your detailed hardware design.");
      } else {
        showToast("🔄 Switched to Sim Simulation Mode (Safe Zero-Wiring Default).");
      }
    }
  } catch (err) {
    console.error("Failed to update simulation mode on backend:", err);
  }

  if (transitionToDetails) {
    const driveTab = document.querySelector(".tab-btn[data-tab='drive-motors']");
    if (driveTab) {
      driveTab.click();
      window.scrollTo({ top: 0, behavior: "smooth" });
    }
  }
}

// Leaving simulation is not a switch: a reference design, or pins, make the robot real. The
// card's buttons take the user to the Reference Designs picker.
function goMakeItReal() {
  document.querySelector(".tab-btn[data-tab='mcu-sim']")?.click();
  const sel = document.getElementById("preset-select");
  if (sel) { sel.scrollIntoView({ block: "center" }); sel.focus(); }
}
function initSimModeWorkflow() {
  document.querySelectorAll(".btn-switch-real-hw").forEach((btn) => btn.addEventListener("click", goMakeItReal));
  document.getElementById("btn-toggle-sim-mode")?.addEventListener("click", goMakeItReal);
  const jumpPreviewBtn = document.getElementById("btn-jump-preview");
  if (jumpPreviewBtn) {
    jumpPreviewBtn.addEventListener("click", () => {
      const slamTab = document.querySelector(".tab-btn[data-tab='slam-nav']");
      if (slamTab) {
        slamTab.click();
        window.scrollTo({ top: 0, behavior: "smooth" });
      }
    });
  }
}

document.addEventListener("DOMContentLoaded", initBaseControllerConfigModule);
