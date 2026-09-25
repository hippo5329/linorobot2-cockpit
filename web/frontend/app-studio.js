// Linorobot2 Cockpit frontend -- AI robotics tuning & custom robot builder studio, params export/merge/promote, path & serial-port pickers.
// Part of the app.js split: a classic script sharing global scope. See app-core.js.

// =========================================================
// AI ROBOTICS TUNING & CUSTOM ROBOT BUILDER STUDIO
// =========================================================
function currentRosDistro() {
  return getDistro();
}

let currentAiTuningAnalysis = null;
let currentCustomRobotSpecs = null;

// 1. AI Tuning Prompt & Chips
const aiTunePrompt = document.getElementById("ai-tune-prompt");
const btnAiTuneAsk = document.getElementById("btn-ai-tune-ask");
const aiTuneOutput = document.getElementById("ai-tune-output");
const aiTuneDiag = document.getElementById("ai-tune-diagnosis");
const aiTuneRecs = document.getElementById("ai-tune-recs");
const btnAiTuneApply = document.getElementById("btn-ai-tune-apply");
const aiTuneStatus = document.getElementById("ai-tune-status");

document.querySelectorAll(".btn-chip").forEach((btn) => {
  btn.addEventListener("click", () => {
    if (aiTunePrompt) {
      aiTunePrompt.value = btn.getAttribute("data-prompt") || "";
      if (btnAiTuneAsk) btnAiTuneAsk.click();
    }
  });
});

if (btnAiTuneAsk) {
  btnAiTuneAsk.addEventListener("click", async () => {
    const prompt = (aiTunePrompt?.value || "").trim();
    if (!prompt) return;
    btnAiTuneAsk.disabled = true;
    if (aiTuneStatus) aiTuneStatus.textContent = "Analyzing robotics dynamics...";
    try {
      const distro = currentRosDistro();
      const base = document.getElementById("tune-base-type")?.value || "2wd";
      const res = await fetch("/api/ai/tune", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ prompt, distro, base }),
      });
      const data = await res.json();
      currentAiTuningAnalysis = data;
      if (aiTuneOutput) aiTuneOutput.style.display = "block";
      if (aiTuneDiag) aiTuneDiag.textContent = "🩺 " + (data.diagnosis || "No diagnosis.");
      if (aiTuneRecs) {
        aiTuneRecs.innerHTML = (data.recommendations || []).map((r) => `<li>${escapeHtml(r)}</li>`).join("");
      }
      if (aiTuneStatus) aiTuneStatus.textContent = "Analysis complete.";
    } catch (e) {
      if (aiTuneStatus) aiTuneStatus.textContent = "Error: " + e.message;
    } finally {
      btnAiTuneAsk.disabled = false;
    }
  });
}

if (btnAiTuneApply) {
  btnAiTuneApply.addEventListener("click", async () => {
    if (!currentAiTuningAnalysis) return;
    btnAiTuneApply.disabled = true;
    if (aiTuneStatus) aiTuneStatus.textContent = "Applying patches...";
    try {
      const distro = currentRosDistro();
      const base = currentAiTuningAnalysis.target_base
        || document.getElementById("tune-base-type")?.value || "2wd";
      const res = await fetch("/api/ai/apply", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({
          nav2_patch: currentAiTuningAnalysis.nav2_patch,
          ekf_patch: currentAiTuningAnalysis.ekf_patch,
          slam_patch: currentAiTuningAnalysis.slam_patch,
          base,
          distro,
        }),
      });
      const data = await res.json();
      if (aiTuneStatus) aiTuneStatus.textContent = "✓ Applied AI recommendations to Nav2, EKF & SLAM!";
      loadNav2Config();
      loadEkfConfig();
      loadSlamConfig();
      setTimeout(() => { if (aiTuneStatus) aiTuneStatus.textContent = ""; }, 5000);
    } catch (e) {
      if (aiTuneStatus) aiTuneStatus.textContent = "Apply failed: " + e.message;
    } finally {
      btnAiTuneApply.disabled = false;
    }
  });
}

// 2. Presets Selector
const btnApplyPreset = document.getElementById("btn-apply-preset");
const tunePresetSelect = document.getElementById("tune-preset-select");
if (btnApplyPreset && tunePresetSelect) {
  btnApplyPreset.addEventListener("click", async () => {
    const preset = tunePresetSelect.value;
    btnApplyPreset.disabled = true;
    try {
      const distro = currentRosDistro();
      const res = await fetch("/api/presets/apply", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ preset, distro }),
      });
      const data = await res.json();
      alert(`Applied preset '${data.label}'! Nav2, EKF, and SLAM configs updated.`);
      loadNav2Config();
      loadEkfConfig();
      loadSlamConfig();
    } catch (e) {
      alert("Failed to apply preset: " + e.message);
    } finally {
      btnApplyPreset.disabled = false;
    }
  });
}

// 3. Interactive Quick Tuning
const btnApplyInteractive = document.getElementById("btn-apply-interactive-tuning");
const interactiveStatus = document.getElementById("tune-interactive-status");
if (btnApplyInteractive) {
  btnApplyInteractive.addEventListener("click", async () => {
    btnApplyInteractive.disabled = true;
    if (interactiveStatus) interactiveStatus.textContent = "Saving tuning...";
    try {
      const distro = currentRosDistro();
      const base = document.getElementById("tune-base-type")?.value || "2wd";
      const max_vel_x = parseFloat(document.getElementById("tune-max-vel-x")?.value || "0.5");
      const max_vel_y = parseFloat(document.getElementById("tune-max-vel-y")?.value || "0.0");
      const max_vel_theta = parseFloat(document.getElementById("tune-max-vel-theta")?.value || "2.5");
      const max_accel_x = parseFloat(document.getElementById("tune-max-accel-x")?.value || "2.5");
      const max_accel_theta = parseFloat(document.getElementById("tune-max-accel-theta")?.value || "3.2");
      const inflation_radius = parseFloat(document.getElementById("tune-inflation-radius")?.value || "0.7");
      const cost_scaling_factor = parseFloat(document.getElementById("tune-cost-scaling")?.value || "3.0");

      const ekf_freq = parseFloat(document.getElementById("tune-ekf-freq")?.value || "50");
      const fuse_vy = Boolean(document.getElementById("tune-fuse-vy")?.checked);
      const fuse_imu_yaw = Boolean(document.getElementById("tune-fuse-imu-yaw")?.checked);

      const slam_res = parseFloat(document.getElementById("tune-slam-res")?.value || "0.05");

      // Patch Nav2
      await fetch("/api/nav2_config/patch", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({
          distro, base, max_vel_x, max_vel_y, max_vel_theta,
          max_accel_x, max_accel_theta, inflation_radius, cost_scaling_factor
        }),
      });

      // Patch EKF
      await fetch("/api/ekf_config/patch", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({
          base, frequency: ekf_freq, fuse_vy, fuse_imu_yaw
        }),
      });

      // Patch SLAM
      await fetch("/api/slam_config/patch", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ resolution: slam_res }),
      });

      if (interactiveStatus) interactiveStatus.textContent = "✓ Applied tuning parameters across configs!";
      loadNav2Config();
      loadEkfConfig();
      loadSlamConfig();
      setTimeout(() => { if (interactiveStatus) interactiveStatus.textContent = ""; }, 4000);
    } catch (e) {
      if (interactiveStatus) interactiveStatus.textContent = "Tuning failed: " + e.message;
    } finally {
      btnApplyInteractive.disabled = false;
    }
  });
}

// 4. EKF & SLAM Editors
const ekfTextarea = document.getElementById("ekf-config-text");
const btnEkfSave = document.getElementById("btn-ekf-save-config");
const btnEkfReset = document.getElementById("btn-ekf-reset-defaults");
const ekfStatus = document.getElementById("ekf-save-status");

async function loadEkfConfig() {
  if (!ekfTextarea) return;
  try {
    const base = document.getElementById("tune-base-type")?.value || "2wd";
    const res = await fetch(`/api/ekf_config?base=${base}`);
    const data = await res.json();
    if (data.config) ekfTextarea.value = data.config;
  } catch (e) {}
}

if (btnEkfSave && ekfTextarea) {
  btnEkfSave.addEventListener("click", async () => {
    try {
      const res = await fetch("/api/ekf_config", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ config: ekfTextarea.value }),
      });
      if (ekfStatus) ekfStatus.textContent = "✓ Saved EKF configuration";
      setTimeout(() => { if (ekfStatus) ekfStatus.textContent = ""; }, 3000);
    } catch (e) {
      if (ekfStatus) ekfStatus.textContent = "Save failed: " + e.message;
    }
  });
}

if (btnEkfReset && ekfTextarea) {
  btnEkfReset.addEventListener("click", async () => {
    try {
      const base = document.getElementById("tune-base-type")?.value || "2wd";
      const res = await fetch("/api/ekf_config/reset", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ base }),
      });
      const data = await res.json();
      if (data.config) ekfTextarea.value = data.config;
      if (ekfStatus) ekfStatus.textContent = "✓ Reset EKF to default";
      setTimeout(() => { if (ekfStatus) ekfStatus.textContent = ""; }, 3000);
    } catch (e) {
      if (ekfStatus) ekfStatus.textContent = "Reset failed: " + e.message;
    }
  });
}

const slamTextarea = document.getElementById("slam-config-text");
const btnSlamSave = document.getElementById("btn-slam-save-config");
const btnSlamReset = document.getElementById("btn-slam-reset-defaults");
const slamStatus = document.getElementById("slam-save-status");

async function loadSlamConfig() {
  if (!slamTextarea) return;
  try {
    const res = await fetch("/api/slam_config");
    const data = await res.json();
    if (data.config) slamTextarea.value = data.config;
  } catch (e) {}
}

if (btnSlamSave && slamTextarea) {
  btnSlamSave.addEventListener("click", async () => {
    try {
      await fetch("/api/slam_config", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ config: slamTextarea.value }),
      });
      if (slamStatus) slamStatus.textContent = "✓ Saved SLAM configuration";
      setTimeout(() => { if (slamStatus) slamStatus.textContent = ""; }, 3000);
    } catch (e) {
      if (slamStatus) slamStatus.textContent = "Save failed: " + e.message;
    }
  });
}

if (btnSlamReset && slamTextarea) {
  btnSlamReset.addEventListener("click", async () => {
    try {
      const res = await fetch("/api/slam_config/reset", { method: "POST" });
      const data = await res.json();
      if (data.config) slamTextarea.value = data.config;
      if (slamStatus) slamStatus.textContent = "✓ Reset SLAM to default";
      setTimeout(() => { if (slamStatus) slamStatus.textContent = ""; }, 3000);
    } catch (e) {
      if (slamStatus) slamStatus.textContent = "Reset failed: " + e.message;
    }
  });
}

// ---------- params export / merge / promote ----------
const paramsOpResult = document.getElementById("params-op-result");
function showParamsResult(obj) {
  if (paramsOpResult) paramsOpResult.textContent =
    typeof obj === "string" ? obj : JSON.stringify(obj, null, 2);
}
async function paramsPost(path, body) {
  const res = await fetch(path, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(body),
  });
  return res.json();
}
function editorDistro() {
  return (nav2EditorDistro && nav2EditorDistro.value) || getDistro();
}

document.getElementById("btn-params-export")?.addEventListener("click", async () => {
  const dir = document.getElementById("params-export-dir").value.trim();
  if (!dir) { showParamsResult("Enter an export directory first."); return; }
  showParamsResult("Exporting…");
  try {
    const d = await paramsPost("/api/params/export", {
      dest_dir: dir,
      distros: ["jazzy", "lyrical", "rolling"],
      depth_costmap: document.getElementById("bringup-depth-sensor")?.value ? "true" : "false",
    });
    showParamsResult(d);
  } catch (e) { showParamsResult("Export failed: " + e.message); }
});

async function runMerge(dryRun, importText) {
  const body = {
    kind: document.getElementById("params-merge-kind").value,
    distro: editorDistro(),
    dry_run: dryRun,
  };
  if (importText != null) { body.target = "active"; body.source_text = importText; }
  else { body.target = document.getElementById("params-merge-target").value; }
  showParamsResult(dryRun ? "Previewing merge…" : "Merging…");
  try {
    const d = await paramsPost("/api/params/merge", body);
    showParamsResult({ status: d.status, target: d.target_path, report: d.report });
    if (d.status === "merged" && !importText && nav2Textarea) loadNav2Config(editorDistro());
  } catch (e) { showParamsResult("Merge failed: " + e.message); }
}
document.getElementById("btn-params-merge-dry")?.addEventListener("click", () => runMerge(true));
document.getElementById("btn-params-merge")?.addEventListener("click", () => runMerge(false));
document.getElementById("btn-params-merge-import")?.addEventListener("click", () => {
  const t = document.getElementById("params-import-text").value;
  if (!t.trim()) { showParamsResult("Paste a params file first."); return; }
  runMerge(false, t);
});

document.getElementById("btn-params-promote")?.addEventListener("click", async () => {
  showParamsResult("Promoting…");
  try {
    const d = await paramsPost("/api/params/promote", {
      kind: document.getElementById("params-merge-kind").value,
      distro: editorDistro(),
      direction: document.getElementById("params-promote-dir").value,
    });
    showParamsResult(d);
    if (d.status === "promoted" && d.direction.endsWith("_to_active") && nav2Textarea) {
      loadNav2Config(editorDistro());
    }
  } catch (e) { showParamsResult("Promote failed: " + e.message); }
});

loadEkfConfig();
loadSlamConfig();

// 5. AI Custom Robot Builder Studio
const aiRobotPrompt = document.getElementById("ai-robot-prompt");
const btnAiRobotGenerate = document.getElementById("btn-ai-robot-generate");
const aiRobotSpecBox = document.getElementById("ai-robot-spec-box");
const btnAiRobotDeploy = document.getElementById("btn-ai-robot-deploy");
const aiRobotDeployStatus = document.getElementById("ai-robot-deploy-status");

document.querySelectorAll(".btn-robot-chip").forEach((btn) => {
  btn.addEventListener("click", () => {
    if (aiRobotPrompt) {
      aiRobotPrompt.value = btn.getAttribute("data-robot") || "";
      if (btnAiRobotGenerate) btnAiRobotGenerate.click();
    }
  });
});

if (btnAiRobotGenerate) {
  btnAiRobotGenerate.addEventListener("click", async () => {
    const description = (aiRobotPrompt?.value || "").trim();
    if (!description) return;
    btnAiRobotGenerate.disabled = true;
    try {
      const res = await fetch("/api/ai/robot_builder", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ description }),
      });
      const data = await res.json();
      currentCustomRobotSpecs = data;

      if (aiRobotSpecBox) aiRobotSpecBox.style.display = "block";
      const des = data.design || {};
      const specBase = document.getElementById("spec-base");
      const specWheel = document.getElementById("spec-wheel");
      const specTrack = document.getElementById("spec-track");
      const specWheelbase = document.getElementById("spec-wheelbase");
      const specLidar = document.getElementById("spec-lidar");

      if (specBase) specBase.textContent = des.title || (data.base || "").toUpperCase();
      const mm = (m) => (m ? Math.round(m * 1000) + "mm" : "-");
      if (specWheel) specWheel.textContent = mm(des.wheel_diameter);
      if (specTrack) specTrack.textContent = mm(des.lr_wheels_distance);
      if (specWheelbase) specWheelbase.textContent = des.fr_wheels_distance ? mm(des.fr_wheels_distance) : "0mm (2WD)";
      if (specLidar) specLidar.textContent = (des.laser_sensor || data.laser_sensor || "").toUpperCase();

      const wf = document.getElementById("spec-workflow");
      if (wf && data.workflow) {
        wf.innerHTML = data.workflow.map((s) => `<div>${escapeHtml(s)}</div>`).join("");
      }
    } catch (e) {
      alert("Failed to generate robot specs: " + e.message);
    } finally {
      btnAiRobotGenerate.disabled = false;
    }
  });
}

if (btnAiRobotDeploy) {
  btnAiRobotDeploy.addEventListener("click", async () => {
    if (!currentCustomRobotSpecs) return;
    btnAiRobotDeploy.disabled = true;
    if (aiRobotDeployStatus) aiRobotDeployStatus.textContent = "Deploying custom robot architecture...";
    try {
      const distro = currentRosDistro();
      const res = await fetch("/api/ai/deploy_robot", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ specs: currentCustomRobotSpecs, distro }),
      });
      const data = await res.json();
      if (aiRobotDeployStatus) aiRobotDeployStatus.textContent = "✓ " + (data.message || "Custom robot deployed successfully!");
      loadNav2Config();
      loadEkfConfig();
      loadSlamConfig();
      setTimeout(() => { if (aiRobotDeployStatus) aiRobotDeployStatus.textContent = ""; }, 6000);
    } catch (e) {
      if (aiRobotDeployStatus) aiRobotDeployStatus.textContent = "Deployment failed: " + e.message;
    } finally {
      btnAiRobotDeploy.disabled = false;
    }
  });
}

// ============ path / serial-port picker (used by .pick-btn buttons) ============
(function () {
  const overlay = document.getElementById("picker-overlay");
  if (!overlay) return;
  const elTitle = document.getElementById("picker-title");
  const elPath = document.getElementById("picker-path");
  const elCwd = document.getElementById("picker-cwd");
  const elList = document.getElementById("picker-list");
  const elHint = document.getElementById("picker-hint");
  const btnUp = document.getElementById("picker-up");
  const btnUse = document.getElementById("picker-use");
  const btnRefresh = document.getElementById("picker-refresh");
  let ctx = null;

  function close() { overlay.classList.remove("open"); ctx = null; }
  function row(txt) {
    const d = document.createElement("div");
    d.className = "pk-row"; d.textContent = txt; return d;
  }
  function pick(value) {
    if (ctx && ctx.target) {
      ctx.target.value = value;
      ctx.target.dispatchEvent(new Event("input", { bubbles: true }));
      ctx.target.dispatchEvent(new Event("change", { bubbles: true }));
    }
    close();
  }

  overlay.addEventListener("click", (e) => { if (e.target === overlay) close(); });
  document.getElementById("picker-close").onclick = close;
  document.getElementById("picker-cancel").onclick = close;
  document.addEventListener("keydown", (e) => {
    if (e.key === "Escape" && overlay.classList.contains("open")) close();
  });

  async function loadDir(path) {
    elList.replaceChildren(row("Loading…"));
    let data;
    try {
      const q = new URLSearchParams({ path: path || "", only: ctx.only, exts: ctx.exts || "" });
      data = await fetch("/api/list_dir?" + q).then((r) => r.json());
    } catch (e) { elList.replaceChildren(row("Error: " + e.message)); return; }
    ctx.cwd = data.path;
    elCwd.textContent = data.path;
    btnUp.disabled = !data.parent;
    btnUp.onclick = () => loadDir(data.parent);
    const rows = (data.entries || []).map((e) => {
      const r = document.createElement("div");
      r.className = "pk-row";
      r.innerHTML = '<span class="pk-ic">' + (e.is_dir ? "📂" : "📄") + "</span>" + escapeHtml(e.name);
      r.onclick = () => (e.is_dir ? loadDir(e.path) : (ctx.only === "dir" ? null : pick(e.path)));
      r.ondblclick = () => e.is_dir && loadDir(e.path);
      return r;
    });
    elList.replaceChildren(...(rows.length ? rows : [row(data.error ? "(" + data.error + ")" : "(empty)")]));
  }

  async function loadSerial() {
    elList.replaceChildren(row("Scanning…"));
    await refreshSerialPorts();
    const rows = (SERIAL_PORTS || []).map((p) => {
      const r = document.createElement("div");
      r.className = "pk-row";
      const id = [p.vendor, p.model].filter(Boolean).join(" ") || "USB serial";
      r.innerHTML = '<span class="pk-ic">🔌</span>' + escapeHtml(id) +
        (p.usb_id ? ' <span class="pk-sub">' + escapeHtml(p.usb_id) + "</span>" : "") +
        '<span class="pk-sub">→ ' + escapeHtml(p.tty) + "</span>";
      r.onclick = () => pick(p.preferred);
      return r;
    });
    elList.replaceChildren(...(rows.length ? rows : [row("No USB serial devices detected.")]));
  }

  function openPicker(target, kind, exts) {
    const isSerial = kind === "serial";
    ctx = {
      target, kind, exts: exts || "",
      only: kind === "file" ? "file" : kind === "dir" ? "dir" : "any",
      cwd: "",
    };
    elTitle.textContent = isSerial ? "Pick a serial port"
      : kind === "dir" ? "Pick a folder" : "Pick a file";
    elPath.hidden = isSerial;
    btnUse.hidden = kind !== "dir";
    btnUse.onclick = () => pick(ctx.cwd);
    elHint.textContent = isSerial ? "" : "click a folder to open it";
    btnRefresh.onclick = () => (isSerial ? loadSerial() : loadDir(ctx.cwd));
    overlay.classList.add("open");
    if (isSerial) loadSerial();
    else loadDir((target.value || "").trim());
  }

  document.querySelectorAll(".pick-btn[data-target]").forEach((b) => {
    b.addEventListener("click", () => {
      const t = document.getElementById(b.dataset.target);
      if (t) openPicker(t, b.dataset.pick, b.dataset.exts);
    });
  });
})();


// ============================================================================

// ============================================================================
// Robot & Nav2 Configuration Engine (robot_config.yaml - Single File of Truth)
// ============================================================================
function downloadFile(filename, content, type = "text/yaml;charset=utf-8") {
  const blob = new Blob([content], { type });
  const url = URL.createObjectURL(blob);
  const a = document.createElement("a");
  a.href = url;
  a.download = filename;
  document.body.appendChild(a);
  a.click();
  setTimeout(() => {
    document.body.removeChild(a);
    URL.revokeObjectURL(url);
  }, 100);
}

async function loadRobotConfig() {
  try {
    const distro = getDistro();
    const res = await fetch(`/api/robot_config?distro=${distro}`);
    const data = await res.json();
    state.robot_config = data;
    const lino = data.linorobot2 || {};

    const bBase = document.getElementById("bringup-base-type");
    if (bBase && (lino.base || data.base)) {
      bBase.value = lino.base || data.base;
      const kine = document.getElementById("cfg-kinematics");
      if (kine && (lino.base || data.base)) kine.value = lino.base || data.base;
    }

    const bLaser = document.getElementById("bringup-laser-sensor");
    if (bLaser && lino.laser_sensor) {
      bLaser.value = lino.laser_sensor;
      const cl = document.getElementById("cfg-laser-sensor");
      if (cl) cl.value = lino.laser_sensor;
      const lm = document.getElementById("laser-driver-model");
      if (lm) lm.value = lino.laser_sensor;
    }

    const bDepth = document.getElementById("bringup-depth-sensor");
    if (bDepth && lino.depth_sensor) {
      bDepth.value = lino.depth_sensor;
      const cd = document.getElementById("cfg-depth-sensor");
      if (cd) cd.value = lino.depth_sensor;
    }

    const bDev = document.getElementById("bringup-agent-device");
    if (bDev && lino.micro_ros_port) {
      bDev.value = lino.micro_ros_port;
      const sp = document.getElementById("cfg-serial-port");
      if (sp) sp.value = lino.micro_ros_port;
    }

    const bBaud = document.getElementById("bringup-agent-baud");
    if (bBaud && lino.micro_ros_baudrate) {
      bBaud.value = lino.micro_ros_baudrate;
      const sb = document.getElementById("cfg-baudrate");
      if (sb) sb.value = lino.micro_ros_baudrate;
    }

    const instBase = document.getElementById("install-base");
    if (instBase && (lino.base || data.base)) instBase.value = lino.base || data.base;

    const editor = document.getElementById("unified-config-editor");
    if (editor && data.yaml) editor.value = data.yaml;

    const statusEl = document.getElementById("unified-status");
    if (statusEl) statusEl.textContent = `Loaded from ${data.path}`;

    if (typeof updateBringupSummary === "function") updateBringupSummary();
  } catch (e) {
    console.warn("Failed to load robot_config.yaml:", e);
  }
}

async function saveRobotConfigFromBringup() {
  const statusEl = document.getElementById("bringup-config-status");
  if (statusEl) statusEl.textContent = "Updating robot_config.yaml...";
  // Fall back to what is already saved, never to a hardcoded default. These
  // controls are populated asynchronously from the sensor registry, so an
  // action fired before that lands used to read them as empty and write the
  // empty string straight over a working robot_config.yaml -- laser_sensor
  // silently became "", the LiDAR port reverted to /dev/ydlidar, and the next
  // bringup came up with no laser at all. The saved value wins over a blank
  // control; only a value the user can actually see may overwrite it.
  const saved = state.config || {};
  const keep = (id, savedVal, fallback) => {
    const v = document.getElementById(id)?.value;
    if (v !== undefined && v !== "") return v;
    if (savedVal !== undefined && savedVal !== "" && savedVal !== null) return savedVal;
    return fallback;
  };
  const payload = {
    base: keep("bringup-base-type", saved.base_type, "2wd"),
    laser_sensor: keep("bringup-laser-sensor", saved.laser_sensor, ""),
    depth_sensor: keep("bringup-depth-sensor", saved.depth_sensor, ""),
    micro_ros_port: keep("bringup-agent-device", saved.agent_device, "/dev/ttyACM0"),
    micro_ros_baudrate: keep("bringup-agent-baud", saved.agent_baud, "1500000"),
  };
  try {
    const res = await fetch("/api/robot_config", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(payload),
    });
    const data = await res.json();
    if (data.status === "saved") {
      if (statusEl) statusEl.innerHTML = `<span style="color: var(--success);">✓ Updated robot_config.yaml</span>`;
      loadRobotConfig();
      logLine(`[console] Updated robot parameters in ${data.path}`);
    } else {
      if (statusEl) statusEl.textContent = data.error || "Update failed";
    }
  } catch (e) {
    if (statusEl) statusEl.textContent = `Error: ${e.message}`;
  }
}

async function saveRobotConfigFromEditor() {
  const editor = document.getElementById("unified-config-editor");
  const statusEl = document.getElementById("unified-status");
  if (!editor || !editor.value.trim()) return;
  if (statusEl) statusEl.textContent = "Saving robot_config.yaml...";
  try {
    const distro = getDistro();
    const res = await fetch("/api/robot_config", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ yaml: editor.value, distro }),
    });
    const data = await res.json();
    if (data.status === "saved") {
      if (statusEl) statusEl.innerHTML = `<span style="color: var(--success);">✓ Saved ${data.path}</span>`;
      loadRobotConfig();
      logLine(`[console] robot_config.yaml saved (single source of truth)`);
      refreshConfigGit();
    } else {
      if (statusEl) statusEl.textContent = data.error || "Save failed";
    }
  } catch (e) {
    if (statusEl) statusEl.textContent = `Error: ${e.message}`;
  }
}

async function exportRobotConfigFile() {
  try {
    const distro = getDistro();
    const res = await fetch(`/api/robot_config?distro=${distro}`);
    const data = await res.json();
    if (data.yaml) {
      downloadFile("robot_config.yaml", data.yaml, "text/yaml;charset=utf-8");
      logLine(`[console] Exported robot_config.yaml`);
    }
  } catch (e) {
    alert(`Export failed: ${e.message}`);
  }
}

// Wire Event Listeners
document.getElementById("btn-save-robot-config-bringup")?.addEventListener("click", saveRobotConfigFromBringup);
document.getElementById("btn-unified-load")?.addEventListener("click", loadRobotConfig);
document.getElementById("btn-unified-save")?.addEventListener("click", saveRobotConfigFromEditor);
document.getElementById("btn-unified-export")?.addEventListener("click", exportRobotConfigFile);
document.getElementById("btn-export-unified")?.addEventListener("click", exportRobotConfigFile);

// Sync kinematics across tabs
document.getElementById("bringup-base-type")?.addEventListener("change", (e) => {
  const inst = document.getElementById("install-base");
  if (inst) inst.value = e.target.value;
});
document.getElementById("install-base")?.addEventListener("change", (e) => {
  const b = document.getElementById("bringup-base-type");
  if (b) b.value = e.target.value;
});

// ---------- Wi-Fi & Network Secrets Management (secrets.yaml) ----------
async function loadSecrets() {
  const pill = document.getElementById("secrets-status-pill");
  try {
    const res = await fetch("/api/secrets");
    const data = await res.json();
    if (!data.success) throw new Error(data.detail || "Failed to load secrets");

    const sec = data.secrets || {};
    const wifi = sec.wifi || {};
    const uros = sec.micro_ros || {};
    const telem = sec.telemetry || {};

    const elSsid = document.getElementById("secrets-wifi-ssid");
    if (elSsid) elSsid.value = wifi.ssid || "";

    const elPw = document.getElementById("secrets-wifi-password");
    if (elPw) elPw.value = wifi.password || "";

    const elIp = document.getElementById("secrets-wifi-static-ip");
    if (elIp) elIp.value = wifi.static_ip || "";

    const elGw = document.getElementById("secrets-wifi-gateway");
    if (elGw) elGw.value = wifi.gateway || "";

    const elSub = document.getElementById("secrets-wifi-subnet");
    if (elSub) elSub.value = wifi.subnet || "255.255.255.0";

    const elDns = document.getElementById("secrets-wifi-dns");
    if (elDns) elDns.value = wifi.dns || "8.8.8.8";

    const elAgentIp = document.getElementById("secrets-agent-ip");
    if (elAgentIp) elAgentIp.value = uros.agent_ip || "";

    const elAgentPort = document.getElementById("secrets-agent-port");
    if (elAgentPort) elAgentPort.value = uros.agent_port || 8888;

    const elSyslogSrv = document.getElementById("secrets-syslog-server");
    if (elSyslogSrv) elSyslogSrv.value = telem.syslog_server || uros.agent_ip || "";

    const elSyslogPort = document.getElementById("secrets-syslog-port");
    if (elSyslogPort) {
      const defaultSyslogPort = (data.is_rootless || data.bound_syslog_port === 5140) ? 5140 : 514;
      elSyslogPort.value = telem.syslog_port || defaultSyslogPort;
      if (data.is_rootless && elSyslogPort.value == 514) {
        elSyslogPort.value = 5140;
      }
    }

    const elOtaPort = document.getElementById("secrets-ota-port");
    if (elOtaPort) elOtaPort.value = telem.ota_port || 3232;

    const rawEditor = document.getElementById("secrets-raw-editor");
    if (rawEditor && data.raw_yaml) rawEditor.value = data.raw_yaml;

    if (pill) {
      if (data.exists) {
        pill.textContent = "✅ secrets.yaml Active";
        pill.className = "pill pill-ok";
      } else {
        pill.textContent = "⚠️ secrets.yaml Not Created (Using defaults)";
        pill.className = "pill pill-warn";
      }
    }
  } catch (err) {
    console.warn("loadSecrets error:", err);
    if (pill) {
      pill.textContent = "Error loading";
      pill.className = "pill pill-error";
    }
  }
}

async function saveSecrets() {
  const statusEl = document.getElementById("secrets-save-status");
  const pill = document.getElementById("secrets-status-pill");
  const rawContainer = document.getElementById("secrets-raw-container");
  const rawEditor = document.getElementById("secrets-raw-editor");

  if (statusEl) statusEl.innerHTML = `<span style="color:var(--text-dim);">Saving secrets.yaml...</span>`;

  let payload = {};
  if (rawContainer && rawContainer.style.display !== "none" && rawEditor && rawEditor.value.trim()) {
    payload = { raw_yaml: rawEditor.value };
  } else {
    payload = {
      secrets: {
        wifi: {
          ssid: document.getElementById("secrets-wifi-ssid")?.value.trim() || "",
          password: document.getElementById("secrets-wifi-password")?.value || "",
          static_ip: document.getElementById("secrets-wifi-static-ip")?.value.trim() || "",
          gateway: document.getElementById("secrets-wifi-gateway")?.value.trim() || "",
          subnet: document.getElementById("secrets-wifi-subnet")?.value.trim() || "255.255.255.0",
          dns: document.getElementById("secrets-wifi-dns")?.value.trim() || "8.8.8.8",
        },
        micro_ros: {
          agent_ip: document.getElementById("secrets-agent-ip")?.value.trim() || "192.168.1.100",
          agent_port: parseInt(document.getElementById("secrets-agent-port")?.value, 10) || 8888,
        },
        telemetry: {
          syslog_server: document.getElementById("secrets-syslog-server")?.value.trim() || "192.168.1.100",
          syslog_port: parseInt(document.getElementById("secrets-syslog-port")?.value, 10) || 514,
          ota_port: parseInt(document.getElementById("secrets-ota-port")?.value, 10) || 3232,
        }
      }
    };
  }

  try {
    const res = await fetch("/api/secrets", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(payload),
    });
    const data = await res.json();
    if (data.success) {
      if (statusEl) statusEl.innerHTML = `<span style="color:var(--success);">✓ Saved secrets.yaml &amp; regenerated firmware headers</span>`;
      if (pill) {
        pill.textContent = "✅ secrets.yaml Active";
        pill.className = "pill pill-ok";
      }
      logLine(`[console] Saved secrets.yaml (gitignored credentials quarantine)`);
      if (data.generator_stdout) {
        logLine(`[generator] ${data.generator_stdout.trim()}`);
      }
      // Reload values & raw YAML to stay in sync
      await loadSecrets();
    } else {
      if (statusEl) statusEl.innerHTML = `<span style="color:var(--danger);">${data.detail || data.message || "Failed to save"}</span>`;
    }
  } catch (err) {
    if (statusEl) statusEl.innerHTML = `<span style="color:var(--danger);">Error: ${err.message}</span>`;
  }
}

document.getElementById("btn-save-secrets")?.addEventListener("click", saveSecrets);
document.getElementById("btn-reload-secrets")?.addEventListener("click", loadSecrets);
document.getElementById("btn-toggle-secrets-pw")?.addEventListener("click", () => {
  const inp = document.getElementById("secrets-wifi-password");
  if (!inp) return;
  inp.type = inp.type === "password" ? "text" : "password";
});
document.getElementById("btn-toggle-secrets-yaml")?.addEventListener("click", () => {
  const box = document.getElementById("secrets-raw-container");
  if (!box) return;
  box.style.display = box.style.display === "none" ? "block" : "none";
});
document.getElementById("link-goto-secrets")?.addEventListener("click", (e) => {
  e.preventDefault();
  const btn = document.querySelector('.tab-btn[data-tab="secrets"]') || document.querySelector('.tab-btn[data-tab="settings"]');
  if (btn) btn.click();
  setTimeout(() => {
    document.getElementById("card-secrets-settings")?.scrollIntoView({ behavior: "smooth" });
  }, 100);
});

// Auto-load on startup
setTimeout(() => {
  loadRobotConfig();
  loadSecrets();
}, 400);



// ---------- micro-ROS Agent Port Conflict Detection & Lifecycle ----------
async function checkAgentPortStatus(opts = {}) {
  const c = state.config || {};
  const transport = document.getElementById("cfg-agent-transport")?.value || c.agent_transport || "serial";
  const device = document.getElementById("cfg-agent-device")?.value || c.agent_device || "/dev/ttyACM0";
  const port = document.getElementById("cfg-agent-port")?.value || c.agent_port || "8888";
  
  const statusPill = document.getElementById("agent-port-status-pill");
  if (statusPill) {
    statusPill.style.display = "inline-block";
    statusPill.textContent = "Checking...";
    statusPill.className = "pill pill-starting";
  }

  try {
    const res = await fetch("/api/agent/port_check", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({
        port: device,
        mode: transport,
        udp_port: parseInt(port, 10) || 8888
      }),
    }).then(r => r.json());

    if (statusPill) {
      if (res.in_use) {
        statusPill.textContent = "Port Busy";
        statusPill.className = "pill pill-warn";
      } else {
        statusPill.textContent = "Port Available";
        statusPill.className = "pill pill-ok";
      }
    }

    if (res.in_use && !opts.silent) {
      document.getElementById("port-conflict-summary").textContent = res.summary || "Port is currently in use";
      document.getElementById("port-conflict-details").textContent = res.details || JSON.stringify(res, null, 2);
      document.getElementById("port-modal-overlay").classList.add("open");
    }
    return res;
  } catch (err) {
    if (statusPill) {
      statusPill.textContent = "Check Failed";
      statusPill.className = "pill pill-off";
    }
    return { in_use: false, error: err.message };
  }
}

async function releaseAgentPort() {
  const c = state.config || {};
  const transport = document.getElementById("cfg-agent-transport")?.value || c.agent_transport || "serial";
  const device = document.getElementById("cfg-agent-device")?.value || c.agent_device || "/dev/ttyACM0";
  const port = document.getElementById("cfg-agent-port")?.value || c.agent_port || "8888";

  const btnRel = document.getElementById("btn-release-port-conflict");
  if (btnRel) {
    btnRel.disabled = true;
    btnRel.textContent = "Releasing...";
  }

  try {
    const res = await fetch("/api/agent/port_release", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({
        port: device,
        mode: transport,
        udp_port: parseInt(port, 10) || 8888
      }),
    }).then(r => r.json());

    document.getElementById("port-modal-overlay").classList.remove("open");
    await checkAgentPortStatus({ silent: true });
    logLine("[console] micro-ROS agent port released successfully.");
  } catch (err) {
    alert("Failed to release port: " + err.message);
  } finally {
    if (btnRel) {
      btnRel.disabled = false;
      btnRel.textContent = "⚡ Release Port & Stop Agent";
    }
  }
}

document.getElementById("btn-agent-port-check")?.addEventListener("click", () => checkAgentPortStatus());
document.getElementById("hdr-btn-agent-check")?.addEventListener("click", () => checkAgentPortStatus());
document.getElementById("btn-close-port-modal")?.addEventListener("click", () => {
  document.getElementById("port-modal-overlay")?.classList.remove("open");
});
document.getElementById("btn-ignore-port-conflict")?.addEventListener("click", () => {
  document.getElementById("port-modal-overlay")?.classList.remove("open");
});
document.getElementById("btn-release-port-conflict")?.addEventListener("click", () => releaseAgentPort());

