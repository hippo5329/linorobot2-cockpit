// Linorobot2 Cockpit frontend -- workflow setup & rootless container controller, autostart-on-boot, UDP syslog server, dashboard integration.
// Part of the app.js split: a classic script sharing global scope. See app-core.js.

// =============================================================================
// WORKFLOW SETUP & ROOTLESS CONTAINER CONTROLLER
// =============================================================================

// Automated Rootless Docker Setup Helper

function showToast(message, duration = 3500) {
  let toast = document.getElementById("toast");
  if (!toast) {
    toast = document.createElement("div");
    toast.id = "toast";
    document.body.appendChild(toast);
  }
  toast.textContent = message;
  toast.classList.add("show");
  if (toast._timer) clearTimeout(toast._timer);
  toast._timer = setTimeout(() => {
    toast.classList.remove("show");
  }, duration);
}

async function ensureContainerEngine(engine) {
  if (engine === "native") return { installed: true };
  const targetEngine = (engine === "podman" || engine === "podman_systemd") ? "podman" : "docker";

  try {
    const status = await fetch("/api/docker/status").then(r => r.json());
    if (targetEngine === "podman" && !status.has_podman) {
      showToast("🦭 Podman not found on system. Installing automatically...", 5000);
      const res = await fetch("/api/container/install", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ engine: "podman" })
      }).then(r => r.json());
      if (res.installed) {
        showToast("✅ Podman installed successfully!", 4000);
      } else {
        showToast("⚠️ Podman auto-install failed: " + res.message, 6000);
      }
      return res;
    } else if (targetEngine === "docker") {
      if (!status.has_docker) {
        showToast("🐳 Docker not found on system. Installing Rootless Docker automatically...", 6000);
        const res = await fetch("/api/container/install", {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify({ engine: "docker" })
        }).then(r => r.json());
        if (res.installed) {
          showToast("✅ Docker (Rootless) installed and configured!", 4000);
        } else {
          showToast("⚠️ Docker auto-install failed: " + res.message, 6000);
        }
        return res;
      } else if (!status.is_rootless_docker && status.platform_system === "Linux") {
        await triggerRootlessDockerSetup(false);
      }
    }
  } catch (e) {
    console.warn("Container auto-check error:", e);
  }
}


async function triggerRootlessDockerSetup(isManual = false) {
  const statusText = document.getElementById("rootless-status-text");
  const btnSetup = document.getElementById("btn-setup-rootless-docker");
  if (statusText) statusText.textContent = "⚡ Configuring Rootless Docker daemon...";
  if (btnSetup) {
    btnSetup.disabled = true;
    btnSetup.textContent = "Setting up...";
  }

  try {
    const res = await fetch("/api/docker/setup_rootless", { method: "POST" }).then(r => r.json());
    if (statusText) {
      if (res.success || res.is_rootless) {
        statusText.textContent = `✅ ${res.message || "Rootless Docker active!"}`;
        if (!isManual) showToast("🐳 Rootless Docker was automatically configured for this session.");
      } else {
        statusText.textContent = `⚠️ ${res.message || "Setup completed with warnings. Check logs."}`;
      }
    }
    return res;
  } catch (err) {
    if (statusText) statusText.textContent = `⚠️ Setup error: ${err.message}`;
    return { success: false, error: err.message };
  } finally {
    if (btnSetup) {
      btnSetup.disabled = false;
      btnSetup.textContent = "⚡ Setup Rootless Now";
    }
  }
}

async function checkAndAutoSetupRootlessDocker() {
  try {
    const status = await fetch("/api/docker/status").then(r => r.json());
    if (status.platform_system === "Linux" && status.has_docker && !status.is_rootless_docker) {
      console.log("[Linorobot2 Console] Auto-configuring rootless Docker...");
      await triggerRootlessDockerSetup(false);
    }
  } catch (e) {}
}


async function openRootlessModal() {
  const modal = document.getElementById("modal-rootless-docker");
  if (!modal) return;
  modal.style.display = "flex";
  const statusText = document.getElementById("rootless-status-text");
  if (statusText) statusText.textContent = "Status: Checking local container engine...";
  try {
    const res = await fetch("/api/docker/rootless_info");
    const info = await res.json();
    if (statusText) {
      if (info.is_rootless) {
        statusText.textContent = `✅ Rootless Docker is active for user '${info.user}' (UID ${info.uid})`;
      } else if (info.has_docker) {
        statusText.textContent = `⚠️ Docker is running in standard (rootful) mode. Run setup below to enable rootless daemon.`;
      } else if (info.has_podman) {
        statusText.textContent = `✅ Podman is available (Rootless by default, no daemon needed).`;
      } else {
        statusText.textContent = `ℹ️ Neither Docker nor Podman found. Follow setup below to install.`;
      }
    }
  } catch (e) {
    if (statusText) statusText.textContent = "ℹ️ Container status check complete.";
  }
}

function closeRootlessModal() {
  const modal = document.getElementById("modal-rootless-docker");
  if (modal) modal.style.display = "none";
}

function initWorkflowSetup() {
  const hdrDistro = document.getElementById("hdr-distro-select");
  const cfgDistro = document.getElementById("cfg-ros-distro");
  const hdrMode = document.getElementById("hdr-install-mode");
  const tabMode = document.getElementById("install-mode");
  const hdrAgent = document.getElementById("hdr-agent-engine");
  const cfgAgent = document.getElementById("cfg-agent-engine");
  const btnRootlessHdr = document.getElementById("btn-rootless-guide");
  const btnRootlessSettings = document.getElementById("btn-settings-rootless-guide");
  const btnCloseModal = document.getElementById("btn-close-rootless-modal");
  const btnCloseModalFoot = document.getElementById("btn-close-rootless-modal-foot");
  const btnTestDaemon = document.getElementById("btn-test-rootless-daemon");
  const btnCopyUbuntu = document.getElementById("btn-copy-rootless-ubuntu");
  const btnCopyPodman = document.getElementById("btn-copy-rootless-podman");

  // Restore saved choices from localStorage if available
  const savedMode = localStorage.getItem("linorobot2_install_mode");
  if (savedMode) {
    if (hdrMode) hdrMode.value = savedMode;
    if (tabMode) {
      tabMode.value = savedMode;
      const isNative = savedMode === "native";
      const natCards = document.getElementById("install-native-cards");
      const dkrCard = document.getElementById("install-docker-card");
      if (natCards) natCards.style.display = isNative ? "block" : "none";
      if (dkrCard) dkrCard.style.display = isNative ? "none" : "block";
    }
  }

  const savedAgent = localStorage.getItem("linorobot2_agent_engine");
  if (savedAgent) {
    if (hdrAgent) hdrAgent.value = savedAgent;
    if (cfgAgent) cfgAgent.value = savedAgent;
  }

  // 1. Distro Sync
  if (hdrDistro) {
    hdrDistro.addEventListener("change", () => {
      const val = hdrDistro.value;
      if (cfgDistro) cfgDistro.value = val;
      localStorage.setItem("linorobot2_ros_distro", val);
      fetch("/api/config", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ ros_distro: val }),
      }).then(refreshStatus);
    });
  }

  // 2. Install / Execution Mode Sync
  const onModeChange = (mode) => {
    if (hdrMode) hdrMode.value = mode;
    if (tabMode) tabMode.value = mode;
    const isNative = mode === "native";
    const natCards = document.getElementById("install-native-cards");
    const dkrCard = document.getElementById("install-docker-card");
    if (natCards) natCards.style.display = isNative ? "block" : "none";
    if (dkrCard) dkrCard.style.display = isNative ? "none" : "block";
    localStorage.setItem("linorobot2_install_mode", mode);
    ensureContainerEngine(mode);
    fetch("/api/config", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ install_mode: mode }),
    }).then(refreshStatus);
  };

  if (hdrMode) {
    hdrMode.addEventListener("change", () => onModeChange(hdrMode.value));
  }
  if (tabMode) {
    tabMode.addEventListener("change", () => onModeChange(tabMode.value));
  }

  // 3. micro-ROS Agent Engine Sync
  const onAgentEngineChange = (engine) => {
    if (hdrAgent) hdrAgent.value = engine;
    if (cfgAgent) cfgAgent.value = engine;
    const chk = document.getElementById("cfg-agent-use-docker");
    if (chk) chk.checked = (engine !== "native");
    localStorage.setItem("linorobot2_agent_engine", engine);
    ensureContainerEngine(engine);
    fetch("/api/config", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ agent_engine: engine }),
    }).then(refreshStatus);
  };

  if (hdrAgent) {
    hdrAgent.addEventListener("change", () => onAgentEngineChange(hdrAgent.value));
  }
  if (cfgAgent) {
    cfgAgent.addEventListener("change", () => onAgentEngineChange(cfgAgent.value));
  }

  // 4. Container Registry Sync
  const hdrReg = document.getElementById("hdr-container-registry");
  const hdrCustomReg = document.getElementById("hdr-custom-registry");
  const cfgReg = document.getElementById("cfg-container-registry");
  const cfgCustomReg = document.getElementById("cfg-custom-registry");

  const syncRegistryState = (val, customVal) => {
    const isCustom = (val === "custom");
    if (hdrReg) hdrReg.value = val;
    if (cfgReg) cfgReg.value = val;
    if (hdrCustomReg) {
      if (customVal !== undefined) hdrCustomReg.value = customVal;
      hdrCustomReg.style.display = isCustom ? "inline-block" : "none";
      if (isCustom) hdrCustomReg.focus();
    }
    if (cfgCustomReg) {
      if (customVal !== undefined) cfgCustomReg.value = customVal;
      cfgCustomReg.style.display = isCustom ? "block" : "none";
    }
  };

  const savedReg = localStorage.getItem("linorobot2_container_registry");
  const savedCustomReg = localStorage.getItem("linorobot2_custom_registry");
  if (savedReg) {
    syncRegistryState(savedReg, savedCustomReg || "");
  }

  const onRegistryChange = (val) => {
    syncRegistryState(val);
    localStorage.setItem("linorobot2_container_registry", val);
    const custom = (hdrCustomReg ? hdrCustomReg.value : (cfgCustomReg ? cfgCustomReg.value : "")).trim();
    fetch("/api/config", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ container_registry: val, custom_registry: custom }),
    }).then(refreshStatus);
  };

  const onCustomRegistryInput = (custom) => {
    if (hdrCustomReg) hdrCustomReg.value = custom;
    if (cfgCustomReg) cfgCustomReg.value = custom;
    localStorage.setItem("linorobot2_custom_registry", custom);
    fetch("/api/config", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ custom_registry: custom }),
    });
  };

  if (hdrReg) hdrReg.addEventListener("change", () => onRegistryChange(hdrReg.value));
  if (cfgReg) cfgReg.addEventListener("change", () => onRegistryChange(cfgReg.value));
  if (hdrCustomReg) hdrCustomReg.addEventListener("input", () => onCustomRegistryInput(hdrCustomReg.value));
  if (cfgCustomReg) cfgCustomReg.addEventListener("input", () => onCustomRegistryInput(cfgCustomReg.value));

  // 4. Modal listeners
  if (btnRootlessHdr) btnRootlessHdr.addEventListener("click", openRootlessModal);
  if (btnRootlessSettings) btnRootlessSettings.addEventListener("click", openRootlessModal);
  if (btnCloseModal) btnCloseModal.addEventListener("click", closeRootlessModal);
  if (btnCloseModalFoot) btnCloseModalFoot.addEventListener("click", closeRootlessModal);
  if (btnTestDaemon) btnTestDaemon.addEventListener("click", openRootlessModal);
  const btnSetupRootless = document.getElementById("btn-setup-rootless-docker");
  if (btnSetupRootless) btnSetupRootless.addEventListener("click", () => triggerRootlessDockerSetup(true));
  checkAndAutoSetupRootlessDocker();

  // Copy buttons
  if (btnCopyUbuntu) {
    btnCopyUbuntu.addEventListener("click", () => {
      const code = document.getElementById("code-rootless-ubuntu")?.textContent || "";
      navigator.clipboard.writeText(code).then(() => {
        btnCopyUbuntu.textContent = "✅ Copied!";
        setTimeout(() => { btnCopyUbuntu.textContent = "📋 Copy Script"; }, 2000);
      });
    });
  }
  if (btnCopyPodman) {
    btnCopyPodman.addEventListener("click", () => {
      const code = document.getElementById("code-rootless-podman")?.textContent || "";
      navigator.clipboard.writeText(code).then(() => {
        btnCopyPodman.textContent = "✅ Copied!";
        setTimeout(() => { btnCopyPodman.textContent = "📋 Copy"; }, 2000);
      });
    });
  }
}

// Call initWorkflowSetup on DOM ready
if (document.readyState === "loading") {
  document.addEventListener("DOMContentLoaded", initWorkflowSetup);
  // The Output console starts folded to its one-line bar: open, it takes a
  // third of the viewport for an empty box. openTerminal() unfolds it the
  // moment anything runs.
  document.addEventListener("DOMContentLoaded", () => toggleConsole(true));
  // The header's once-only settings rows fold behind one button; the choice
  // is remembered per browser and starts folded.
  document.addEventListener("DOMContentLoaded", () => {
    const btn = document.getElementById("btn-hdr-settings");
    const rows = document.getElementById("hdr-settings-rows");
    if (!btn || !rows) return;
    const apply = (open) => {
      rows.hidden = !open;
      btn.setAttribute("aria-expanded", open ? "true" : "false");
      btn.textContent = open ? "⚙ Settings ▴" : "⚙ Settings ▾";
    };
    let open = false;
    try { open = localStorage.getItem("hdr_settings_open") === "1"; } catch { /* default folded */ }
    apply(open);
    btn.addEventListener("click", () => {
      open = rows.hidden;
      apply(open);
      try { localStorage.setItem("hdr_settings_open", open ? "1" : "0"); } catch { /* ignore */ }
    });
  });
} else {
  initWorkflowSetup();
}

// =============================================================================
// AUTOSTART ON BOOT CONTROLLER
// =============================================================================
async function refreshAutostartStatus(opts = {}) {
  const pill = document.getElementById("autostart-status-pill");
  const infoBox = document.getElementById("autostart-info-box");
  const summaryText = document.getElementById("autostart-summary-text");
  const detailsText = document.getElementById("autostart-details-text");

  try {
    const res = await fetch("/api/autostart/status").then(r => r.json());
    if (pill) {
      if (res.active) {
        pill.textContent = "active & running";
        pill.className = "pill pill-ok";
      } else if (res.enabled) {
        pill.textContent = "enabled on boot";
        pill.className = "pill pill-starting";
      } else {
        pill.textContent = "disabled";
        pill.className = "pill pill-off";
      }
    }
    if (opts.showInfo && infoBox && summaryText && detailsText) {
      infoBox.style.display = "block";
      summaryText.textContent = `Service: ${res.service_name} | Enabled: ${res.enabled ? "YES" : "NO"} | Active: ${res.active ? "RUNNING" : "STOPPED"} | Lingering: ${res.lingering ? "ENABLED" : "OFF"}`;
      detailsText.textContent = res.details || "No active process status available.";
    }
    return res;
  } catch (err) {
    if (pill) {
      pill.textContent = "check error";
      pill.className = "pill pill-off";
    }
    return { enabled: false, active: false, error: err.message };
  }
}

async function enableBootAutostart() {
  const btn = document.getElementById("btn-autostart-enable");
  const stack = document.getElementById("autostart-stack-select")?.value || "full_nav2";
  const mapPath = document.getElementById("autostart-map-path")?.value || "";
  const distro = getDistro();
  const mode = document.getElementById("hdr-install-mode")?.value || "native";
  const agentEngine = getAgentEngine();

  if (btn) {
    btn.disabled = true;
    btn.textContent = "Enabling...";
  }

  try {
    const res = await fetch("/api/autostart/enable", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({
        stack,
        map_path: mapPath,
        distro,
        mode,
        agent_engine: agentEngine
      })
    }).then(r => r.json());

    if (res.enabled) {
      const infoBox = document.getElementById("autostart-info-box");
      const summaryText = document.getElementById("autostart-summary-text");
      const detailsText = document.getElementById("autostart-details-text");
      if (infoBox && summaryText && detailsText) {
        infoBox.style.display = "block";
        summaryText.textContent = `✅ ${res.message}`;
        detailsText.textContent = `Unit: ${res.service_path}\nScript: ${res.script_path}\n\nStack is set to launch on power-on automatically.`;
      }
    }
    await refreshAutostartStatus();
  } catch (err) {
    alert("Failed to enable autostart: " + err.message);
  } finally {
    if (btn) {
      btn.disabled = false;
      btn.textContent = "⚡ Enable Boot Autostart";
    }
  }
}

async function disableBootAutostart() {
  const btn = document.getElementById("btn-autostart-disable");
  if (btn) {
    btn.disabled = true;
    btn.textContent = "Disabling...";
  }

  try {
    const res = await fetch("/api/autostart/disable", {
      method: "POST",
      headers: { "Content-Type": "application/json" }
    }).then(r => r.json());

    const infoBox = document.getElementById("autostart-info-box");
    const summaryText = document.getElementById("autostart-summary-text");
    const detailsText = document.getElementById("autostart-details-text");
    if (infoBox && summaryText && detailsText) {
      infoBox.style.display = "block";
      summaryText.textContent = `🛑 ${res.message}`;
      detailsText.textContent = "Autostart on boot has been removed.";
    }
    await refreshAutostartStatus();
  } catch (err) {
    alert("Failed to disable autostart: " + err.message);
  } finally {
    if (btn) {
      btn.disabled = false;
      btn.textContent = "🛑 Disable Autostart";
    }
  }
}

async function viewBootAutostartLogs() {
  const infoBox = document.getElementById("autostart-info-box");
  const summaryText = document.getElementById("autostart-summary-text");
  const detailsText = document.getElementById("autostart-details-text");

  try {
    const res = await fetch("/api/autostart/logs").then(r => r.json());
    if (infoBox && summaryText && detailsText) {
      infoBox.style.display = "block";
      summaryText.textContent = `📜 Journald Logs (linorobot2-autostart.service)`;
      detailsText.textContent = res.logs || "No logs recorded.";
    }
  } catch (err) {
    alert("Failed to read autostart logs: " + err.message);
  }
}

function initAutostartListeners() {
  const btnEnable = document.getElementById("btn-autostart-enable");
  const btnDisable = document.getElementById("btn-autostart-disable");
  const btnStatus = document.getElementById("btn-autostart-status");
  const btnLogs = document.getElementById("btn-autostart-logs");

  if (btnEnable) btnEnable.addEventListener("click", enableBootAutostart);
  if (btnDisable) btnDisable.addEventListener("click", disableBootAutostart);
  if (btnStatus) btnStatus.addEventListener("click", () => refreshAutostartStatus({ showInfo: true }));
  if (btnLogs) btnLogs.addEventListener("click", viewBootAutostartLogs);

  refreshAutostartStatus();
}

if (document.readyState === "loading") {
  document.addEventListener("DOMContentLoaded", initAutostartListeners);
} else {
  initAutostartListeners();
}


// ---------- virtual gamepad ----------
// Drives /cmd_vel straight from the page. The server keeps a single rclpy node
// alive (gamepad_publisher.py) and we feed it target velocities; publishing is
// its job, not ours, because a robot has to be told to keep going -- cmd_vel
// that goes quiet means stop. That also gives us the deadman for free: stop
// sending and the node zeroes the robot on its own.
const vgpPad = document.getElementById("vgp-pad");
if (vgpPad) {
  const vgpPost = (url, body) =>
    fetch(url, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(body),
    }).then((r) => r.json());

  const vgpKnob = document.getElementById("vgp-knob");
  const vgpStart = document.getElementById("btn-vgp-start");
  const vgpStop = document.getElementById("btn-vgp-stop");
  const vgpState = document.getElementById("vgp-state");
  const SEND_MS = 100;      // 10 Hz to the server; the node republishes at 20 Hz
  const KNOB_TRAVEL = 0.42; // knob centre stays inside the pad at full deflection

  let vgpRunning = false;
  let vgpTimer = null;
  let vgpStallTimer = null;
  let axes = { x: 0, y: 0 };   // -1..1, y positive = forward
  const keysHeld = new Set();

  const num = (id, fallback) => {
    const v = parseFloat(document.getElementById(id).value);
    return Number.isFinite(v) ? v : fallback;
  };

  function twist() {
    return {
      linear_x: axes.y * num("vgp-max-linear", 0.4),
      linear_y: 0,
      angular_z: -axes.x * num("vgp-max-angular", 1.2),
    };
  }

  function render() {
    document.getElementById("vgp-lx").textContent = twist().linear_x.toFixed(2);
    document.getElementById("vgp-az").textContent = twist().angular_z.toFixed(2);
    vgpKnob.style.left = `${50 + axes.x * KNOB_TRAVEL * 100}%`;
    vgpKnob.style.top = `${50 - axes.y * KNOB_TRAVEL * 100}%`;
  }

  function setAxes(x, y) {
    // clamp into the unit circle so a corner drag isn't faster than a straight one
    const mag = Math.hypot(x, y);
    if (mag > 1) { x /= mag; y /= mag; }
    axes = { x, y };
    render();
  }

  async function tick() {
    if (!vgpRunning) return;
    try {
      const r = await vgpPost("/api/gamepad/cmd", twist());
      if (r && r.running === false) stopGamepad("publisher exited");
    } catch (e) {
      stopGamepad("lost contact with Console");
    }
  }

  // Told to move but not moving: driven into something. Checked on the server,
  // which compares the commanded twist against measured odometry, so it fires
  // for a real robot caught on furniture as readily as for a simulated one
  // against a simulated wall. Two consecutive hits, because a single sample
  // during spin-up is just the robot not having accelerated yet.
  let stallHits = 0;
  async function checkStall() {
    if (!vgpRunning) return;
    try {
      const r = await vgpPost("/api/gamepad/stall", {});
      stallHits = (r && r.stalled) ? stallHits + 1 : 0;
      const el = document.getElementById("vgp-stall");
      if (el) {
        const hit = stallHits >= 2;
        el.style.display = hit ? "block" : "none";
        if (hit) {
          const m = r.measured || {};
          el.textContent = `\u26a0 Robot is not moving \u2014 commanded `
            + `${(r.commanded?.linear_x ?? 0).toFixed(2)} m/s but measuring `
            + `${Math.abs(m.linear_x ?? 0).toFixed(2)} m/s. Something is in the way.`;
        }
      }
    } catch (e) { /* transient */ }
  }

  async function startGamepad() {
    const topic = document.getElementById("vgp-topic").value.trim() || "/cmd_vel";
    vgpState.textContent = "starting...";
    const r = await vgpPost("/api/gamepad/start", { topic });
    if (!r || !r.started) {
      vgpState.textContent = "could not start the publisher (is ROS 2 sourced?)";
      return;
    }
    vgpRunning = true;
    vgpStart.disabled = true;
    vgpStop.disabled = false;
    vgpState.textContent = `publishing ${topic}`;
    vgpTimer = setInterval(tick, SEND_MS);
    vgpStallTimer = setInterval(checkStall, 2000);
    vgpPad.focus();
  }

  async function stopGamepad(reason) {
    vgpRunning = false;
    if (vgpTimer) { clearInterval(vgpTimer); vgpTimer = null; }
    if (vgpStallTimer) { clearInterval(vgpStallTimer); vgpStallTimer = null; }
    stallHits = 0;
    const stallEl = document.getElementById("vgp-stall");
    if (stallEl) stallEl.style.display = "none";
    setAxes(0, 0);
    keysHeld.clear();
    vgpStart.disabled = false;
    vgpStop.disabled = true;
    vgpState.textContent = reason || "stopped";
    try { await vgpPost("/api/gamepad/kill", {}); } catch (e) { /* already gone */ }
  }

  vgpStart.addEventListener("click", startGamepad);
  vgpStop.addEventListener("click", () => stopGamepad());

  // ---- pointer drag ----
  function pointerAxes(ev) {
    const r = vgpPad.getBoundingClientRect();
    const nx = (ev.clientX - (r.left + r.width / 2)) / (r.width / 2);
    const ny = (ev.clientY - (r.top + r.height / 2)) / (r.height / 2);
    setAxes(nx, -ny);
  }
  vgpPad.addEventListener("pointerdown", (ev) => {
    vgpPad.setPointerCapture(ev.pointerId);
    vgpPad.classList.add("vgp-active");
    vgpPad.focus();
    pointerAxes(ev);
  });
  vgpPad.addEventListener("pointermove", (ev) => {
    if (vgpPad.hasPointerCapture(ev.pointerId)) pointerAxes(ev);
  });
  const release = (ev) => {
    if (ev.pointerId !== undefined && vgpPad.hasPointerCapture(ev.pointerId)) {
      vgpPad.releasePointerCapture(ev.pointerId);
    }
    vgpPad.classList.remove("vgp-active");
    setAxes(0, 0);   // spring back to centre: let go and the robot stops
  };
  vgpPad.addEventListener("pointerup", release);
  vgpPad.addEventListener("pointercancel", release);

  // ---- keyboard, only while the pad has focus so it can't hijack form typing ----
  const KEY_AXES = {
    ArrowUp: [0, 1], KeyW: [0, 1],
    ArrowDown: [0, -1], KeyS: [0, -1],
    ArrowLeft: [-1, 0], KeyA: [-1, 0],
    ArrowRight: [1, 0], KeyD: [1, 0],
  };
  function applyKeys() {
    let x = 0, y = 0;
    for (const code of keysHeld) { x += KEY_AXES[code][0]; y += KEY_AXES[code][1]; }
    setAxes(Math.max(-1, Math.min(1, x)), Math.max(-1, Math.min(1, y)));
  }
  vgpPad.addEventListener("keydown", (ev) => {
    if (ev.code === "Space") { ev.preventDefault(); stopGamepad("stopped (space)"); return; }
    if (!KEY_AXES[ev.code]) return;
    ev.preventDefault();
    keysHeld.add(ev.code);
    applyKeys();
  });
  vgpPad.addEventListener("keyup", (ev) => {
    if (!KEY_AXES[ev.code]) return;
    keysHeld.delete(ev.code);
    applyKeys();
  });
  vgpPad.addEventListener("blur", () => { keysHeld.clear(); applyKeys(); });

  // a page unload would otherwise leave the robot driving until the deadman trips
  window.addEventListener("pagehide", () => {
    if (vgpRunning) navigator.sendBeacon("/api/gamepad/kill", "{}");
  });

  render();
}

// ==============================================================================
// UDP Syslog Server Controller
// ==============================================================================

let syslogEventSource = null;

function updateSyslogUIState(syslogInfo) {
  if (!syslogInfo) return;
  const badge = document.getElementById("syslog-status-badge");
  const btnStart = document.getElementById("btn-syslog-start");
  const btnStop = document.getElementById("btn-syslog-stop");
  const portInput = document.getElementById("cfg-syslog-server-port");
  const statFile = document.getElementById("syslog-stat-file");
  const statPkts = document.getElementById("syslog-stat-packets");
  const statClient = document.getElementById("syslog-stat-client");

  if (badge) {
    if (syslogInfo.running) {
      badge.className = "pill pill-ok";
      badge.textContent = `Syslog: Active (:${syslogInfo.port}${syslogInfo.fallback ? " fallback" : ""})`;
    } else {
      badge.className = "pill pill-warn";
      badge.textContent = "Syslog: Offline";
    }
  }

  if (btnStart) btnStart.style.display = syslogInfo.running ? "none" : "inline-block";
  if (btnStop) btnStop.style.display = syslogInfo.running ? "inline-block" : "none";
  if (portInput) {
    portInput.disabled = syslogInfo.running;
    if (syslogInfo.port && document.activeElement !== portInput) {
      portInput.value = syslogInfo.port;
    }
  }
  if (statFile && syslogInfo.log_file) {
    statFile.textContent = syslogInfo.log_file;
  }
  if (statPkts && typeof syslogInfo.packets_received === "number") {
    statPkts.textContent = syslogInfo.packets_received;
  }
  if (statClient) {
    if (syslogInfo.last_client && syslogInfo.last_client.ip) {
      statClient.textContent = `${syslogInfo.last_client.ip}:${syslogInfo.last_client.port || ""}`;
    } else {
      statClient.textContent = "--";
    }
  }
}

async function startSyslogServer() {
  const portInput = document.getElementById("cfg-syslog-server-port");
  const port = portInput ? parseInt(portInput.value, 10) || 5140 : 5140;
  logLine(`[syslog] Starting UDP Syslog server on port ${port}...`);
  try {
    const res = await fetch("/api/syslog/start", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ port })
    });
    const data = await res.json();
    if (data.running || data.status === "running" || data.status === "ok") {
      logLine(`[syslog] Server online on port ${data.port}${data.fallback ? " (fallback to unprivileged port)" : ""}. Logging to ${data.logfile || data.log_file}`);
    } else {
      logLine(`[syslog error] ${data.error || "Failed to start syslog server"}`);
    }
    refreshStatus();
  } catch (err) {
    logLine(`[syslog error] ${err.message}`);
  }
}

async function stopSyslogServer() {
  logLine("[syslog] Stopping UDP Syslog server...");
  try {
    const res = await fetch("/api/syslog/stop", { method: "POST" });
    const data = await res.json();
    logLine(`[syslog] ${data.message || "Server stopped"}`);
    refreshStatus();
  } catch (err) {
    logLine(`[syslog error] ${err.message}`);
  }
}

async function viewSyslogLogs() {
  try {
    const res = await fetch("/api/syslog/logs?lines=50");
    const data = await res.json();
    if (data.lines && data.lines.length > 0) {
      logLine(`--- Syslog Recent Logs (${data.file}) ---`);
      for (const line of data.lines) {
        logLine(line);
      }
      logLine(`--- End Syslog Logs (${data.total_lines} total lines) ---`);
    } else {
      logLine(`[syslog] Log file empty or not yet created: ${data.file || ""}`);
    }
  } catch (err) {
    logLine(`[syslog error] Could not retrieve logs: ${err.message}`);
  }
}

function connectSyslogStream() {
  if (syslogEventSource) return;
  try {
    syslogEventSource = new EventSource("/api/syslog/stream");
    syslogEventSource.onmessage = (ev) => {
      try {
        const payload = JSON.parse(ev.data);
        const chk = document.getElementById("chk-stream-syslog-terminal");
        if (chk && chk.checked) {
          const sender = payload.client_ip ? `[${payload.client_ip}]` : "";
          logLine(`📡 SYSLOG ${sender} ${payload.raw || payload.message || ev.data}`);
        }
        const statPkts = document.getElementById("syslog-stat-packets");
        if (statPkts && typeof payload.packets_received === "number") {
          statPkts.textContent = payload.packets_received;
        }
        const statClient = document.getElementById("syslog-stat-client");
        if (statClient && payload.client_ip) {
          statClient.textContent = `${payload.client_ip}:${payload.client_port || ""}`;
        }
      } catch {
        const chk = document.getElementById("chk-stream-syslog-terminal");
        if (chk && chk.checked) {
          logLine(`📡 SYSLOG: ${ev.data}`);
        }
      }
    };
    syslogEventSource.onerror = () => {
      // Reconnection handled automatically by browser EventSource
    };
  } catch (e) {
    console.warn("Could not connect syslog stream:", e);
  }
}

// ==============================================================================
// Linorobot2 Cockpit Dashboard Integration
// ==============================================================================

function initCockpitDashboard() {
  const btnOneClick = document.getElementById("btn-cockpit-oneclick");
  const btnStopPipeline = document.getElementById("btn-cockpit-stop-pipeline");
  const targetSelect = document.getElementById("cockpit-target-select");
  const exploreSec = document.getElementById("cockpit-explore-sec");
  const noNav2Check = document.getElementById("cockpit-no-nav2");
  const pipelineBadge = document.getElementById("cockpit-pipeline-badge");
  const pipelineMsg = document.getElementById("cockpit-pipeline-msg");

  const btnVerifyTopics = document.getElementById("btn-cockpit-verify-topics");
  const verifyStatus = document.getElementById("cockpit-verify-status");

  const btnReloadParams = document.getElementById("btn-cockpit-reload-params");
  const btnSaveParams = document.getElementById("btn-cockpit-save-params");
  const paramsYaml = document.getElementById("cockpit-params-yaml");
  const paramsMsg = document.getElementById("cockpit-params-msg");

  const btnRefreshMaps = document.getElementById("btn-cockpit-refresh-maps");
  const mapsContainer = document.getElementById("cockpit-maps-container");

  let activePipelineAbort = null;

  // --- 1. Load Parameters ---
  async function loadParams() {
    if (!paramsYaml) return;
    try {
      if (paramsMsg) paramsMsg.textContent = "Loading parameters from disk...";
      const res = await fetch("/api/params/raw");
      if (!res.ok) throw new Error(`HTTP ${res.status}`);
      const text = await res.text();
      paramsYaml.value = text;
      if (paramsMsg) paramsMsg.textContent = `Parameters loaded from ${state.status?.robot_config_path || "the robot config"}.`;
    } catch (e) {
      if (paramsMsg) paramsMsg.textContent = "Failed to load parameters: " + e.message;
    }
  }

  // --- 2. Save Parameters ---
  async function saveParams() {
    if (!paramsYaml) return;
    try {
      if (paramsMsg) paramsMsg.textContent = "Saving and regenerating firmware headers...";
      const res = await fetch("/api/params/raw", {
        method: "POST",
        headers: { "Content-Type": "text/plain" },
        body: paramsYaml.value,
      });
      const data = await res.json();
      if (res.ok && data.success) {
        if (paramsMsg) paramsMsg.textContent = "✅ " + data.message;
        logLine("[params] " + data.message);
        if (data.generator_stdout) {
          logLine(data.generator_stdout);
        }
      } else {
        throw new Error(data.detail || "Failed to save");
      }
    } catch (e) {
      if (paramsMsg) paramsMsg.textContent = "❌ Error: " + e.message;
      logLine("[params error] " + e.message);
    }
  }

  // --- 3. Saved Maps Gallery ---
  async function loadMaps() {
    if (!mapsContainer) return;
    try {
      mapsContainer.innerHTML = "<span class='hint'>Loading maps...</span>";
      const res = await fetch("/api/maps");
      const data = await res.json();
      const maps = data.maps || [];
      if (maps.length === 0) {
        mapsContainer.innerHTML = "<span class='hint'>No maps found in maps/. Run 1-Click Pipeline or SLAM to create one.</span>";
        return;
      }
      mapsContainer.innerHTML = "";
      for (const m of maps) {
        const card = document.createElement("div");
        card.className = "card";
        card.style.cssText = "background:#0c1220; padding:12px; border-radius:8px; min-width:200px; display:flex; flex-direction:column; gap:8px;";
        
        let imgHtml = "";
        if (m.image) {
          imgHtml = `<img src="/api/maps/${encodeURIComponent(m.image)}" style="max-width:180px; max-height:140px; border-radius:4px; background:#000; object-fit:contain;" alt="${m.name}">`;
        } else {
          imgHtml = `<div style="width:180px; height:120px; background:#161f30; border-radius:4px; display:flex; align-items:center; justify-content:center; color:var(--text-dim); font-size:11px;">No Preview</div>`;
        }

        card.innerHTML = `
          <div style="font-weight:600; font-size:13px; font-family:var(--font-mono);">${m.name}</div>
          <div style="text-align:center;">${imgHtml}</div>
          <div style="display:flex; gap:6px; margin-top:4px;">
            <a href="/api/maps/${encodeURIComponent(m.yaml)}" target="_blank" class="btn btn-mini" style="text-decoration:none; text-align:center; flex:1;">YAML</a>
            ${m.image ? `<a href="/api/maps/${encodeURIComponent(m.image)}" download class="btn btn-mini" style="text-decoration:none; text-align:center; flex:1;">PGM</a>` : ""}
          </div>
        `;
        mapsContainer.appendChild(card);
      }
    } catch (e) {
      mapsContainer.innerHTML = "<span class='hint'>Failed to load maps: " + e.message + "</span>";
    }
  }

  // --- 4. ROS 2 Topic Echo & Rate Audit ---
  async function verifyTopics() {
    if (!btnVerifyTopics) return;
    btnVerifyTopics.disabled = true;
    btnVerifyTopics.textContent = "Auditing (dual QoS)...";
    openTerminal("ROS 2 Topic Rate & Echo Audit");
    logLine("[audit] Initiating ROS 2 Topic Echo & Rate Verification Gate...");

    const setTopicBadge = (topic, badgeId, echoId) => {
      const b = document.getElementById(badgeId);
      const e = document.getElementById(echoId);
      if (b) { b.className = "pill pill-unknown"; b.textContent = "Auditing..."; }
      if (e) { e.textContent = "Measuring frequency & echo payload..."; }
    };
    setTopicBadge("/odom", "topic-badge-odom", "topic-echo-odom");
    setTopicBadge("/imu/data", "topic-badge-imu", "topic-echo-imu");
    setTopicBadge("/scan", "topic-badge-scan", "topic-echo-scan");

    if (verifyStatus) {
      verifyStatus.textContent = "auditing…";
      verifyStatus.className = "hint";
    }

    try {
      const res = await fetch("/api/topics/verify");
      const json = await res.json();
      if (json.raw) logLine(json.raw);

      if (json.data && json.data.topics) {
        const topics = json.data.topics;
        const updateUI = (name, badgeId, echoId) => {
          const t = topics[name];
          const badge = document.getElementById(badgeId);
          const echo = document.getElementById(echoId);
          if (!t) return;
          if (badge) {
            badge.className = t.passed ? "pill pill-ok" : "pill pill-off";
            badge.textContent = `${t.hz} Hz`;
          }
          if (echo) {
            echo.textContent = t.echo || (t.received ? "Received" : "NO DATA");
          }
        };
        updateUI("/odom", "topic-badge-odom", "topic-echo-odom");
        updateUI("/imu/data", "topic-badge-imu", "topic-echo-imu");
        updateUI("/scan", "topic-badge-scan", "topic-echo-scan");
      }
      // The verdict, beside the button. It was only ever written to the
      // terminal, where an audit run before a long bringup scrolls out of
      // sight -- and `verifyStatus` was declared here and then never assigned,
      // so the element had no writer at all.
      if (verifyStatus) {
        const topics = (json.data && json.data.topics) || {};
        const names = Object.keys(topics);
        const passed = names.filter((n) => topics[n].passed);
        if (!names.length) {
          verifyStatus.textContent = "no topics — is the stack running?";
          verifyStatus.className = "hint pill-off";
        } else {
          verifyStatus.textContent = `${passed.length}/${names.length} topics passed`;
          verifyStatus.className = passed.length === names.length
            ? "hint pill-ok" : "hint pill-warn";
        }
      }
    } catch (e) {
      logLine("[audit error] " + e.message);
      if (verifyStatus) {
        verifyStatus.textContent = "audit failed: " + e.message;
        verifyStatus.className = "hint pill-off";
      }
    } finally {
      btnVerifyTopics.disabled = false;
      btnVerifyTopics.textContent = "⚡ Run Rate & Echo Audit";
    }
  }

  // --- 7. One-Click Autonomous Pipeline ---
  const hdrMode = document.getElementById("hdr-pipeline-mode");
  const cockpitMode = document.getElementById("cockpit-pipeline-mode");
  if (hdrMode && cockpitMode) {
    hdrMode.addEventListener("change", () => { cockpitMode.value = hdrMode.value; });
    cockpitMode.addEventListener("change", () => { hdrMode.value = cockpitMode.value; });
  }

  const btnHdrDeploy = document.getElementById("btn-header-start-pipeline");
  const btnHdrDeployText = document.getElementById("hdr-deploy-btn-text");

  async function runOneClick() {
    if (!btnOneClick && !btnHdrDeploy) return;
    const controller = targetSelect ? targetSelect.value : (state.status?.controller || "pico2");
    const sec = exploreSec ? parseInt(exploreSec.value, 10) || 15 : 15;
    const noNav2 = noNav2Check ? noNav2Check.checked : false;
    const mode = (hdrMode && hdrMode.value) || (cockpitMode && cockpitMode.value) || "fake";
    // Force a write even over a board that already runs this build (--flash).
    // Unticked by default; auto-update below is what handles the ordinary case.
    const updateFw = document.getElementById("cockpit-update-firmware")?.checked || false;
    // Ticked by default, and the default must survive a missing element: `?? true`
    // rather than `|| true`, since `|| true` would read an UNTICKED box as on and
    // make the setting impossible to turn off from the UI.
    const autoUpdate = document.getElementById("cockpit-auto-update")?.checked ?? true;

    if (btnOneClick) btnOneClick.disabled = true;
    if (btnHdrDeploy) btnHdrDeploy.disabled = true;
    if (btnHdrDeployText) btnHdrDeployText.innerHTML = "<strong>Running...</strong>";
    if (btnStopPipeline) btnStopPipeline.disabled = false;
    if (pipelineBadge) {
      pipelineBadge.className = "pill pill-ok";
      pipelineBadge.textContent = "Running...";
    }
    if (pipelineMsg) pipelineMsg.textContent = `Pipeline active: ${controller} [mode: ${mode}], exploring for ${sec}s...`;

    openTerminal(`One-Click Pipeline (${controller})`);
    logLine(`[pipeline] Starting automated pipeline for robot '${state.robot_name || "?"}', base controller '${controller}' (explore: ${sec}s, no_nav2: ${noNav2}, mode: ${mode}, auto_update: ${autoUpdate}, force_update: ${updateFw})...`);

    const abortCtrl = new AbortController();
    activePipelineAbort = abortCtrl;
    // flash_mcu.py prints one "NEXT ACTION: ..." line when a flash fails; it
    // becomes the banner instead of "see terminal for recovery instructions".
    let nextAction = "";
    hideActionBanner();

    try {
      const distroSel = document.getElementById("hdr-distro-select");
      const distroVal = distroSel ? distroSel.value : "";
      const distroParam = distroVal ? `&distro=${encodeURIComponent(distroVal)}` : "";
      // Send the robot, not just the controller. Without it the pipeline picks
      // the config from the controller name alone and takes the alphabetically
      // first match when two robots share one -- so a run with rover_pico2
      // selected in this very header executed linorobot2_config.yaml's
      // kinematics, pins, EKF, SLAM and Nav2 parameters, and said so only in
      // one line deep in the log.
      const robotName = state.robot_name || document.getElementById("hdr-robot-name")?.value || "";
      const robotParam = robotName ? `&robot=${encodeURIComponent(robotName)}` : "";
      const url = `/api/workflow/one-click/stream?controller=${encodeURIComponent(controller)}&explore_sec=${sec}&no_nav2=${noNav2}&mode=${encodeURIComponent(mode)}&flash_firmware=${updateFw}&auto_update=${autoUpdate}${distroParam}${robotParam}`;
      const res = await fetch(url, { signal: abortCtrl.signal });
      if (!res.ok || !res.body) {
        logLine(`[pipeline] Failed to start: HTTP ${res.status}`);
        return;
      }
      const reader = res.body.getReader();
      const decoder = new TextDecoder();
      let buf = "";
      while (true) {
        const { value, done } = await reader.read();
        if (done) break;
        buf += decoder.decode(value, { stream: true });
        const frames = buf.split("\n\n");
        buf = frames.pop();
        for (const frame of frames) {
          const evMatch = frame.match(/^event: (.+)$/m);
          const dataMatch = frame.match(/^data: (.+)$/m);
          if (!dataMatch) continue;
          const evType = evMatch ? evMatch[1] : "output";
          try {
            const payload = JSON.parse(dataMatch[1]);
            if (evType === "output") {
              logLine(payload.line);
              const na = /NEXT ACTION: (.+)$/.exec(payload.line || "");
              if (na) nextAction = na[1].trim();
            } else if (evType === "done") {
              logLine(`[pipeline] Mission finished with exit code ${payload.exit_code}`);
              if (payload.exit_code !== 0 && nextAction) {
                showActionBanner(`Flashing failed. Next: ${nextAction}`, "The full recovery steps are in the Output console.");
              }
              if (pipelineBadge) {
                pipelineBadge.className = payload.exit_code === 0 ? "pill pill-ok" : "pill pill-off";
                pipelineBadge.textContent = payload.exit_code === 0 ? "PASSED" : `FAILED (${payload.exit_code})`;
              }
              if (pipelineMsg) {
                pipelineMsg.textContent = payload.exit_code === 0 ? "✅ Mission succeeded! Map saved and Nav2 verified." : `❌ Pipeline halted (exit code ${payload.exit_code}). Flashing or verification failed; see terminal for recovery instructions.`;
              }
              if (payload.exit_code !== 0) {
                showToast(`❌ Pipeline halted: Execution failed (exit code ${payload.exit_code}). Check terminal for recovery steps!`, 7000);
              }
            }
          } catch {}
        }
      }
    } catch (e) {
      if (e.name === "AbortError") {
        logLine("[pipeline] Aborted by user.");
      } else {
        logLine("[pipeline error] " + e.message);
      }
    } finally {
      activePipelineAbort = null;
      if (btnOneClick) btnOneClick.disabled = false;
      if (btnHdrDeploy) btnHdrDeploy.disabled = false;
      if (btnHdrDeployText) btnHdrDeployText.innerHTML = "<strong>Start 1-Click</strong>";
      if (btnStopPipeline) btnStopPipeline.disabled = true;
      loadMaps();
    }
  }

  function abortOneClick() {
    if (activePipelineAbort) {
      activePipelineAbort.abort();
      activePipelineAbort = null;
    }
    if (btnHdrDeploy) btnHdrDeploy.disabled = false;
    if (btnHdrDeployText) btnHdrDeployText.innerHTML = "<strong>Start 1-Click</strong>";
    if (pipelineBadge) {
      pipelineBadge.className = "pill pill-off";
      pipelineBadge.textContent = "Aborted";
    }
  }

  // Wire event listeners
  if (btnOneClick) btnOneClick.addEventListener("click", runOneClick);
  if (btnHdrDeploy) btnHdrDeploy.addEventListener("click", runOneClick);
  if (btnStopPipeline) btnStopPipeline.addEventListener("click", abortOneClick);
  if (btnVerifyTopics) btnVerifyTopics.addEventListener("click", verifyTopics);
  if (btnReloadParams) btnReloadParams.addEventListener("click", loadParams);
  if (btnSaveParams) btnSaveParams.addEventListener("click", saveParams);
  if (btnRefreshMaps) btnRefreshMaps.addEventListener("click", loadMaps);

  function updateWifiHint() {
    const wifiHint = document.getElementById("cockpit-wifi-hint");
    if (!wifiHint || !targetSelect) return;
    const val = targetSelect.value;
    // Every ESP32 build compiles the radio in; the W boards have one too. The
    // esp32_wifi entry went with its config on 2026-09-20.
    const isWifi = ["pico2w", "picow", "esp32", "gendrv", "esp32s3"].includes(val);
    wifiHint.style.display = isWifi ? "block" : "none";
  }
  if (targetSelect) {
    targetSelect.addEventListener("change", updateWifiHint);
    updateWifiHint();
  }

  // Wire Syslog controls
  const btnSyslogStart = document.getElementById("btn-syslog-start");
  const btnSyslogStop = document.getElementById("btn-syslog-stop");
  const btnSyslogLogs = document.getElementById("btn-syslog-logs");
  if (btnSyslogStart) btnSyslogStart.addEventListener("click", startSyslogServer);
  if (btnSyslogStop) btnSyslogStop.addEventListener("click", stopSyslogServer);
  if (btnSyslogLogs) btnSyslogLogs.addEventListener("click", viewSyslogLogs);
  connectSyslogStream();

  // Initial loads
  loadParams();
  loadMaps();
}

document.addEventListener("DOMContentLoaded", initCockpitDashboard);


