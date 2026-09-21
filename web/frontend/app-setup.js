// Linorobot2 Cockpit frontend -- sensor registry, config import, install actions, Docker/Podman install mode.
// Part of the app.js split: a classic script sharing global scope. See app-core.js.

// ---------- sensor registry (single source of truth) ----------
// Everything sensor-related -- Install driver list, Bringup model codes, the
// Docker/.env choices, the Sensors/LiDAR launcher, install/udev commands --
// comes from ONE /api/sensors payload (server.py's LASER_SENSORS/DEPTH_SENSORS).
// No parallel copies live in this file anymore.
let SENSORS = { laser: {}, depth: {} };
let SERIAL_PORTS = [];

function addOpt(sel, value, text) {
  if (!sel) return;
  const o = document.createElement("option");
  o.value = value;
  o.textContent = text;
  sel.appendChild(o);
}

function laserEntryForModel(code) {
  return Object.entries(SENSORS.laser).find(
    ([, e]) => (e.models || []).some((m) => m.code === code)
  );
}

function populateSensorSelects() {
  // ... and re-apply the saved laser at the end, because this and the /api/status
  // fetch race: whichever lands last has to be the one that decides the panel.
  // Install tab -- one entry per driver package
  Object.entries(SENSORS.laser).forEach(([k, e]) => addOpt(document.getElementById("install-laser"), k, e.label));
  Object.entries(SENSORS.depth).forEach(([k, e]) => addOpt(document.getElementById("install-depth"), k, e.label));

  // Bringup tab -- one entry per model code (LINOROBOT2_*_SENSOR value)
  Object.values(SENSORS.laser).forEach((e) =>
    (e.models || []).forEach((m) => addOpt(document.getElementById("bringup-laser-sensor"), m.code, `${m.code} — ${m.label}`)));
  Object.values(SENSORS.depth).forEach((e) =>
    (e.models || []).forEach((m) => addOpt(document.getElementById("bringup-depth-sensor"), m.code, `${m.code} — ${m.label}`)));

  // Docker tab -- only drivers that have a docker/.env key
  Object.values(SENSORS.laser).forEach((e) => e.docker_key && addOpt(document.getElementById("docker-laser-sensor"), e.docker_key, `${e.docker_key} (${e.label})`));
  Object.values(SENSORS.depth).forEach((e) => e.docker_key && addOpt(document.getElementById("docker-depth-sensor"), e.docker_key, `${e.docker_key} (${e.label})`));

  // Sensors/LiDAR tab launcher -- one entry per model code
  const lm = document.getElementById("laser-driver-model");
  Object.values(SENSORS.laser).forEach((e) =>
    (e.models || []).forEach((m) => addOpt(lm, m.code, `${m.code} — ${m.label}`)));
  if (typeof updateLaserDriverFieldsVisibility === "function") updateLaserDriverFieldsVisibility();

  // Sensors tab LiDAR & Depth selects
  const cfgLaser = document.getElementById("cfg-laser-sensor");
  if (cfgLaser) {
    cfgLaser.innerHTML = "";
    Object.values(SENSORS.laser).forEach((e) =>
      (e.models || []).forEach((m) => addOpt(cfgLaser, m.code, `${m.code} — ${m.label}`)));
    if (!cfgLaser._hasSync) {
      cfgLaser._hasSync = true;
      cfgLaser.addEventListener("change", () => {
        const v = cfgLaser.value;
        if (lm && lm.value !== v) {
          lm.value = v;
          if (typeof updateLaserDriverFieldsVisibility === "function") updateLaserDriverFieldsVisibility();
        }
        const bl = document.getElementById("bringup-laser-sensor");
        if (bl) bl.value = v;
        if (typeof updateBringupSummary === "function") updateBringupSummary();
      });
    }
  }

  if (lm && !lm._hasSync) {
    lm._hasSync = true;
    lm.addEventListener("change", () => {
      const v = lm.value;
      if (cfgLaser && cfgLaser.value !== v) cfgLaser.value = v;
      const bl = document.getElementById("bringup-laser-sensor");
      if (bl) bl.value = v;
      if (typeof updateBringupSummary === "function") updateBringupSummary();
    });
  }

  const cfgDepth = document.getElementById("cfg-depth-sensor");
  if (cfgDepth) {
    cfgDepth.innerHTML = "";
    addOpt(cfgDepth, "", "None (Disabled)");
    Object.values(SENSORS.depth).forEach((e) =>
      (e.models || []).forEach((m) => addOpt(cfgDepth, m.code, `${m.code} — ${m.label}`)));
    if (!cfgDepth._hasSync) {
      cfgDepth._hasSync = true;
      cfgDepth.addEventListener("change", () => {
        const v = cfgDepth.value;
        const bd = document.getElementById("bringup-depth-sensor");
        if (bd) bd.value = v;
        const id = document.getElementById("install-depth");
        if (id) id.value = v;
        if (typeof updateBringupSummary === "function") updateBringupSummary();
      });
    }
  }

  // There is no second pair of sensor selects, and there must not be. This
  // used to fill `env-laser-sensor` / `env-depth-sensor` for a "Robot
  // Environment" tab that does not exist in the cockpit -- the working
  // controls are `cfg-laser-sensor` and `cfg-depth-sensor` above, which are
  // the ones the config is read from and written to. Two dropdowns for one
  // setting is the failure this file already carries a comment about: they
  // disagree, and the one the user changed is not the one that is saved.
  applyLaserConfigToPanel();
  if (typeof updateBringupSummary === "function") updateBringupSummary();
}

function renderSerialPortList() {
  const dl = document.getElementById("serial-ports-list");
  if (dl) {
    dl.innerHTML = "";
    SERIAL_PORTS.forEach((p) => {
      const o = document.createElement("option");
      o.value = p.preferred;
      o.label = `${p.vendor || "?"} ${p.model || ""} ${p.usb_id ? "[" + p.usb_id + "]" : ""} → ${p.tty}`.trim();
      dl.appendChild(o);
    });
  }
  const box = document.getElementById("serial-ports-detected");
  if (box) {
    if (!SERIAL_PORTS.length) {
      box.textContent = "No USB serial devices detected.";
    } else {
      box.innerHTML = SERIAL_PORTS.map((p) => {
        // Most lidars use a generic CP2102/CH340/FTDI bridge, so VID:PID and
        // the model string rarely tell devices apart -- the by-path (physical
        // USB port) is the reliable identifier and what we store.
        const idbits = [p.usb_id, p.vendor, p.model].filter(Boolean).join(" · ") || "generic UART";
        const sn = p.serial ? ` · SN ${p.serial}` : " · no serial#";
        return `<div style="margin-bottom:6px">` +
          `<code>${escapeHtml(p.preferred)}</code><br>` +
          `<span class="hint">${escapeHtml(idbits)}${escapeHtml(sn)} · now ${escapeHtml(p.tty)}</span>` +
          `</div>`;
      }).join("");
    }
  }
}

async function refreshSerialPorts() {
  try {
    const r = await fetch("/api/serial_ports");
    SERIAL_PORTS = (await r.json()).ports || [];
  } catch (e) {
    SERIAL_PORTS = [];
  }
  renderSerialPortList();
}

fetch("/api/sensors")
  .then((r) => r.json())
  .then((data) => {
    SENSORS = data;
    populateSensorSelects();
  })
  .catch(() => {});
refreshSerialPorts();

// ---------- import config ----------
document.getElementById("btn-import").addEventListener("click", async () => {
  const path = document.getElementById("import-path").value.trim();
  if (!path) return;
  const res = await fetch("/api/import_config", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ path }),
  });
  const data = await res.json();
  const resultEl = document.getElementById("import-result");
  if (data.error) {
    resultEl.textContent = data.error;
    return;
  }
  if (data.type === "unified_yaml") {
    resultEl.innerHTML = `<span style="color: var(--success);">✓ Imported Unified Configuration! Linorobot2, Nav2, EKF, and SLAM synchronized.</span>`;
    loadRobotEnv();
    loadUnifiedConfig();
    return;
  }
  if (data.type === "robot_env") {
    resultEl.innerHTML = `<span style="color: var(--success);">✓ Imported robot.env! Environment settings updated and synced to ~/.bashrc.</span>`;
    loadRobotEnv();
    return;
  }
  if (data.base) document.getElementById("install-base").value = data.base;
  if (data.transport) document.getElementById("cfg-agent-transport").value = data.transport;
  if (data.agent_baud) document.getElementById("cfg-agent-baud").value = data.agent_baud;
  if (data.agent_port) document.getElementById("cfg-agent-port").value = data.agent_port;
  if (data.agent_ip) document.getElementById("cfg-agent-device").placeholder = data.agent_ip;
  // The header also settles how the scan reaches this computer. The server has
  // already persisted that, so re-read the config and let the laser panel
  // follow it instead of mapping the same fields a second time here.
  if (data.lidar_transport) {
    fetch("/api/config")
      .then((r) => r.json())
      .then((c) => { state.config = c; applyLaserConfigToPanel(); })
      .catch(() => {});
  }
  const lidarBit = data.lidar_transport
    ? ` lidar=${data.lidar_transport}` +
      (data.lidar_transport === "udp_bridge" ? `:${data.lidar_udp_port || "8889"}` : "") +
      (data.lidar_baud ? `@${data.lidar_baud}` : "")
    : "";
  resultEl.textContent =
    `Imported: base=${data.base || "?"} transport=${data.transport} ` +
    `has_imu=${data.has_imu} has_mag=${data.has_mag}` + lidarBit +
    (data.mag_bias ? ` mag_bias=[${data.mag_bias.join(", ")}]` : "");
  (data.warnings || []).forEach((w) => {
    const line = document.createElement("div");
    line.style.color = "var(--accent-warn)";
    line.textContent = "\u26a0 " + w;
    resultEl.appendChild(line);
  });
});

// ---------- install actions ----------
document.getElementById("btn-install-base").addEventListener("click", () => {
  const workspace = document.getElementById("install-workspace").value.trim();
  if (workspace) {
    fetch("/api/config", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ workspace_path: workspace }),
    });
  }
  checkAndBuildWorkspace();
});

// Install/udev command assembly lives on the server now (build_sensor_install_cmd);
// the client asks for the joined string via POST /api/sensor_install_cmd. No
// mirrored command table here.
async function fetchSensorInstallCmd(kind, key, { skipUdev = false, udevOnly = false } = {}) {
  const r = await fetch("/api/sensor_install_cmd", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ kind, key, skip_udev: skipUdev, udev_only: udevOnly, workspace_path: ws() }),
  });
  if (!r.ok) return null;
  return (await r.json()).handle || null;
}

async function runSensorInstall(kind, selId, skipId, titlePrefix) {
  const key = document.getElementById(selId).value;
  if (!key) return;
  const skip = document.getElementById(skipId).checked;
  const handle = await fetchSensorInstallCmd(kind, key, { skipUdev: skip });
  if (!handle) {
    logLine(`[console] no install steps defined for ${kind} "${key}" -- see its own driver docs.`);
    return;
  }
  runCommand({ handle }, { title: `${titlePrefix}: ${key}` });
}

document.getElementById("btn-install-laser").addEventListener("click", () =>
  runSensorInstall("laser", "install-laser", "laser-skip-udev", "Install laser"));
document.getElementById("btn-install-depth").addEventListener("click", () =>
  runSensorInstall("depth", "install-depth", "depth-skip-udev", "Install depth camera"));

// ---------- Docker / Podman install mode ----------
// The console has its OWN compose stack at tools/console/docker/
// (docker-compose.yaml + a generated .env + devices.generated.yaml). It reuses
// the upstream linorobot2 image + docker/Dockerfile (built, not modified) but
// no ROS install on this host at all -- sensor drivers install *inside* the
// image via the Dockerfile's own `bash install.bash ...` step (that's
// linorobot2's documented build process, not something Console runs on the
// host, so it doesn't conflict with the "no install.bash on the host" rule
// the native install path follows).
const installModeSel = document.getElementById("install-mode");
installModeSel.addEventListener("change", () => {
  const isNative = installModeSel.value === "native";
  document.getElementById("install-native-cards").style.display = isNative ? "block" : "none";
  document.getElementById("install-docker-card").style.display = isNative ? "none" : "block";
});

// The console's OWN compose dir -- it never writes into the repo's upstream
// docker/ dir. `.env` + `devices.generated.yaml` are written here by
// btn-docker-build; the checked-in docker-compose.yaml drives nav/SLAM
// through the console's own launch_nav2.py / launch_bringup.py.
function dockerDir() {
  return `${ws()}/src/linorobot2/tools/console/docker`;
}
// The -f overlay + --env-file the console always passes to compose.
function dockerComposeFlags() {
  return `--env-file .env -f docker-compose.yaml -f devices.generated.yaml`;
}

// Resolved at command-run time (not build time) since we can't be sure which
// of `podman compose` (the compose plugin) or the standalone `podman-compose`
// tool is actually installed -- docker itself only has one real option.
function composeResolveSnippet() {
  if (installModeSel.value === "podman") {
    return `if command -v podman-compose >/dev/null 2>&1; then COMPOSE="podman-compose"; else COMPOSE="podman compose"; fi; `;
  }
  return `COMPOSE="docker compose"; `;
}

// Docker/.env's LASER_SENSOR/DEPTH_SENSOR use a coarser name than
// lasers.launch.py's per-model `sensor` codes (e.g. "rplidar", not a1/.../s3).
// Look up the driver-package key from the registry's docker_key so the
// *udev-rules-only* step below (driver install itself happens inside the
// image) can reuse the same server-side command builder.
function sensorKeyForDockerValue(kind, dockerVal) {
  const table = kind === "laser" ? SENSORS.laser : SENSORS.depth;
  const hit = Object.entries(table).find(([, e]) => e.docker_key === dockerVal);
  return hit ? hit[0] : null;
}
function dockerLaserDevice(dockerVal) {
  const k = sensorKeyForDockerValue("laser", dockerVal);
  return k ? SENSORS.laser[k].symlink : null;
}

function cloneLinorobot2Command() {
  const workspace = document.getElementById("install-workspace").value.trim() || ws();
  return [
    `mkdir -p ${workspace}/src`,
    `cd ${workspace}/src`,
    gitCloneDistroSnippet("https://github.com/linorobot/linorobot2", "linorobot2"),
  ].join(" && ");
}

document.getElementById("btn-docker-build").addEventListener("click", () => {
  const baseImage = document.getElementById("docker-base-image").value;
  const robotBase = document.getElementById("install-base").value;
  const laser = document.getElementById("docker-laser-sensor").value;
  const depth = document.getElementById("docker-depth-sensor").value;
  const serialPort = document.getElementById("docker-base-serial-port").value.trim() || "/dev/ttyACM0";
  const domainId = document.getElementById("docker-ros-domain-id").value.trim() || "0";
  const gpuId = document.getElementById("docker-gpu-id").value.trim() || "0";
  const distro = (state.status && state.status.ros_distro) || "jazzy";

  // The whole build (clone + .env + device overlay + compose build) is assembled
  // SERVER-SIDE now (actions.py docker_build); here we only pass the fields.
  runCommand({ action: "docker_build", args: {
    distro, base_image: baseImage, robot_base: robotBase,
    laser, depth, serial_port: serialPort, domain_id: domainId, gpu_id: gpuId,
    robot_name: state.robot_name || "linorobot2",
    workspace: document.getElementById("install-workspace").value.trim() || ws(),
    docker_dir: dockerDir(),
    laser_device: dockerLaserDevice(laser) || "",
    engine: installModeSel.value === "podman" ? "podman" : "docker",
  } }, { title: `Docker/Podman build (${baseImage})` });
});

document.getElementById("btn-docker-udev").addEventListener("click", async () => {
  const laser = document.getElementById("docker-laser-sensor").value;
  const depth = document.getElementById("docker-depth-sensor").value;
  const handles = [];
  for (const [kind, dockerVal] of [["laser", laser], ["depth", depth]]) {
    if (!dockerVal) continue;
    const key = sensorKeyForDockerValue(kind, dockerVal);
    if (!key) {
      logLine(`[console] no registry entry for ${kind} "${dockerVal}" -- check its own driver docs (e.g. ZED SDK).`);
      continue;
    }
    const h = await fetchSensorInstallCmd(kind, key, { udevOnly: true });
    if (h) handles.push(h);
    else logLine(`[console] "${dockerVal}" has no persistent udev symlink -- it'll enumerate as a plain /dev/ttyUSBx or /dev/ttyACMx.`);
  }
  if (!handles.length) return;
  // Each is an independent, idempotent step -- run them in turn.
  for (const handle of handles) {
    await new Promise((resolve) => runCommand({ handle }, { title: "Install udev rules (host)", onDone: resolve }));
  }
});

const btnDockerServiceStart = document.getElementById("btn-docker-service-start");
const btnDockerServiceStop = document.getElementById("btn-docker-service-stop");
btnDockerServiceStart.addEventListener("click", async () => {
  const service = document.getElementById("docker-service").value;
  if (["slam", "navigate"].includes(service)) {
    if (isAutoBringupEnabled()) {
      await ensureBringupRunning();
    }
  }
  // linorobot2's own Tmuxinator profiles (docker/profiles/*.yml) always
  // `export DISPLAY=:200` before `docker compose up` -- GUI services
  // (gazebo, rviz, slam/navigate with rviz:=true) render into that
  // virtual display, which the kasmvnc service then streams to a browser.
  // Skipping it isn't just "no picture" -- gz sim's GUI process crashes
  // outright trying to open an unset/invalid display.
  btnDockerServiceStart.disabled = true;
  btnDockerServiceStop.disabled = false;
  runCommand({ action: "docker_service_up", args: {
    engine: installModeSel.value === "podman" ? "podman" : "docker",
    docker_dir: dockerDir(), service,
  } }, {
    title: `Docker/Podman service: ${service}`,
    onDone: () => {
      btnDockerServiceStart.disabled = false;
      btnDockerServiceStop.disabled = true;
    },
  });
});
btnDockerServiceStop.addEventListener("click", () => killSlot("main"));

const vncBtn = document.getElementById("btn-open-vnc");
if (vncBtn) {
  const host = window.location.hostname || "localhost";
  vncBtn.href = `http://${host}:3000/`;
}

document.getElementById("btn-docker-down").addEventListener("click", () => {
  runCommand({ action: "docker_down", args: {
    engine: installModeSel.value === "podman" ? "podman" : "docker", docker_dir: dockerDir(),
  } }, { title: "Docker/Podman: stop + remove all containers" });
});

