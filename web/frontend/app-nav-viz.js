// Linorobot2 Cockpit frontend -- teleop, SLAM/Nav, RViz-over-noVNC, magnetometer cal, laser driver, LiDAR viewer, settings, Nav2 editor.
// Part of the app.js split: a classic script sharing global scope. See app-core.js.

// ---------- SLAM / navigation ----------
wireStartStop({
  startBtn: document.getElementById("btn-slam-start"),
  stopBtn: document.getElementById("btn-slam-stop"),
  stackTag: "slam",
  slot: "main",
  title: "SLAM",
  needsBringup: true,
  // slam.launch.py brings nav2 up alongside slam_toolbox, using
  // linorobot2_navigation's own navigation.yaml. That file is a Jazzy-era
  // layout, so on Lyrical/Rolling controller_server failed to configure
  // ("Failed to get 'primary_controller.plugin' parameter") and
  // lifecycle_manager aborted the whole bringup -- taking SLAM with it. It
  // also meant every parameter tuned in Console was quietly ignored, which is
  // the opposite of what this tab claims. Go through Console's own launcher
  // with slam:=true, exactly as the Navigation button does.
  // launchers/slam.launch.py with the robot's own slam block, as 1-Click runs it.
  buildCommand: async () => ({ action: "slam", args: {
    distro: getDistro(),
    config_path: state.status?.robot_config_path || "",
  } }),
});

document.getElementById("btn-map-save").addEventListener("click", () => {
  const name = document.getElementById("map-save-name").value.trim();
  if (!name) return;
  const mapsDir = `${ws()}/src/linorobot2/linorobot2_navigation/maps`;
  // Saving needs SLAM still running -- it is what publishes /map -- so this
  // cannot take the slot SLAM is holding. On "main" it was refused with a 409
  // every time, i.e. the map could never be saved from the UI at all.
  runCommand(
    { action: "map_save", args: { name, maps_dir: mapsDir, distro: getDistro() } },
    { slot: "tool", title: `Save map: ${name}`, onDone: refreshMaps }
  );
});

function refreshMaps() {
  fetch("/api/maps").then((r) => r.json()).then((data) => {
    const sel = document.getElementById("nav-map-select");
    sel.innerHTML = "";
    // /api/maps returns objects ({name, yaml, image}), not bare strings, and
    // the option value is fed straight to `map:=` so it must be the absolute
    // path. Interpolating the object gave every option the literal value
    // "undefined/[object Object].yaml", which made Start Navigation unable to
    // load any saved map at all.
    data.maps.forEach((m) => {
      const opt = document.createElement("option");
      opt.value = `${data.maps_dir}/${m.yaml}`;
      opt.textContent = m.name;
      sel.appendChild(opt);
    });
  });
}
document.getElementById("btn-refresh-maps").addEventListener("click", refreshMaps);
refreshMaps();

wireStartStop({
  startBtn: document.getElementById("btn-nav-start"),
  stopBtn: document.getElementById("btn-nav-stop"),
  stackTag: "nav2",
  slot: "main",
  title: "Navigation",
  needsBringup: true,
  buildCommand: async () => {
    const distro = getDistro();
    // launchers/nav2.launch.py with the robot's own nav2 block, as 1-Click runs it.
    return { action: "nav2", args: {
      distro,
      config_path: state.status?.robot_config_path || "",
      map: document.getElementById("nav-map-select").value || "",
    } };
  },
});

// ---------- RViz via noVNC (headless-friendly) ----------
// Native equivalent of the `kasmvnc` service in linorobot2's own
// docker/docker-compose.yaml: Xvfb gives rviz2 a virtual display to open,
// x11vnc exposes that display over VNC, and websockify (from the `novnc`
// package) fronts it as a plain browser page -- no native VNC client needed,
// and it works the same whether the browser is on the robot computer itself
// or a separate laptop.
const btnVncStart = document.getElementById("btn-vnc-start");
const btnVncStop = document.getElementById("btn-vnc-stop");
// RViz configs shipped in this repo under rviz/, resolved against repo_root
// from /api/status. They used to be resolved against a `web_dir` field the
// status endpoint never returned, so every path fell back to "." and RViz
// always opened empty. The description view still comes from the sibling
// linorobot2_description package, which may not be in the workspace.
const RVIZ_CONFIGS = {
  teleop: "/rviz/teleop.rviz",
  slam: "/rviz/slam.rviz",
  navigation: "/rviz/navigation.rviz",
  description: "/../linorobot2_description/rviz/description.rviz",
};

btnVncStart.addEventListener("click", () => {
  const display = document.getElementById("vnc-display").value.trim() || ":99";
  const novncPort = document.getElementById("vnc-novnc-port").value.trim() || "6080";
  const which = document.getElementById("vnc-rviz-config").value;
  const rel = RVIZ_CONFIGS[which];
  const root = state.status?.repo_root || state.status?.web_dir || ".";
  // Reclaim the display through its own lock file (a single inspected PID) --
  // `pkill -f "Xvfb :99"` is a broad string-matching kill, banned outright by
  // AGENTS.md section 6.
  btnVncStart.disabled = true;
  btnVncStop.disabled = false;
  const link = document.getElementById("vnc-link");
  link.href = `http://${location.hostname}:${novncPort}/vnc.html`;
  link.style.display = "inline";
  runCommand({ action: "rviz_novnc", args: {
    distro: getDistro(), display, novnc_port: novncPort,
    rviz_config: rel ? `${root}${rel}` : "",
  } }, {
    slot: "viewer",
    title: "RViz via noVNC",
    onDone: () => {
      btnVncStart.disabled = false;
      btnVncStop.disabled = true;
      link.style.display = "none";
    },
  });
});
btnVncStop.addEventListener("click", () => killSlot("viewer"));

// ---------- magnetometer calibration ----------
// Needs Bringup already running elsewhere (cmd_vel to spin the base, IMU/mag
// topics to read) -- a bare standalone agent wouldn't be enough, same
// reasoning as teleop/SLAM/navigation above, so this doesn't call
// ensureAgentRunning() either.
document.getElementById("btn-mag-cal").addEventListener("click", async () => {
  if (!confirm("The robot will spin in place for about a minute. Clear the area, then continue?")) return;
  if (isAutoBringupEnabled()) {
    await ensureBringupRunning();
  }
  const resultEl = document.getElementById("mag-cal-result");
  resultEl.textContent = "";
  runCommand({ action: "mag_calibrate", args: { distro: getDistro() } }, {
    title: "Magnetometer calibration",
    onLine: (line) => {
      if (/mag_bias|bias_x|bias_y|bias_z/i.test(line)) {
        resultEl.textContent += line + "\n";
      }
    },
  });
});

// ---------- laser driver (standalone, independent of Bringup/agent) ----------
// Model list + persistent-symlink + baud all come from the sensor registry.
// Two families, per linorobot2_bringup/launch/lasers.launch.py:
// - registry model has NO `product` field  -> delegate to lasers.launch.py
//   `sensor:=<code>`, passing lidar_serial_port:= / lidar_transport:= (those
//   ARE declared launch args). ydlidar/rplidar/xv11.
// - registry model HAS `product`/`bins`/`baud` (ld06/ld19/stl27l) -> run the
//   ldlidar_stl_ros2 node directly with `--ros-args -p`, which also unlocks
//   its UDP-bridge / native-network modes.
const laserModelSel = document.getElementById("laser-driver-model");

function laserModelMeta(code) {
  const hit = laserEntryForModel(code);
  if (!hit) return null;
  const [key, entry] = hit;
  const m = (entry.models || []).find((x) => x.code === code) || {};
  return {
    key, entry, code,
    isLd: Boolean(m.product),
    product: m.product,
    bins: m.bins,
    baud: m.baud || entry.default_baud || "",
    symlink: entry.symlink,
  };
}

// The MCU and the LiDAR sit on two different tty devices, and the MCU's is
// already known -- so the LiDAR's can be guessed instead of left blank:
//
//   agent on /dev/ttyUSB0  -> LiDAR on /dev/ttyUSB1   (both USB-serial bridges;
//                                                      the agent took the first)
//   agent on /dev/ttyACM0  -> LiDAR on /dev/ttyUSB0   (the MCU is native USB CDC,
//                                                      so no ttyUSB is taken yet)
//
// Only when the agent actually holds a serial port: over WiFi it holds none, and
// pairing off a stale agent_device would push the LiDAR one slot too far. This is
// a starting value, not an identity -- ttyUSBn is assignment order and flips on
// replug, so a rig that cares should save a /dev/serial/by-id path instead.
function laserPortPairedWithAgent(c) {
  if ((c.agent_transport || "serial") !== "serial") return "";
  const dev = c.agent_device || "";
  const usb = dev.match(/^\/dev\/ttyUSB(\d+)$/);
  if (usb) return `/dev/ttyUSB${Number(usb[1]) + 1}`;
  if (/^\/dev\/ttyACM\d+$/.test(dev)) return "/dev/ttyUSB0";
  return "";
}

function laserDefaultPort(meta) {
  const c = state.config || {};
  return c.laser_serial_port || laserPortPairedWithAgent(c) || meta.symlink || "";
}

// The laser panel opened on whichever option happened to be first in the list
// (ydlidar) no matter what the robot's saved laser_sensor said, and its port
// and baud were filled in before the config had loaded. "Start laser driver"
// then launched the wrong driver on the wrong device -- silently, because a
// driver that finds nothing on a port looks the same as one that is starting
// up. robot_config.yaml is the source of truth, so follow it once it arrives.
function applyLaserConfigToPanel() {
  if (!laserModelSel || !laserModelSel.options.length) return;
  const c = state.config || {};
  const has = (v) => [...laserModelSel.options].some((o) => o.value === v);
  let code = "";
  if (c.laser_model && has(c.laser_model)) {
    code = c.laser_model;                      // exact model, when recorded
  } else if (c.laser_sensor) {
    // laser_sensor names a driver family (e.g. "ldlidar"); the select holds
    // models (ld06/ld19/stl27l). Take the family's first model as the default.
    const entry = (SENSORS.laser || {})[c.laser_sensor];
    const first = entry && (entry.models || [])[0];
    if (first && has(first.code)) code = first.code;
  }
  if (code) laserModelSel.value = code;
  // The connection route has to be restored before the visibility pass, which
  // reads it to decide which row to show. An imported firmware header sets it:
  // USE_LIDAR_UDP means the scan arrives as datagrams, not on a cable.
  const modeSel = document.getElementById("laser-driver-mode");
  if (modeSel && c.laser_transport &&
      [...modeSel.options].some((o) => o.value === c.laser_transport)) {
    modeSel.value = c.laser_transport;
  }
  const udpField = document.getElementById("laser-driver-udp-port");
  if (udpField && c.laser_udp_port) udpField.value = c.laser_udp_port;
  updateLaserDriverFieldsVisibility();
}

function updateLaserDriverFieldsVisibility() {
  const meta = laserModelMeta(laserModelSel.value);
  if (!meta) return;
  document.getElementById("laser-driver-ld-fields").style.display = meta.isLd ? "block" : "none";
  const hint = document.getElementById("laser-driver-simple-hint");
  if (hint) {
    hint.textContent = meta.isLd
      ? ""
      : `Delegates to lasers.launch.py sensor:=${meta.code}. Serial port below is passed as lidar_serial_port:= (blank = the driver's own /dev symlink).`;
  }
  const portField = document.getElementById("laser-driver-serial-port");
  if (portField) portField.value = laserDefaultPort(meta);
  const baudField = document.getElementById("laser-driver-baud");
  if (baudField && meta.isLd) baudField.value = (state.config || {}).laser_baud || meta.baud;
  updateLaserDriverModeVisibility();
}

function updateLaserDriverModeVisibility() {
  const meta = laserModelMeta(laserModelSel.value);
  const mode = meta && meta.isLd ? document.getElementById("laser-driver-mode").value : "serial";
  document.getElementById("laser-driver-serial-row").style.display = mode === "serial" ? "flex" : "none";
  document.getElementById("laser-driver-udpbridge-row").style.display = mode === "udp_bridge" ? "flex" : "none";
  document.getElementById("laser-driver-netaddr-row").style.display =
    (mode === "udp_server" || mode === "udp_client") ? "flex" : "none";
}

laserModelSel.addEventListener("change", updateLaserDriverFieldsVisibility);
document.getElementById("laser-driver-mode").addEventListener("change", updateLaserDriverModeVisibility);
const btnDetectPorts = document.getElementById("btn-laser-detect-ports");
if (btnDetectPorts) btnDetectPorts.addEventListener("click", refreshSerialPorts);

function persistLaserPort() {
  const port = document.getElementById("laser-driver-serial-port").value.trim();
  const meta = laserModelMeta(laserModelSel.value);
  const baud = meta && meta.isLd ? document.getElementById("laser-driver-baud").value.trim() : "";
  const payload = {};
  // The route is worth saving even when there is no serial port to save with
  // it: on the udp_bridge route the port row is hidden and the field is empty,
  // so returning early on a blank port dropped the route on the floor and a
  // reload came back on "Direct serial" pointed at a device nobody is feeding.
  if (meta && meta.isLd) {
    payload.laser_transport = document.getElementById("laser-driver-mode").value;
    const udpPort = document.getElementById("laser-driver-udp-port").value.trim();
    if (udpPort) payload.laser_udp_port = udpPort;
  }
  // Only record a port the panel actually holds. Before the saved config has
  // been applied this field still shows the registry's first entry
  // (/dev/ydlidar), and writing that back overwrote the real device -- the
  // panel's own defaults ended up saved as if the user had chosen them.
  if (port) {
    payload.laser_serial_port = port;
    payload.laser_baud = baud;
    // Record the model too, so the exact LiDAR survives a reload; the driver
    // family alone cannot say whether this is an LD06 or an LD19.
    payload.laser_model = meta ? meta.code : "";
  }
  if (!Object.keys(payload).length) return;
  fetch("/api/config", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(payload),
  }).then((r) => r.json()).then((c) => { state.config = c; }).catch(() => {});
}

function ldNodeParams(meta, overrides) {
  const base = {
    product_name: meta.product,
    topic_name: "scan",
    frame_id: "laser",
    laser_scan_dir: "true",
    bins: String(meta.bins),
    enable_angle_crop_func: "false",
    angle_crop_min: "135.0",
    angle_crop_max: "225.0",
  };
  Object.assign(base, overrides);
  return Object.entries(base).map(([k, v]) => `-p ${k}:=${v}`).join(" ");
}

// Returns a laser_driver action SPEC (built server-side, actions.py). The four
// modes -- non-LD sensor, LD serial, LD udp_bridge (socat), LD udp server/client
// -- are all reproduced there from these fields.
function buildLaserDriverCommand() {
  const meta = laserModelMeta(laserModelSel.value);
  const port = document.getElementById("laser-driver-serial-port").value.trim();
  persistLaserPort();
  const args = {
    distro: getDistro(),
    is_ld: !!meta.isLd,
    code: meta.code,
    product: meta.product,
    bins: meta.bins,
    port,
    symlink: meta.symlink,
    baud: document.getElementById("laser-driver-baud").value.trim() || meta.baud,
    mode: document.getElementById("laser-driver-mode")?.value || "serial",
    udp_port: document.getElementById("laser-driver-udp-port")?.value.trim() || "8889",
    bridge_path: document.getElementById("laser-driver-bridge-path")?.value.trim() || "/dev/lidar_udp_bridge",
    server_ip: document.getElementById("laser-driver-server-ip")?.value.trim() || "0.0.0.0",
    server_port: document.getElementById("laser-driver-server-port")?.value.trim() || "8889",
  };
  return { spec: { action: "laser_driver", args } };
}

const btnLaserStart = document.getElementById("btn-laser-driver-start");
const btnLaserStop = document.getElementById("btn-laser-driver-stop");

function isLaserRunning() {
  return Boolean(state.status && state.status.laser_busy);
}

// Shared by the button and by the 1-Click chain, so both start the driver the
// same way. Resolves once the process has been accepted, not once it exits --
// the driver is long-lived and the caller has to carry on to SLAM.
function startLaserDriver() {
  const { spec } = buildLaserDriverCommand();
  btnLaserStart.disabled = true;
  btnLaserStop.disabled = false;
  runCommand(spec, {
    // Its own slot: the driver has to keep running while SLAM and Nav2 do, and
    // bringup does not start the LiDAR, so sharing "main" made /scan and SLAM
    // mutually exclusive -- the second one was refused with a 409 that showed
    // up as simply nothing happening.
    slot: "laser",
    title: `Laser driver: ${laserModelSel.value}`,
    onDone: () => {
      btnLaserStart.disabled = false;
      btnLaserStop.disabled = true;
    },
  });
  // Give the driver a moment to open the port and start publishing before the
  // caller launches SLAM on top of it.
  return new Promise((resolve) => setTimeout(async () => {
    await refreshStatus();
    resolve(isLaserRunning());
  }, 4000));
}

btnLaserStart.addEventListener("click", () => { startLaserDriver(); });
btnLaserStop.addEventListener("click", () => killSlot("laser"));

// ---------- LiDAR viewer ----------
let lidarSource = null;
const lidarCanvas = document.getElementById("lidar-canvas");
const lidarCtx = lidarCanvas.getContext("2d");

function drawScan(scan) {
  const w = lidarCanvas.width, h = lidarCanvas.height;
  lidarCtx.clearRect(0, 0, w, h);
  lidarCtx.strokeStyle = "#232b3d";
  lidarCtx.beginPath();
  lidarCtx.arc(w / 2, h / 2, Math.min(w, h) / 2 - 4, 0, Math.PI * 2);
  lidarCtx.stroke();

  const ranges = scan.ranges || [];
  if (!ranges.length) return;
  const maxRange = Math.max(...ranges.filter((r) => isFinite(r) && r > 0), 1);
  const scale = (Math.min(w, h) / 2 - 8) / maxRange;

  lidarCtx.fillStyle = "#6366f1";
  ranges.forEach((r, i) => {
    if (!isFinite(r) || r <= 0) return;
    const angle = scan.angle_min + i * scan.angle_increment;
    const x = w / 2 + r * Math.cos(angle) * scale;
    const y = h / 2 - r * Math.sin(angle) * scale;
    lidarCtx.fillRect(x - 1.5, y - 1.5, 3, 3);
  });
}

function setLidarViewerStatus(message, level) {
  const el = document.getElementById("lidar-viewer-status");
  if (!el) return;
  el.textContent = message || "";
  el.className = "hint lidar-status" + (level ? ` lidar-status-${level}` : "");
}

document.getElementById("btn-lidar-start").addEventListener("click", async () => {
  if (lidarSource) lidarSource.close();
  const distro = document.getElementById("hdr-distro-select")?.value || "jazzy";
  const topic = document.getElementById("laser-driver-topic")?.value?.trim() || "/scan";
  const qs = new URLSearchParams({ distro, topic });
  // EventSource cannot carry the access-token header, so it carries a one-shot
  // ticket instead -- the token itself never goes into a URL.
  const ticket = await streamTicket();
  // No ticket means the cockpit has no token yet: streamTicket() swallows that
  // and returns "". Opening the stream anyway gets a 401 that EventSource
  // reports as a bare `error`, and the viewer used to blame the supervisor for
  // it -- "is the supervisor still running?" while the supervisor was serving
  // this very page. Say the one thing that fixes it instead.
  if (!ticket) {
    setLidarViewerStatus(
      "Not authorised: this cockpit needs its access token before it will stream. " +
      "Paste it in the banner at the top, or open the ?token= URL the supervisor printed.",
      "err");
    return;
  }
  lidarSource = new EventSource(`/api/lidar_stream?${qs}&ticket=${encodeURIComponent(ticket)}`);
  setLidarViewerStatus(`Connecting to ${topic}...`, "info");

  lidarSource.addEventListener("scan", (ev) => {
    try {
      const scan = JSON.parse(ev.data);
      drawScan(scan);
      setLidarViewerStatus(
        `${scan.frame_id || "laser"} — ${scan.count} points, frame ${scan.seq}`,
        "ok"
      );
    } catch (e) {
      setLidarViewerStatus(`Malformed scan payload: ${e.message}`, "err");
    }
  });
  // The backend reports "no publisher yet" and startup noise out of band, so a
  // viewer that draws nothing can say why instead of showing an empty circle.
  lidarSource.addEventListener("status", (ev) => {
    try {
      const st = JSON.parse(ev.data);
      setLidarViewerStatus(st.message || "", st.level === "warn" ? "warn" : st.level === "error" ? "err" : "info");
    } catch {}
  });
  lidarSource.onerror = () => {
    // Do not name a cause this has not checked. The stream drops for a stopped
    // supervisor, a rejected ticket, and a network that went away, and only one
    // of those is worth restarting anything over.
    setLidarViewerStatus(
      "Stream disconnected. Check the supervisor is up, and that this cockpit is "
      + "still authorised — a ticket is good once and expires.", "err");
  };

  document.getElementById("btn-lidar-start").disabled = true;
  document.getElementById("btn-lidar-stop").disabled = false;
});
document.getElementById("btn-lidar-stop").addEventListener("click", () => {
  if (lidarSource) {
    lidarSource.close();
    lidarSource = null;
  }
  setLidarViewerStatus("Stopped.", "info");
  document.getElementById("btn-lidar-start").disabled = false;
  document.getElementById("btn-lidar-stop").disabled = true;
});

// ---------- settings ----------
document.getElementById("btn-save-workspace").addEventListener("click", () => {
  const workspace_path = document.getElementById("cfg-workspace").value.trim();
  fetch("/api/config", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ workspace_path }),
  }).then(refreshStatus);
});

document.getElementById("btn-save-agent").addEventListener("click", () => {
  const body = {
    agent_transport: document.getElementById("cfg-agent-transport").value,
    agent_device: document.getElementById("cfg-agent-device").value,
    agent_port: document.getElementById("cfg-agent-port").value,
    agent_baud: document.getElementById("cfg-agent-baud").value,
  };
  fetch("/api/config", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(body),
  }).then(refreshStatus);
});

// ---------- Nav2 configuration editor (per-distro) ----------
const nav2EditorBox = document.getElementById("nav2-editor-box");
const btnNav2Toggle = document.getElementById("btn-nav2-toggle-editor");
const nav2Textarea = document.getElementById("nav2-config-text");
const nav2EditorDistro = document.getElementById("nav2-editor-distro");
const btnNav2Save = document.getElementById("btn-nav2-save-config");
const btnNav2Reset = document.getElementById("btn-nav2-reset-defaults");
const nav2Status = document.getElementById("nav2-save-status");

async function loadNav2Config(distro) {
  const d = distro || (nav2EditorDistro ? nav2EditorDistro.value : getDistro());
  if (nav2EditorDistro && nav2EditorDistro.value !== d) {
    nav2EditorDistro.value = d;
  }
  try {
    const res = await fetch(`/api/nav2_config?distro=${d}`);
    const data = await res.json();
    if (nav2Textarea && data.config) {
      nav2Textarea.value = data.config;
    }
    const hint = document.getElementById("nav2-depth-costmap-hint");
    if (hint) {
      const on = data.depth_pointcloud_active;
      hint.textContent = on === null || on === undefined
        ? ""
        : `This file lists observation_sources: ${on ? "scan pointcloud" : "scan"}. ` +
          `At launch, nav2.launch.py gates depth sources if needed — your saved YAML is not modified.`;
    }
  } catch (e) {}
}

if (nav2EditorDistro) {
  nav2EditorDistro.addEventListener("change", () => loadNav2Config(nav2EditorDistro.value));
}

if (btnNav2Toggle) {
  btnNav2Toggle.addEventListener("click", () => {
    if (!nav2EditorBox) return;
    const isHidden = nav2EditorBox.style.display === "none";
    nav2EditorBox.style.display = isHidden ? "block" : "none";
    btnNav2Toggle.textContent = isHidden ? "Hide Nav2 Parameters" : "Edit Nav2 Parameters (YAML)";
    if (isHidden) {
      loadNav2Config(getDistro());
    }
  });
}

if (btnNav2Save) {
  btnNav2Save.addEventListener("click", async () => {
    if (!nav2Textarea) return;
    const d = nav2EditorDistro ? nav2EditorDistro.value : getDistro();
    btnNav2Save.disabled = true;
    if (nav2Status) nav2Status.textContent = `Saving config for ${d}...`;
    try {
      const res = await fetch("/api/nav2_config", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ distro: d, config: nav2Textarea.value }),
      });
      const data = await res.json();
      if (nav2Status) {
        const savedTarget = data.path || (state.status?.robot_config_path ? state.status.robot_config_path.split("/").pop() : "robot config");
        nav2Status.textContent = data.status === "ok" ? `✓ Saved to ${savedTarget}` : "Error saving";
        setTimeout(() => { if (nav2Status) nav2Status.textContent = ""; }, 4000);
      }
    } catch (e) {
      if (nav2Status) nav2Status.textContent = "Error: " + e.message;
    } finally {
      btnNav2Save.disabled = false;
    }
  });
}

if (btnNav2Reset) {
  btnNav2Reset.addEventListener("click", async () => {
    const d = nav2EditorDistro ? nav2EditorDistro.value : getDistro();
    if (!confirm(`Reset Nav2 configuration for ${d} to default parameters?`)) return;
    btnNav2Reset.disabled = true;
    try {
      const res = await fetch("/api/nav2_config/reset", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ distro: d }),
      });
      const data = await res.json();
      if (data.config && nav2Textarea) {
        nav2Textarea.value = data.config;
        if (nav2Status) {
          nav2Status.textContent = `✓ Reset ${d} to default parameters`;
          setTimeout(() => { if (nav2Status) nav2Status.textContent = ""; }, 4000);
        }
      }
    } catch (e) {
      if (nav2Status) nav2Status.textContent = "Reset failed: " + e.message;
    } finally {
      btnNav2Reset.disabled = false;
    }
  });
}

loadNav2Config();

// ---------- distro & auto-bringup settings sync ----------
const hdrDistroSelect = document.getElementById("hdr-distro-select");
if (hdrDistroSelect) {
  hdrDistroSelect.addEventListener("change", () => {
    const val = hdrDistroSelect.value;
    fetch("/api/config", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ ros_distro: val }),
    }).then(refreshStatus);
  });
}

const btnSaveDistro = document.getElementById("btn-save-distro");
if (btnSaveDistro) {
  btnSaveDistro.addEventListener("click", () => {
    const distro = document.getElementById("cfg-ros-distro")?.value || "jazzy";
    const autoBringup = Boolean(document.getElementById("cfg-auto-bringup")?.checked);
    fetch("/api/config", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ ros_distro: distro, auto_bringup: autoBringup }),
    }).then(refreshStatus);
  });
}

const bringupAutoToggle = document.getElementById("bringup-auto-toggle");
if (bringupAutoToggle) {
  bringupAutoToggle.addEventListener("change", () => {
    const autoBringup = bringupAutoToggle.checked;
    const cfgAuto = document.getElementById("cfg-auto-bringup");
    if (cfgAuto) cfgAuto.checked = autoBringup;
    fetch("/api/config", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ auto_bringup: autoBringup }),
    });
  });
}


