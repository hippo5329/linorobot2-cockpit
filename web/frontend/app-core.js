// Linorobot2 Cockpit frontend. Vanilla JS, no build step, no framework.
//
// The UI names a server-side ACTION plus structured args (see runCommand below
// and web/backend/actions.py); the server owns the command text and refuses raw
// command strings unless COCKPIT_ALLOW_RAW_EXEC is set. SSE streams carry the
// output back. This replaced the older "the browser builds a bash string, the
// server just runs it" design.
//
// Originally one ~7.7k-line file, now split into ordered CLASSIC scripts (not ES
// modules) that share a single global scope, loaded by index.html in this order:
//   app-core -> app-setup -> app-agent-bringup -> app-nav-viz -> app-studio ->
//   app-workflow -> app-hardware -> app-adc -> app-presets
// Order matters: a later file may call a function declared in an earlier one at
// load time, but not the reverse -- function declarations hoist only within
// their own script. Cross-file calls therefore always point backward in this
// list, and event/timer callbacks (deferred to after every script has loaded)
// may call anything.

const state = {
  config: null,
  status: null,
  mainBusy: false,
  agentBusy: false,
  robot_name: "linorobot2",
  robots: [],
  git_branch: "",
  git_branches: [],
};

// ---------- access token ----------
// Every write to /api/ needs the per-install token (web/backend/access.py).
// It arrives once in the URL the supervisor prints at start (?token=...), is
// kept in this browser, and rides on every same-origin /api request as a
// header. A 401 raises a banner asking for it.
const TOKEN_KEY = "cockpit_token";
(function bootstrapToken() {
  try {
    const url = new URL(window.location.href);
    const t = url.searchParams.get("token");
    if (t) {
      localStorage.setItem(TOKEN_KEY, t);
      url.searchParams.delete("token");
      history.replaceState(null, "", url.pathname + url.search + url.hash);
    }
  } catch { /* storage or URL unavailable: the banner will ask */ }
})();
function cockpitToken() {
  try { return localStorage.getItem(TOKEN_KEY) || ""; } catch { return ""; }
}
// A one-shot ticket for an EventSource stream. EventSource cannot set the
// token header, so instead of putting the long-lived token in the stream URL
// (where it lands in server and proxy logs), we spend a minute-long ticket the
// backend mints for us -- the POST below carries the real token in its header.
async function streamTicket() {
  try {
    const r = await fetch("/api/stream_ticket", { method: "POST" });
    if (r.ok) return (await r.json()).ticket || "";
  } catch { /* fall through: caller opens without a ticket, banner will ask */ }
  return "";
}
function showTokenBanner() {
  if (document.getElementById("token-banner")) return;
  const bar = document.createElement("div");
  bar.id = "token-banner";
  bar.className = "token-banner";
  bar.innerHTML =
    '<span>This cockpit needs its access token. It is in the URL the supervisor printed at start ' +
    '(<code>docker compose logs</code>) and in <code>~/linorobot2-config/.cockpit_token</code>.</span>' +
    '<input id="token-input" type="password" placeholder="paste token" autocomplete="off">' +
    '<button type="button" id="token-save" class="btn btn-primary btn-sm">Use token</button>';
  document.body.prepend(bar);
  const save = () => {
    const v = (document.getElementById("token-input").value || "").trim();
    if (!v) return;
    try { localStorage.setItem(TOKEN_KEY, v); } catch { /* ignore */ }
    window.location.reload();
  };
  document.getElementById("token-save").addEventListener("click", save);
  document.getElementById("token-input").addEventListener("keydown", (e) => { if (e.key === "Enter") save(); });
}
const _nativeFetch = window.fetch.bind(window);
window.fetch = function (input, init) {
  const url = typeof input === "string" ? input : (input && input.url) || "";
  const isApi = url.startsWith("/api/") || url.startsWith(window.location.origin + "/api/");
  if (!isApi) return _nativeFetch(input, init);
  const opts = Object.assign({}, init || {});
  const headers = new Headers(opts.headers || (input instanceof Request ? input.headers : undefined));
  const tok = cockpitToken();
  if (tok) headers.set("X-Cockpit-Token", tok);
  opts.headers = headers;
  return _nativeFetch(input, opts).then((r) => {
    if (r.status === 401) showTokenBanner();
    return r;
  });
};

// A banner with the one thing to do next. Used when a run fails in a way
// the user has to fix at the bench (a flash that did not take), where the
// answer otherwise sits in grey monospace at the bottom of the console.
function showActionBanner(title, detail) {
  hideActionBanner();
  const bar = document.createElement("div");
  bar.id = "action-banner";
  bar.className = "action-banner";
  bar.innerHTML = `<strong>${escapeHtml(title)}</strong>` +
    (detail ? `<span>${escapeHtml(detail)}</span>` : "") +
    '<button type="button" id="action-banner-close" class="btn btn-secondary btn-sm" aria-label="Dismiss">Dismiss</button>';
  document.body.prepend(bar);
  document.getElementById("action-banner-close").addEventListener("click", hideActionBanner);
  bar.scrollIntoView({ block: "start" });
}
function hideActionBanner() {
  document.getElementById("action-banner")?.remove();
}

// A git value worth painting. The Docker image carries no .git, so the
// backend answers "unknown"; a chip that can only ever say that is hidden
// (hideGitChips) rather than shown.
function knownGit(v) {
  return Boolean(v) && !["unknown", "no-git", "none", ""].includes(String(v).trim().toLowerCase());
}
function hideGitChips() {
  document.getElementById("git-version-badge")?.closest(".git-version-row")?.setAttribute("hidden", "");
  document.getElementById("hdr-git-branch")?.parentElement?.setAttribute("hidden", "");
}

const consolePane = document.getElementById("console-pane");
const consoleTitle = document.getElementById("console-title");
const consoleWrap = document.getElementById("console-wrap");

function escapeHtml(value) {
  return String(value == null ? "" : value)
    .replace(/&/g, "&amp;")
    .replace(/</g, "&lt;")
    .replace(/>/g, "&gt;")
    .replace(/"/g, "&quot;")
    .replace(/'/g, "&#39;");
}

function toggleConsole(forceCollapse) {
  if (!consoleWrap) return;
  const willCollapse = typeof forceCollapse === "boolean" ? forceCollapse : !consoleWrap.classList.contains("collapsed");
  if (willCollapse) {
    consoleWrap.classList.add("collapsed");
    document.body.classList.add("console-collapsed");
  } else {
    consoleWrap.classList.remove("collapsed");
    document.body.classList.remove("console-collapsed");
    if (consolePane) {
      consolePane.scrollTop = consolePane.scrollHeight;
    }
  }

  const iconElem = document.getElementById("console-collapse-icon");
  const labelElem = document.getElementById("console-collapse-label");
  const btnCollapse = document.getElementById("console-collapse");
  if (iconElem) iconElem.textContent = willCollapse ? "▲" : "_";
  if (labelElem) labelElem.textContent = willCollapse ? "Expand" : "1-Line";
  if (btnCollapse) {
    btnCollapse.title = willCollapse ? "Expand terminal window" : "Shrink terminal into one line";
  }
}

function openTerminal(title) {
  if (title) {
    setConsoleTitle(title);
  }
  if (consoleWrap && consoleWrap.classList.contains("collapsed")) {
    toggleConsole(false);
  }
  if (consolePane) {
    consolePane.scrollTop = consolePane.scrollHeight;
  }
}

// The pane used to be a plain append onto the pane's textContent, which copies
// the entire buffer on every line and then reads scrollHeight, forcing a
// synchronous layout each time. That is quadratic, and a chatty slot makes it
// fatal: the micro-ROS agent emits a few hundred lines a second against a 50 Hz
// board, and on a fresh Jazzy box the tab pegged a core for 23 of its 24
// minutes of life. Timers stopped firing, so the 1-Click chain simply stopped
// between two steps -- no error, no failed request, nothing in any log.
// Keep a bounded ring buffer and repaint on a timer instead, so console volume
// can never starve the chain that is driving the robot.
const CONSOLE_MAX_LINES = 2000;
const CONSOLE_FLUSH_MS = 100;
const consoleLines = [];
let consoleFlushTimer = null;

function flushConsole() {
  consoleFlushTimer = null;
  if (!consolePane) return;
  consolePane.textContent = consoleLines.length ? consoleLines.join("\n") + "\n" : "";
  consolePane.scrollTop = consolePane.scrollHeight;
}

// setTimeout, not requestAnimationFrame: rAF does not fire in a background tab,
// and a user who switches away mid-SLAM must still get the log when they come
// back. Background throttling clamps this to ~1 Hz, which is plenty.
function queueConsoleFlush() {
  if (consoleFlushTimer !== null) return;
  consoleFlushTimer = setTimeout(flushConsole, CONSOLE_FLUSH_MS);
}

function logLine(text) {
  const lineStr = String(text);
  consoleLines.push(lineStr);
  if (consoleLines.length > CONSOLE_MAX_LINES) {
    consoleLines.splice(0, consoleLines.length - CONSOLE_MAX_LINES);
  }
  const lastLineElem = document.getElementById("console-last-line");
  if (lastLineElem) {
    lastLineElem.textContent = lineStr;
  }
  queueConsoleFlush();
}

function clearConsole() {
  consoleLines.length = 0;
  const lastLineElem = document.getElementById("console-last-line");
  if (lastLineElem) {
    lastLineElem.textContent = "";
  }
  queueConsoleFlush();
}

function setConsoleTitle(title) {
  consoleTitle.textContent = title;
}

const btnConsoleClear = document.getElementById("console-clear");
if (btnConsoleClear) {
  btnConsoleClear.addEventListener("click", (e) => {
    e.stopPropagation();
    clearConsole();
  });
}

const btnConsoleCollapse = document.getElementById("console-collapse");
if (btnConsoleCollapse) {
  btnConsoleCollapse.addEventListener("click", (e) => {
    e.stopPropagation();
    toggleConsole();
  });
}

const consoleHeader = document.getElementById("console-header");
if (consoleHeader) {
  consoleHeader.addEventListener("click", () => {
    toggleConsole();
  });
}

// ---------- tabs ----------
document.querySelectorAll(".tab-btn").forEach((btn) => {
  btn.addEventListener("click", () => {
    document.querySelectorAll(".tab-btn").forEach((b) => b.classList.remove("active"));
    document.querySelectorAll(".tab-pane").forEach((p) => p.classList.remove("active"));
    btn.classList.add("active");
    document.getElementById("tab-" + btn.dataset.tab).classList.add("active");
    // The config repo's state is asked for when its tab opens, not on the
    // status poll: `git status` on a bind mount is not free every two seconds.
    if (btn.dataset.tab === "settings") refreshConfigGit();
  });
});

// ---------- the config directory and its git repo ----------
// The user's robot configs are their own repository (scripts/cockpit_paths.py
// seeds and `git init`s it). Until this card there was nowhere in the UI that
// said where it is or let anyone record a change, so a robot tuned through
// Config Studio just accumulated uncommitted edits.
async function refreshConfigGit() {
  const pill = document.getElementById("config-git-pill");
  const pathEl = document.getElementById("config-dir-path");
  const detail = document.getElementById("config-git-detail");
  const changes = document.getElementById("config-git-changes");
  const commitBtn = document.getElementById("btn-config-commit");
  if (!pill || !pathEl) return;
  try {
    const g = await fetch("/api/config/git").then((r) => r.json());
    pathEl.textContent = g.path || "(unknown)";
    changes.style.display = "none";
    changes.textContent = "";
    if (!g.git_available) {
      pill.textContent = "no git";
      pill.className = "pill pill-unknown";
      detail.textContent = "git is not installed here, so the configs are files without a history.";
      if (commitBtn) commitBtn.disabled = true;
      return;
    }
    if (!g.is_repo) {
      pill.textContent = "not a repo";
      pill.className = "pill pill-unknown";
      detail.textContent = "This directory is not a git repository.";
      if (commitBtn) commitBtn.disabled = true;
      return;
    }
    if (commitBtn) commitBtn.disabled = !g.dirty;
    const last = g.last_commit ? ` Last commit: ${g.last_commit}.` : "";
    if (g.dirty) {
      pill.textContent = `${g.changes.length} uncommitted`;
      pill.className = "pill pill-off";
      detail.textContent = `Branch ${g.branch || "?"}.${last}`;
      changes.textContent = g.changes.join("\n");
      changes.style.display = "block";
    } else {
      pill.textContent = "clean";
      pill.className = "pill pill-ok";
      detail.textContent = `Branch ${g.branch || "?"}.${last}`;
    }
  } catch (e) {
    pill.textContent = "unavailable";
    pill.className = "pill pill-unknown";
    detail.textContent = String(e);
  }
}

document.getElementById("btn-config-git-refresh")?.addEventListener("click", refreshConfigGit);
document.getElementById("btn-config-commit")?.addEventListener("click", async () => {
  const input = document.getElementById("config-commit-message");
  const btn = document.getElementById("btn-config-commit");
  const detail = document.getElementById("config-git-detail");
  if (btn) btn.disabled = true;
  try {
    const res = await fetch("/api/config/commit", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ message: (input?.value || "").trim() }),
    }).then((r) => r.json());
    if (res.status === "ok") {
      if (input) input.value = "";
      logLine(`[console] config committed: ${res.detail}`);
    } else if (res.status === "noop") {
      logLine(`[console] ${res.detail}`);
    } else {
      logLine(`[console] commit failed: ${res.detail || res.error || "unknown error"}`);
      if (detail) detail.textContent = res.detail || "Commit failed.";
    }
  } catch (e) {
    logLine(`[console] commit failed: ${e}`);
  }
  refreshConfigGit();
});

// ---------- generic SSE command runner ----------
// slot: "main" -> /api/exec ; "agent" -> /api/agent/exec
//
// `spec` names WHAT to run; the server owns the command text (web/backend/
// actions.py). It is one of:
//   { action: "agent_start", args: {...} }   a named server-side action
//   { handle: "..." }                        a command another endpoint prepared
//   "some shell string"                      legacy raw (server refuses it unless
//                                            COCKPIT_ALLOW_RAW_EXEC is set)
function runCommand(spec, { slot = "main", title = "Running", action, onDone, onLine } = {}) {
  const endpoint = slot === "agent" ? "/api/agent/exec" : (slot === "bringup" ? "/api/bringup/exec" : "/api/exec");
  openTerminal(title);
  logLine(`$ [${slot}] ${title}`);
  let body;
  if (typeof spec === "string") body = { command: spec, slot };
  else if (spec && spec.handle) body = { action: "prepared", handle: spec.handle, slot };
  else if (spec && spec.action) body = { action: spec.action, args: spec.args || {}, slot };
  else body = { slot };
  return fetch(endpoint, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(body),
  }).then(async (response) => {
    if (response.status === 409) {
      logLine("[console] that slot is already busy -- stop the running action first.");
      if (onDone) onDone(-1);
      return;
    }
    if (!response.ok || !response.body) {
      logLine(`[console] failed to start: HTTP ${response.status}`);
      if (onDone) onDone(-1);
      return;
    }
    const reader = response.body.getReader();
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
        const evType = evMatch ? evMatch[1] : "message";
        let payload;
        try {
          payload = JSON.parse(dataMatch[1]);
        } catch {
          continue;
        }
        if (evType === "output") {
          const prefix = (slot !== "main") ? `[${slot}] ` : "";
          logLine(`${prefix}${payload.line}`);
          if (onLine) onLine(payload.line);
        } else if (evType === "done") {
          logLine(`[console] exited with code ${payload.exit_code}`);
          if (onDone) onDone(payload.exit_code);
        }
      }
    }
  });
}

function killSlot(slot) {
  const endpoint = slot === "agent" ? "/api/agent/kill" : (slot === "bringup" ? "/api/bringup/kill" : "/api/kill");
  return fetch(endpoint, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ slot }),
  });
}

// pairs up a Start/Stop button with a command builder for long-running actions
function wireStartStop({ startBtn, stopBtn, slot, title, buildCommand, needsAgent, needsBringup, stackTag }) {
  startBtn.addEventListener("click", async () => {
    startBtn.disabled = true;
    try {
      if (needsBringup) {
        await ensureBringupRunning(title);
      } else if (needsAgent) {
        await ensureAgentRunning();
      }
      const command = await buildCommand();
      stopBtn.disabled = false;
      runCommand(command, {
        slot,
        title,
        onDone: () => {
          startBtn.disabled = false;
          stopBtn.disabled = true;
        },
      });
    } catch (err) {
      console.error(`Failed to start ${title}:`, err);
      logLine(`[console] ✖ Failed to start ${title}: ${err.message || err}`);
      startBtn.disabled = false;
      stopBtn.disabled = true;
    }
  });
  stopBtn.addEventListener("click", async () => {
    killSlot(slot);
    // A 1-Click run leaves bringup, SLAM and Nav2 running on purpose, and they
    // are not this backend's children -- the pipeline started them and exited.
    // Stop has to reach those too, or the button only appears to work.
    if (stackTag) {
      try {
        await fetch("/api/stack/stop", {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify({ tag: stackTag }),
        });
      } catch (err) {
        console.warn(`stack stop (${stackTag}):`, err);
      }
    }
  });
}

// ---------- workspace / ros-env prefix ----------
function getDistro() {
  return (state.config && state.config.ros_distro) || (state.status && state.status.ros_distro) || "jazzy";
}

function envPrefix() {
  const distro = getDistro();
  const ws = (state.config && state.config.workspace_path) || "~/linorobot2_ws";
  // Service replies are dropped when rmw_fastrtps cannot match the response
  // writer to the client's reader inside max_blocking_time (100 ms by default),
  // which strands nav2's lifecycle manager mid-transition. The XML raises that
  // ceiling for service endpoints only -- see config/fastdds_service_qos.xml.
  // Everything nav2/SLAM is launched through here, not through server.py's
  // ros_env_prefix(), so the export has to live in this string too.
  const webDir = (state.status && state.status.web_dir) || "";
  const qosPrefix = webDir
    ? `export FASTDDS_DEFAULT_PROFILES_FILE=${webDir}/../config/fastdds_service_qos.xml; `
    : "";
  return qosPrefix +
         `export PATH=/usr/bin:$PATH; export ROS_DISTRO=${distro}; ` +
         `if [ -f /opt/ros/${distro}/setup.bash ]; then source /opt/ros/${distro}/setup.bash 2>/dev/null || true; ` +
         `elif [ -f /opt/ros/jazzy/setup.bash ]; then source /opt/ros/jazzy/setup.bash 2>/dev/null || true; ` +
         `elif [ -f /opt/ros/rolling/setup.bash ]; then source /opt/ros/rolling/setup.bash 2>/dev/null || true; ` +
         `fi; ` +
         `[ -f ${ws}/install/setup.bash ] && source ${ws}/install/setup.bash 2>/dev/null || true; `;
}

function gitCloneDistroSnippet(repoUrl, targetDir) {
  const distro = getDistro();
  return `[ -d ${targetDir} ] || git clone -b ${distro} ${repoUrl} ${targetDir} 2>/dev/null || ` +
         `git clone -b main ${repoUrl} ${targetDir} 2>/dev/null || ` +
         `git clone -b jazzy ${repoUrl} ${targetDir} 2>/dev/null || ` +
         `git clone ${repoUrl} ${targetDir}`;
}

function ws() {
  return (state.config && state.config.workspace_path) || "~/linorobot2_ws";
}

// ---------- status polling ----------
async function refreshStatus() {
  try {
    // Tell the server which controller 1-Click would actually run, so the
    // board-mismatch verdict it computes is about the run the user would get
    // -- not about the config's base_controller, which a Reference Build
    // preset can leave behind.
    const sel = document.getElementById("cockpit-target-select");
    const q = sel && sel.value ? `?controller=${encodeURIComponent(sel.value)}` : "";
    const res = await fetch(`/api/status${q}`);
    const s = await res.json();
    state.status = s;
    state.config = s.config;
    // On first load the robot is already chosen server-side, so selectRobot()
    // never runs and #cockpit-target-select keeps its hardcoded pico2 default --
    // Start 1-Click would then run the wrong controller against the plugged
    // board (a CP2102 ESP32 is not reliably told apart from an RP2 by VID:PID,
    // so the MCU guard cannot always catch it, and the mismatch only surfaces at
    // flash time). Sync the Operations select to the active robot's controller
    // ONCE, then leave it alone so a deliberate later override still stands.
    if (!state._ctrlSelSynced) {
      const bcName = s.config && s.config.base_controller && s.config.base_controller.name;
      const tsel = document.getElementById("cockpit-target-select");
      if (tsel && bcName && [...tsel.options].some((o) => o.value === bcName)) {
        state._ctrlSelSynced = true;
        if (tsel.value !== bcName) {
          tsel.value = bcName;
          if (window.__syncControllerSelects) window.__syncControllerSelects(bcName, "cockpit-target-select");
          logLine(`[console] base controller -> ${bcName} (synced to the active robot)`);
        }
      }
    }
    applyLaserConfigToPanel();
    state.mainBusy = s.main_busy;
    state.agentBusy = s.agent_busy_console;

    const distroSel = document.getElementById("hdr-distro-select");
    if (distroSel && s.ros_distro && document.activeElement !== distroSel) {
      distroSel.value = s.ros_distro;
    }

    // Robot name + branch header (don't clobber a field the user is editing)
    if (s.robot_name) state.robot_name = s.robot_name;
    if (Array.isArray(s.robots)) state.robots = s.robots;
    if (typeof s.git_branch === "string") state.git_branch = s.git_branch;
    const robotInput = document.getElementById("hdr-robot-name");
    if (robotInput && document.activeElement !== robotInput) {
      robotInput.value = state.robot_name;
    }
    const branchInput = document.getElementById("hdr-git-branch");
    if (branchInput && document.activeElement !== branchInput) {
      branchInput.value = state.git_branch;
    }
    if (s.git) {
      const gvText = document.getElementById("git-version-text");
      const gvBadge = document.getElementById("git-version-badge");
      const ver = [s.git.version_at_start, s.git.version, s.git.commit].find(knownGit);
      if (!ver) hideGitChips();
      if (gvText && ver) gvText.textContent = ver;
      if (gvBadge) {
        if (s.git.dirty || s.git.moved_since_start) gvBadge.classList.add("is-dirty");
        else gvBadge.classList.remove("is-dirty");
      }
    }
    const cfgDistroSel = document.getElementById("cfg-ros-distro");
    if (cfgDistroSel && s.ros_distro) {
      cfgDistroSel.value = s.ros_distro;
    }

    if (s.detected_mcu || s.controller) {
      const detectedMcu = s.detected_mcu || s.controller;
      const detectedChip = s.detected_chip || detectedMcu;
      // The board's own name for itself, when it has one. The VID:PID above says
      // which KIND of part is plugged in; this says WHICH ONE, which is the only
      // thing that tells two identical boards apart on one bench. The key is the
      // claim and the two are never merged: `uid` is the silicon's own id (RP2350
      // chip info, ESP32 eFuse MAC), `flashid` is the external flash chip's, all
      // an RP2040 has to offer -- it moves with the flash, not the MCU.
      const bid = s.board_id;
      const idKind = bid && bid.kind === "flashid" ? "flash" : "chip";
      const idLabel = bid ? `${idKind} ${bid.id}` : "";
      const mcuBadge = document.getElementById("hdr-detected-mcu-badge");
      if (mcuBadge) {
        mcuBadge.textContent = detectedChip;
        mcuBadge.title = bid
          ? `Auto-detected MCU: ${detectedMcu} (${detectedChip})\n` +
            (bid.kind === "flashid"
              ? `Flash chip id ${bid.id} — this RP2040 has no id of its own, so this names the board but moves if the flash is replaced.`
              : `Chip uid ${bid.id} — burned into the silicon.`) +
            (bid.confirmed ? "" : " (from a flash this host did not hear confirmed)")
          : `Auto-detected MCU: ${detectedMcu} (${detectedChip})`;
      }
      const mcuIdBadge = document.getElementById("hdr-board-id-badge");
      if (mcuIdBadge) {
        mcuIdBadge.textContent = idLabel;
        mcuIdBadge.hidden = !bid;
        if (mcuBadge) mcuIdBadge.title = mcuBadge.title;
      }
      const baseMcuName = document.getElementById("base-detected-mcu-name");
      if (baseMcuName) {
        baseMcuName.textContent = detectedChip;
      }
      const busPorts = (s.ports && s.ports.local_ports) || [];
      const busWhere = "this robot computer";
      const baseMcuDesc = document.getElementById("base-detected-mcu-desc");
      if (baseMcuDesc) {
        const lp = busPorts.find(p => p.mcu_hint === detectedMcu) || busPorts[0];
        if (s.mcu_detected && lp) {
          baseMcuDesc.innerHTML = `Probed via USB sysfs VID:PID <code>${lp.vid || ""}:${lp.pid || ""}</code> on <code>${lp.path}</code> at ${escapeHtml(busWhere)} (${lp.product || lp.chip}).`;
        } else {
          // Nothing answered. Say that, and clear the probe line -- leaving the
          // previous poll's VID:PID under a heading that says "Auto-Detected"
          // presents a configured fallback as a live reading. This is the
          // normal state while a board sits in BOOTSEL, i.e. mid-flash.
          baseMcuDesc.innerHTML = `No board is answering on the USB bus right now — showing the ` +
            `controller from your robot config (<code>${escapeHtml(detectedMcu)}</code>), not a probe ` +
            `result. This is expected while a board is in BOOTSEL during flashing.`;
        }
      }
      if (baseMcuName) {
        baseMcuName.textContent = s.mcu_detected ? detectedChip : `${detectedMcu} (from config — nothing detected)`;
      }

      // The bus contradicting the config, said before the user presses Start
      // rather than twenty seconds into a run. The verdict is computed server
      // side from the one vid/pid table, so it cannot disagree with the guard
      // that refuses the flash.
      const mmEl = document.getElementById("hdr-mcu-mismatch");
      if (mmEl) {
        const mm = s.mcu_mismatch;
        if (mm) {
          mmEl.innerHTML = `⚠️ <b>Board does not match this robot.</b> ` +
            `<code>${escapeHtml(mm.controller)}</code> builds for <b>${escapeHtml(mm.expected)}</b>, ` +
            `but the board on the bus is <b>${escapeHtml(mm.detected)}</b> (${escapeHtml(mm.chip)}). ` +
            `1-Click will refuse to flash. Pick the robot that matches the board, or plug in the ` +
            `board this robot is written for.`;
          mmEl.hidden = false;
        } else {
          mmEl.hidden = true;
        }
      }
      const baseMcuPill = document.getElementById("base-mcu-status-pill");
      if (baseMcuPill) {
        const lp = busPorts.find(p => p.mcu_hint === detectedMcu) || busPorts[0];
        if (lp) {
          baseMcuPill.textContent = lp.busy ? "In Use / Busy" : "Online & Free";
          baseMcuPill.className = "pill " + (lp.busy ? "pill-busy" : "pill-ok");
        } else {
          baseMcuPill.textContent = "Offline / Not Connected";
          baseMcuPill.className = "pill pill-unknown";
        }
      }
      if (!window.hasInitializedPresets && typeof updateReferenceDesigns === "function") {
        window.hasInitializedPresets = true;
        updateReferenceDesigns(detectedMcu);
      }
    }

    if (s.fake_mode_active !== undefined && !window.hasSyncedFakeMode) {
      window.hasSyncedFakeMode = true;
      if (typeof updateFakeModeUI === "function") {
        updateFakeModeUI(s.fake_mode_active);
      }
    }
    if (typeof updateBringupSummary === "function") {
      updateBringupSummary();
    }

    const wsPill = document.getElementById("hdr-workspace-pill");
    if (wsPill) {
      wsPill.textContent = s.workspace_built ? "built" : "unbuilt";
      wsPill.className = "pill " + (s.workspace_built ? "pill-ok" : "pill-unknown");
    }
    // `hdr-workspace-pill` above already says built/unbuilt, and it is the
    // element the header actually has. A second readout of the same flag can
    // only ever agree with it or be a bug.

    // Both pills carry `pill-unknown` in the markup until the first poll lands;
    // setting only textContent left them grey forever, which read as "the
    // header never updates".
    const liveWhere = (s.liveness && s.liveness.where) || "host";
    const agentPill = document.getElementById("hdr-agent-pill");
    const agentAlive = s.agent_alive_external || s.agent_busy_console;
    if (agentPill) {
      agentPill.textContent = agentAlive ? "running" : "down";
      agentPill.className = "pill " + (agentAlive ? "pill-ok" : "pill-off");
      agentPill.title = agentAlive
        ? `micro-ROS agent detected on ${liveWhere}` + ((s.liveness && s.liveness.detail) ? `: ${s.liveness.detail}` : "")
        : `No micro-ROS agent process or container found on ${liveWhere}`;
    }
    const btnAgentStop = document.getElementById("btn-agent-stop");
    if (btnAgentStop) btnAgentStop.disabled = !s.agent_busy_console;

    const bringupPill = document.getElementById("hdr-bringup-pill");
    const bringupAlive = s.bringup_alive_external || s.bringup_busy_console;
    if (bringupPill) {
      bringupPill.textContent = bringupAlive ? "running" : "down";
      bringupPill.className = "pill " + (bringupAlive ? "pill-ok" : "pill-off");
      bringupPill.title = bringupAlive
        ? `Bringup nodes detected on ${liveWhere}`
        : `No bringup.launch.py / ekf_node / robot_state_publisher on ${liveWhere}`;
    }
    if (s.syslog) {
      updateSyslogUIState(s.syslog);
    }
    const bringupStartBtn = document.getElementById("btn-bringup-start");
    const bringupStopBtn = document.getElementById("btn-bringup-stop");
    if (bringupStartBtn) bringupStartBtn.disabled = s.bringup_busy_console;
    if (bringupStopBtn) bringupStopBtn.disabled = !s.bringup_busy_console;

    const autoBringupCfg = document.getElementById("cfg-auto-bringup");
    const autoBringupToggle = document.getElementById("bringup-auto-toggle");
    if (s.config && typeof s.config.auto_bringup === "boolean") {
      if (autoBringupCfg) autoBringupCfg.checked = s.config.auto_bringup;
      if (autoBringupToggle) autoBringupToggle.checked = s.config.auto_bringup;
    }

    if (!document.getElementById("install-workspace").value) {
      document.getElementById("install-workspace").value = s.workspace_path;
    }
    if (!document.getElementById("cfg-workspace").value) {
      document.getElementById("cfg-workspace").value = s.workspace_path;
    }
    if (s.config) {
      const c = s.config;
      const setIfEmpty = (id, val) => {
        const el = document.getElementById(id);
        if (el && !el.value) el.value = val;
      };
      document.getElementById("cfg-agent-transport").value = c.agent_transport;
      setIfEmpty("cfg-agent-device", c.agent_device);
      setIfEmpty("cfg-agent-port", c.agent_port);
      setIfEmpty("cfg-agent-baud", c.agent_baud);

      if (c.install_mode) {
        const hdrM = document.getElementById("hdr-install-mode");
        const tabM = document.getElementById("install-mode");
        if (hdrM && !localStorage.getItem("linorobot2_install_mode")) hdrM.value = c.install_mode;
        if (tabM && !localStorage.getItem("linorobot2_install_mode")) {
          tabM.value = c.install_mode;
          const isNative = c.install_mode === "native";
          const natCards = document.getElementById("install-native-cards");
          const dkrCard = document.getElementById("install-docker-card");
          if (natCards) natCards.style.display = isNative ? "block" : "none";
          if (dkrCard) dkrCard.style.display = isNative ? "none" : "block";
        }
      }

      if (c.agent_engine) {
        const hdrA = document.getElementById("hdr-agent-engine");
        const cfgA = document.getElementById("cfg-agent-engine");
        if (hdrA && !localStorage.getItem("linorobot2_agent_engine")) hdrA.value = c.agent_engine;
        if (cfgA && !localStorage.getItem("linorobot2_agent_engine")) cfgA.value = c.agent_engine;
      }

      if (c.container_registry) {
        const hdrR = document.getElementById("hdr-container-registry");
        const cfgR = document.getElementById("cfg-container-registry");
        const regMode = (c.container_registry === "custom" || (!["auto", "cluster", "dockerhub"].includes(c.container_registry))) ? "custom" : c.container_registry;
        const customVal = c.custom_registry || (!["auto", "cluster", "dockerhub"].includes(c.container_registry) ? c.container_registry : "");
        if (hdrR && !localStorage.getItem("linorobot2_container_registry")) hdrR.value = regMode;
        if (cfgR && !localStorage.getItem("linorobot2_container_registry")) cfgR.value = regMode;
        const hdrCust = document.getElementById("hdr-custom-registry");
        const cfgCust = document.getElementById("cfg-custom-registry");
        if (hdrCust) {
          if (!localStorage.getItem("linorobot2_custom_registry")) hdrCust.value = customVal;
          hdrCust.style.display = (regMode === "custom") ? "inline-block" : "none";
        }
        if (cfgCust) {
          if (!localStorage.getItem("linorobot2_custom_registry")) cfgCust.value = customVal;
          cfgCust.style.display = (regMode === "custom") ? "block" : "none";
        }
      }
    }
  } catch (e) {
    // server not reachable yet / transient -- ignore, next poll will retry
  }
}
setInterval(refreshStatus, 4000);
refreshStatus();

// ---------- Robot Name + Branch header ----------
// <robot>_config.yaml is the single source of truth; it is git
// auto-committed on the active branch before every action (server-side).
async function loadGitInfo() {
  try {
    const gi = await fetch("/api/gitinfo", { cache: "no-cache" }).then((r) => r.json());
    state.git_branch = gi.branch || state.git_branch;
    state.git_branches = gi.branches || [];
    const branchInput = document.getElementById("hdr-git-branch");
    if (branchInput && knownGit(state.git_branch) && document.activeElement !== branchInput) branchInput.value = state.git_branch;
    const text = document.getElementById("git-version-text");
    const badge = document.getElementById("git-version-badge");
    const shown = [gi.version_at_start, gi.version, gi.commit].find(knownGit);
    if (!shown || !knownGit(state.git_branch)) hideGitChips();
    if (text && shown) {
      text.textContent = shown;
    }
    if (badge) {
      if (gi.dirty || gi.moved_since_start) badge.classList.add("is-dirty");
      else badge.classList.remove("is-dirty");
    }
  } catch (e) { /* ignore */ }
}

async function loadRobotList() {
  try {
    const r = await fetch("/api/robots").then((x) => x.json());
    state.robots = r.robots || [];
    state.robot_name = r.active || state.robot_name;
  } catch (e) { /* ignore */ }
}

// Dropdown picker shared by the Robot and Branch header fields. Ported from
// robot_config_engine's tested initBranchPicker(): the input itself opens the
// list, ArrowUp/Down opens it, Escape closes it, typing filters it live, and
// the entry matching the current value carries a green dot.
function initHeaderPicker({ inputId, caretId, menuId, loadItems, onPick, emptyText }) {
  const input = document.getElementById(inputId);
  const caret = document.getElementById(caretId);
  const menu = document.getElementById(menuId);
  if (!input || !menu) return;

  const close = () => {
    menu.hidden = true;
    if (caret) caret.setAttribute("aria-expanded", "false");
  };

  const pick = (name) => {
    input.value = name;
    close();
    onPick(name);
  };

  const render = (res) => {
    const items = (res && Array.isArray(res.items)) ? res.items : (Array.isArray(res) ? res : []);
    const current = res && res.current;
    if (!items.length) {
      menu.innerHTML = `<div class="branch-empty">${escapeHtml(emptyText)}</div>`;
      return;
    }
    const typed = input.value.trim();
    menu.innerHTML = items.map((b) => `
      <button type="button" role="option" class="branch-item${b === current ? " is-current" : ""}${b === typed ? " is-active" : ""}" data-name="${escapeHtml(b)}">
        <span class="branch-cur-dot"></span><span>${escapeHtml(b)}</span>${
          b === current ? '<span style="margin-left:auto;font-size:0.68rem;opacity:0.6">current</span>' : ""
        }
      </button>`).join("");
    menu.querySelectorAll(".branch-item").forEach((btn) => {
      btn.addEventListener("click", () => pick(btn.dataset.name));
    });
  };

  const open = async () => {
    menu.hidden = false;
    if (caret) caret.setAttribute("aria-expanded", "true");
    menu.innerHTML = `<div class="branch-empty">Loading…</div>`;
    try {
      const res = await loadItems();
      render(res);
    } catch (err) {
      console.error("[initHeaderPicker] Error loading items:", err);
      menu.innerHTML = `<div class="branch-empty">Failed to load items.</div>`;
    }
  };

  input.addEventListener("click", (e) => { e.stopPropagation(); if (menu.hidden) open(); });
  input.addEventListener("keydown", (e) => {
    if ((e.key === "ArrowDown" || e.key === "ArrowUp") && menu.hidden) { e.preventDefault(); open(); }
  });
  if (caret) {
    caret.addEventListener("click", (e) => {
      e.stopPropagation();
      if (menu.hidden) open(); else close();
    });
  }
  // Re-filter the visible list as the user types.
  input.addEventListener("input", () => {
    if (menu.hidden) return;
    const typed = input.value.trim().toLowerCase();
    menu.querySelectorAll(".branch-item").forEach((btn) => {
      btn.style.display = btn.dataset.name.toLowerCase().includes(typed) ? "" : "none";
      btn.classList.toggle("is-active", btn.dataset.name === input.value.trim());
    });
  });
  document.addEventListener("click", (e) => {
    if (!menu.hidden && !menu.contains(e.target) && e.target !== input && e.target !== caret) close();
  });
  document.addEventListener("keydown", (e) => {
    if (e.key === "Escape" && !menu.hidden) { close(); input.blur(); }
  });
}

async function selectRobot(name) {
  name = (name || "").trim();
  if (!name || !/^[a-z0-9_]+$/.test(name)) {
    logLine(`[console] invalid robot name: "${name}" (use lowercase, digits, _)`);
    const ri = document.getElementById("hdr-robot-name");
    if (ri) ri.value = state.robot_name;
    return;
  }
  if (name === state.robot_name) return;
  try {
    const res = await fetch("/api/robot/select", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ name }),
    }).then((r) => r.json());
    if (res.error || res.detail) {
      logLine(`[console] ${res.error || res.detail}`);
      const ri = document.getElementById("hdr-robot-name");
      if (ri) ri.value = state.robot_name;
      return;
    }
    state.robot_name = res.active || res.robot_name || name;
    state.robots = res.robots || [];
    state.config = res.config || state.config;
    // Push the switched robot's workflow settings into the header selects.
    const c = state.config || {};
    const put = (id, v) => { const el = document.getElementById(id); if (el && v != null) el.value = v; };
    put("hdr-distro-select", c.ros_distro);
    put("hdr-install-mode", c.install_mode);
    put("hdr-agent-engine", c.agent_engine);
    put("install-mode", c.install_mode);
    put("cfg-agent-engine", c.agent_engine);
    put("hdr-container-registry", c.container_registry);
    put("cfg-container-registry", c.container_registry);
    const ri = document.getElementById("hdr-robot-name");
    if (ri) ri.value = state.robot_name;

    // The base controller has to follow the robot, or the two silently diverge.
    // 1-Click reads its controller from #cockpit-target-select, NOT from the
    // robot selector, so leaving this stale means switching robots and pressing
    // Start runs the PREVIOUS robot's controller -- against the previous
    // robot's port, with no warning anywhere in the UI. That is the §3 single
    // source of truth inverted: the config file is the selection, so the select
    // must reflect it. Only adopt a value the select actually offers; assigning
    // an unknown one blanks the element and is worse than leaving it alone.
    const bcName = (c.base_controller || {}).name;
    const tsel = document.getElementById("cockpit-target-select");
    if (tsel && bcName) {
      if ([...tsel.options].some((o) => o.value === bcName)) {
        if (tsel.value !== bcName) {
          tsel.value = bcName;
          tsel.dispatchEvent(new Event("change"));
          logLine(`[console] base controller -> ${bcName}`);
        }
        if (window.__syncControllerSelects) window.__syncControllerSelects(bcName, "cockpit-target-select");
      } else {
        logLine(`[console] warning: no '${bcName}' entry in the base-controller list; 1-Click will use '${tsel.value}'`);
      }
    }

    logLine(`[console] active robot -> ${state.robot_name}  (${res.robot_config_path || "<robot>_config.yaml"})`);
    refreshStatus();
  } catch (e) {
    logLine(`[console] robot select failed: ${e}`);
  }
}

function checkoutBranch(branch) {
  branch = (branch || "").trim();
  if (!branch || !/^[A-Za-z0-9._/-]+$/.test(branch)) {
    logLine(`[console] invalid branch name: "${branch}"`);
    return;
  }
  if (branch === state.git_branch) {
    logLine(`[console] already on branch "${branch}"`);
    return;
  }
  setConsoleTitle(`git checkout ${branch}`);
  logLine(`$ git checkout ${branch}`);
  fetch("/api/gitinfo/branch", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ branch }),
  }).then(async (response) => {
    if (!response.ok || !response.body) {
      logLine(`[console] checkout failed: HTTP ${response.status}`);
      return;
    }
    const reader = response.body.getReader();
    const decoder = new TextDecoder();
    let buf = "";
    for (;;) {
      const { done, value } = await reader.read();
      if (done) break;
      buf += decoder.decode(value, { stream: true });
      const parts = buf.split("\n\n");
      buf = parts.pop();
      for (const chunk of parts) {
        const m = /^data: (.*)$/m.exec(chunk);
        if (!m) continue;
        try {
          const payload = JSON.parse(m[1]);
          if (payload.line) logLine(payload.line);
          if (typeof payload.exit_code === "number") {
            logLine(`[console] checkout exited ${payload.exit_code}`);
            loadGitInfo();
            refreshStatus();
          }
        } catch (e) { /* ignore */ }
      }
    }
  }).catch((e) => logLine(`[console] checkout error: ${e}`));
}

function setupRobotBranchHeader() {
  const robotInput = document.getElementById("hdr-robot-name");
  const branchInput = document.getElementById("hdr-git-branch");
  const toNameBtn = document.getElementById("btn-branch-to-name");
  if (!robotInput || !branchInput) return;

  robotInput.addEventListener("keydown", (e) => {
    if (e.key === "Enter") { e.preventDefault(); robotInput.blur(); }
  });
  robotInput.addEventListener("blur", () => selectRobot(robotInput.value));

  branchInput.addEventListener("keydown", (e) => {
    if (e.key === "Enter") { e.preventDefault(); checkoutBranch(branchInput.value); }
  });

  initHeaderPicker({
    inputId: "hdr-robot-name",
    caretId: "btn-robot-menu",
    menuId: "robot-menu",
    emptyText: "No saved robot configs.",
    loadItems: async () => {
      await loadRobotList();
      // A robot is named by its file's CONTENT (robot.name), so two files can
      // claim one name. Every file still gets a row -- losing one to a name
      // clash is how a robot disappears from the cockpit while the CLI can
      // still see it -- and `select` is the handle that reaches THIS file: the
      // name when it is unique, the filename stem when it is not. The clash is
      // shown rather than resolved, because only the user can fix it.
      return {
        items: state.robots.map((r) => (r.conflict ? `${r.name} (${r.filename})` : r.name)),
        current: state.robot_name,
      };
    },
    onPick: (label) => {
      const hit = state.robots.find(
        (r) => label === r.name || label === `${r.name} (${r.filename})`);
      return selectRobot(hit ? hit.select : label);
    },
  });

  initHeaderPicker({
    inputId: "hdr-git-branch",
    caretId: "btn-branch-menu",
    menuId: "branch-menu",
    emptyText: "No local git branches.",
    loadItems: async () => {
      await loadGitInfo();
      return { items: state.git_branches, current: state.git_branch };
    },
    onPick: checkoutBranch,
  });

  toNameBtn.addEventListener("click", async () => {
    const n = (robotInput.value || state.robot_name || "").trim();
    if (!n) return;
    if (n !== state.robot_name) {
      await selectRobot(n);
    }
    branchInput.value = n;
    checkoutBranch(n);
  });

  loadRobotList();
  loadGitInfo();
}
setupRobotBranchHeader();

// =============================================================================
// Header Git Version Badge — shows the 7-char commit the web server booted on;
// click to reveal the branch, remotes and last 10 commits (GET /api/gitinfo).
// =============================================================================
function initGitVersionBadge() {
  const badge = document.getElementById("git-version-badge");
  const text = document.getElementById("git-version-text");
  const popover = document.getElementById("git-version-popover");
  if (!badge || !text || !popover) return;

  const esc = (s) => String(s == null ? "" : s).replace(/[&<>"']/g, (c) => (
    { "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;" }[c]
  ));

  let loaded = null;

  const render = (info) => {
    const remotes = (info.remotes || []).map((r) => `
      <div class="gv-line">
        <span class="gv-remote-name">${esc(r.name)}</span>
        <span class="gv-val">${esc(r.url)}</span>
      </div>`).join("") || `<div class="gv-line"><span class="gv-val">(no remotes)</span></div>`;

    const commits = (info.commits || []).map((c) => `
      <li>
        <div><span class="gv-hash">${esc(c.hash)}</span> <span class="gv-subject">${esc(c.subject)}</span></div>
        <div class="gv-meta">${esc(c.author)} · ${esc(c.date)} (${esc(c.reldate)})</div>
      </li>`).join("") || `<li><span class="gv-meta">(no commit history)</span></li>`;

    const movedNote = info.moved_since_start
      ? `<div class="gv-note">⚠ HEAD is now at <code>${esc(info.version)}</code> — the server is still running the <code>${esc(info.version_at_start)}</code> build. Restart server.py to pick up the new code.</div>`
      : "";
    const dirtyNote = info.dirty
      ? `<div class="gv-note">Working tree has uncommitted changes.</div>`
      : "";

    popover.innerHTML = `
      <h4>Version</h4>
      <div class="gv-line"><span class="gv-key">server @</span><span class="gv-val">${esc(info.version_at_start || info.version)}</span></div>
      <div class="gv-line"><span class="gv-key">branch</span><span class="gv-val">${esc(info.branch)}</span></div>
      <h4>Remotes</h4>
      ${remotes}
      <h4>Last 10 commits</h4>
      <ol class="gv-commits">${commits}</ol>
      ${movedNote}
      ${dirtyNote}`;
  };

  const load = async () => {
    try {
      const res = await fetch("/api/gitinfo", { cache: "no-cache" });
      if (!res.ok) return null;
      return await res.json();
    } catch (e) {
      return null;
    }
  };

  const closePopover = () => {
    popover.hidden = true;
    badge.setAttribute("aria-expanded", "false");
  };

  const openPopover = async () => {
    const fresh = await load();
    if (fresh) {
      loaded = fresh;
      render(loaded);
      const branchInput = document.getElementById("hdr-git-branch");
      if (branchInput && document.activeElement !== branchInput) {
        branchInput.value = fresh.branch;
      }
    }
    if (!loaded) return;
    popover.hidden = false;
    badge.setAttribute("aria-expanded", "true");
  };

  badge.addEventListener("click", (e) => {
    e.stopPropagation();
    if (popover.hidden) openPopover();
    else closePopover();
  });
  document.addEventListener("click", (e) => {
    if (!popover.hidden && !popover.contains(e.target) && e.target !== badge) closePopover();
  });
  document.addEventListener("keydown", (e) => {
    if (e.key === "Escape" && !popover.hidden) closePopover();
  });

  // Prime the badge label at startup.
  load().then((info) => {
    // A deployment that cannot know its git state (the Docker image carries
    // no .git) shows nothing rather than "unknown"/"no-git": a chip that can
    // only ever say "unknown" makes the chips that matter look unreliable.
    const version = info && [info.version_at_start, info.version, info.commit].find(knownGit);
    if (!info || !version) {
      hideGitChips();
      return;
    }
    loaded = info;
    render(info);
    text.textContent = version;
    if (info.dirty || info.moved_since_start) badge.classList.add("is-dirty");
    const branchInput = document.getElementById("hdr-git-branch");
    if (branchInput && !knownGit(info.branch)) {
      hideGitChips();
    } else if (branchInput && document.activeElement !== branchInput) {
      branchInput.value = info.branch;
    }
  });
}
initGitVersionBadge();

