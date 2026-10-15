// The tool panel (Hardware Tests). The board's test shell, held by the backend
// (web/backend/routes_tool.py, one session per robot) and shown here: every button types
// its command into the terminal, so what the page does is what a person could type.
//
// Safety, from the 2026-10-13 web-designer review: STOP sends `stop` to the board (and
// lino-stop on Wi-Fi) -- it never just closes a reader; the backend keeps the board's
// keep-alive going only while this page beats once a second, so a closed lid or a dropped
// tab stops a drive; nothing here writes the env's `app` key.
"use strict";

const tool = {
  open: false,
  watch: null,
  es: null,
  beatTimer: null,
  stateTimer: null,
  running: null,
  // The robot the session was opened for. STOP, commands and beats name it: resolved by the
  // ACTIVE robot instead, a header switch made STOP answer 404 with a wheel still turning.
  robot: null,
  wheels: 4,
  simulatedWheels: false,
  scanThenLeave: false,
  lines: 0,
  // Each [TOOL_JSON] object, to whoever listens (the Bring-up checklist, app-bringup.js).
  jsonHooks: [],
};

const $t = (id) => document.getElementById(id);

function toolStatus(text, failed = false) {
  const el = $t("tool-status");
  if (el) { el.textContent = text || ""; el.classList.toggle("tool-stop-failed", !!failed); }
}

// The body of every request about the session: its robot, when the page knows it.
function toolBody(extra = {}) {
  return JSON.stringify(tool.robot ? { robot: tool.robot, ...extra } : extra);
}

function toolTermAppend(line) {
  const term = $t("tool-term");
  if (!term) return;
  term.append(document.createTextNode(line + (line.endsWith("tool> ") ? "" : "\n")));
  tool.lines++;
  if (tool.lines > 2000) {           // keep the page light: the last ~1500 lines
    const text = term.textContent;
    term.textContent = text.slice(text.length - 120000);
    tool.lines = 1500;
  }
  term.scrollTop = term.scrollHeight;
}

function toolSetControls() {
  const open = tool.open;
  const checks = $t("tool-checks"), wheel = $t("tool-wheel");
  if (checks) checks.disabled = !open;
  if (wheel) wheel.disabled = !open || tool.simulatedWheels;
  for (const b of document.querySelectorAll("#tool-panel .tool-drive")) {
    b.disabled = !open || tool.simulatedWheels;
  }
  const inp = $t("tool-input"), send = $t("btn-tool-send");
  if (inp) inp.disabled = !open;
  if (send) send.disabled = !open;
  const op = $t("btn-tool-open"), cl = $t("btn-tool-close");
  if (op) op.disabled = open;
  if (cl) cl.disabled = !open;
  const st = $t("tool-state");
  if (st) st.textContent = open ? (tool.running ? `running ${tool.running}` : "in tool") : "closed";
  const stop = $t("btn-tool-stop");
  if (stop) stop.hidden = !(open && tool.running);
  // At phone width STOP is a bar fixed over the bottom of the page: room is made for it.
  document.body?.classList.toggle("tool-stop-on", !!(open && tool.running));
  // A 2wd robot has wheels 1 and 2 only (the firmware refuses 3 and 4).
  for (const n of [3, 4]) {
    const w = document.querySelector(`#tool-wheel input[name="tool-wheel"][value="${n}"]`);
    if (w) { w.disabled = tool.wheels < n; w.closest("label").hidden = tool.wheels < n; }
  }
  const note = $t("tool-wheel-note");
  if (note) note.textContent = tool.simulatedWheels
    ? "This robot's config simulates its wheels: the tool runs on real parts only. Turn the simulation off on the Sensors tab and press Flash MCU."
    : "";
}

async function toolRefreshState() {
  try {
    const r = await fetch("/api/tool/state");
    if (!r.ok) return;
    const s = await r.json();
    if ($t("tool-robot")) $t("tool-robot").textContent = s.robot || "—";
    if ($t("tool-link")) $t("tool-link").textContent = s.wifi ? "Wi-Fi" : "USB serial";
    const ref = $t("tool-refusal");
    if (ref) { ref.hidden = !s.refusal; ref.textContent = s.refusal || ""; }
    const op = $t("btn-tool-open");
    if (op && s.refusal) op.disabled = true;
    if (s.open && !tool.es) {
      // A session the backend already holds (an earlier visit, another window): attach, and
      // say so -- Enter tool is disabled then, and without a word the page looked broken.
      tool.robot = s.robot || tool.robot;
      toolAttach();
      toolStatus(`In the tool on ${tool.robot || "this robot"} -- it was already open (another window ` +
                 "or an earlier visit). Back to robot ends it.");
    }
  } catch { /* the next refresh says it */ }
}

function toolParse(line) {
  // The board's answers (scripts/tool_answer.py): "ok: <line> (running)", "fail: <line> --
  // why", "done: <line>", "stopped: <line> (why)". Shown as they come -- they are the words
  // a person reads -- and the end of a running command clears STOP.
  const ans = /^\s*(ok|fail|done|stopped): /.exec(line);
  // "fail" ends a running command too: refused at its start, or ended without its work
  // ("fail: acc 500 30 -- the wheels are simulated ...").
  if (ans && (ans[1] === "done" || ans[1] === "stopped" ||
              (ans[1] === "fail" && tool.running && line.includes(`fail: ${tool.running}`)))) {
    tool.running = null;
    toolStatus(line.trim());
    toolSetControls();
  }
  // [TOOL_JSON]: the live readouts (firmware test_motors.cpp / test_sensors.cpp).
  const tj = line.indexOf("[TOOL_JSON] ");
  if (tj >= 0) {
    try {
      const j = JSON.parse(line.slice(tj + 12));
      if (Array.isArray(j.rpm)) {
        j.rpm.forEach((v, i) => {
          const e = $t(`tool-live-m${i + 1}`);
          const enc = j.enc && j.enc[i] ? `  ENC ${j.enc[i][0]}/${j.enc[i][1]}` : "";
          if (e) e.textContent = `${Number(v).toFixed(1)} RPM${enc}`;
        });
      }
      if (Array.isArray(j.count)) {
        j.count.forEach((v, i) => { const e = $t(`tool-live-m${i + 1}`); if (e) e.textContent = `count ${v}`; });
      }
      if ("bat" in j) { const e = $t("tool-live-bat"); if (e) e.textContent = j.bat == null ? "none" : `${j.bat.toFixed(2)} V`; }
      if (j.rpy) { const e = $t("tool-live-imu"); if (e) e.textContent = `RPY ${j.rpy.map((v) => v.toFixed(1)).join(" ")}`; }
      else if (j.acc) { const e = $t("tool-live-imu"); if (e) e.textContent = `ACC ${j.acc.map((v) => v.toFixed(2)).join(" ")}`; }
      // The gyro and the magnetometer too: turning the robot in your hands shows here.
      if (j.gyr) { const e = $t("tool-live-gyr"); if (e) e.textContent = j.gyr.map((v) => v.toFixed(2)).join(" "); }
      if (j.mag_ut) { const e = $t("tool-live-mag"); if (e) e.textContent = `${j.mag_ut.map((v) => v.toFixed(1)).join(" ")} uT`; }
      if ("range" in j) { const e = $t("tool-live-range"); if (e) e.textContent = j.range == null ? "none" : `${j.range.toFixed(2)} m`; }
      for (const h of tool.jsonHooks) { try { h(j); } catch { /* one listener's fault is its own */ } }
    } catch { /* a cut line: the next one comes in a second */ }
    return true;            // not shown in the terminal: the readouts are its view
  }
  // [I2C_JSON]: the bus, as a table.
  const ij = line.indexOf("[I2C_JSON] ");
  if (ij >= 0) {
    try { toolI2cTable(JSON.parse(line.slice(ij + 11))); } catch { /* ignore */ }
  }
  if (typeof adcCurveFromLine === "function") adcCurveFromLine(line);
  const w = /\[tool\] wheels: (\d), (simulated|real)/.exec(line);
  if (w) {
    tool.wheels = parseInt(w[1], 10);
    tool.simulatedWheels = w[2] === "simulated";
    if ($t("tool-wheels")) $t("tool-wheels").textContent = `${w[1]}, ${w[2]}`;
    toolSetControls();
  }
  if (/\[test_acc\] STOPPED/.test(line)) toolStatus(line.replace(/^\s*\[\w+\]\s*/, ""));
  return false;
}

function toolI2cTable(j) {
  const rows = (j.devices || []).map((d) =>
    `<tr><td class="font-mono">${d.addr}</td><td>${d.category}</td><td>${d.model}</td><td>${d.desc || ""}</td></tr>`).join("");
  const html = rows
    ? `<table class="data-table"><caption class="sr-only">I2C devices</caption><thead><tr><th>Address</th><th>Kind</th><th>Chip</th><th>Description</th></tr></thead><tbody>${rows}</tbody></table>`
    : "<p>No device answered on the I2C bus.</p>";
  for (const id of ["tool-i2c-table", "i2c-detect-results"]) {
    const box = $t(id);
    if (box) { box.hidden = false; box.style.display = "block"; box.innerHTML = html; }
  }
  if (tool.scanThenLeave) { tool.scanThenLeave = false; setTimeout(toolClose, 500); }
}

async function toolAttach() {
  const ticket = typeof streamTicket === "function" ? await streamTicket() : "";
  const q = [];
  if (ticket) q.push(`ticket=${encodeURIComponent(ticket)}`);
  if (tool.robot) q.push(`robot=${encodeURIComponent(tool.robot)}`);
  const es = new EventSource("/api/tool/stream" + (q.length ? `?${q.join("&")}` : ""));
  tool.es = es;
  es.onmessage = (ev) => {
    let m;
    try { m = JSON.parse(ev.data); } catch { return; }
    if (m.watch) { tool.watch = m.watch; tool.open = true; if (m.robot) tool.robot = m.robot; }
    if (m.line !== undefined) {
      if (!toolParse(m.line)) toolTermAppend(m.line);
    }
    const st = m.state || (m.watch ? m : null);
    if (st) {
      tool.running = st.running || null;
      toolSetControls();
    }
    if (m.closed) toolDetached("The tool session closed.");
  };
  es.onerror = () => { if (es.readyState === EventSource.CLOSED) toolDetached("Lost the tool session."); };
  clearInterval(tool.beatTimer);
  // The board's keep-alive is sent only while a page beats: a closed tab stops a drive.
  tool.beatTimer = setInterval(() => {
    if (tool.watch == null) return;
    fetch("/api/tool/beat", { method: "POST", headers: { "Content-Type": "application/json" },
      body: toolBody({ watch: tool.watch }) }).catch(() => {});
  }, 1000);
  toolSetControls();
}

function toolDetached(why) {
  if (tool.es) { tool.es.close(); tool.es = null; }
  clearInterval(tool.beatTimer);
  tool.open = false;
  tool.watch = null;
  tool.running = null;
  tool.robot = null;
  toolStatus(why);
  toolSetControls();
}

async function toolOpen() {
  toolStatus("Restarting the board into the tool (about 5 s)…");
  const op = $t("btn-tool-open");
  if (op) op.disabled = true;
  try {
    const port = ($t("hw-flash-port")?.value || "").trim();
    const r = await fetch("/api/tool/open", { method: "POST", headers: { "Content-Type": "application/json" },
      body: JSON.stringify(port ? { port } : {}) });
    const j = await r.json().catch(() => ({}));
    if (!r.ok) { toolStatus(j.detail || `Could not open the tool (${r.status}).`); if (op) op.disabled = false; return false; }
    toolStatus("In the tool. The robot is out of micro-ROS until Back to robot.");
    tool.robot = j.robot || (typeof state !== "undefined" ? state.robot_name : null) || null;
    // Open from the moment the backend holds the session -- not from the stream's first
    // message: a step that opens the tool and sends at once (the Bring-up card) found it
    // "closed" and was told "Enter the tool first" (the 2026-10-14 Sim MCU walk).
    tool.open = true;
    await toolAttach();
    return true;
  } catch (e) {
    toolStatus(`Could not open the tool: ${e.message}`);
    if (op) op.disabled = false;
    return false;
  }
}

// Commands that keep running until stop or their own end (web/backend/tool_session.py RUNNING).
const TOOL_RUNNING = new Set(["sensors", "enc", "motor", "motors", "ramp", "acc", "adc", "i2c"]);
const TOOL_DRIVES = new Set(["motor", "motors", "ramp", "acc"]);

async function toolSend(line) {
  if (!tool.open) { toolStatus("Enter the tool first."); return false; }
  const r = await fetch("/api/tool/cmd", { method: "POST", headers: { "Content-Type": "application/json" },
    body: toolBody({ line }) });
  const j = await r.json().catch(() => ({}));
  if (!r.ok) { toolStatus(j.detail || "Refused."); return false; }
  // Command and response: the board answers every line (ok / fail); the backend waits for
  // it (2 s) and hands it back. A refusal or silence is said here, and the caller is told.
  const a = j.answer || {};
  const word = String(j.sent || line).split(/\s+/)[0];
  tool.lastAnswer = a;
  if (a.ok === false) {
    toolStatus(`fail: ${j.sent} -- ${a.msg}`);
    if (tool.running === word) { tool.running = null; toolSetControls(); }
    return false;
  }
  if (a.ok == null) {
    if (j.answers) {
      // The board answers commands, and not this one: the link is in trouble. A drive it
      // may have started is stopped rather than left to the deadman.
      toolStatus(`no answer: ${j.sent} (nothing from the board in 2 s -- check the link)`);
      if (TOOL_DRIVES.has(word)) toolStop();
      return false;
    }
    toolStatus(`Sent: ${j.sent} (this firmware does not answer commands: flash the MCU for answers)`);
  } else {
    toolStatus(`ok: ${j.sent}${a.msg ? ` (${a.msg})` : ""}`);
  }
  // STOP shows the moment a command that keeps running is accepted -- not when the next
  // state message arrives (the 2026-10-13 walk found it missing at phone width while a
  // wheel turned). The stream's state confirms it, or clears it on a refusal.
  if (TOOL_RUNNING.has(word)) { tool.running = word; toolSetControls(); }
  return true;
}

// A STOP that did not reach the board must not read "Stop sent.": the wheel may still turn.
const TOOL_STOP_FAILED = "STOP FAILED -- press Esc / unplug the motors' power";

async function toolStop() {
  let ok = false, why = "";
  try {
    const r = await fetch("/api/tool/stop", { method: "POST", headers: { "Content-Type": "application/json" },
      body: toolBody() });
    ok = r.ok;
    if (!ok) why = ((await r.json().catch(() => ({}))).detail) || `HTTP ${r.status}`;
  } catch (e) { why = e.message; }
  if (ok) { toolStatus("Stop sent."); return true; }
  toolStatus(`${TOOL_STOP_FAILED} (${why})`, true);
  if (typeof showToast === "function") showToast(`⛔ ${TOOL_STOP_FAILED}`, 10000);
  return false;
}

async function toolClose() {
  toolStatus("Back to the robot: restarting into micro-ROS…");
  try {
    const r = await fetch("/api/tool/close", { method: "POST" });
    const j = await r.json().catch(() => ({}));
    toolDetached("The robot is back in micro-ROS.");
    if (j.needs_env_base && j.handle && typeof runCommand === "function") {
      // An older cockpit left `app=test_*` in the env: every power-on would run that test
      // again. One env-only write (4 KB, no reflash) makes it boot the robot.
      toolStatus("The board's env still named a test: writing app=base (env only)…");
      await runCommand({ handle: j.handle }, { slot: "main", title: "Back to robot: env app=base" });
    }
  } catch (e) {
    toolStatus(`Back to robot failed: ${e.message}`);
  }
}

function toolInit() {
  if (!$t("tool-panel")) return;
  $t("btn-tool-open")?.addEventListener("click", toolOpen);
  $t("btn-tool-close")?.addEventListener("click", toolClose);
  $t("btn-tool-stop")?.addEventListener("click", toolStop);
  for (const b of document.querySelectorAll("#tool-panel [data-cmd]")) {
    b.addEventListener("click", () => toolSend(b.dataset.cmd));
  }
  $t("btn-tool-motors")?.addEventListener("click", () => toolSend(`motors ${$t("tool-motors-pct").value || 25}`));
  $t("btn-tool-acc")?.addEventListener("click", () =>
    toolSend(`acc ${$t("tool-acc-ms").value || 1000} ${$t("tool-acc-pct").value || 50}`));
  const power = $t("tool-power");
  const dir = () => parseInt(document.querySelector('input[name="tool-dir"]:checked')?.value || "1", 10);
  const powerText = () => {
    const p = parseInt(power.value, 10);
    $t("tool-power-val").textContent = `${p} %`;
    power.setAttribute("aria-valuetext", `${p} percent ${dir() > 0 ? "forward" : "reverse"}`);
  };
  power?.addEventListener("input", powerText);
  // On release, not on every input event: a running wheel takes the new power at once.
  power?.addEventListener("change", () => {
    powerText();
    if (tool.running === "motor") toolSend(`set tm_pct ${dir() * parseInt(power.value, 10)}`);
  });
  for (const r of document.querySelectorAll('input[name="tool-dir"]')) r.addEventListener("change", powerText);
  $t("btn-tool-run-wheel")?.addEventListener("click", () => {
    const w = document.querySelector('input[name="tool-wheel"]:checked')?.value || "1";
    const secs = parseInt($t("tool-secs").value || "10", 10);
    toolSend(`motor ${w} ${dir() * parseInt(power.value, 10)} ${isNaN(secs) ? 10 : secs}`);
  });
  $t("btn-tool-ramp-wheel")?.addEventListener("click", () => {
    const w = document.querySelector('input[name="tool-wheel"]:checked')?.value || "1";
    toolSend(`ramp ${w}`);
  });
  $t("tool-form")?.addEventListener("submit", (e) => {
    e.preventDefault();
    const inp = $t("tool-input");
    const line = (inp.value || "").trim();
    if (!line) { toolStop(); return; }          // an empty line stops, as on the board
    toolSend(line).then((ok) => { if (ok) inp.value = ""; });
  });
  // Esc stops: anywhere on the Hardware Tests tab while the tool is open, and on every other
  // tab while a command runs (STOP itself shows on every tab then).
  document.addEventListener("keydown", (e) => {
    if (e.key !== "Escape" || !tool.open) return;
    if ($t("tab-hw-tests")?.classList.contains("active") || tool.running) toolStop();
  });
  // The Sensors tab's I2C card: open the tool, scan once, show the table, leave.
  $t("btn-auto-detect-i2c")?.addEventListener("click", async () => {
    const box = $t("i2c-detect-results");
    if (box) { box.style.display = "block"; box.textContent = "Opening the tool and scanning the I2C bus…"; }
    const wasOpen = tool.open;
    if (!wasOpen && !(await toolOpen())) {
      if (box) box.textContent = $t("tool-status")?.textContent || "Could not open the tool.";
      return;
    }
    tool.scanThenLeave = !wasOpen;
    toolSend("i2c");
  });
  // The ADC studio's button runs the tool's `adc`.
  $t("btn-adc-run-cal")?.addEventListener("click", async () => {
    if (!tool.open && !(await toolOpen())) return;
    toolSend("adc");
  });
  document.querySelector('.tab-btn[data-tab="hw-tests"]')?.addEventListener("click", toolRefreshState);
  toolRefreshState();
  toolSetControls();
}

document.addEventListener("DOMContentLoaded", toolInit);
