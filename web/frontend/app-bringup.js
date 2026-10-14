// The Real-robot bring-up card (Hardware Tests). Each step drives the board through the tool
// session (app-tool.js: every command is typed into its terminal, STOP and Esc stop it) or runs
// scripts/robot_bringup.py on the robot computer (lidar, speed, acc), and records its result in
// the same file the command line writes (web/backend/routes_bringup.py). The arithmetic --
// invert flags, counts per rev, the battery divider -- is the backend's, never this page's.
"use strict";

const bu = {
  list: null,
  phase: null,          // "rest" | "moving" | "battery" | "spin" | "rpm" | "turns"
  rest: [], moving: [], plot: [], batt: [],
  spin: null,           // {wheel, rpms: [], t0}
  mot: {},              // wheel -> {rpm, answer, fix, changes, result}
  rpm: {},              // wheel -> {samples: [[t, rpm]], enc, tacho}
  rpmChanges: null, batChanges: null,
  turn: null,           // {wheel, first, last}
  timer: null,
};

const $b = (id) => document.getElementById(id);

async function buJson(url, body) {
  const r = await fetch(url, body === undefined ? {} : {
    method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify(body) });
  const j = await r.json().catch(() => ({}));
  if (!r.ok) throw new Error(j.detail || `HTTP ${r.status}`);
  return j;
}

const BU_PILL = { pass: ["pill-ok", "passed"], fail: ["pill-off", "failed"], skip: ["pill-unknown", "skipped"],
  stale: ["pill-off", "config changed"], todo: ["pill-unknown", "to do"], "not needed": ["pill-unknown", "not needed"] };

async function buRefresh() {
  try { bu.list = await buJson("/api/bringup/checklist"); } catch { return; }
  const c = bu.list, p = c.parts;
  const gate = $b("bu-gate");
  if (gate) {
    gate.className = "pill " + (!c.real_robot ? "pill-unknown" : (c.gate_ok ? "pill-ok" : "pill-off"));
    gate.textContent = !c.real_robot ? "simulated robot" : (c.gate_ok ? "1-Click: ready" : `1-Click held: ${c.missing.length} step(s)`);
    gate.title = c.missing.join("\n");
  }
  const has = ["imu", "mag", "battery", "lidar"].filter((k) => p[k]);
  if ($b("bu-robot")) $b("bu-robot").textContent =
    `Robot ${c.robot} (${c.config}): ${p.wheels} wheels ${p.wheels_real ? "real" : "simulated"}, `
    + `${has.length ? has.join(", ") : "no sensors"}${p.battery_chip ? ` (${p.battery_chip})` : ""}, ${p.wifi ? "Wi-Fi" : "serial"}.`;
  if ($b("bu-simnote")) $b("bu-simnote").hidden = c.real_robot;
  const ol = $b("bu-steps");
  if (ol) {
    ol.innerHTML = "";
    for (const s of c.stages) {
      const li = document.createElement("li");
      const [cls, word] = BU_PILL[s.state] || ["pill-unknown", s.state];
      li.innerHTML = `<a href="#bu-step-${s.stage}" data-open="${s.stage}"></a> <span class="pill ${cls}"></span>`
        + (s.gate && s.required ? ' <span class="tool-small">needed by the 1-Click</span>' : "")
        + (s.summary ? `<div class="tool-small"></div>` : "");
      li.querySelector("a").textContent = s.title;
      li.querySelector(".pill").textContent = word + (s.at ? ` ${s.at.slice(0, 16).replace("T", " ")} (${s.by})` : "");
      if (s.summary) li.querySelector("div").textContent = s.summary;
      ol.append(li);
      const pill = document.querySelector(`.bu-pill[data-for="${s.stage}"]`);
      if (pill) { pill.className = `bu-pill pill ${cls}`; pill.textContent = word; }
      const det = $b(`bu-step-${s.stage}`);
      if (det) det.hidden = !s.required && s.state === "not needed";
    }
  }
  if ($b("bu-rpm-cpr")) $b("bu-rpm-cpr").textContent = c.current.counts_per_rev ?? "—";
  if ($b("bu-rpm-max")) $b("bu-rpm-max").textContent = c.current.max_rpm ?? "—";
  buBuildWheelRows();
  buOneClickState();
}

// ---------------------------------------------------------------- the tool
async function buTool() {
  if (tool.open) return true;
  return await toolOpen();
}

function buCountdown(secs, label, done) {
  clearInterval(bu.timer);
  const end = Date.now() + secs * 1000;
  const el = $b("bu-sens-timer");
  bu.timer = setInterval(() => {
    const left = Math.max(0, Math.ceil((end - Date.now()) / 1000));
    if (el) el.textContent = `${label}: ${left} s`;
    if (left <= 0) { clearInterval(bu.timer); if (el) el.textContent = ""; done(); }
  }, 200);
}

function buOnJson(j) {
  if (j.cmd === "sensors") {
    if (bu.phase === "rest") bu.rest.push(j);
    else if (bu.phase === "moving") bu.moving.push(j);
    else if (bu.phase === "battery" && typeof j.bat === "number") bu.batt.push(j.bat);
    buSensorView(j);
  } else if (Array.isArray(j.rpm)) {
    const now = performance.now() / 1000;
    if (bu.spin && bu.phase === "spin" && now - bu.spin.t0 > 1.2) bu.spin.rpms.push(Number(j.rpm[bu.spin.wheel - 1]));
    if (bu.phase === "rpm") {
      for (const [w, r] of Object.entries(bu.rpm)) {
        if (!r.running) continue;
        if (now - r.t0 > 1.5) r.samples.push([now, Math.abs(Number(j.rpm[w - 1]))]);
        r.samples = r.samples.filter(([t]) => now - t <= 2.5);
        const med = buMedian(r.samples.map(([, v]) => v));
        if (med != null) { r.enc = med; const e = $b(`bu-rpm-enc-${w}`); if (e) e.textContent = med.toFixed(1); }
      }
    }
  } else if (j.cmd === "enc" && Array.isArray(j.count) && bu.turn) {
    const v = Number(j.count[bu.turn.wheel - 1]);
    if (bu.turn.first == null) bu.turn.first = v;
    bu.turn.last = v;
    if ($b("bu-turn-count")) $b("bu-turn-count").textContent = `${v - bu.turn.first >= 0 ? "+" : ""}${v - bu.turn.first}`;
  }
}

function buMedian(a) {
  if (!a.length) return null;
  const s = [...a].sort((x, y) => x - y);
  const m = Math.floor(s.length / 2);
  return s.length % 2 ? s[m] : (s[m - 1] + s[m]) / 2;
}

// ---------------------------------------------------------------- 1. sensors
function buSensorView(j) {
  const all = bu.rest.concat(bu.moving);
  const rows = [["acc", "acc", 2], ["gyr", "gyr", 2], ["mag", "mag_ut", 1], ["rpy", "rpy", 1]];
  for (const [id, key, dp] of rows) {
    const v = j[key];
    ["x", "y", "z"].forEach((ax, i) => { const e = $b(`bu-${id}-${ax}`); if (e) e.textContent = Array.isArray(v) ? Number(v[i]).toFixed(dp) : "—"; });
    const seen = all.map((s) => s[key]).filter(Array.isArray);
    const sp = $b(`bu-${id}-s`);
    if (sp) sp.textContent = seen.length ? [0, 1, 2].map((i) => {
      const xs = seen.map((r) => r[i]);
      return (Math.max(...xs) - Math.min(...xs)).toFixed(dp);
    }).join(" / ") : "—";
  }
  if ($b("bu-bat-live")) $b("bu-bat-live").textContent = typeof j.bat === "number" ? `${j.bat.toFixed(2)} V` : "none";
  if ($b("bu-range-live")) $b("bu-range-live").textContent = typeof j.range === "number" ? `${j.range.toFixed(2)} m` : "none";
  const t = performance.now() / 1000;
  const norm = (v) => Array.isArray(v) ? Math.hypot(...v) : null;
  bu.plot.push([t, norm(j.gyr), norm(j.mag_ut)]);
  bu.plot = bu.plot.filter(([tt]) => t - tt <= 10);
  buDrawPlot(t);
}

function buDrawPlot(now) {
  const cv = $b("bu-sens-plot");
  if (!cv) return;
  const g = cv.getContext("2d"), W = cv.width, H = cv.height;
  g.clearRect(0, 0, W, H);
  g.strokeStyle = "#33415580"; g.beginPath(); g.moveTo(0, H - 1); g.lineTo(W, H - 1); g.stroke();
  const series = [[1, "#38bdf8", 4], [2, "#f59e0b", 100]];     // full scale: |gyro| 4 rad/s, |mag| 100 uT
  for (const [k, col, div] of series) {
    g.strokeStyle = col; g.lineWidth = 2; g.beginPath();
    let first = true;
    for (const row of bu.plot) {
      if (row[k] == null) continue;
      const x = W - (now - row[0]) / 10 * W;
      const v = row[k] / div;
      const y = H - 4 - Math.min(1, v) * (H - 8);
      if (first) { g.moveTo(x, y); first = false; } else g.lineTo(x, y);
    }
    g.stroke();
  }
}

async function buSensorsStill() {
  if (!(await buTool())) return;
  bu.rest = []; bu.moving = []; bu.plot = [];
  $b("bu-sens-verdict").innerHTML = "";
  bu.phase = "rest";
  await toolSend("sensors 10");
  $b("bu-sens-move").disabled = true;
  buCountdown(5, "Hold still", () => {
    bu.phase = null;
    $b("bu-sens-move").disabled = false;
    $b("bu-sens-timer").textContent = bu.rest.length ? "Now press ② and turn the robot." : "No readings came: is a sensor fitted?";
  });
}

async function buSensorsMove() {
  if (!tool.open) { $b("bu-sens-timer").textContent = "The tool closed: start again with ①."; return; }
  bu.phase = "moving";
  $b("bu-sens-move").disabled = true;
  buCountdown(20, "Turn it, tilt it, shake it", async () => {
    bu.phase = null;
    await toolSend("stop");
    try {
      const v = await buJson("/api/bringup/sensors", { rest: bu.rest, moving: bu.moving });
      const ul = $b("bu-sens-verdict");
      ul.innerHTML = "";
      for (const ln of v.lines) { const li = document.createElement("li"); li.textContent = ln; ul.append(li); }
      const li = document.createElement("li");
      li.innerHTML = `<b>${v.ok ? "PASS" : "FAIL"}</b> (${bu.rest.length + bu.moving.length} readings)`;
      ul.prepend(li);
    } catch (e) { $b("bu-sens-timer").textContent = e.message; }
    buRefresh();
  });
}

// ---------------------------------------------------------------- 2. battery
async function buBatteryRead() {
  if (!(await buTool())) return;
  bu.batt = []; bu.phase = "battery";
  $b("bu-bat-board").textContent = "reading…";
  await toolSend("sensors 5");
  setTimeout(async () => {
    bu.phase = null;
    await toolSend("stop");
    const v = buMedian(bu.batt);
    bu.batBoard = v;
    $b("bu-bat-board").textContent = v == null ? "no voltage reported" : `${v.toFixed(2)} V`;
    $b("bu-bat-check").disabled = v == null;
  }, 5000);
}

async function buBatteryCheck() {
  const meas = parseFloat($b("bu-bat-meter").value);
  const res = $b("bu-bat-result"), apply = $b("bu-bat-apply");
  apply.hidden = true;
  if (!(meas > 0) || bu.batBoard == null) { res.textContent = "Read the board, then type the multimeter's reading."; return; }
  const p = bu.list.parts, rep = bu.batBoard, diff = rep - meas;
  const line = `Board ${rep.toFixed(2)} V, multimeter ${meas.toFixed(2)} V (${diff >= 0 ? "+" : ""}${diff.toFixed(2)} V).`;
  if (p.battery_adc) {
    if (Math.abs(diff) <= 0.02 * meas) {
      res.textContent = `${line} Within 2 %: passed.`;
      await buJson("/api/bringup/mark", { stage: "battery", status: "pass", summary: line, values: { reported: rep, measured: meas } });
      return buRefresh();
    }
    try {
      const t = await buJson("/api/bringup/calc", { kind: "battery", reported: rep, measured: meas });
      bu.batChanges = t.changes;
      res.textContent = `${line} The divider: R1 ${bu.list.current.battery.r1} -> ${t.r1} (R2 ${t.r2}), ratio x${t.scale}. Save it, then read the board again.`;
      apply.hidden = false;
    } catch (e) {
      res.textContent = `${line} ${e.message}`;
      await buJson("/api/bringup/mark", { stage: "battery", status: "fail", summary: e.message });
      buRefresh();
    }
  } else {
    const ok = Math.abs(diff) <= 1.0;
    res.textContent = `${line} ${ok ? "Passed" : "More than 1 V apart: check the wiring"}${ok && Math.abs(diff) > 0.2 ? " (a monitor behind an input diode reads ~0.6 V under the pack)" : ""}.`;
    await buJson("/api/bringup/mark", { stage: "battery", status: ok ? "pass" : "fail", summary: line, values: { reported: rep, measured: meas } });
    buRefresh();
  }
}

// ---------------------------------------------------------------- writing a calibration
async function buApply(changes, title) {
  const j = await buJson("/api/bringup/apply", { changes, port: ($b("hw-flash-port")?.value || "").trim() });
  const wrote = Object.entries(j.written || {}).map(([k, v]) => `${k.split(".").slice(-2).join(".")} ${v.from} -> ${v.to}`).join(", ");
  if (j.tool_closed && typeof toolDetached === "function") toolDetached("The tool closed for the env write.");
  return await new Promise((resolve) => {
    runCommand({ handle: j.handle }, {
      slot: "main", title: `${title}: env write`,
      onDone: (code) => { buRefresh(); resolve({ ok: code === 0, wrote }); },
    });
  });
}

// ---------------------------------------------------------------- 3. motors
function buBuildWheelRows() {
  const n = bu.list?.parts?.wheels || 2;
  const inv = bu.list?.current?.invert || {};
  const mt = $b("bu-mot-rows"), rt = $b("bu-rpm-rows");
  if (mt && mt.dataset.n !== String(n)) {
    mt.dataset.n = String(n);
    mt.innerHTML = "";
    for (let w = 1; w <= n; w++) {
      const tr = document.createElement("tr");
      tr.innerHTML = `<th>${w}</th><td><button type="button" class="btn btn-secondary btn-sm tool-drive" data-spin="${w}">Spin forward</button></td>`
        + `<td id="bu-mot-enc-${w}">—</td><td class="u-flex u-gap-6">`
        + `<button type="button" class="btn btn-sm" data-ans="${w}:f" disabled>Forward</button>`
        + `<button type="button" class="btn btn-sm" data-ans="${w}:b" disabled>Backward</button>`
        + `<button type="button" class="btn btn-sm" data-ans="${w}:n" disabled>Did not turn</button></td>`
        + `<td id="bu-mot-res-${w}">—</td><td id="bu-mot-inv-${w}">—</td>`;
      mt.append(tr);
    }
  }
  for (let w = 1; w <= n; w++) {
    const e = $b(`bu-mot-inv-${w}`);
    if (e && inv[w]) e.textContent = `${inv[w].motor ? "yes" : "no"} / ${inv[w].encoder ? "yes" : "no"}`;
  }
  if (rt && rt.dataset.n !== String(n)) {
    rt.dataset.n = String(n);
    rt.innerHTML = "";
    for (let w = 1; w <= n; w++) {
      const tr = document.createElement("tr");
      tr.innerHTML = `<th>${w}</th><td><button type="button" class="btn btn-secondary btn-sm tool-drive" data-rpm="${w}">Run</button></td>`
        + `<td id="bu-rpm-enc-${w}">—</td><td><label class="sr-only" for="bu-rpm-tacho-${w}">Tachometer RPM, wheel ${w}</label>`
        + `<input type="number" id="bu-rpm-tacho-${w}" class="form-input tool-num" min="5" max="3000" step="0.1" inputmode="decimal"></td>`
        + `<td id="bu-rpm-cpr-${w}">—</td>`;
      rt.append(tr);
    }
  }
}

async function buSpin(w) {
  if (!(await buTool())) return;
  const pct = Math.max(10, Math.min(100, parseInt($b("bu-mot-pct").value, 10) || 40));
  const secs = Math.max(2, Math.min(10, parseInt($b("bu-mot-secs").value, 10) || 3));
  bu.spin = { wheel: w, rpms: [], t0: performance.now() / 1000 };
  bu.phase = "spin";
  $b(`bu-mot-enc-${w}`).textContent = "turning…";
  if (!(await toolSend(`motor ${w} ${pct} ${secs}`))) { bu.phase = null; return; }
  setTimeout(async () => {
    bu.phase = null;
    await toolSend("stop");
    const rpm = buMedian(bu.spin.rpms) ?? 0;
    bu.mot[w] = { rpm };
    $b(`bu-mot-enc-${w}`).textContent = `${rpm >= 0 ? "+" : ""}${rpm.toFixed(1)} RPM`;
    for (const b of document.querySelectorAll(`[data-ans^="${w}:"]`)) b.disabled = false;
  }, (secs + 0.6) * 1000);
}

async function buAnswer(w, a) {
  const m = bu.mot[w] || { rpm: 0 };
  for (const b of document.querySelectorAll(`[data-ans^="${w}:"]`)) b.disabled = true;
  const res = $b(`bu-mot-res-${w}`);
  if (a === "n") {
    Object.assign(m, { answer: a, result: "did not turn: check its supply, driver pins and battery", ok: false, changes: {} });
  } else {
    try {
      const r = await buJson("/api/bringup/calc", { kind: "invert", wheel: w, turned_forward: a === "f", rpm: m.rpm });
      const what = [r.fix.flip_motor ? "invert the motor" : "", r.fix.flip_encoder ? "invert the encoder" : ""].filter(Boolean).join(" and ");
      Object.assign(m, { answer: a, changes: r.changes, ok: !what, result: what || `right (${m.rpm.toFixed(0)} RPM)` });
    } catch (e) {
      Object.assign(m, { answer: a, changes: {}, ok: false, result: e.message });
    }
  }
  bu.mot[w] = m;
  res.textContent = m.result;
  await buMotorsVerdict();
}

async function buMotorsVerdict() {
  const n = bu.list.parts.wheels;
  const done = Object.entries(bu.mot).filter(([, m]) => m.answer);
  const changes = Object.assign({}, ...done.map(([, m]) => m.changes || {}));
  $b("bu-mot-apply").hidden = !Object.keys(changes).length;
  bu.motChanges = changes;
  if (done.length < n) { $b("bu-mot-msg").textContent = `${done.length} of ${n} wheels checked.`; return; }
  const ok = done.every(([, m]) => m.ok);
  const summary = done.map(([w, m]) => `wheel ${w}: ${m.result}`).join("; ");
  $b("bu-mot-msg").textContent = ok ? "Every wheel is right: passed." : (Object.keys(changes).length
    ? "Save the fixes, then spin those wheels again." : "Not passed: see the wheels above.");
  await buJson("/api/bringup/mark", { stage: "motors", status: ok ? "pass" : "fail", summary,
    values: Object.fromEntries(done.map(([w, m]) => [w, m.result])) });
  buRefresh();
}

async function buMotorsApply() {
  $b("bu-mot-apply").hidden = true;
  $b("bu-mot-msg").textContent = "Writing the config and the board's env…";
  const r = await buApply(bu.motChanges, "Motor directions");
  // the wheels that changed are checked again
  for (const [w, m] of Object.entries(bu.mot)) {
    if (m.changes && Object.keys(m.changes).length) {
      delete bu.mot[w];
      $b(`bu-mot-res-${w}`).textContent = "fixed: spin it again";
      $b(`bu-mot-enc-${w}`).textContent = "—";
    }
  }
  $b("bu-mot-msg").textContent = r.ok ? `Written (${r.wrote}). Spin the fixed wheels again to confirm.` : "The env write failed: see the console.";
}

// ---------------------------------------------------------------- 4. rpm
async function buRpmRun(w) {
  const r = bu.rpm[w] || (bu.rpm[w] = { samples: [] });
  const btn = document.querySelector(`[data-rpm="${w}"]`);
  if (r.running) {
    r.running = false;
    await toolSend("stop");
    bu.phase = Object.values(bu.rpm).some((x) => x.running) ? "rpm" : null;
    if (btn) btn.textContent = "Run";
    return;
  }
  if (!(await buTool())) return;
  for (const [ow, o] of Object.entries(bu.rpm)) {
    if (o.running && Number(ow) !== w) { o.running = false; document.querySelector(`[data-rpm="${ow}"]`).textContent = "Run"; }
  }
  const pct = Math.max(20, Math.min(100, parseInt($b("bu-rpm-pct").value, 10) || 100));
  Object.assign(r, { running: true, samples: [], t0: performance.now() / 1000, pct });
  bu.phase = "rpm";
  if (!(await toolSend(`motor ${w} ${pct} 0`))) { r.running = false; return; }
  if (btn) btn.textContent = "Stop";
  $b("bu-rpm-msg").textContent = `Wheel ${w} at ${pct} %: read the tachometer, type it, press Stop.`;
}

async function buRpmCalc() {
  const n = bu.list.parts.wheels;
  const rows = [];
  for (let w = 1; w <= n; w++) {
    const r = bu.rpm[w];
    const tacho = parseFloat($b(`bu-rpm-tacho-${w}`).value);
    if (!r || !(r.enc > 0) || !(tacho > 0)) continue;
    try {
      const c = await buJson("/api/bringup/calc", { kind: "cpr_tacho", encoder_rpm: r.enc, tacho_rpm: tacho });
      $b(`bu-rpm-cpr-${w}`).textContent = String(c.cpr);
      rows.push({ w, enc: r.enc, tacho, cpr: c.cpr, from: c.from, pct: r.pct });
    } catch (e) { $b(`bu-rpm-cpr-${w}`).textContent = e.message; }
  }
  const msg = $b("bu-rpm-msg"), apply = $b("bu-rpm-apply");
  apply.hidden = true;
  if (!rows.length) { msg.textContent = "Run a wheel and type its tachometer reading first."; return; }
  const cpr = Math.round(buMedian(rows.map((r) => r.cpr)));
  const from = rows[0].from, off = Math.abs(cpr - from) / from;
  const changes = {};
  if (off > 0.02) changes["kinematics.counts_per_rev"] = cpr;
  const full = rows.filter((r) => r.pct === 100);
  let maxNote = "";
  if (full.length === n) {
    const slow = Math.min(...full.map((r) => r.tacho));
    const cur = bu.list.current.max_rpm;
    if (cur == null || Math.abs(slow - cur) / slow > 0.05) {
      Object.assign(changes, (await buJson("/api/bringup/calc", { kind: "max_rpm", rpm: slow })).changes);
      maxNote = `; max_rpm ${cur ?? "unset"} -> ${Math.round(slow)} (the slowest wheel at full power)`;
    }
  }
  const summary = rows.map((r) => `wheel ${r.w}: encoder ${r.enc.toFixed(0)} / tacho ${r.tacho.toFixed(0)} RPM`).join("; ");
  bu.rpmChanges = changes;
  if (!Object.keys(changes).length && rows.length === n) {
    msg.textContent = `counts_per_rev ${from} agrees within 2 %: passed.`;
    await buJson("/api/bringup/mark", { stage: "rpm", status: "pass", summary: `${summary}; counts_per_rev ${from} agrees`,
      values: Object.fromEntries(rows.map((r) => [r.w, { encoder: r.enc, tacho: r.tacho }])) });
  } else if (Object.keys(changes).length) {
    msg.textContent = `counts_per_rev ${from} -> ${cpr} (${(off * 100).toFixed(1)} % off)${maxNote}. Save it, then run the wheels again to confirm.`;
    apply.hidden = false;
    if (rows.length === n) await buJson("/api/bringup/mark", { stage: "rpm", status: "fail", summary: `${summary}; counts_per_rev ${from} -> ${cpr}` });
  } else {
    msg.textContent = `${rows.length} of ${n} wheels measured; counts_per_rev agrees so far.`;
  }
  buRefresh();
}

async function buRpmApply(changes) {
  $b("bu-rpm-apply").hidden = true;
  $b("bu-rpm-msg").textContent = "Writing the config and the board's env…";
  const r = await buApply(changes || bu.rpmChanges, "Wheel speed");
  bu.rpm = {};
  for (const el of document.querySelectorAll('[id^="bu-rpm-enc-"], [id^="bu-rpm-cpr-"]')) el.textContent = "—";
  $b("bu-rpm-msg").textContent = r.ok ? `Written (${r.wrote}). Run the wheels again to confirm.` : "The env write failed: see the console.";
}

async function buTurnStart() {
  if (!(await buTool())) return;
  bu.turn = { wheel: parseInt($b("bu-turn-wheel").value, 10), first: null, last: null };
  $b("bu-turn-count").textContent = "0";
  if (!(await toolSend("enc"))) { bu.turn = null; return; }
  $b("bu-turn-done").disabled = false;
  $b("bu-turn-msg").textContent = "Turn the wheel forward by hand now, then press Done.";
}

async function buTurnDone() {
  $b("bu-turn-done").disabled = true;
  await toolSend("stop");
  const t = bu.turn;
  bu.turn = null;
  const msg = $b("bu-turn-msg");
  if (!t || t.first == null) { msg.textContent = "The encoder reported nothing."; return; }
  try {
    const c = await buJson("/api/bringup/calc", { kind: "cpr_turns", counts: t.last - t.first, turns: parseFloat($b("bu-turn-n").value) || 10 });
    msg.textContent = `${t.last - t.first} counts: ${c.cpr} counts per rev (config ${c.from}, ${c.off_pct} % off)`
      + (c.counted_down ? ". It counted DOWN on a forward turn: the encoder needs inverting (step 3)." : ".");
    if (c.off_pct > 2) {
      bu.rpmChanges = c.changes;
      $b("bu-rpm-apply").hidden = false;
      $b("bu-rpm-msg").textContent = `From the hand count: counts_per_rev ${c.from} -> ${c.cpr}.`;
    }
  } catch (e) { msg.textContent = e.message; }
}

// ---------------------------------------------------------------- 5-7. the robot computer's steps
async function buRun(stage, msgId, extra = {}) {
  const msg = $b(msgId);
  try {
    const j = await buJson("/api/bringup/run", { stage, ...extra });
    msg.textContent = "Running: see the console.";
    runCommand({ handle: j.handle }, {
      slot: "main", title: `Bring-up: ${stage}`,
      onDone: (code) => { msg.textContent = code === 0 ? "Passed." : (code === 2 ? "Refused: nothing moved (see the console)." : `Not passed (exit ${code}): see the console.`); buRefresh(); },
    });
  } catch (e) { msg.textContent = e.message; }
}

// ---------------------------------------------------------------- the 1-Click card
function buOneClickState() {
  const grp = $b("grp-cockpit-real");
  if (!grp || !bu.list) return;
  grp.hidden = !bu.list.real_robot;
  const st = $b("cockpit-bringup-state");
  if (st) st.textContent = bu.list.gate_ok ? "Bring-up: every bench step has passed."
    : `Bring-up not complete -- the 1-Click will refuse: ${bu.list.missing.join("; ")}`;
}

async function buOneClickGate() {
  // runOneClick (app-workflow.js) asks before it starts: null = go, else the reason it stops.
  await buRefresh();
  if (!bu.list || !bu.list.real_robot || bu.list.gate_ok) return null;
  if ($b("cockpit-skip-bringup")?.checked) return null;
  return `This real robot's bring-up is not complete (Hardware Tests > Real-robot bring-up): ${bu.list.missing.join("; ")}.`;
}

function buInit() {
  if (!$b("bu-card")) return;
  tool.jsonHooks.push(buOnJson);
  $b("bu-sens-still")?.addEventListener("click", buSensorsStill);
  $b("bu-sens-move")?.addEventListener("click", buSensorsMove);
  $b("bu-bat-read")?.addEventListener("click", buBatteryRead);
  $b("bu-bat-check")?.addEventListener("click", buBatteryCheck);
  $b("bu-bat-apply")?.addEventListener("click", async () => {
    $b("bu-bat-apply").hidden = true;
    const r = await buApply(bu.batChanges, "Battery divider");
    $b("bu-bat-result").textContent = r.ok ? `Written (${r.wrote}). Read the board again and compare.` : "The env write failed: see the console.";
    $b("bu-bat-check").disabled = true;
  });
  $b("bu-mot-rows")?.addEventListener("click", (e) => {
    const s = e.target.closest("[data-spin]"); if (s) return buSpin(parseInt(s.dataset.spin, 10));
    const a = e.target.closest("[data-ans]");
    if (a) { const [w, k] = a.dataset.ans.split(":"); buAnswer(parseInt(w, 10), k); }
  });
  $b("bu-mot-apply")?.addEventListener("click", buMotorsApply);
  $b("bu-rpm-rows")?.addEventListener("click", (e) => {
    const r = e.target.closest("[data-rpm]"); if (r) buRpmRun(parseInt(r.dataset.rpm, 10));
  });
  $b("bu-rpm-calc")?.addEventListener("click", buRpmCalc);
  $b("bu-rpm-apply")?.addEventListener("click", () => buRpmApply());
  $b("bu-turn-start")?.addEventListener("click", buTurnStart);
  $b("bu-turn-done")?.addEventListener("click", buTurnDone);
  $b("bu-lidar-run")?.addEventListener("click", () => buRun("lidar", "bu-lidar-msg"));
  $b("bu-speed-run")?.addEventListener("click", () => buRun("speed", "bu-speed-msg"));
  $b("bu-acc-run")?.addEventListener("click", () => buRun("acc", "bu-acc-msg", { write: !!$b("bu-acc-write")?.checked }));
  $b("bu-steps")?.addEventListener("click", (e) => {
    const a = e.target.closest("[data-open]");
    if (a) { e.preventDefault(); const d = $b(`bu-step-${a.dataset.open}`); if (d) { d.open = true; d.scrollIntoView({ behavior: "smooth" }); } }
  });
  for (const a of document.querySelectorAll("[data-goto-tab]")) {
    a.addEventListener("click", (e) => {
      e.preventDefault();
      document.querySelector(`.tab-btn[data-tab="${a.dataset.gotoTab}"]`)?.click();
    });
  }
  document.querySelector('.tab-btn[data-tab="hw-tests"]')?.addEventListener("click", buRefresh);
  document.querySelector('.tab-btn[data-tab="cockpit"]')?.addEventListener("click", buRefresh);
  buRefresh();
}

document.addEventListener("DOMContentLoaded", buInit);
