// Linorobot2 Cockpit frontend -- ESP32 ADC transfer-function plot & LUT calibration UI.
// Part of the app.js split: a classic script sharing global scope. See app-core.js.

// ==============================================================================
// ADC transfer-function plot
// ==============================================================================
//
// Upstream's studio charts the 4096-entry array the tool prints, because there
// the user has to paste it into a header anyway. Here the table goes to the
// `adclut` partition and is never echoed -- so the board emits one tagged line
// instead, `[ADC_JSON]`, carrying the 257 knots the sweep measured and a sample
// of the inverse it stored. That is the SHAPE, which is all a chart needs, at a
// tenth of the bytes.
let adcCurve = null;        // {knots, lut, span, entries, dac_pin, adc_pin}

const ADC_CHART_COLOURS = { raw: "#f43f5e", ideal: "#38bdf8", corrected: "#10b981" };

function adcMetrics(c) {
  const span = c.span || 16;
  const full = (c.entries || 4096) - 1;
  // How far the measured curve strays from the converter a datasheet promises.
  let maxDev = 0, sxx = 0, syy = 0, sxy = 0, sx = 0, sy = 0, n = 0;
  c.knots.forEach((y, k) => {
    const x = Math.min(k * span, full);
    maxDev = Math.max(maxDev, Math.abs(y - x));
    sx += x; sy += y; sxx += x * x; syy += y * y; sxy += x * y; n++;
  });
  // R² of the straight-line fit: 1.0 is a perfectly linear converter.
  const num = n * sxy - sx * sy;
  const den = Math.sqrt((n * sxx - sx * sx) * (n * syy - sy * sy));
  const r2 = den > 0 ? (num / den) ** 2 : 0;
  let maxDelta = 0;
  (c.lut || []).forEach((v, k) => {
    maxDelta = Math.max(maxDelta, Math.abs(v - Math.min(k * span, full)));
  });
  return { r2, maxDev, maxDeltaCounts: maxDelta, maxDevMv: maxDev * 3300 / full };
}

function renderAdcChart(hoveredX = null) {
  const canvas = document.getElementById("adc-chart-canvas");
  if (!canvas) return;
  const empty = document.getElementById("adc-chart-empty");
  if (empty) empty.style.display = adcCurve ? "none" : "";
  const ctx = canvas.getContext("2d");
  // Redraw at device resolution, or the traces are soft on a HiDPI screen.
  const dpr = window.devicePixelRatio || 1;
  const w = canvas.clientWidth || 600;
  const h = 320;
  canvas.width = w * dpr;
  canvas.height = h * dpr;
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, w, h);

  const pad = { l: 46, r: 12, t: 12, b: 28 };
  const plotW = w - pad.l - pad.r;
  const plotH = h - pad.t - pad.b;
  const full = adcCurve ? (adcCurve.entries || 4096) - 1 : 4095;
  const X = (v) => pad.l + (v / full) * plotW;
  const Y = (v) => pad.t + plotH - (v / full) * plotH;

  // grid + axes
  ctx.strokeStyle = "rgba(148,163,184,0.18)";
  ctx.fillStyle = "#94a3b8";
  ctx.font = "10px var(--font-mono, monospace)";
  ctx.lineWidth = 1;
  for (let i = 0; i <= 4; i++) {
    const v = (full * i) / 4;
    const y = Y(v), x = X(v);
    ctx.beginPath(); ctx.moveTo(pad.l, y); ctx.lineTo(w - pad.r, y); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(x, pad.t); ctx.lineTo(x, h - pad.b); ctx.stroke();
    ctx.fillText(String(Math.round(v)), 6, y + 3);
    ctx.fillText(String(Math.round(v)), x - 12, h - pad.b + 14);
  }
  ctx.fillText("raw ADC counts →", w - pad.r - 96, h - 4);

  if (!adcCurve) return;
  const span = adcCurve.span || 16;

  const trace = (points, colour, width) => {
    ctx.strokeStyle = colour;
    ctx.lineWidth = width;
    ctx.beginPath();
    points.forEach(([x, y], i) => (i ? ctx.lineTo(X(x), Y(y)) : ctx.moveTo(X(x), Y(y))));
    ctx.stroke();
  };

  trace([[0, 0], [full, full]], ADC_CHART_COLOURS.ideal, 1.5);
  trace(adcCurve.knots.map((y, k) => [Math.min(k * span, full), y]),
        ADC_CHART_COLOURS.raw, 2);
  if (adcCurve.lut && adcCurve.lut.length)
    trace(adcCurve.lut.map((y, k) => [Math.min(k * span, full), y]),
          ADC_CHART_COLOURS.corrected, 2);

  // crosshair + readout
  const tip = document.getElementById("adc-chart-tooltip");
  if (hoveredX === null || !tip) { if (tip) tip.style.display = "none"; return; }
  const raw = Math.max(0, Math.min(full, ((hoveredX - pad.l) / plotW) * full));
  const k = Math.max(0, Math.min(adcCurve.knots.length - 1, Math.round(raw / span)));
  ctx.strokeStyle = "rgba(226,232,240,0.45)";
  ctx.setLineDash([4, 4]);
  ctx.beginPath(); ctx.moveTo(X(k * span), pad.t); ctx.lineTo(X(k * span), h - pad.b); ctx.stroke();
  ctx.setLineDash([]);
  const measured = adcCurve.knots[k];
  const corrected = adcCurve.lut ? adcCurve.lut[Math.min(k, adcCurve.lut.length - 1)] : null;
  const volts = (v) => (v * 3.3 / full).toFixed(3) + " V";
  tip.style.display = "block";
  tip.style.left = Math.min(w - 170, X(k * span) + 10) + "px";
  tip.style.top = pad.t + 8 + "px";
  tip.textContent =
    `ideal     ${k * span}  (${volts(k * span)})\n` +
    `measured  ${measured}  (${volts(measured)})\n` +
    (corrected === null ? "" : `corrected ${corrected}\n`) +
    `error     ${measured - k * span} counts`;
}

function showAdcCurve(curve) {
  adcCurve = curve;
  try { localStorage.setItem("cockpit_adc_curve", JSON.stringify(curve)); } catch { /* ignore */ }
  const m = adcMetrics(curve);
  const put = (id, text) => { const e = document.getElementById(id); if (e) e.textContent = text; };
  put("adc-metric-r2", m.r2.toFixed(5));
  put("adc-metric-dev", `${Math.round(m.maxDev)} counts (${Math.round(m.maxDevMv)} mV)`);
  put("adc-metric-delta", `±${Math.round(m.maxDeltaCounts)} counts`);
  put("adc-metric-pins", `DAC GPIO${curve.dac_pin} → ADC GPIO${curve.adc_pin}`);
  renderAdcChart();
}

// Pick the board's curve out of the streamed tool output. Same convention as
// i2c_detect's [I2C_JSON]: one tagged line, so the reader never parses prose.
function adcCurveFromLine(line) {
  const at = line.indexOf("[ADC_JSON]");
  if (at < 0) return false;
  try {
    const curve = JSON.parse(line.slice(at + "[ADC_JSON]".length).trim());
    if (!Array.isArray(curve.knots) || !curve.knots.length) return false;
    showAdcCurve(curve);
    showToast("📊 ADC curve captured — the plot is on the Hardware Tests tab.");
    return true;
  } catch {
    return false;
  }
}

function initAdcChart() {
  const canvas = document.getElementById("adc-chart-canvas");
  if (!canvas) return;
  // The last calibration, so the plot is not blank after a reload.
  try {
    const saved = localStorage.getItem("cockpit_adc_curve");
    if (saved) { adcCurve = JSON.parse(saved); showAdcCurve(adcCurve); }
  } catch { /* ignore */ }
  canvas.addEventListener("mousemove", (e) => {
    const r = canvas.getBoundingClientRect();
    renderAdcChart(e.clientX - r.left);
  });
  canvas.addEventListener("mouseleave", () => renderAdcChart());
  window.addEventListener("resize", () => renderAdcChart());
  renderAdcChart();
}

// Which silicon can build an ADC linearisation table at all.
//
// Sweeping a hardware DAC is the only way to measure the curve, and only the
// classic ESP32 and the ESP32-S2 have one -- the ESP32-S3, the C-series and the
// RP2040/RP2350 do not. adc_lut.h gates the whole facility on exactly this
// condition and adc_calibrate refuses to run anywhere else, printing
// "adc_calibrate is an ESP32-only tool" and then idling forever.
//
// Which is the problem: the panel offered it on every board. Flashing it to a
// Pico spends a flash cycle and leaves the robot running a tool that can only
// apologise -- no /odom, no /scan, until somebody reflashes. The firmware's own
// comment says "the app list is supposed to exclude it there"; nothing did.
function mcuHasDac(env) {
  const e = String(env || "").toLowerCase();
  return e === "esp32" || e === "esp32s2";
}

// Hides the calibration tool wherever it cannot run, and says why rather than
// silently dropping a control the user saw a moment ago.
function applyDacAvailability() {
  const env = document.getElementById("hw-flash-env")?.value
    || document.getElementById("cfg-mcu")?.value || "";
  const usable = mcuHasDac(env);

  const opt = document.querySelector('#hw-flash-target option[value="adc_calibrate"]');
  if (opt) {
    opt.disabled = !usable;
    opt.hidden = !usable;
    // Never leave the selector pointing at something that cannot be flashed.
    const sel = document.getElementById("hw-flash-target");
    if (!usable && sel && sel.value === "adc_calibrate") sel.value = "test_sensors";
  }
  const uploadBtn = document.getElementById("btn-upload-adc");
  if (uploadBtn) {
    uploadBtn.disabled = !usable;
    uploadBtn.style.display = usable ? "" : "none";
  }
  const card = document.getElementById("adc-studio-card");
  if (card) {
    card.style.display = usable ? "" : "none";
  }
  const note = document.getElementById("adc-no-dac-note");
  if (note) {
    note.style.display = usable ? "none" : "";
    note.textContent = `${env || "this board"} has no hardware DAC, so it cannot `
      + "sweep one into an ADC pin to measure the curve. ADC calibration and the "
      + "linearisation table are ESP32 / ESP32-S2 only.";
  }
}

// The wiring chart. Markdown from /api/wiring_table, rendered as a plain
// table here (no Markdown library: two pipe-tables and a few headings are a
// forty-line converter, not a dependency) and offered as a file, because the
// place this gets read is next to the robot with a screwdriver in hand.
let wiringMarkdown = "";

function markdownTablesToHtml(md) {
  const esc = (t) => t.replace(/&/g, "&amp;").replace(/</g, "&lt;").replace(/>/g, "&gt;");
  const inline = (t) => esc(t)
    .replace(/\*\*(.+?)\*\*/g, "<b>$1</b>")
    .replace(/`(.+?)`/g, "<code>$1</code>")
    .replace(/_(.+?)_/g, "<i>$1</i>");
  const out = [];
  const lines = md.split("\n");
  let i = 0;
  while (i < lines.length) {
    const line = lines[i];
    if (/^#\s/.test(line)) { out.push(`<h3 style="margin:8px 0 4px;">${inline(line.slice(2))}</h3>`); i++; continue; }
    if (/^##\s/.test(line)) { out.push(`<h4 style="margin:10px 0 4px;">${inline(line.slice(3))}</h4>`); i++; continue; }
    if (/^\|/.test(line) && i + 1 < lines.length && /^\|[-:| ]+\|$/.test(lines[i + 1])) {
      const cells = (l) => l.replace(/^\||\|$/g, "").split("|").map((c) => c.trim());
      const head = cells(line);
      i += 2;
      const rows = [];
      while (i < lines.length && /^\|/.test(lines[i])) { rows.push(cells(lines[i])); i++; }
      out.push('<table class="pin-table" style="width:100%; font-size:0.82rem;"><thead><tr>'
        + head.map((h) => `<th style="text-align:left;">${inline(h)}</th>`).join("") + "</tr></thead><tbody>"
        + rows.map((r) => "<tr>" + r.map((c) => `<td>${inline(c)}</td>`).join("") + "</tr>").join("")
        + "</tbody></table>");
      continue;
    }
    if (/^- /.test(line)) {
      const items = [];
      while (i < lines.length && /^- /.test(lines[i])) { items.push(`<li>${inline(lines[i].slice(2))}</li>`); i++; }
      out.push(`<ul style="margin:4px 0 8px 18px;">${items.join("")}</ul>`);
      continue;
    }
    if (line.trim()) out.push(`<p class="hint" style="margin:4px 0;">${inline(line)}</p>`);
    i++;
  }
  return out.join("\n");
}

async function showWiringTable() {
  const panel = document.getElementById("wiring-table-panel");
  const body = document.getElementById("wiring-table-body");
  if (!panel || !body) return;
  panel.hidden = false;
  body.innerHTML = '<p class="hint">Generating…</p>';
  try {
    const res = await fetch("/api/wiring_table");
    const data = await res.json();
    if (!res.ok || !data.markdown) throw new Error(data.detail || `HTTP ${res.status}`);
    wiringMarkdown = data.markdown;
    body.innerHTML = markdownTablesToHtml(data.markdown);
  } catch (err) {
    body.innerHTML = `<p class="hint">Could not generate the wiring chart: ${err.message}</p>`;
  }
}

function initWiringTable() {
  document.getElementById("btn-wiring-table")?.addEventListener("click", showWiringTable);
  document.getElementById("btn-wiring-close")?.addEventListener("click", () => {
    const panel = document.getElementById("wiring-table-panel");
    if (panel) panel.hidden = true;
  });
  document.getElementById("btn-wiring-copy")?.addEventListener("click", async () => {
    if (!wiringMarkdown) return;
    try { await navigator.clipboard.writeText(wiringMarkdown); showToast("📋 Wiring chart copied."); }
    catch {
      const ta = document.createElement("textarea");
      ta.value = wiringMarkdown; document.body.appendChild(ta); ta.select();
      try { document.execCommand("copy"); showToast("📋 Wiring chart copied."); }
      catch { showToast("Could not copy — use Download instead."); }
      ta.remove();
    }
  });
  document.getElementById("btn-wiring-download")?.addEventListener("click", () => {
    if (!wiringMarkdown) return;
    const name = (state.robot_name || "robot") + "_wiring.md";
    const blob = new Blob([wiringMarkdown], { type: "text/markdown" });
    const a = document.createElement("a");
    a.href = URL.createObjectURL(blob); a.download = name;
    document.body.appendChild(a); a.click(); a.remove();
    setTimeout(() => URL.revokeObjectURL(a.href), 1000);
  });
}

function initBaseControllerConfigModule() {
  loadHardwareConfig();

  // Inputs change listeners. The wheelbase and the two motor voltages are here
  // because the performance half of the HUD depends on them: the wheelbase sets
  // a mecanum's rotation radius, and a motor rated above the pack never reaches
  // its no-load rpm. Without them the HUD silently described a different robot
  // than the form did.
  ["cfg-wheel-diameter", "cfg-track-width", "cfg-wheelbase", "cfg-max-rpm", "cfg-cpr",
   "cfg-headroom", "cfg-motor-voltage", "cfg-motor-max-voltage",
   // The load and the losses. Mass is the field the HUD is most sensitive to, so
   // a HUD that did not follow it would be answering for a different robot than
   // the one on screen -- which is the whole reason the HUD exists.
   "cfg-sim-mass", "cfg-sim-gear-eff", "cfg-sim-gear-drag", "cfg-sim-sag",
   "cfg-sim-sag-tau", "cfg-sim-drv-drop", "cfg-sim-drv-r",
   "cfg-sim-stall-a", "cfg-sim-ilimit-a", "cfg-pwm-bits"].forEach(id => {
    const el = document.getElementById(id);
    if (el) el.addEventListener("input", updateKinematicsHUD);
  });

  ["cfg-bat-max", "cfg-bat-r1", "cfg-bat-r2"].forEach(id => {
    const el = document.getElementById(id);
    if (el) el.addEventListener("input", updateAdcCalculations);
  });

  const elAuto = document.getElementById("cfg-nav2-auto");
  if (elAuto) elAuto.addEventListener("change", updateKinematicsHUD);

  const elKine = document.getElementById("cfg-kinematics");
  if (elKine) elKine.addEventListener("change", () => {
    updateKinematicsVisibility();
    validateHardwareSafety();
    // The drivetrain decides the rotation radius, so the HUD is wrong until it
    // is told: mecanum turns on (lr + fr)/2, skid steer on lr/2 times the
    // scrub factor, and the same Nav2 request costs them different wheel speeds.
    updateKinematicsHUD();
  });

  const elMcu = document.getElementById("cfg-mcu");
  if (elMcu) elMcu.addEventListener("change", (e) => {
    const elHwEnv = document.getElementById("hw-flash-env");
    if (elHwEnv) elHwEnv.value = e.target.value;
    syncMcuSerialSettings();
    validateHardwareSafety();
    applyDacAvailability();
  });

  const elHwEnvSel = document.getElementById("hw-flash-env");
  if (elHwEnvSel) elHwEnvSel.addEventListener("change", applyDacAvailability);
  applyDacAvailability();

  const elDrv = document.getElementById("cfg-driver-type");
  if (elDrv) elDrv.addEventListener("change", () => {
    updateMotorPinVisibility();
    validateHardwareSafety();
  });

  const elSerialPort = document.getElementById("cfg-serial-port");
  if (elSerialPort) {
    elSerialPort.addEventListener("input", syncMcuSerialSettings);
    elSerialPort.addEventListener("change", syncMcuSerialSettings);
  }

  const elBaud = document.getElementById("cfg-baudrate");
  if (elBaud) {
    elBaud.addEventListener("change", syncMcuSerialSettings);
  }

  const bringupDev = document.getElementById("bringup-agent-device");
  if (bringupDev) {
    bringupDev.addEventListener("input", () => {
      const p = document.getElementById("cfg-serial-port");
      if (p) p.value = bringupDev.value;
      syncMcuSerialSettings();
    });
  }

  const hwPort = document.getElementById("hw-flash-port");
  if (hwPort) {
    hwPort.addEventListener("input", () => {
      const p = document.getElementById("cfg-serial-port");
      if (p) p.value = hwPort.value;
      syncMcuSerialSettings();
    });
  }

  // Pin inputs validation listeners
  document.querySelectorAll(".pin-input").forEach(inp => {
    inp.addEventListener("input", validateHardwareSafety);
  });

  // Buttons
  const btnSmartAlloc = document.getElementById("btn-smart-alloc");
  if (btnSmartAlloc) btnSmartAlloc.addEventListener("click", autoAssignPins);

  ["btn-save-base-config", "btn-save-drive-config", "btn-save-sensors-config", "btn-save-pins-config"].forEach(id => {
    const btn = document.getElementById(id);
    if (btn) btn.addEventListener("click", saveCurrentHardwareConfig);
  });

  const btnRefreshPorts = document.getElementById("btn-refresh-ports-hw");
  if (btnRefreshPorts) btnRefreshPorts.addEventListener("click", refreshHwSerialPorts);

  const btnHwFlash = document.getElementById("btn-hw-flash");
  if (btnHwFlash) btnHwFlash.addEventListener("click", () => executeHardwareAction("upload"));

  const btnHwBuild = document.getElementById("btn-hw-build");
  if (btnHwBuild) btnHwBuild.addEventListener("click", () => executeHardwareAction("build"));

  const btnHwMonitor = document.getElementById("btn-hw-monitor");
  if (btnHwMonitor) btnHwMonitor.addEventListener("click", () => executeHardwareAction("monitor"));

  const btnHwStop = document.getElementById("btn-hw-stop");
  if (btnHwStop) btnHwStop.addEventListener("click", () => killSlot("main"));

  // 1-Click Upload Diagnostics Buttons
  const directBtns = [
    { id: "btn-upload-sensors", firmware: "test_sensors" },
    { id: "btn-upload-motors", firmware: "test_motors" },
    { id: "btn-upload-i2c", firmware: "i2c_detect" },
    { id: "btn-upload-acc", firmware: "test_acc" },
    { id: "btn-upload-adc", firmware: "adc_calibrate" },
    { id: "btn-upload-firmware", firmware: "base" },
  ];

  directBtns.forEach(({ id, firmware }) => {
    const btn = document.getElementById(id);
    if (btn) {
      btn.addEventListener("click", () => {
        const elSel = document.getElementById("hw-flash-target");
        if (elSel) elSel.value = firmware;
        executeHardwareAction("upload", firmware);
      });
    }
  });

  // Auto-Detect I2C button
  const btnAutoI2c = document.getElementById("btn-auto-detect-i2c");
  if (btnAutoI2c) {
    btnAutoI2c.addEventListener("click", async () => {
      const resultsEl = document.getElementById("i2c-detect-results");
      if (resultsEl) {
        resultsEl.style.display = "block";
        resultsEl.innerHTML = "⏳ Switching the board to the <code>i2c_detect</code> app &amp; scanning I2C bus...";
      }
      await executeHardwareAction("upload", "i2c_detect");
    });
  }

  // ADC Calibration button
  const btnAdcCal = document.getElementById("btn-adc-run-cal");
  if (btnAdcCal) {
    btnAdcCal.addEventListener("click", () => {
      executeHardwareAction("upload", "adc_calibrate");
    });
  }

  // The LUT the calibration prints is meant to be pasted into a config; the
  // Copy button beside it had no handler at all, so the only way to take it
  // was to select 4096 entries by hand.
  const btnCopyLut = document.getElementById("btn-copy-lut-code");
  if (btnCopyLut) {
    btnCopyLut.addEventListener("click", async () => {
      const box = document.getElementById("adc-lut-code-display");
      const text = (box ? box.innerText : "").trim();
      if (!text) { showToast("No LUT to copy yet — run the calibration first."); return; }
      try {
        await navigator.clipboard.writeText(text);
        showToast("📋 ADC LUT copied to the clipboard.");
      } catch {
        // Clipboard access needs a secure context; over plain http on a LAN
        // address it is refused, so fall back rather than fail silently.
        const ta = document.createElement("textarea");
        ta.value = text;
        document.body.appendChild(ta);
        ta.select();
        try { document.execCommand("copy"); showToast("📋 ADC LUT copied."); }
        catch { showToast("Could not copy — select the block and copy manually."); }
        ta.remove();
      }
    });
  }

  // "Run Magnetometer Calibration" was pure markup: no handler, and no backend
  // endpoint either. The firmware carries the application (bno085_cal), so the
  // button now does what every other tool button does -- flash it and stream
  // its output, which is where the hard-iron offsets are printed.
  const btnMagCalHw = document.getElementById("btn-mag-cal-hw");
  if (btnMagCalHw) {
    btnMagCalHw.addEventListener("click", () => {
      const result = document.getElementById("mag-cal-hw-result");
      if (result) {
        result.textContent = "Flashing bno085_cal and streaming its output — "
          + "spin the robot slowly in place; the offsets appear in the terminal below.";
      }
      executeHardwareAction("upload", "bno085_cal");
    });
  }

  // Re-scan MCU button in Base & MCU tab
  const btnRedetectMcu = document.getElementById("btn-redetect-mcu");
  if (btnRedetectMcu) {
    btnRedetectMcu.addEventListener("click", async () => {
      btnRedetectMcu.disabled = true;
      btnRedetectMcu.textContent = "🔄 Scanning...";
      await refreshStatus();
      // The bus that matters is the robot computer's as soon as one is named,
      // so a re-scan has to ask THAT machine, not just re-read this one's /dev.
      btnRedetectMcu.disabled = false;
      btnRedetectMcu.textContent = "🔄 Re-scan Hardware";
      showToast("🔍 USB hardware ports re-scanned!");
    });
  }

  // Workflow Next / Prev step navigation buttons
  document.querySelectorAll(".btn-workflow-next").forEach((btn) => {
    btn.addEventListener("click", () => {
      const nextTab = btn.dataset.next;
      if (nextTab) {
        const tabBtn = document.querySelector(`.tab-btn[data-tab="${nextTab}"]`);
        if (tabBtn) {
          tabBtn.click();
          window.scrollTo({ top: 0, behavior: "smooth" });
        }
      }
    });
  });

  document.querySelectorAll(".btn-workflow-prev").forEach((btn) => {
    btn.addEventListener("click", () => {
      const prevTab = btn.dataset.prev;
      if (prevTab) {
        const tabBtn = document.querySelector(`.tab-btn[data-tab="${prevTab}"]`);
        if (tabBtn) {
          tabBtn.click();
          window.scrollTo({ top: 0, behavior: "smooth" });
        }
      }
    });
  });

  initReferenceDesigns();
}

