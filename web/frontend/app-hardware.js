// Linorobot2 Cockpit frontend -- base controller configuration & hardware test studio, body/sensor URDF placement.
// Part of the app.js split: a classic script sharing global scope. See app-core.js.

// ==============================================================================
// Base Controller Configuration & Hardware Test Studio (ported from config engine)
// ==============================================================================

const ESP32_FLASH_PINS = [6, 7, 8, 9, 10, 11];
const ESP32_INPUT_ONLY_PINS = [34, 35, 36, 39];
const ESP32_STRAPPING_PINS = [0, 2, 12, 14, 15];
const ESP32S3_STRAPPING_PINS = [0, 3, 45, 46];

let activeHwConfig = null;

// What the pin catalogue (scripts/pin_catalog.py) says about the saved
// config, shown under the Pin Matrix header. An error is a config the board
// cannot run; a warning is for a human. Empty means every pin checked out.
function renderPinFindings(findings, savedOk) {
  const box = document.getElementById("pin-findings");
  if (!box) return;
  const list = Array.isArray(findings) ? findings : [];
  if (!list.length && savedOk === undefined) { box.hidden = true; box.innerHTML = ""; return; }
  const rows = list.map((f) =>
    `<div class="pf ${escapeHtml(f.level)}"><span class="lvl">${escapeHtml(f.level)}</span><span>${escapeHtml(f.message)}</span></div>`);
  if (savedOk === true && !list.some((f) => f.level === "error")) {
    rows.unshift('<div class="pf ok"><span class="lvl">saved</span><span>Config saved and the firmware header regenerated. Next: flash from Hardware Tests, or press Start 1-Click.</span></div>');
  } else if (savedOk === false) {
    rows.unshift('<div class="pf error"><span class="lvl">not built</span><span>Config saved, but the header was not regenerated. Fix the pins named below and save again.</span></div>');
  }
  box.innerHTML = rows.join("");
  box.hidden = false;
}

async function loadHardwareConfig() {
  try {
    const res = await fetch("/api/hardware/config");
    if (!res.ok) return;
    const data = await res.json();
    activeHwConfig = data;
    renderPinFindings(data.pin_findings);

    const tgt = data.base_controller || {};
    const kine = data.kinematics || {};
    const sensors = tgt.sensors || {};
    const pins = tgt.pins || {};

    // 1. Base & MCU
    // The selects list silicon; a config may name its board (gendrv, yb_eet01).
    loadedControllerName = data.controller || "pico2";
    const elMcu = document.getElementById("cfg-mcu");
    if (elMcu) elMcu.value = siliconOf(loadedControllerName);

    const elHwEnv = document.getElementById("hw-flash-env");
    if (elHwEnv) elHwEnv.value = siliconOf(loadedControllerName);

    initAdcChart();
  initWiringTable();

  const elSonar = document.getElementById("cfg-sonar");
  if (elSonar) {
    elSonar.addEventListener("change", syncSonarFields);
    syncSonarFields();
  }

  const elKine = document.getElementById("cfg-kinematics");
    if (elKine) elKine.value = kine.base_type || "2wd";

    const elBaud = document.getElementById("cfg-baudrate");
    if (elBaud && tgt.baudrate) elBaud.value = String(tgt.baudrate);
    const elConsole = document.getElementById("cfg-console");
    if (elConsole) elConsole.value = tgt.console || "usb";

    const elPort = document.getElementById("hw-flash-port");
    if (elPort && tgt.serial_port) elPort.value = tgt.serial_port;

    const elWheelD = document.getElementById("cfg-wheel-diameter");
    if (elWheelD) elWheelD.value = kine.wheel_diameter || 0.152;

    const elTrackW = document.getElementById("cfg-track-width");
    if (elTrackW) elTrackW.value = kine.lr_wheels_distance || 0.271;

    const elWheelbase = document.getElementById("cfg-wheelbase");
    if (elWheelbase) elWheelbase.value = kine.fr_wheels_distance || 0.0;

    const elMaxRpm = document.getElementById("cfg-max-rpm");
    if (elMaxRpm) elMaxRpm.value = kine.max_rpm || 140;

    const elCpr = document.getElementById("cfg-cpr");
    if (elCpr) elCpr.value = kine.counts_per_rev || 144000;

    const elHeadroom = document.getElementById("cfg-headroom");
    if (elHeadroom) elHeadroom.value = kine.max_rpm_ratio || 0.85;

    const elKp = document.getElementById("cfg-pid-kp");
    if (elKp && kine.pid) elKp.value = kine.pid.kp !== undefined ? kine.pid.kp : 0.6;
    const elKi = document.getElementById("cfg-pid-ki");
    if (elKi && kine.pid) elKi.value = kine.pid.ki !== undefined ? kine.pid.ki : 0.8;
    const elKd = document.getElementById("cfg-pid-kd");
    if (elKd && kine.pid) elKd.value = kine.pid.kd !== undefined ? kine.pid.kd : 0.5;

    const elOpV = document.getElementById("cfg-motor-voltage");
    if (elOpV) elOpV.value = kine.motor_operating_voltage || 24.0;
    const elMaxV = document.getElementById("cfg-motor-max-voltage");
    if (elMaxV) elMaxV.value = kine.motor_power_max_voltage || 12.0;
    loadSimForm(tgt.simulation || {});
    // Default ON when the key is absent: the same default drivetrain_report.py
    // applies, so an old config gets the derived limits rather than keeping
    // whatever it happened to ship with.
    const elAuto = document.getElementById("cfg-nav2-auto");
    if (elAuto) elAuto.checked = kine.auto_nav2_limits !== false;
    loadGeometryForm(data.geometry || {}, data.geometry_warnings || []);

    // 2. Drive & Motors
    const elDriver = document.getElementById("cfg-driver-type");
    if (elDriver) elDriver.value = tgt.driver_type || "GENERIC_2_IN";

    const elPwmFreq = document.getElementById("cfg-pwm-freq");
    if (elPwmFreq) elPwmFreq.value = kine.pwm_frequency || 20000;

    const elPwmBits = document.getElementById("cfg-pwm-bits");
    if (elPwmBits) elPwmBits.value = kine.pwm_bits || 10;

    const m1 = pins.motor1 || {};
    const m2 = pins.motor2 || {};
    const m3 = pins.motor3 || {};
    const m4 = pins.motor4 || {};

    const chkM1Inv = document.getElementById("pin-m1-inv");
    if (chkM1Inv) chkM1Inv.checked = !!m1.invert;
    const chkM2Inv = document.getElementById("pin-m2-inv");
    if (chkM2Inv) chkM2Inv.checked = m2.invert !== undefined ? !!m2.invert : true;
    const chkM3Inv = document.getElementById("pin-m3-inv");
    if (chkM3Inv) chkM3Inv.checked = !!m3.invert;
    const chkM4Inv = document.getElementById("pin-m4-inv");
    if (chkM4Inv) chkM4Inv.checked = m4.invert !== undefined ? !!m4.invert : true;

    const enc1 = pins.encoder1 || {};
    const enc2 = pins.encoder2 || {};
    const enc3 = pins.encoder3 || {};
    const enc4 = pins.encoder4 || {};

    const chkE1Inv = document.getElementById("pin-enc-1-inv");
    if (chkE1Inv) chkE1Inv.checked = !!enc1.invert;
    const chkE2Inv = document.getElementById("pin-enc-2-inv");
    if (chkE2Inv) chkE2Inv.checked = !!enc2.invert;
    const chkE3Inv = document.getElementById("pin-enc-3-inv");
    if (chkE3Inv) chkE3Inv.checked = !!enc3.invert;
    const chkE4Inv = document.getElementById("pin-enc-4-inv");
    if (chkE4Inv) chkE4Inv.checked = !!enc4.invert;

    // 3. Sensors
    const elImu = document.getElementById("cfg-imu");
    if (elImu) elImu.value = sensors.imu || (sensors.use_sim_imu ? "SIM" : "NONE");

    const elMag = document.getElementById("cfg-mag");
    if (elMag) elMag.value = sensors.mag || "NONE";

    const elCurrent = document.getElementById("cfg-battery");
    if (elCurrent) {
      if (sensors.current === "INA219") elCurrent.value = "INA219";
      else if (pins.battery && pins.battery.pin !== undefined && pins.battery.pin >= 0) elCurrent.value = "ADC_DIVIDER";
      else elCurrent.value = "NONE";
    }

    const bat = pins.battery || {};
    const elBatR1 = document.getElementById("cfg-bat-r1");
    if (elBatR1 && bat.r1) elBatR1.value = bat.r1;
    const elBatR2 = document.getElementById("cfg-bat-r2");
    if (elBatR2 && bat.r2) elBatR2.value = bat.r2;
    // The pack itself. These fields existed for years and were never saved;
    // the firmware reads them now (bat_min / bat_max / bat_cap in the env)
    // for /battery percentage and capacity.
    const elBatMin = document.getElementById("cfg-bat-min");
    if (elBatMin && bat.min_v !== undefined) elBatMin.value = bat.min_v;
    const elBatMax = document.getElementById("cfg-bat-max");
    if (elBatMax && bat.max_v !== undefined) elBatMax.value = bat.max_v;
    const elBatCap = document.getElementById("cfg-bat-cap");
    if (elBatCap && bat.capacity_ah !== undefined) elBatCap.value = bat.capacity_ah;
    const elBatDip = document.getElementById("cfg-bat-dip");
    if (elBatDip) elBatDip.value = bat.dip_pct !== undefined ? bat.dip_pct : 2;
    // Nominal pack voltage and the ADC filter capacitor: shown for years,
    // saved by nothing, so they reverted on every reload.
    const elBatNom = document.getElementById("cfg-bat-nom");
    if (elBatNom && bat.nominal_v !== undefined) elBatNom.value = bat.nominal_v;
    const elBatCapVal = document.getElementById("cfg-bat-cap-val");
    if (elBatCapVal && bat.filter_cap_pf !== undefined) elBatCapVal.value = bat.filter_cap_pf;

    // Hard-iron offsets. The firmware subtracts them when MAG_BIAS is defined
    // (firmware/src/main.cpp), and scripts/gen_firmware_header.py emits it from
    // exactly these three keys.
    const magBias = Array.isArray(sensors.mag_bias) ? sensors.mag_bias : [];
    [["cfg-mag-bias-x", 0], ["cfg-mag-bias-y", 1], ["cfg-mag-bias-z", 2]].forEach(([id, i]) => {
      const el = document.getElementById(id);
      if (el && magBias[i] !== undefined && magBias[i] !== null) el.value = magBias[i];
    });

    const elEnv = document.getElementById("cfg-env");
    // AUTO when the config says nothing: the barometer is in every image and
    // found on the bus at boot. NONE is an explicit "never publish".
    if (elEnv) elEnv.value = sensors.env ? String(sensors.env).toUpperCase() : "AUTO";

    const chkSimImu = document.getElementById("chk-sim-imu");
    if (chkSimImu) chkSimImu.checked = !!sensors.use_sim_imu;
    const chkSimMag = document.getElementById("chk-sim-mag");
    if (chkSimMag) chkSimMag.checked = !!sensors.use_sim_mag;
    const chkSimWheel = document.getElementById("chk-sim-wheel");
    if (chkSimWheel) chkSimWheel.checked = !!sensors.use_sim_wheel;
    const chkSimLd19 = document.getElementById("chk-sim-ld19");
    if (chkSimLd19) chkSimLd19.checked = !!sensors.use_sim_ld19;
    const chkSimEnv = document.getElementById("chk-sim-env");
    if (chkSimEnv) chkSimEnv.checked = !!sensors.use_sim_env;
    // Absent means on, as mcu_env reads it: the cone exists whenever the
    // simulated LiDAR does, unless the config turns it off.
    const chkSimSonar = document.getElementById("chk-sim-sonar");
    if (chkSimSonar) chkSimSonar.checked = sensors.use_sim_sonar !== false;
    const chkSimBattery = document.getElementById("chk-sim-battery");
    if (chkSimBattery) chkSimBattery.checked = !!sensors.use_sim_battery;

    // 4. Pin Matrix
    const elLed = document.getElementById("pin-led");
    if (elLed && pins.led !== undefined) {
      elLed.value = typeof pins.led === "object" ? (pins.led.pin !== undefined ? pins.led.pin : -1) : pins.led;
    }

    // Motor pins
    if (document.getElementById("pin-m1-p1")) document.getElementById("pin-m1-p1").value = m1.pwm !== undefined ? m1.pwm : -1;
    if (document.getElementById("pin-m1-p2")) document.getElementById("pin-m1-p2").value = m1.in_a !== undefined ? m1.in_a : -1;
    if (document.getElementById("pin-m1-p3")) document.getElementById("pin-m1-p3").value = m1.in_b !== undefined ? m1.in_b : -1;

    if (document.getElementById("pin-m2-p1")) document.getElementById("pin-m2-p1").value = m2.pwm !== undefined ? m2.pwm : -1;
    if (document.getElementById("pin-m2-p2")) document.getElementById("pin-m2-p2").value = m2.in_a !== undefined ? m2.in_a : -1;
    if (document.getElementById("pin-m2-p3")) document.getElementById("pin-m2-p3").value = m2.in_b !== undefined ? m2.in_b : -1;

    if (document.getElementById("pin-m3-p1")) document.getElementById("pin-m3-p1").value = m3.pwm !== undefined ? m3.pwm : -1;
    if (document.getElementById("pin-m3-p2")) document.getElementById("pin-m3-p2").value = m3.in_a !== undefined ? m3.in_a : -1;
    if (document.getElementById("pin-m3-p3")) document.getElementById("pin-m3-p3").value = m3.in_b !== undefined ? m3.in_b : -1;

    if (document.getElementById("pin-m4-p1")) document.getElementById("pin-m4-p1").value = m4.pwm !== undefined ? m4.pwm : -1;
    if (document.getElementById("pin-m4-p2")) document.getElementById("pin-m4-p2").value = m4.in_a !== undefined ? m4.in_a : -1;
    if (document.getElementById("pin-m4-p3")) document.getElementById("pin-m4-p3").value = m4.in_b !== undefined ? m4.in_b : -1;

    // Encoder pins
    if (document.getElementById("pin-enc-1a")) document.getElementById("pin-enc-1a").value = enc1.pin_a !== undefined ? enc1.pin_a : -1;
    if (document.getElementById("pin-enc-1b")) document.getElementById("pin-enc-1b").value = enc1.pin_b !== undefined ? enc1.pin_b : -1;

    if (document.getElementById("pin-enc-2a")) document.getElementById("pin-enc-2a").value = enc2.pin_a !== undefined ? enc2.pin_a : -1;
    if (document.getElementById("pin-enc-2b")) document.getElementById("pin-enc-2b").value = enc2.pin_b !== undefined ? enc2.pin_b : -1;

    if (document.getElementById("pin-enc-3a")) document.getElementById("pin-enc-3a").value = enc3.pin_a !== undefined ? enc3.pin_a : -1;
    if (document.getElementById("pin-enc-3b")) document.getElementById("pin-enc-3b").value = enc3.pin_b !== undefined ? enc3.pin_b : -1;

    if (document.getElementById("pin-enc-4a")) document.getElementById("pin-enc-4a").value = enc4.pin_a !== undefined ? enc4.pin_a : -1;
    if (document.getElementById("pin-enc-4b")) document.getElementById("pin-enc-4b").value = enc4.pin_b !== undefined ? enc4.pin_b : -1;

    // Bus & aux pins
    const i2c = pins.i2c || {};
    if (document.getElementById("pin-i2c-sda")) document.getElementById("pin-i2c-sda").value = i2c.sda !== undefined ? i2c.sda : -1;
    if (document.getElementById("pin-i2c-scl")) document.getElementById("pin-i2c-scl").value = i2c.scl !== undefined ? i2c.scl : -1;

    const elBatPin = document.getElementById("pin-battery");
    if (elBatPin) elBatPin.value = bat.pin !== undefined ? bat.pin : -1;

    const sonar = pins.sonar || {};
    if (document.getElementById("pin-sonar-trig")) document.getElementById("pin-sonar-trig").value = sonar.trigger !== undefined ? sonar.trigger : -1;
    if (document.getElementById("pin-sonar-echo")) document.getElementById("pin-sonar-echo").value = sonar.echo !== undefined ? sonar.echo : -1;
    // The enable select is the truth the user sees; wired pins are what it
    // means. Derived rather than stored, so the two can never disagree.
    const elSonarEn = document.getElementById("cfg-sonar");
    if (elSonarEn) {
      const on = Number(sonar.trigger) >= 0 && Number(sonar.echo) >= 0;
      elSonarEn.value = on ? "true" : "false";
    }

    const elDacPin = document.getElementById("cfg-dac-pin");
    if (elDacPin && pins.dac !== undefined && pins.dac !== null) elDacPin.value = String(pins.dac);

    const lidar = tgt.lidar || {};
    if (document.getElementById("cfg-lidar-rxd")) document.getElementById("cfg-lidar-rxd").value = lidar.rx_pin !== undefined ? lidar.rx_pin : -1;

    const elSerialPort = document.getElementById("cfg-serial-port");
    if (elSerialPort) {
      const portVal = tgt.serial_port || tgt.port || (targetMcu.includes("pico") ? "/dev/ttyACM0" : "/dev/ttyUSB0");
      elSerialPort.value = portVal;
    }
    syncMcuSerialSettings();

    updateKinematicsHUD();
    updateAdcCalculations();
    updateKinematicsVisibility();
    updateMotorPinVisibility();
    validateHardwareSafety();
    // AFTER the selects are populated. Running it only at init read whatever
    // the markup happened to default to, so an ESP32 board was told "pico2 has
    // no hardware DAC" and the calibration studio stayed hidden on the one
    // family that can use it.
    applyDacAvailability();
    refreshHwSerialPorts();
  } catch (err) {
    console.error("[HardwareConfig] Error loading hardware config:", err);
  }
}

// --- Body & sensor placement (the generated URDF) ---------------------------
// The backend fills every gap from the kinematics (effective_geometry), so the
// form is never blank; what is saved is exactly what the URDF is built from.
function loadGeometryForm(geo, warnings) {
  const set = (id, v) => { const el = document.getElementById(id); if (el && v !== undefined && v !== null) el.value = v; };
  const body = geo.body || {}, wheel = geo.wheel || {}, laser = geo.laser || {}, imu = geo.imu || {}, casters = geo.casters || {};
  set("geo-body-length", body.length); set("geo-body-width", body.width);
  set("geo-body-height", body.height); set("geo-body-mass", body.mass);
  set("geo-wheel-width", wheel.width); set("geo-wheel-z", wheel.z);
  set("geo-laser-x", laser.x); set("geo-laser-y", laser.y); set("geo-laser-z", laser.z); set("geo-laser-yaw", laser.yaw);
  set("geo-imu-x", imu.x); set("geo-imu-y", imu.y); set("geo-imu-z", imu.z);
  const cf = document.getElementById("geo-caster-front"); if (cf) cf.checked = !!casters.front;
  const cr = document.getElementById("geo-caster-rear"); if (cr) cr.checked = !!casters.rear;
  renderGeometryWarnings(warnings);
}

function readGeometryForm(kineType) {
  const num = (id) => { const v = parseFloat(document.getElementById(id)?.value); return Number.isFinite(v) ? v : undefined; };
  const strip = (o) => Object.fromEntries(Object.entries(o).filter(([, v]) => v !== undefined));
  const geo = {
    body: strip({ length: num("geo-body-length"), width: num("geo-body-width"), height: num("geo-body-height"), mass: num("geo-body-mass") }),
    wheel: strip({ width: num("geo-wheel-width"), z: num("geo-wheel-z") }),
    laser: strip({ x: num("geo-laser-x"), y: num("geo-laser-y"), z: num("geo-laser-z"), yaw: num("geo-laser-yaw") }),
    imu: strip({ x: num("geo-imu-x"), y: num("geo-imu-y"), z: num("geo-imu-z") }),
  };
  if (kineType === "2wd") {
    geo.casters = { front: !!document.getElementById("geo-caster-front")?.checked,
                    rear: !!document.getElementById("geo-caster-rear")?.checked };
  }
  return geo;
}

function renderGeometryWarnings(warnings) {
  const box = document.getElementById("geometry-warnings");
  if (!box) return;
  const list = Array.isArray(warnings) ? warnings : [];
  box.innerHTML = list.length
    ? list.map((w) => `<div class="warn">⚠ ${escapeHtml(String(w))}</div>`).join("")
    : "";
}

// The load and the drivetrain's losses: form field <-> base_controller.simulation
// key. One table, used by the loader, the save payload and the HUD's request, so
// a field cannot be readable and unsaveable (or worse, saved under a name the
// firmware does not read -- the env key contract's whole subject).
const SIM_FIELDS = [
  ["cfg-sim-mass",      "robot_mass"],
  ["cfg-sim-gear-eff",  "gear_efficiency"],
  ["cfg-sim-gear-drag", "gear_drag_rpm"],
  ["cfg-sim-sag",       "battery_sag"],
  ["cfg-sim-sag-tau",   "battery_sag_tau_ms"],
  ["cfg-sim-drv-drop",  "driver_drop"],
  ["cfg-sim-drv-r",     "driver_resistance"],
  ["cfg-sim-stall-a",   "motor_stall_amps"],
  ["cfg-sim-ilimit-a",  "driver_current_limit"],
];

// Blank means "use the firmware's default", never zero. A gearbox of efficiency
// 0 or a pack that never sags are both meaningful values somebody might type, so
// the empty string is the only thing that means unset -- which is why this
// returns undefined rather than 0 and the caller drops the key entirely.
function readSimForm() {
  const out = {};
  SIM_FIELDS.forEach(([id, key]) => {
    const raw = document.getElementById(id)?.value;
    if (raw === undefined || raw === null || String(raw).trim() === "") return;
    const v = parseFloat(raw);
    if (Number.isFinite(v)) out[key] = v;
  });
  return out;
}

function loadSimForm(sim) {
  SIM_FIELDS.forEach(([id, key]) => {
    const el = document.getElementById(id);
    if (!el) return;
    // The placeholder already shows the firmware default, so an absent key
    // leaves the box empty rather than pretending the user chose that value.
    el.value = (sim && sim[key] !== undefined && sim[key] !== null) ? sim[key] : "";
  });
}

function updateKinematicsHUD() {
  const wheelD = parseFloat(document.getElementById("cfg-wheel-diameter")?.value || 0.152);
  const trackW = parseFloat(document.getElementById("cfg-track-width")?.value || 0.271);
  const maxRpm = parseFloat(document.getElementById("cfg-max-rpm")?.value || 140);
  const cpr = parseFloat(document.getElementById("cfg-cpr")?.value || 144000);
  const headroom = parseFloat(document.getElementById("cfg-headroom")?.value || 0.85);

  if (wheelD > 0 && maxRpm > 0) {
    const circ = Math.PI * wheelD;
    const maxLinear = (circ * maxRpm / 60.0) * headroom;
    const maxAngular = trackW > 0 ? (2.0 * maxLinear) / trackW : 0;
    const ticksPerM = circ > 0 && cpr > 0 ? cpr / circ : 0;

    const elSpeed = document.getElementById("hud-max-speed");
    if (elSpeed) elSpeed.textContent = `${maxLinear.toFixed(2)} m/s`;

    const elOmega = document.getElementById("hud-max-omega");
    if (elOmega) elOmega.textContent = `${maxAngular.toFixed(2)} rad/s`;

    const elCirc = document.getElementById("hud-wheel-circ");
    if (elCirc) elCirc.textContent = `${circ.toFixed(3)} m`;

    const elTicks = document.getElementById("hud-ticks-per-m");
    if (elTicks) elTicks.textContent = `${Math.round(ticksPerM).toLocaleString()} ticks/m`;
  }
  updateDrivetrainHUD();
}

// --- motor performance --------------------------------------------------------
//
// pi*d*rpm/60 above is arithmetic, and arithmetic cannot answer the question
// somebody designing a robot actually has: does it accelerate fast enough, and
// is the Nav2 tuning in this same config asking the motors for more than they
// can give? That needs the brushed DC torque-speed model, and until now the only
// way to see it was to flash test_acc and drive the board.
//
// The model is NOT reimplemented here. /api/drivetrain/performance runs
// scripts/drivetrain_report.py, which parses its constants out of sim_wheel.h --
// so this HUD and the firmware cannot disagree about the robot. A JS copy would
// be a second opinion that drifts silently, which is the exact fault the report
// was written to avoid.
let drivetrainHudTimer = null;
let drivetrainHudSeq = 0;

function updateDrivetrainHUD() {
  // Debounced: this fires on every keystroke in the kinematics form.
  if (drivetrainHudTimer) clearTimeout(drivetrainHudTimer);
  drivetrainHudTimer = setTimeout(fetchDrivetrainHUD, 250);
}

async function fetchDrivetrainHUD() {
  const num = (id, dflt) => {
    const v = parseFloat(document.getElementById(id)?.value);
    return Number.isFinite(v) ? v : dflt;
  };
  const payload = {
    kinematics: {
      base_type: document.getElementById("cfg-kinematics")?.value || "2wd",
      wheel_diameter: num("cfg-wheel-diameter", 0.152),
      lr_wheels_distance: num("cfg-track-width", 0.271),
      fr_wheels_distance: num("cfg-wheelbase", 0.0),
      max_rpm: num("cfg-max-rpm", 140),
      max_rpm_ratio: num("cfg-headroom", 0.85),
      counts_per_rev: num("cfg-cpr", 144000),
      // The loop's output range. A 12-bit board reaches the same speed with
      // four times the counts, so its plant gain is a quarter of a 10-bit
      // board's -- and gains derived without knowing that are 4x wrong.
      pwm_bits: num("cfg-pwm-bits", 10),
      motor_operating_voltage: num("cfg-motor-voltage", 24.0),
      motor_power_max_voltage: num("cfg-motor-max-voltage", 12.0),
      auto_nav2_limits: document.getElementById("cfg-nav2-auto")?.checked !== false,
    },
    // Unsaved edits count: the HUD's job is to answer while the user is still
    // deciding, so it sends what the form says, not what the file says.
    base_controller: { simulation: readSimForm() },
  };
  // A late reply must not overwrite a newer one: the user keeps typing while
  // these are in flight, and out-of-order responses would show the HUD of a
  // config that no longer matches the form.
  const seq = ++drivetrainHudSeq;
  let data;
  try {
    const res = await fetch("/api/drivetrain/performance", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(payload),
    });
    data = await res.json();
  } catch (err) {
    return;                       // offline or backend restarting; keep the last numbers
  }
  if (seq !== drivetrainHudSeq) return;
  renderDrivetrainHUD(data);
}

function renderDrivetrainHUD(d) {
  const set = (id, text) => {
    const el = document.getElementById(id);
    if (el) el.textContent = text;
  };
  const checks = document.getElementById("hud-perf-checks");
  if (!d || d.ready === false) {
    ["hud-perf-speed", "hud-perf-accel-first", "hud-perf-accel-held", "hud-perf-t90",
     "hud-perf-radius", "hud-perf-budget", "hud-acc-max-vel", "hud-acc-max-acc",
     "hud-acc-t90", "hud-acc-stop"].forEach((id) => set(id, "--"));
    set("hud-perf-model", d && d.reason ? d.reason : "");
    if (checks) checks.innerHTML = "";
    return;
  }

  set("hud-perf-speed", `${d.max_linear.toFixed(2)} m/s`);
  set("hud-perf-speed-sub", `${d.max_angular.toFixed(2)} rad/s about the centre`);
  set("hud-perf-accel-first", `${d.accel_first.toFixed(2)} m/s²`);
  set("hud-perf-accel-held", `${d.accel_held.toFixed(2)} m/s²`);
  set("hud-perf-t90", `${d.t_to_90.toFixed(2)} s`);
  set("hud-perf-stop", `stops in ${d.stop_distance.toFixed(2)} m (τ ${Math.round(d.tau_ms)} ms)`);
  set("hud-perf-radius", `${d.radius.toFixed(3)} m`);
  set("hud-perf-radius-note", d.radius_note || "");
  set("hud-perf-budget", `${Math.round(d.command_rpm)} rpm`);
  set("hud-perf-budget-sub",
      `motor reaches ${Math.round(d.motor_rpm)} rpm`
      + (d.volt_ratio < 0.999 ? ` (pack derates it to ${Math.round(d.volt_ratio * 100)}%)` : ""));
  set("hud-perf-model",
      `${d.mass.toFixed(2)} kg · gearbox ${Math.round(d.model.gear_efficiency * 100)}% · `
      + `drag ${Math.round(d.model.gear_drag_rpm)} rpm/s · sag ${Math.round(d.model.battery_sag * 100)}%`
      + ` over ${Math.round(d.model.battery_sag_tau_ms)} ms · driver `
      + `${Math.round(d.model.driver_drop * 100)}% + ${Math.round(d.model.driver_resistance * 100)}%`
      + (d.model.driver_current_limit > 0
         ? ` · limited to ${d.model.driver_current_limit} A of ${d.model.motor_stall_amps} A stall`
         : ` · no current limiter`));

  // The simulated test_acc run. Same four lines the tool prints on a board, in
  // the same units and the same order, so the two are directly comparable when
  // somebody does put motors on it.
  const m = d.measured || {};
  if (m.max_vel !== undefined) {
    set("hud-acc-max-vel", `${m.max_vel.toFixed(2)} m/s`);
    set("hud-acc-max-vel-ang", `${m.max_vel_ang.toFixed(2)} rad/s turning on the spot`);
    set("hud-acc-max-acc", `${m.max_acc.toFixed(2)} m/s²`);
    set("hud-acc-max-acc-ang", `${m.max_acc_ang.toFixed(2)} rad/s² turning`);
    set("hud-acc-t90", `${m.t_to_90.toFixed(2)} s`);
    set("hud-acc-stop", `${m.stop.toFixed(3)} m`);
    set("hud-acc-stop-ang", `${m.stop_ang.toFixed(2)} rad turning`);
  }

  const badge = document.getElementById("hud-kinematics-badge");
  if (badge) {
    // The badge used to say "85% Headroom Safe" whatever the numbers were --
    // a reassurance with nothing behind it. It now reports the verdict.
    badge.textContent = d.over_budget ? "⚠ Config asks for more than the motors give"
                                      : "✓ Within the motors' budget";
    badge.classList.toggle("over", !!d.over_budget);
  }

  set("hud-nav2-auto-state", d.auto_limits
      ? "auto-tuning is on — saving rewrites these from the motors"
      : "auto-tuning is off — these are yours to set");
  renderSuggestedLimits(d);

  if (!checks) return;
  const rows = (d.checks || []).map((c) => {
    if (!c.set) {
      return `<div class="hud-check"><span class="hud-check-label">${escapeHtml(c.label)}</span>`
           + `<span>not set in this config</span></div>`;
    }
    const cls = c.over ? "over" : "ok";
    const flag = c.over ? "OVER BUDGET" : "ok";
    const detail = c.need_rpm !== undefined
      ? `${c.linear.toFixed(2)} m/s + ${c.angular.toFixed(2)} rad/s → `
        + `${c.need_rpm.toFixed(1)} rpm = ${c.percent.toFixed(0)}% of ${Math.round(c.budget_rpm)}`
      : `${c.asked.toFixed(2)} m/s² → ${c.percent.toFixed(0)}% of the `
        + `${c.have.toFixed(2)} m/s² it holds`;
    return `<div class="hud-check ${cls}"><span class="hud-check-label">`
         + `${escapeHtml(c.label)}</span><span>${escapeHtml(detail)}</span>`
         + `<span>${flag}</span></div>`;
  });
  if (d.auto_limits) {
    // With auto-tuning on, an over-budget row is a statement about the FILE as
    // it stands, not about what the robot will run: saving rewrites it. Saying
    // so stops the red row reading as an unfixable fault.
    rows.unshift('<div class="hud-check"><span class="hud-check-label">In the config now'
               + "</span><span>auto-tuning is on, so saving replaces these with the "
               + "values below</span></div>");
  }
  checks.innerHTML = rows.join("");
}

// What auto-tuning would write. Shown whether it is on or off: with it on this
// is a preview of the save, and with it off it is the advice a manual tuner
// wanted without having their own numbers overwritten to see it.
function renderSuggestedLimits(d) {
  const box = document.getElementById("hud-perf-suggested");
  if (!box) return;
  const sug = d.suggested || {};
  const keys = Object.keys(sug);
  if (!keys.length) { box.innerHTML = ""; return; }
  const fmt = (v) => (Array.isArray(v) ? `[${v.join(", ")}]` : String(v));
  const head = d.auto_limits
    ? "Derived from the motors — written on save"
    : "Auto-tuning is OFF — these are what the motors suggest, nothing is rewritten";
  box.innerHTML = `<div class="hud-check"><span class="hud-check-label">${escapeHtml(head)}`
    + "</span></div>"
    + keys.map((k) => `<div class="hud-check"><span class="hud-check-label">`
        + `${escapeHtml(k)}</span><span>${escapeHtml(fmt(sug[k]))}</span></div>`).join("");
}

function updateAdcCalculations() {
  const batMax = parseFloat(document.getElementById("cfg-bat-max")?.value || 12.6);
  const r1 = parseFloat(document.getElementById("cfg-bat-r1")?.value || 30000);
  const r2 = parseFloat(document.getElementById("cfg-bat-r2")?.value || 7500);

  if (r1 + r2 > 0) {
    const ratio = r2 / (r1 + r2);
    const maxAdcV = batMax * ratio;

    const elRatio = document.getElementById("lbl-divider-ratio");
    if (elRatio) elRatio.textContent = `${ratio.toFixed(4)} (1:${(1/ratio).toFixed(1)})`;

    const elAdcV = document.getElementById("lbl-adc-max-v");
    if (elAdcV) elAdcV.textContent = `${maxAdcV.toFixed(2)} V`;

    const banner = document.getElementById("bat-adc-safety-banner");
    const icon = document.getElementById("safety-banner-icon");
    const title = document.getElementById("safety-banner-title");
    const desc = document.getElementById("safety-banner-desc");

    if (banner) {
      if (maxAdcV <= 3.0) {
        banner.className = "adc-safety-banner safe";
        if (icon) icon.textContent = "🛡️";
        if (title) title.textContent = `ADC Input Voltage: Safe (${maxAdcV.toFixed(2)}V ≤ 3.0V)`;
      } else {
        banner.className = "adc-safety-banner danger";
        if (icon) icon.textContent = "⚠️";
        if (title) title.textContent = `EXCEEDS 3.0V LIMIT! (${maxAdcV.toFixed(2)}V > 3.0V)`;
        if (desc) desc.textContent = `Divider delivers ${maxAdcV.toFixed(2)}V at full charge, exceeding 3.0V linear range! Increase R1 or reduce R2 to prevent pin damage.`;
      }
    }
  }
}

function updateKinematicsVisibility() {
  const kine = document.getElementById("cfg-kinematics")?.value || "2wd";
  const is4wd = (kine === "4wd" || kine === "mecanum");

  const grpWb = document.getElementById("group-wheelbase");
  if (grpWb) grpWb.style.display = is4wd ? "flex" : "none";
  const grpCasters = document.getElementById("group-casters");
  if (grpCasters) grpCasters.style.display = is4wd ? "none" : "flex";

  const rowM3 = document.getElementById("row-motor-3");
  const rowM4 = document.getElementById("row-motor-4");
  if (rowM3) rowM3.style.display = is4wd ? "table-row" : "none";
  if (rowM4) rowM4.style.display = is4wd ? "table-row" : "none";

  const rowE3 = document.getElementById("row-enc-3");
  const rowE4 = document.getElementById("row-enc-4");
  if (rowE3) rowE3.style.display = is4wd ? "table-row" : "none";
  if (rowE4) rowE4.style.display = is4wd ? "table-row" : "none";

  document.querySelectorAll(".row-m3-item, .row-m4-item").forEach(el => {
    el.style.display = is4wd ? "flex" : "none";
  });

  // "Front Left"/"Front Right" only mean something when there is a rear pair.
  // On a 2WD base motors 3 and 4 are hidden, so those two labels described
  // wheels the robot does not have.
  document.querySelectorAll(".motor-pos").forEach(el => {
    const side = el.dataset.motor === "m1" ? "Left" : "Right";
    el.textContent = is4wd ? `Front ${side}` : side;
  });

  if (!is4wd) {
    const unusedPins = [
      "pin-m3-p1", "pin-m3-p2", "pin-m3-p3",
      "pin-m4-p1", "pin-m4-p2", "pin-m4-p3",
      "pin-enc-3a", "pin-enc-3b",
      "pin-enc-4a", "pin-enc-4b"
    ];
    unusedPins.forEach(id => {
      const el = document.getElementById(id);
      if (el) el.value = "-1";
    });
  }
}

function updateMotorPinVisibility() {
  const drv = document.getElementById("cfg-driver-type")?.value || "GENERIC_2_IN";
  const thP1 = document.getElementById("th-m-p1");
  const thP2 = document.getElementById("th-m-p2");
  const thP3 = document.getElementById("th-m-p3");

  let p1Used = true, p2Used = true, p3Used = true;
  let p1Label = "PWM (Speed)", p2Label = "IN_A (Dir 1)", p3Label = "IN_B (Dir 2)";

  if (drv === "BTS7960") {
    // Two-PWM drivers: RPWM (IN_A) and LPWM (IN_B) carry the speed, and the
    // pin the config calls `pwm` is the driver's ENABLE (R_EN/L_EN on a
    // BTS7960 module, PWMA/PWMB on the Waveshare General Driver's TB6612),
    // which the firmware holds HIGH. It used to be hidden here as "unused" --
    // on the GenDrv that is GPIO 25/26, and losing it disables the motors.
    p1Used = true;
    p2Used = true;
    p3Used = true;
    p1Label = "EN (enable, -1 if tied high)";
    p2Label = "RPWM (IN_A)";
    p3Label = "LPWM (IN_B)";
  } else if (drv === "GENERIC_1_IN") {
    // Cytron MD10C: PWM speed + DIR. IN_B is unused placeholder.
    p1Used = true;
    p2Used = true;
    p3Used = false;
    p1Label = "PWM (Speed)";
    p2Label = "DIR (Direction)";
  } else if (drv === "ESC") {
    // RC ESC: single PWM signal. IN_A and IN_B are unused placeholders.
    p1Used = true;
    p2Used = false;
    p3Used = false;
    p1Label = "PWM Signal";
  } else {
    // GENERIC_2_IN: L298N, TB6612: PWM + IN_A + IN_B
    p1Used = true;
    p2Used = true;
    p3Used = true;
    p1Label = "PWM (Speed)";
    p2Label = "IN_A (Dir 1)";
    p3Label = "IN_B (Dir 2)";
  }

  if (thP1) {
    thP1.textContent = p1Label;
    thP1.style.display = p1Used ? "" : "none";
  }
  if (thP2) {
    thP2.textContent = p2Label;
    thP2.style.display = p2Used ? "" : "none";
  }
  if (thP3) {
    thP3.textContent = p3Label;
    thP3.style.display = p3Used ? "" : "none";
  }

  document.querySelectorAll(".cell-m-p1").forEach(td => {
    td.style.display = p1Used ? "" : "none";
    const inp = td.querySelector("input");
    if (!p1Used && inp && (inp.value === "" || inp.value === undefined)) {
      inp.value = "-1";
    }
  });
  document.querySelectorAll(".cell-m-p2").forEach(td => {
    td.style.display = p2Used ? "" : "none";
    const inp = td.querySelector("input");
    if (!p2Used && inp && (inp.value === "" || inp.value === undefined)) {
      inp.value = "-1";
    }
  });
  document.querySelectorAll(".cell-m-p3").forEach(td => {
    td.style.display = p3Used ? "" : "none";
    const inp = td.querySelector("input");
    if (!p3Used && inp && (inp.value === "" || inp.value === undefined)) {
      inp.value = "-1";
    }
  });
}

function autoAssignPins() {
  // A board name checks as its silicon: the Yahboom YB-EET01 is an ESP32-S3.
  const mcuSel = (document.getElementById("cfg-mcu")?.value || "pico2").toLowerCase();
  const mcu = mcuSel === "yb_eet01" ? "esp32s3" : mcuSel;
  const kine = document.getElementById("cfg-kinematics")?.value || "2wd";
  const is4wd = (kine === "4wd" || kine === "mecanum");

  if (mcu.includes("pico")) {
    document.getElementById("pin-led").value = (mcu === "picow" || mcu === "pico2w") ? 32 : 25;
    document.getElementById("pin-i2c-sda").value = 0;
    document.getElementById("pin-i2c-scl").value = 1;
    document.getElementById("pin-battery").value = 26;
    document.getElementById("pin-sonar-trig").value = 22;
    document.getElementById("pin-sonar-echo").value = 27;

    document.getElementById("pin-m1-p1").value = 10;
    document.getElementById("pin-m1-p2").value = 11;
    document.getElementById("pin-m1-p3").value = 12;

    document.getElementById("pin-m2-p1").value = 13;
    document.getElementById("pin-m2-p2").value = 14;
    document.getElementById("pin-m2-p3").value = 15;

    document.getElementById("pin-enc-1a").value = 2;
    document.getElementById("pin-enc-1b").value = 3;
    document.getElementById("pin-enc-2a").value = 4;
    document.getElementById("pin-enc-2b").value = 5;

    if (is4wd) {
      document.getElementById("pin-m3-p1").value = 16;
      document.getElementById("pin-m3-p2").value = 17;
      document.getElementById("pin-m3-p3").value = 18;
      document.getElementById("pin-m4-p1").value = 19;
      document.getElementById("pin-m4-p2").value = 20;
      document.getElementById("pin-m4-p3").value = 21;
      document.getElementById("pin-enc-3a").value = 6;
      document.getElementById("pin-enc-3b").value = 7;
      document.getElementById("pin-enc-4a").value = 8;
      document.getElementById("pin-enc-4b").value = 9;
    }
  } else if (mcu === "esp32" || mcu === "gendrv") {
    document.getElementById("pin-led").value = 2;
    document.getElementById("pin-i2c-sda").value = 21;
    document.getElementById("pin-i2c-scl").value = 22;
    document.getElementById("pin-battery").value = 36;
    document.getElementById("pin-sonar-trig").value = 5;
    document.getElementById("pin-sonar-echo").value = 17;

    document.getElementById("pin-m1-p1").value = 13;
    document.getElementById("pin-m1-p2").value = 14;
    document.getElementById("pin-m1-p3").value = 27;

    document.getElementById("pin-m2-p1").value = 25;
    document.getElementById("pin-m2-p2").value = 26;
    document.getElementById("pin-m2-p3").value = 33;

    document.getElementById("pin-enc-1a").value = 4;
    document.getElementById("pin-enc-1b").value = 32;
    document.getElementById("pin-enc-2a").value = 35;
    document.getElementById("pin-enc-2b").value = 34;

    if (is4wd) {
      document.getElementById("pin-m3-p1").value = 18;
      document.getElementById("pin-m3-p2").value = 19;
      document.getElementById("pin-m3-p3").value = 23;
      document.getElementById("pin-m4-p1").value = 15;
      document.getElementById("pin-m4-p2").value = 16;
      document.getElementById("pin-m4-p3").value = 17;
    }
  } else if (mcu === "esp32s3") {
    document.getElementById("pin-led").value = 48;
    document.getElementById("pin-i2c-sda").value = 41;
    document.getElementById("pin-i2c-scl").value = 42;
    document.getElementById("pin-battery").value = 3;
    document.getElementById("pin-sonar-trig").value = 47;
    document.getElementById("pin-sonar-echo").value = 40;

    document.getElementById("pin-m1-p1").value = 1;
    document.getElementById("pin-m1-p2").value = 2;
    document.getElementById("pin-m1-p3").value = 4;

    document.getElementById("pin-m2-p1").value = 5;
    document.getElementById("pin-m2-p2").value = 6;
    document.getElementById("pin-m2-p3").value = 7;

    document.getElementById("pin-enc-1a").value = 14;
    document.getElementById("pin-enc-1b").value = 15;
    document.getElementById("pin-enc-2a").value = 16;
    document.getElementById("pin-enc-2b").value = 17;

    if (is4wd) {
      document.getElementById("pin-m3-p1").value = 8;
      document.getElementById("pin-m3-p2").value = 9;
      document.getElementById("pin-m3-p3").value = 10;
      document.getElementById("pin-m4-p1").value = 11;
      document.getElementById("pin-m4-p2").value = 12;
      document.getElementById("pin-m4-p3").value = 13;
    }
  }

  validateHardwareSafety();
  logLine(`[config-engine] Auto-assigned pins for ${mcu.toUpperCase()} (${kine.toUpperCase()})`);
}

function validateHardwareSafety() {
  // A board name checks as its silicon: the Yahboom YB-EET01 is an ESP32-S3.
  const mcuSel = (document.getElementById("cfg-mcu")?.value || "pico2").toLowerCase();
  const mcu = mcuSel === "yb_eet01" ? "esp32s3" : mcuSel;
  const kine = document.getElementById("cfg-kinematics")?.value || "2wd";
  const is4wd = (kine === "4wd" || kine === "mecanum");

  const assigned = {};
  const errors = [];
  const warnings = [];

  function checkPin(id, label, isOutput = false) {
    const el = document.getElementById(id);
    if (!el) return;
    const val = parseInt(el.value, 10);
    if (isNaN(val) || val < 0) return;

    if (!assigned[val]) assigned[val] = [];
    assigned[val].push(label);

    // Microcontroller specific checks
    if (mcu.includes("pico")) {
      if (val > 29) errors.push(`GP${val} (${label}) is out of range for RP2040/RP2350 (0-29).`);
      if ((mcu === "picow" || mcu === "pico2w") && [23, 24, 25, 29].includes(val)) {
        warnings.push(`GP${val} (${label}) is connected to CYW43439 Wi-Fi chip.`);
      }
    } else if (mcu === "esp32" || mcu === "gendrv") {
      if (ESP32_FLASH_PINS.includes(val)) {
        errors.push(`GPIO ${val} (${label}) connects to internal SPI flash!`);
      }
      if (isOutput && ESP32_INPUT_ONLY_PINS.includes(val)) {
        errors.push(`GPIO ${val} (${label}) is INPUT-ONLY and cannot drive output.`);
      }
      if (!isOutput && ESP32_STRAPPING_PINS.includes(val)) {
        warnings.push(`GPIO ${val} (${label}) is a boot strapping pin.`);
      }
    } else if (mcu === "esp32s3") {
      if (!isOutput && ESP32S3_STRAPPING_PINS.includes(val)) {
        warnings.push(`GPIO ${val} (${label}) is an ESP32-S3 strapping pin.`);
      }
    }
  }

  checkPin("pin-led", "LED Pin", true);
  checkPin("pin-m1-p1", "Motor 1 PWM", true);
  checkPin("pin-m1-p2", "Motor 1 IN_A", true);
  checkPin("pin-m1-p3", "Motor 1 IN_B", true);

  checkPin("pin-m2-p1", "Motor 2 PWM", true);
  checkPin("pin-m2-p2", "Motor 2 IN_A", true);
  checkPin("pin-m2-p3", "Motor 2 IN_B", true);

  if (is4wd) {
    checkPin("pin-m3-p1", "Motor 3 PWM", true);
    checkPin("pin-m3-p2", "Motor 3 IN_A", true);
    checkPin("pin-m3-p3", "Motor 3 IN_B", true);

    checkPin("pin-m4-p1", "Motor 4 PWM", true);
    checkPin("pin-m4-p2", "Motor 4 IN_A", true);
    checkPin("pin-m4-p3", "Motor 4 IN_B", true);
  }

  checkPin("pin-enc-1a", "Encoder 1 A", false);
  checkPin("pin-enc-1b", "Encoder 1 B", false);
  checkPin("pin-enc-2a", "Encoder 2 A", false);
  checkPin("pin-enc-2b", "Encoder 2 B", false);

  if (is4wd) {
    checkPin("pin-enc-3a", "Encoder 3 A", false);
    checkPin("pin-enc-3b", "Encoder 3 B", false);
    checkPin("pin-enc-4a", "Encoder 4 A", false);
    checkPin("pin-enc-4b", "Encoder 4 B", false);
  }

  checkPin("pin-i2c-sda", "I2C SDA", false);
  checkPin("pin-i2c-scl", "I2C SCL", false);
  checkPin("pin-battery", "Battery ADC", false);
  checkPin("pin-sonar-trig", "Sonar Trig", true);
  checkPin("pin-sonar-echo", "Sonar Echo", false);

  // Duplicate pin check. One pin, one job -- except a shared enable line on a
  // two-PWM driver (BTS7960 type), which is the same job on several motors:
  // the firmware holds it HIGH once per motor, which is idempotent. The same
  // rule as scripts/pin_catalog.py, so the page and the flash agree:
  // pico2_mecanum enables all four bridges from GP22 and was flagged here.
  const drvType = document.getElementById("cfg-driver-type")?.value || "";
  for (const [pin, names] of Object.entries(assigned)) {
    if (names.length < 2) continue;
    if (drvType === "BTS7960" && names.every((n) => /^Motor \d PWM$/.test(n))) continue;
    errors.push(`GPIO Pin ${pin} is assigned to multiple devices: ${names.join(", ")}`);
  }

  // Update UI card
  const card = document.getElementById("hardware-safety-card");
  const dot = document.getElementById("safety-dot");
  const badge = document.getElementById("safety-badge");
  const msgBox = document.getElementById("safety-messages");

  if (!card || !dot || !badge || !msgBox) return;

  if (errors.length > 0) {
    dot.className = "status-dot status-err";
    badge.className = "safety-badge badge-err";
    badge.textContent = `${errors.length} Error${errors.length > 1 ? "s" : ""}`;
    msgBox.innerHTML = errors.map(e => `<div class="safety-msg-item safety-msg-err">❌ ${e}</div>`).join("");
  } else if (warnings.length > 0) {
    dot.className = "status-dot status-warn";
    badge.className = "safety-badge badge-warn";
    badge.textContent = `${warnings.length} Warning${warnings.length > 1 ? "s" : ""}`;
    msgBox.innerHTML = warnings.map(w => `<div class="safety-msg-item safety-msg-warn">⚠️ ${w}</div>`).join("");
  } else {
    dot.className = "status-dot status-ok";
    badge.className = "safety-badge badge-ok";
    badge.textContent = "0 Errors / 0 Warnings";
    msgBox.innerHTML = `<div class="safety-msg-item safety-msg-ok">✅ Conflict-free pin configuration verified for target microcontroller.</div>`;
  }
}

// The sonar enable select. "Disabled (Bare Module Default)" has to mean the
// pins go to -1, because a bare module that claims a sonar publishes /sonar
// from a pin nothing is wired to.
function sonarEnabled() {
  const el = document.getElementById("cfg-sonar");
  if (!el) {
    // No select on the page: fall back to the pins themselves.
    const t = parseInt(document.getElementById("pin-sonar-trig")?.value ?? -1, 10);
    const e = parseInt(document.getElementById("pin-sonar-echo")?.value ?? -1, 10);
    return t >= 0 && e >= 0;
  }
  return el.value === "true";
}

// Greys the pin fields out when the sensor is off, so the page cannot show a
// pin pair that will not be written.
function syncSonarFields() {
  const on = sonarEnabled();
  ["pin-sonar-trig", "pin-sonar-echo"].forEach((id) => {
    const el = document.getElementById(id);
    if (!el) return;
    el.disabled = !on;
    el.style.opacity = on ? "" : "0.45";
  });
}

async function saveCurrentHardwareConfig() {
  const activeController = controllerForSave(document.getElementById("cfg-mcu")?.value || "pico2");
  const kineType = document.getElementById("cfg-kinematics")?.value || "2wd";
  const driverType = document.getElementById("cfg-driver-type")?.value || "GENERIC_2_IN";
  const baudrate = parseInt(document.getElementById("cfg-baudrate")?.value || 921600, 10);
  const serialPort = document.getElementById("cfg-serial-port")?.value.trim() || "/dev/ttyACM0";

  const parsePin = (id, fallback = -1) => {
    const el = document.getElementById(id);
    if (!el) return fallback;
    const v = String(el.value || "").trim();
    if (v === "" || v === undefined || v === null) return -1;
    const n = parseInt(v, 10);
    return isNaN(n) ? -1 : n;
  };

  const is4wd = (kineType === "4wd" || kineType === "mecanum");

  // Motor 1. On a two-PWM driver (BTS7960 type) `pwm` is the ENABLE line,
  // shown as "EN" -- it used to be saved as -1 here, so pressing Save on
  // pico2_mecanum (GP22) or the GenDrv (GPIO 25/26) disabled every motor.
  const m1_pwm = parsePin("pin-m1-p1", -1);
  const m1_ina = driverType === "ESC" ? -1 : parsePin("pin-m1-p2", -1);
  const m1_inb = (driverType === "GENERIC_1_IN" || driverType === "ESC") ? -1 : parsePin("pin-m1-p3", -1);

  // Motor 2
  const m2_pwm = parsePin("pin-m2-p1", -1);
  const m2_ina = driverType === "ESC" ? -1 : parsePin("pin-m2-p2", -1);
  const m2_inb = (driverType === "GENERIC_1_IN" || driverType === "ESC") ? -1 : parsePin("pin-m2-p3", -1);

  // Motor 3 (unused if !is4wd)
  const m3_pwm = !is4wd ? -1 : parsePin("pin-m3-p1", -1);
  const m3_ina = (!is4wd || driverType === "ESC") ? -1 : parsePin("pin-m3-p2", -1);
  const m3_inb = (!is4wd || driverType === "GENERIC_1_IN" || driverType === "ESC") ? -1 : parsePin("pin-m3-p3", -1);

  // Motor 4 (unused if !is4wd)
  const m4_pwm = !is4wd ? -1 : parsePin("pin-m4-p1", -1);
  const m4_ina = (!is4wd || driverType === "ESC") ? -1 : parsePin("pin-m4-p2", -1);
  const m4_inb = (!is4wd || driverType === "GENERIC_1_IN" || driverType === "ESC") ? -1 : parsePin("pin-m4-p3", -1);

  // Encoders
  const enc1_a = parsePin("pin-enc-1a", -1);
  const enc1_b = parsePin("pin-enc-1b", -1);
  const enc2_a = parsePin("pin-enc-2a", -1);
  const enc2_b = parsePin("pin-enc-2b", -1);
  const enc3_a = !is4wd ? -1 : parsePin("pin-enc-3a", -1);
  const enc3_b = !is4wd ? -1 : parsePin("pin-enc-3b", -1);
  const enc4_a = !is4wd ? -1 : parsePin("pin-enc-4a", -1);
  const enc4_b = !is4wd ? -1 : parsePin("pin-enc-4b", -1);

  const payload = {
    controller: activeController,
    driver_type: driverType,
    baudrate: baudrate,
    serial_port: serialPort,
    console: document.getElementById("cfg-console")?.value || "usb",
    geometry: readGeometryForm(kineType),
    simulation: readSimForm(),
    kinematics: {
      base_type: kineType,
      wheel_diameter: parseFloat(document.getElementById("cfg-wheel-diameter")?.value || 0.152),
      lr_wheels_distance: parseFloat(document.getElementById("cfg-track-width")?.value || 0.271),
      fr_wheels_distance: parseFloat(document.getElementById("cfg-wheelbase")?.value || 0.0),
      max_rpm: parseInt(document.getElementById("cfg-max-rpm")?.value || 140, 10),
      max_rpm_ratio: parseFloat(document.getElementById("cfg-headroom")?.value || 0.85),
      counts_per_rev: parseInt(document.getElementById("cfg-cpr")?.value || 144000, 10),
      auto_nav2_limits: document.getElementById("cfg-nav2-auto")?.checked !== false,
      pwm_frequency: parseInt(document.getElementById("cfg-pwm-freq")?.value || 20000, 10),
      pwm_bits: parseInt(document.getElementById("cfg-pwm-bits")?.value || 10, 10),
      motor_operating_voltage: parseFloat(document.getElementById("cfg-motor-voltage")?.value || 24.0),
      motor_power_max_voltage: parseFloat(document.getElementById("cfg-motor-max-voltage")?.value || 12.0),
      pid: {
        kp: parseFloat(document.getElementById("cfg-pid-kp")?.value || 0.6),
        ki: parseFloat(document.getElementById("cfg-pid-ki")?.value || 0.8),
        kd: parseFloat(document.getElementById("cfg-pid-kd")?.value || 0.5)
      }
    },
    sensors: {
      // NONE, not SIM, when the control is absent -- and it is absent now that
      // I2C sensors are detected at boot. NONE means "not declared, find it";
      // SIM would mean "simulate one", which is a different robot.
      imu: document.getElementById("cfg-imu")?.value || "NONE",
      mag: document.getElementById("cfg-mag")?.value || "NONE",
      current: document.getElementById("cfg-battery")?.value || "NONE",
      env: document.getElementById("cfg-env")?.value || "AUTO",
      use_sim_imu: !!document.getElementById("chk-sim-imu")?.checked,
      use_sim_mag: !!document.getElementById("chk-sim-mag")?.checked,
      use_sim_wheel: !!document.getElementById("chk-sim-wheel")?.checked,
      use_sim_ld19: !!document.getElementById("chk-sim-ld19")?.checked,
      use_sim_env: !!document.getElementById("chk-sim-env")?.checked,
      use_sim_sonar: !!document.getElementById("chk-sim-sonar")?.checked,
      use_sim_battery: !!document.getElementById("chk-sim-battery")?.checked,
    },
    pins: {
      led: parsePin("pin-led", -1),
      motor1: {
        pwm: m1_pwm,
        in_a: m1_ina,
        in_b: m1_inb,
        invert: !!document.getElementById("pin-m1-inv")?.checked,
      },
      motor2: {
        pwm: m2_pwm,
        in_a: m2_ina,
        in_b: m2_inb,
        invert: !!document.getElementById("pin-m2-inv")?.checked,
      },
      motor3: {
        pwm: m3_pwm,
        in_a: m3_ina,
        in_b: m3_inb,
        invert: !!document.getElementById("pin-m3-inv")?.checked,
      },
      motor4: {
        pwm: m4_pwm,
        in_a: m4_ina,
        in_b: m4_inb,
        invert: !!document.getElementById("pin-m4-inv")?.checked,
      },
      encoder1: {
        pin_a: enc1_a,
        pin_b: enc1_b,
        invert: !!document.getElementById("pin-enc-1-inv")?.checked,
      },
      encoder2: {
        pin_a: enc2_a,
        pin_b: enc2_b,
        invert: !!document.getElementById("pin-enc-2-inv")?.checked,
      },
      encoder3: {
        pin_a: enc3_a,
        pin_b: enc3_b,
        invert: !!document.getElementById("pin-enc-3-inv")?.checked,
      },
      encoder4: {
        pin_a: enc4_a,
        pin_b: enc4_b,
        invert: !!document.getElementById("pin-enc-4-inv")?.checked,
      },
      i2c: {
        sda: parsePin("pin-i2c-sda", -1),
        scl: parsePin("pin-i2c-scl", -1),
      },
      battery: {
        pin: parsePin("pin-battery", -1),
        r1: parseFloat(document.getElementById("cfg-bat-r1")?.value || 30000),
        r2: parseFloat(document.getElementById("cfg-bat-r2")?.value || 7500),
        min_v: parseFloat(document.getElementById("cfg-bat-min")?.value || 0),
        max_v: parseFloat(document.getElementById("cfg-bat-max")?.value || 0),
        capacity_ah: parseFloat(document.getElementById("cfg-bat-cap")?.value || 0),
        dip_pct: parseFloat(document.getElementById("cfg-bat-dip")?.value || 2),
        nominal_v: parseFloat(document.getElementById("cfg-bat-nom")?.value || 0),
        filter_cap_pf: parseFloat(document.getElementById("cfg-bat-cap-val")?.value || 0),
      },
      // "Disabled" means disabled: a bare module must not come up claiming a
      // sonar, so the select forces both pins to -1 rather than leaving
      // whatever the pin fields happen to hold.
      sonar: sonarEnabled()
        ? { trigger: parsePin("pin-sonar-trig", -1), echo: parsePin("pin-sonar-echo", -1) }
        : { trigger: -1, echo: -1 },
      // The DAC pin adc_calibrate sweeps; the firmware reads it from the env
      // (envU16("dac_pin", DAC_PIN) in firmware/src/tools/adc_calibrate.cpp).
      dac: parseInt(document.getElementById("cfg-dac-pin")?.value ?? -1, 10),
    }
  };

  const magBias = ["cfg-mag-bias-x", "cfg-mag-bias-y", "cfg-mag-bias-z"]
    .map((id) => (document.getElementById(id)?.value ?? "").trim());
  if (magBias.some((v) => v !== "")) {
    payload.sensors = Object.assign({}, payload.sensors, {
      mag_bias: magBias.map((v) => (v === "" ? 0 : parseFloat(v))),
    });
  }

  logLine(`[config-engine] Saving hardware configuration for [${activeController}]...`);
  try {
    const res = await fetch("/api/hardware/config", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(payload)
    });
    const result = await res.json();
    if (result.success) {
      logLine(`${result.header_ok === false ? "⚠️" : "✅"} ${result.message}`);
      (result.pin_findings || []).forEach((f) => logLine(`[pins] ${f.level}: ${f.message}`));
      renderPinFindings(result.pin_findings, result.header_ok !== false);
      renderGeometryWarnings(result.geometry_warnings);
      if (result.urdf_ok === false) logLine(`⚠️ URDF not generated: ${result.urdf_error || "unknown error"}`);
      else if (result.urdf_path) logLine(`[urdf] ${result.urdf_path} regenerated`);
      if (result.header_ok === false) {
        document.querySelector('.tab-btn[data-tab="pin-matrix"]')?.click();
        document.getElementById("pin-findings")?.scrollIntoView({ block: "start" });
      }
    } else {
      logLine(`❌ Error saving hardware config: ${result.error || "Unknown error"}`);
    }
  } catch (err) {
    logLine(`❌ Failed to save configuration: ${err.message}`);
  }
}

function updateBringupSummary() {
  const kine = document.getElementById("cfg-kinematics")?.value || document.getElementById("bringup-base-type")?.value || "2wd";
  const port = document.getElementById("cfg-serial-port")?.value || document.getElementById("bringup-agent-device")?.value || "/dev/ttyACM0";
  const baud = document.getElementById("cfg-baudrate")?.value || document.getElementById("bringup-agent-baud")?.value || "921600";
  const laser = document.getElementById("cfg-laser-sensor")?.value || document.getElementById("laser-driver-model")?.value || document.getElementById("bringup-laser-sensor")?.value || "ld19";
  const depth = document.getElementById("cfg-depth-sensor")?.value || document.getElementById("bringup-depth-sensor")?.value || "";

  const elBase = document.getElementById("summary-bringup-base");
  if (elBase) {
    elBase.textContent = kine.toUpperCase() + (kine === "2wd" ? " (Differential)" : kine === "4wd" ? " (Skid Steer)" : " (Omni)");
  }

  const elPort = document.getElementById("summary-bringup-port");
  if (elPort) {
    elPort.textContent = `${port} @ ${baud}`;
  }

  const elLaser = document.getElementById("summary-bringup-laser");
  if (elLaser) {
    elLaser.textContent = laser ? laser.toUpperCase() : "None";
  }

  const elDepth = document.getElementById("summary-bringup-depth");
  if (elDepth) {
    elDepth.textContent = depth ? depth.toUpperCase() : "None";
  }
}

function syncMcuSerialSettings() {
  const basePort = document.getElementById("cfg-serial-port")?.value.trim() || "/dev/ttyACM0";
  const baseBaud = document.getElementById("cfg-baudrate")?.value || "921600";
  const mcu = document.getElementById("cfg-mcu")?.value || "pico2";

  const setVal = (id, val) => {
    const el = document.getElementById(id);
    if (el) el.value = val;
  };

  setVal("hw-flash-port", basePort);
  setVal("hw-flash-env", mcu);
  setVal("bringup-agent-device", basePort);
  setVal("bringup-agent-baud", baseBaud);
  setVal("cfg-agent-device", basePort);
  setVal("cfg-agent-baud", baseBaud);
  setVal("docker-base-serial-port", basePort);

  updateBringupSummary();
}

// ---------------------------------------------------------------------------
// Serial port chips and MCU auto-detection (this robot computer's USB bus)
// ---------------------------------------------------------------------------
function renderPortChips(ports, whereLabel) {
  const containers = [
    document.getElementById("base-port-chips"),
    document.getElementById("hw-port-chips")
  ].filter(Boolean);
  containers.forEach(container => {
    container.innerHTML = "";
    (ports || []).forEach(p => {
      const path = p.path || p.port || String(p);
      // udev's by-id name is what gets saved, because /dev/ttyACM0 is
      // enumeration order and not a board: two Picos on one bench swapped
      // numbers overnight and a flash went at the wrong one. The chip still
      // shows the short tty; the stable name is what the click puts in the
      // config, and the tooltip says both.
      const byId = p.by_id || "";
      const chip = document.createElement("span");
      chip.className = "port-chip";
      chip.textContent = path + (byId ? " ·id" : "");
      chip.title = `${p.chip || p.product || "USB serial"} on ${whereLabel || "this machine"}`
        + (p.busy ? " (in use)" : "")
        + (byId ? ` — click to select its stable name ${byId} (follows this board across reboots)`
                : " — click to select (no udev by-id name for this port)");
      chip.addEventListener("click", () => {
        const inp = document.getElementById("cfg-serial-port");
        if (inp) inp.value = byId || path;
        syncMcuSerialSettings();
      });
      container.appendChild(chip);
    });
  });
}

function applyDetectedMcu(sug, data) {
  // The bus wins over the config, for the same reason the I2C bus does
  // (AGENTS.md §5): the YAML is a claim about the hardware and the tty is the
  // hardware. The configured port is kept when the robot actually has it --
  // switching a correct value to "the first port found" would be noise on a
  // two-board bench.
  const portEl = document.getElementById("cfg-serial-port");
  if (portEl) {
    const present = (data.ports || []).some(p => (p.path || p.port) === portEl.value.trim());
    if (!present && sug.port) portEl.value = sug.port;
  }
  const mcuEl = document.getElementById("cfg-mcu");
  if (mcuEl && sug.mcu) {
    const hasOption = Array.from(mcuEl.options).some(o => o.value === sug.mcu);
    // pico2 vs pico2w (and esp32 vs gendrv) cannot be told apart over USB, so a
    // wireless or board-specific variant already chosen is never downgraded to
    // the family the VID:PID names.
    const variantOf = { pico2: ["pico2", "pico2w"], pico: ["pico", "picow"],
                        esp32: ["esp32", "gendrv"], esp32s3: ["esp32s3"] };
    const family = variantOf[sug.mcu] || [sug.mcu];
    if (hasOption && !family.includes(mcuEl.value)) {
      mcuEl.value = sug.mcu;
      mcuEl.dispatchEvent(new Event("change"));
    }
  }
  syncMcuSerialSettings();
}

async function refreshHwSerialPorts() {
  try {
    const res = await fetch("/api/serial_ports");
    if (!res.ok) return;
    const data = await res.json();
    // The bus a run will flash is this machine's: the board is plugged into
    // the robot computer, and that is where the Cockpit runs.
    const ports = data.ports || [];
    const detail = data.local_ports || ports.map(path => ({ path }));
    renderPortChips(detail, "this robot computer");
    if (data.suggested && data.suggested.port) applyDetectedMcu(data.suggested, { ports: detail });

    if (ports.length > 0 && document.getElementById("cfg-serial-port")) {
      const current = document.getElementById("cfg-serial-port").value;
      if (!current || (current === "/dev/ttyACM0" && !ports.includes("/dev/ttyACM0"))) {
        document.getElementById("cfg-serial-port").value = ports[0];
        syncMcuSerialSettings();
      }
    }
  } catch (err) {
    console.error("[HardwareConfig] Error refreshing serial ports:", err);
  }
}

async function executeHardwareAction(action, customFirmware = null) {
  // This selects which APPLICATION the one image boots. It is the `app` key of
  // the env partition, not a directory and not a separate build (test_sensors,
  // base) -- not a controller and not a nav goal.
  const firmwareName = customFirmware || document.getElementById("hw-flash-target")?.value || "test_sensors";
  const port = document.getElementById("cfg-serial-port")?.value.trim() || document.getElementById("hw-flash-port")?.value.trim() || "/dev/ttyACM0";
  const mcuEnv = document.getElementById("hw-flash-env")?.value || document.getElementById("cfg-mcu")?.value || "pico2";
  const baud = parseInt(document.getElementById("cfg-baudrate")?.value || 921600, 10);

  const btnStop = document.getElementById("btn-hw-stop");
  if (btnStop) btnStop.style.display = "inline-block";

  logLine(`========================================================================`);
  logLine(`🚀 [Hardware Test] Action: ${action.toUpperCase()} | Firmware: ${firmwareName} | Env: ${mcuEnv} | Port: ${port} @ ${baud}`);
  logLine(`========================================================================`);

  if (action === "upload" || action === "monitor") {
    logLine(`🛑 [Safe Port Protocol] Checking micro-ROS agent and releasing port ${port}...`);
    try {
      await fetch("/api/agent/port_release", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ port: port, mode: "serial" })
      });
      // /api/agent/stop has never existed; the endpoint is /api/agent/kill
      // (web/backend/main.py). The 404 landed in the catch below as a
      // console.warn, so the agent was left holding the port on every upload
      // and monitor from this panel -- which is exactly the state the "Safe
      // Port Protocol" line above claims to have cleared.
      await fetch("/api/agent/kill", { method: "POST" });
    } catch (err) {
      console.warn("Pre-action port release warning:", err);
    }
  }

  try {
    const res = await fetch("/api/hardware/test", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({
        firmware: firmwareName,
        port: port,
        mcu_env: mcuEnv,
        action: action,
        baud: baud
      })
    });

    if (!res.ok) {
      const err = await res.json();
      logLine(`❌ Error triggering hardware action: ${err.detail || res.statusText}`);
      if (btnStop) btnStop.style.display = "none";
      return;
    }

    const data = await res.json();
    await runCommand({ handle: data.handle }, {
      slot: "main",
      title: `${action === "upload" ? "Flashing" : action === "monitor" ? "Monitoring" : "Building"} ${firmwareName} (${mcuEnv})`,
      // adc_calibrate ends by printing one [ADC_JSON] line: the curve it just
      // measured. Catch it as it streams past and draw it. flash_mcu also prints
      // one "NEXT ACTION: ..." line when a flash fails -- surface it in a banner
      // that outlives the (default-collapsed) console, so a beginner sees the one
      // thing to do next instead of hunting grey monospace at the bottom.
      onLine: (line) => {
        adcCurveFromLine(line);
        const na = /NEXT ACTION: (.+)$/.exec(line || "");
        if (na) showActionBanner(`Flashing failed. Next: ${na[1].trim()}`,
                                 "The full recovery steps are in the Output console.");
      },
      onDone: (exitCode) => {
        if (btnStop) btnStop.style.display = "none";
        if (exitCode === 0) {
          hideActionBanner();
          logLine(`✅ [Hardware Test] ${firmwareName} completed successfully.`);
          showToast(`✅ ${firmwareName} flashed successfully!`, 4000);
          // A diagnostic application exists to be READ. Flashing one and then
          // showing nothing is what made these tools look broken: they print
          // to serial, and until now nothing opened the port afterwards. The
          // base controller is the exception -- its serial belongs to
          // micro-ROS, and the agent needs it.
          if (action === "upload" && firmwareName !== "base") {
            logLine(`📡 [Hardware Test] streaming ${firmwareName} output — press Stop when done.`);
            executeHardwareAction("monitor", firmwareName);
          }
        } else {
          logLine(`❌ [Hardware Test] ${firmwareName} finished with exit code ${exitCode}. Check terminal above for troubleshooting & recovery steps.`);
          showToast(`❌ Flashing ${firmwareName} failed. Check terminal for recovery steps!`, 7000);
        }
      }
    });
  } catch (err) {
    logLine(`❌ Hardware test execution failed: ${err.message}`);
    if (btnStop) btnStop.style.display = "none";
  }
}

