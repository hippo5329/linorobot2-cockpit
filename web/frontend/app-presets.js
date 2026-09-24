// Linorobot2 Cockpit frontend -- reference build presets & hardware design engine, simulation-mode.
// Part of the app.js split: a classic script sharing global scope. See app-core.js.

// ==============================================================================
// Reference Build Presets & Hardware Design Engine
// ==============================================================================

const REFERENCE_DESIGNS = {
  pico2: [
    {
      id: "bare_pico2",
      name: "🧩 Bare Module (RP2350) — all pins N/C (Safe Default / Sim Mode)",
      mcu: "pico2",
      kinematics: "2wd",
      driver: "BTS7960",
      wheel_diameter: 0.1,
      lr_wheels_distance: 0.271,
      fr_wheels_distance: 0.0,
      max_rpm: 140,
      cpr: 4000,
      operating_voltage: 12.0,
      imu: "SIM",
      mag: "NONE",
      use_sim_imu: true,
      use_sim_mag: true,
      use_sim_wheel: true,
      use_sim_ld19: true,
      pins: {
        led: 25,
        motor1: { in_a: -1, in_b: -1, pwm: -1 },
        motor2: { in_a: -1, in_b: -1, pwm: -1 },
        motor3: { in_a: -1, in_b: -1, pwm: -1 },
        motor4: { in_a: -1, in_b: -1, pwm: -1 },
        encoder1: { a: -1, b: -1 },
        encoder2: { a: -1, b: -1 },
        encoder3: { a: -1, b: -1 },
        encoder4: { a: -1, b: -1 },
        i2c: { sda: -1, scl: -1 },
        battery: -1,
        sonar: { trig: -1, echo: -1 },
      }
    },
    {
      id: "pico2_diff",
      name: "⚡ Pico 2 · 2WD Differential · BTS7960 (Simulated IMU Default)",
      mcu: "pico2",
      kinematics: "2wd",
      driver: "BTS7960",
      wheel_diameter: 0.1,
      lr_wheels_distance: 0.271,
      fr_wheels_distance: 0.0,
      max_rpm: 140,
      cpr: 4000,
      operating_voltage: 12.0,
      imu: "SIM",
      mag: "NONE",
      use_sim_imu: true,
      use_sim_mag: true,
      use_sim_wheel: false,
      use_sim_ld19: true,
      pins: {
        led: 25,
        motor1: { in_a: 10, in_b: 11, pwm: 12 },
        motor2: { in_a: 13, in_b: 14, pwm: 15 },
        motor3: { in_a: -1, in_b: -1, pwm: -1 },
        motor4: { in_a: -1, in_b: -1, pwm: -1 },
        encoder1: { a: 2, b: 3 },
        encoder2: { a: 4, b: 5 },
        encoder3: { a: -1, b: -1 },
        encoder4: { a: -1, b: -1 },
        i2c: { sda: 0, scl: 1 },
        battery: 26,
        sonar: { trig: 22, echo: 27 },
      }
    },
    {
      id: "pico2_mecanum",
      name: "⚡ Pico 2 · 4WD Mecanum · BTS7960 (Simulated IMU)",
      mcu: "pico2",
      kinematics: "mecanum",
      driver: "BTS7960",
      wheel_diameter: 0.1,
      lr_wheels_distance: 0.271,
      fr_wheels_distance: 0.18,
      max_rpm: 140,
      cpr: 4000,
      operating_voltage: 12.0,
      imu: "SIM",
      mag: "NONE",
      use_sim_imu: true,
      use_sim_mag: true,
      use_sim_wheel: false,
      use_sim_ld19: true,
      pins: {
        led: 25,
        motor1: { in_a: 6, in_b: 7, pwm: 8 },
        motor2: { in_a: 9, in_b: 10, pwm: 11 },
        motor3: { in_a: 12, in_b: 13, pwm: 14 },
        motor4: { in_a: 15, in_b: 16, pwm: 17 },
        encoder1: { a: 0, b: 1 },
        encoder2: { a: 2, b: 3 },
        encoder3: { a: 4, b: 5 },
        encoder4: { a: 18, b: 19 },
        i2c: { sda: 20, scl: 21 },
        battery: 26,
        sonar: { trig: 22, echo: 27 },
      }
    },
    {
      id: "scout_pico2",
      name: "Raspberry Pi Pico 2 (2WD Diff + TB6612 / L298N)",
      mcu: "pico2",
      kinematics: "2wd",
      driver: "GENERIC_2_IN",
      wheel_diameter: 0.1,
      lr_wheels_distance: 0.271,
      fr_wheels_distance: 0.0,
      max_rpm: 140,
      cpr: 4000,
      operating_voltage: 12.0,
      imu: "SIM",
      mag: "NONE",
      use_sim_imu: true,
      use_sim_mag: true,
      use_sim_wheel: false,
      use_sim_ld19: true,
      pins: {
        led: 25,
        motor1: { in_a: 11, in_b: 12, pwm: 10 },
        motor2: { in_a: 14, in_b: 15, pwm: 13 },
        motor3: { in_a: -1, in_b: -1, pwm: -1 },
        motor4: { in_a: -1, in_b: -1, pwm: -1 },
        encoder1: { a: 2, b: 3 },
        encoder2: { a: 4, b: 5 },
        encoder3: { a: -1, b: -1 },
        encoder4: { a: -1, b: -1 },
        i2c: { sda: 0, scl: 1 },
        battery: 26,
        sonar: { trig: 22, echo: 27 },
      }
    },
    {
      id: "mech_pico2",
      name: "Raspberry Pi Pico 2 (4WD Mecanum + TB6612)",
      mcu: "pico2",
      kinematics: "mecanum",
      driver: "GENERIC_2_IN",
      wheel_diameter: 0.1,
      lr_wheels_distance: 0.271,
      fr_wheels_distance: 0.18,
      max_rpm: 140,
      cpr: 4000,
      operating_voltage: 12.0,
      imu: "SIM",
      mag: "NONE",
      use_sim_imu: true,
      use_sim_mag: true,
      use_sim_wheel: false,
      use_sim_ld19: true,
      pins: {
        led: 25,
        motor1: { in_a: 11, in_b: 12, pwm: 10 },
        motor2: { in_a: 14, in_b: 15, pwm: 13 },
        motor3: { in_a: 17, in_b: 18, pwm: 16 },
        motor4: { in_a: 20, in_b: 21, pwm: 19 },
        encoder1: { a: 2, b: 3 },
        encoder2: { a: 4, b: 5 },
        encoder3: { a: 6, b: 7 },
        encoder4: { a: 8, b: 9 },
        i2c: { sda: 0, scl: 1 },
        battery: 26,
        sonar: { trig: 22, echo: 27 },
      }
    }
  ],
  pico: [
    {
      id: "bare_pico",
      name: "🧩 Bare Module (RP2040) — all pins N/C (Safe Default / Sim Mode)",
      mcu: "pico",
      kinematics: "2wd",
      driver: "BTS7960",
      wheel_diameter: 0.1,
      lr_wheels_distance: 0.271,
      fr_wheels_distance: 0.0,
      max_rpm: 140,
      cpr: 4000,
      operating_voltage: 12.0,
      imu: "SIM",
      mag: "NONE",
      use_sim_imu: true,
      use_sim_mag: true,
      use_sim_wheel: true,
      use_sim_ld19: true,
      pins: {
        led: 25,
        motor1: { in_a: -1, in_b: -1, pwm: -1 },
        motor2: { in_a: -1, in_b: -1, pwm: -1 },
        motor3: { in_a: -1, in_b: -1, pwm: -1 },
        motor4: { in_a: -1, in_b: -1, pwm: -1 },
        encoder1: { a: -1, b: -1 },
        encoder2: { a: -1, b: -1 },
        encoder3: { a: -1, b: -1 },
        encoder4: { a: -1, b: -1 },
        i2c: { sda: -1, scl: -1 },
        battery: -1,
        sonar: { trig: -1, echo: -1 },
      }
    },
    {
      id: "scout_pico",
      name: "Raspberry Pi Pico (2WD Diff + TB6612 / L298N)",
      mcu: "pico",
      kinematics: "2wd",
      driver: "GENERIC_2_IN",
      wheel_diameter: 0.1,
      lr_wheels_distance: 0.271,
      fr_wheels_distance: 0.0,
      max_rpm: 140,
      cpr: 4000,
      operating_voltage: 12.0,
      imu: "SIM",
      mag: "NONE",
      use_sim_imu: true,
      use_sim_mag: true,
      use_sim_wheel: false,
      use_sim_ld19: true,
      pins: {
        led: 25,
        motor1: { in_a: 11, in_b: 12, pwm: 10 },
        motor2: { in_a: 14, in_b: 15, pwm: 13 },
        motor3: { in_a: -1, in_b: -1, pwm: -1 },
        motor4: { in_a: -1, in_b: -1, pwm: -1 },
        encoder1: { a: 2, b: 3 },
        encoder2: { a: 4, b: 5 },
        encoder3: { a: -1, b: -1 },
        encoder4: { a: -1, b: -1 },
        i2c: { sda: 0, scl: 1 },
        battery: 26,
        sonar: { trig: 22, echo: 27 },
      }
    }
  ],
  esp32: [
    {
      id: "bare_esp32",
      name: "🧩 Bare Module (ESP32) — all pins N/C (Safe Default / Sim Mode)",
      mcu: "esp32",
      kinematics: "2wd",
      driver: "BTS7960",
      wheel_diameter: 0.1,
      lr_wheels_distance: 0.271,
      fr_wheels_distance: 0.0,
      max_rpm: 140,
      cpr: 4000,
      operating_voltage: 12.0,
      imu: "SIM",
      mag: "NONE",
      use_sim_imu: true,
      use_sim_mag: true,
      use_sim_wheel: true,
      use_sim_ld19: true,
      pins: {
        led: 2,
        motor1: { in_a: -1, in_b: -1, pwm: -1 },
        motor2: { in_a: -1, in_b: -1, pwm: -1 },
        motor3: { in_a: -1, in_b: -1, pwm: -1 },
        motor4: { in_a: -1, in_b: -1, pwm: -1 },
        encoder1: { a: -1, b: -1 },
        encoder2: { a: -1, b: -1 },
        encoder3: { a: -1, b: -1 },
        encoder4: { a: -1, b: -1 },
        i2c: { sda: -1, scl: -1 },
        battery: -1,
        sonar: { trig: -1, echo: -1 },
      }
    },
    {
      id: "waveshare_gendrv",
      name: "Waveshare General Driver Board (ESP32 + BTS7960 + QMI8658)",
      mcu: "gendrv",
      kinematics: "2wd",
      driver: "BTS7960",
      wheel_diameter: 0.1,
      lr_wheels_distance: 0.271,
      fr_wheels_distance: 0.0,
      max_rpm: 140,
      cpr: 4000,
      operating_voltage: 12.0,
      imu: "QMI8658",
      mag: "AK09918",
      use_sim_imu: false,
      use_sim_mag: false,
      use_sim_wheel: false,
      use_sim_ld19: false,
      pins: {
        led: -1,
        motor1: { in_a: 17, in_b: 21, pwm: -1 },
        motor2: { in_a: 23, in_b: 22, pwm: -1 },
        motor3: { in_a: -1, in_b: -1, pwm: -1 },
        motor4: { in_a: -1, in_b: -1, pwm: -1 },
        encoder1: { a: 34, b: 35 },
        encoder2: { a: 16, b: 27 },
        encoder3: { a: -1, b: -1 },
        encoder4: { a: -1, b: -1 },
        i2c: { sda: 32, scl: 33 },
        battery: -1,
        sonar: { trig: -1, echo: -1 },
      }
    },
    {
      id: "mech_esp32",
      name: "ESP32 DevKit (4WD Mecanum + WiFi UDP + BNO085)",
      mcu: "esp32",
      kinematics: "mecanum",
      driver: "GENERIC_2_IN",
      wheel_diameter: 0.1,
      lr_wheels_distance: 0.271,
      fr_wheels_distance: 0.18,
      max_rpm: 140,
      cpr: 4000,
      operating_voltage: 12.0,
      imu: "BNO085",
      mag: "NONE",
      use_sim_imu: false,
      use_sim_mag: true,
      use_sim_wheel: false,
      use_sim_ld19: true,
      pins: {
        led: 2,
        motor1: { in_a: 14, in_b: 27, pwm: 13 },
        motor2: { in_a: 26, in_b: 33, pwm: 25 },
        motor3: { in_a: 19, in_b: 23, pwm: 18 },
        motor4: { in_a: 16, in_b: 17, pwm: 15 },
        encoder1: { a: 34, b: 35 },
        encoder2: { a: 36, b: 39 },
        encoder3: { a: 4, b: 32 },
        encoder4: { a: 5, b: 12 },
        i2c: { sda: 21, scl: 22 },
        battery: 36,
        sonar: { trig: 0, echo: 0 },
      }
    }
  ],
  esp32s3: [
    {
      id: "bare_esp32s3",
      name: "🧩 Bare Module (ESP32-S3) — all pins N/C (Safe Default / Sim Mode)",
      mcu: "esp32s3",
      kinematics: "2wd",
      driver: "BTS7960",
      wheel_diameter: 0.1,
      lr_wheels_distance: 0.271,
      fr_wheels_distance: 0.0,
      max_rpm: 140,
      cpr: 4000,
      operating_voltage: 12.0,
      imu: "SIM",
      mag: "NONE",
      use_sim_imu: true,
      use_sim_mag: true,
      use_sim_wheel: true,
      use_sim_ld19: true,
      pins: {
        led: 48,
        motor1: { in_a: -1, in_b: -1, pwm: -1 },
        motor2: { in_a: -1, in_b: -1, pwm: -1 },
        motor3: { in_a: -1, in_b: -1, pwm: -1 },
        motor4: { in_a: -1, in_b: -1, pwm: -1 },
        encoder1: { a: -1, b: -1 },
        encoder2: { a: -1, b: -1 },
        encoder3: { a: -1, b: -1 },
        encoder4: { a: -1, b: -1 },
        i2c: { sda: -1, scl: -1 },
        battery: -1,
        sonar: { trig: -1, echo: -1 },
      }
    },
    {
      id: "yb_eet01",
      name: "Yahboom microROS Control Board (ESP32-S3, YB-EET01-V2.0)",
      mcu: "esp32s3",
      kinematics: "2wd",
      console: "uart0",
      driver: "BTS7960",
      wheel_diameter: 0.1,
      lr_wheels_distance: 0.271,
      fr_wheels_distance: 0,
      max_rpm: 140,
      cpr: 4000,
      operating_voltage: 8.4,
      imu: "ICM42670",
      mag: "NONE",
      use_sim_imu: false,
      use_sim_mag: true,
      use_sim_wheel: false,
      use_sim_ld19: true,
      pins: {
        led: 45,
        motor1: { in_a: 4, in_b: 5, pwm: -1 },
        motor2: { in_a: 15, in_b: 16, pwm: -1 },
        encoder1: { a: 6, b: 7 },
        encoder2: { a: 47, b: 48 },
        i2c: { sda: 40, scl: 39 },
        battery: 3,
        sonar: { trig: -1, echo: -1 },
      }
    },
    {
      id: "crawler_esp32s3",
      name: "ESP32-S3 (4WD Skid + TB6612 + Sonar + ADC)",
      mcu: "esp32s3",
      kinematics: "4wd",
      driver: "GENERIC_2_IN",
      wheel_diameter: 0.1,
      lr_wheels_distance: 0.271,
      fr_wheels_distance: 0.18,
      max_rpm: 140,
      cpr: 4000,
      operating_voltage: 12.0,
      imu: "MPU6050",
      mag: "NONE",
      use_sim_imu: false,
      use_sim_mag: true,
      use_sim_wheel: false,
      use_sim_ld19: true,
      pins: {
        led: 48,
        motor1: { in_a: 2, in_b: 4, pwm: 1 },
        motor2: { in_a: 6, in_b: 7, pwm: 5 },
        motor3: { in_a: 9, in_b: 10, pwm: 8 },
        motor4: { in_a: 12, in_b: 13, pwm: 11 },
        encoder1: { a: 14, b: 15 },
        encoder2: { a: 16, b: 17 },
        encoder3: { a: 18, b: 21 },
        encoder4: { a: 38, b: 39 },
        i2c: { sda: 41, scl: 42 },
        battery: 3,
        sonar: { trig: 47, echo: 40 },
      }
    }
  ]
};

function normalizeMcuFamily(mcu) {
  if (!mcu) return "pico2";
  const s = String(mcu).toLowerCase();
  if (s.includes("pico2") || s.includes("rp2350")) return "pico2";
  if (s.includes("pico") || s.includes("rp2040")) return "pico";
  if (s.includes("s3") || s.includes("esp32s3")) return "esp32s3";
  if (s.includes("esp32") || s.includes("gendrv")) return "esp32";
  return "pico2";
}

function updateReferenceDesigns(mcuHint) {
  const select = document.getElementById("preset-select");
  if (!select) return;
  const family = normalizeMcuFamily(mcuHint);
  const currentVal = select.value;

  const mcuLabels = {
    pico2: "Raspberry Pi Pico 2 (RP2350)",
    pico: "Raspberry Pi Pico (RP2040)",
    esp32: "ESP32 / GenDrv",
    esp32s3: "ESP32-S3",
  };

  let html = "";
  const primaryDesigns = REFERENCE_DESIGNS[family] || [];
  html += `<optgroup label="⚡ Detected MCU: ${mcuLabels[family] || family.toUpperCase()}">`;
  for (const d of primaryDesigns) {
    html += `<option value="${d.id}">${d.name}</option>`;
  }
  html += `</optgroup>`;

  html += `<optgroup label="🌐 Other MCU Architectures">`;
  for (const [fKey, list] of Object.entries(REFERENCE_DESIGNS)) {
    if (fKey === family) continue;
    for (const d of list) {
      html += `<option value="${d.id}">${d.name}</option>`;
    }
  }
  html += `</optgroup>`;

  select.innerHTML = html;
  const hasCurrent = select.querySelector(`option[value="${currentVal}"]`);
  if (hasCurrent) {
    select.value = currentVal;
  } else if (primaryDesigns.length > 0) {
    select.value = primaryDesigns[0].id;
  }
}

async function applyReferenceDesign(designId) {
  let found = null;
  for (const list of Object.values(REFERENCE_DESIGNS)) {
    found = list.find((d) => d.id === designId);
    if (found) break;
  }
  if (!found) return;

  // Base & MCU. A preset that changes the controller must change it
  // everywhere, or it silently splits the two selects apart -- see
  // syncControllerSelects.
  const mcuSel = document.getElementById("cfg-mcu");
  if (mcuSel && found.mcu) {
    mcuSel.value = found.mcu;
    if (window.__syncControllerSelects) window.__syncControllerSelects(found.mcu, "cfg-mcu");
  }

  const baseSel = document.getElementById("cfg-kinematics");
  if (baseSel && found.kinematics) baseSel.value = found.kinematics;

  const setVal = (id, val) => {
    const el = document.getElementById(id);
    if (el && val !== undefined) el.value = val;
  };
  setVal("cfg-console", found.console || "usb");
  setVal("cfg-wheel-diameter", found.wheel_diameter);
  setVal("cfg-track-width", found.lr_wheels_distance);
  setVal("cfg-wheelbase", found.fr_wheels_distance);
  setVal("cfg-max-rpm", found.max_rpm);
  setVal("cfg-cpr", found.cpr);
  setVal("cfg-motor-voltage", found.operating_voltage);

  // Drive & Motors
  const drvSel = document.getElementById("cfg-driver-type");
  if (drvSel && found.driver) drvSel.value = found.driver;

  // Sensors
  const imuSel = document.getElementById("cfg-imu");
  if (imuSel && found.imu) imuSel.value = found.imu;

  const magSel = document.getElementById("cfg-mag");
  if (magSel && found.mag) magSel.value = found.mag;

  const setChk = (id, val) => {
    const el = document.getElementById(id);
    if (el && val !== undefined) el.checked = !!val;
  };
  setChk("chk-sim-imu", found.use_sim_imu);
  setChk("chk-sim-mag", found.use_sim_mag);
  setChk("chk-sim-wheel", found.use_sim_wheel);
  setChk("chk-sim-ld19", found.use_sim_ld19);

  // Pins
  if (found.pins) {
    setVal("pin-led", found.pins.led);
    setVal("pin-m1-p1", found.pins.motor1?.pwm);
    setVal("pin-m1-p2", found.pins.motor1?.in_a);
    setVal("pin-m1-p3", found.pins.motor1?.in_b);

    setVal("pin-m2-p1", found.pins.motor2?.pwm);
    setVal("pin-m2-p2", found.pins.motor2?.in_a);
    setVal("pin-m2-p3", found.pins.motor2?.in_b);

    setVal("pin-m3-p1", found.pins.motor3?.pwm);
    setVal("pin-m3-p2", found.pins.motor3?.in_a);
    setVal("pin-m3-p3", found.pins.motor3?.in_b);

    setVal("pin-m4-p1", found.pins.motor4?.pwm);
    setVal("pin-m4-p2", found.pins.motor4?.in_a);
    setVal("pin-m4-p3", found.pins.motor4?.in_b);

    setVal("pin-enc-1a", found.pins.encoder1?.a);
    setVal("pin-enc-1b", found.pins.encoder1?.b);
    setVal("pin-enc-2a", found.pins.encoder2?.a);
    setVal("pin-enc-2b", found.pins.encoder2?.b);
    setVal("pin-enc-3a", found.pins.encoder3?.a);
    setVal("pin-enc-3b", found.pins.encoder3?.b);
    setVal("pin-enc-4a", found.pins.encoder4?.a);
    setVal("pin-enc-4b", found.pins.encoder4?.b);

    setVal("pin-i2c-sda", found.pins.i2c?.sda);
    setVal("pin-i2c-scl", found.pins.i2c?.scl);
    setVal("pin-battery", found.pins.battery);
    setVal("pin-sonar-trig", found.pins.sonar?.trig);
    setVal("pin-sonar-echo", found.pins.sonar?.echo);
  }

  const mcuTarget = found.mcu || "pico2";
  const defaultPort = mcuTarget.includes("pico") ? "/dev/ttyACM0" : "/dev/ttyUSB0";
  const portToUse = found.serial_port || defaultPort;
  setVal("cfg-serial-port", portToUse);
  setVal("cfg-baudrate", 921600);
  syncMcuSerialSettings();

  updateKinematicsVisibility();
  updateMotorPinVisibility();
  updateKinematicsHUD();
  updateAdcCalculations();
  validateHardwareSafety();

  await saveCurrentHardwareConfig();
  showToast(`⚡ Reference Build Loaded: ${found.name}`);
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

  const hdrMode = document.getElementById("hdr-pipeline-mode");
  const cockpitMode = document.getElementById("cockpit-pipeline-mode");
  if (hdrMode) hdrMode.value = enabled ? "sim" : "real";
  if (cockpitMode) cockpitMode.value = enabled ? "sim" : "real";

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
      descElem.innerHTML = `Linorobot2 defaults to safe <b>Sim Mode</b> simulation. Embedded firmware generates synthetic wheel encoder ticks (<code>USE_SIM_WHEEL</code>), simulated 6-DOF IMU quaternion telemetry (<code>USE_SIM_IMU</code>), and simulated planar LiDAR scans (<code>USE_SIM_LD19</code>). This enables complete end-to-end Map, SLAM, and Nav2 testing on a bare MCU module before physical wheels or motors are wired.`;
    } else {
      descElem.innerHTML = `<b>Real Physical Hardware Mode Active.</b> Synthetic simulation flags (<code>USE_SIM_WHEEL</code>, <code>USE_SIM_IMU</code>, <code>USE_SIM_LD19</code>) are turned OFF. The microcontroller interacts with real physical motor drivers, wheel encoders, and real I2C sensors. Proceed to Step 2 (Drive &amp; Motors) and Step 4 (Pin Matrix) to finalize wiring pinouts.`;
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
      ? `<span style="color:#94a3b8;">Simulated IMU:</span> <b style="color:#34d399;">Active (50 Hz /imu/data_raw)</b>`
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
}

async function setSimMode(enabled, transitionToDetails = false) {
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

function initSimModeWorkflow() {
  document.querySelectorAll(".btn-switch-real-hw").forEach((btn) => {
    btn.addEventListener("click", () => {
      setSimMode(false, true);
    });
  });

  const toggleBtn = document.getElementById("btn-toggle-sim-mode");
  if (toggleBtn) {
    toggleBtn.addEventListener("click", () => {
      setSimMode(!currentSimMode, !currentSimMode ? false : true);
    });
  }

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

  const hdrMode = document.getElementById("hdr-pipeline-mode");
  const cockpitMode = document.getElementById("cockpit-pipeline-mode");
  if (hdrMode) {
    hdrMode.addEventListener("change", (e) => {
      if (e.target.value === "real") {
        setSimMode(false, false);
      } else if (e.target.value === "sim") {
        setSimMode(true, false);
      }
    });
  }
  if (cockpitMode) {
    cockpitMode.addEventListener("change", (e) => {
      if (e.target.value === "real") {
        setSimMode(false, false);
      } else if (e.target.value === "sim") {
        setSimMode(true, false);
      }
    });
  }
}

document.addEventListener("DOMContentLoaded", initBaseControllerConfigModule);
