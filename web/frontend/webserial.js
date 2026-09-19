// ============================================================================
// webserial.js — in-browser raw serial debug monitor
//
// WebSerial is deliberately limited to READING AND WRITING TEXT. Firmware
// flashing is never routed through the browser: it runs natively on the robot
// computer via esptool / picotool, which is the only path that can pause
// micro_ros_agent, verify the port was released, and resume it afterwards.
// A browser tab can do none of that, and a half-written flash bricks the board.
//
// This is the equivalent of `pio device monitor` / minicom, for the machine the
// operator is sitting at -- useful for watching a board's boot log or talking to
// the test_sensors / i2c_detect diagnostic firmwares.
//
// Requires a Chromium-family browser over HTTPS or localhost; Firefox and Safari
// do not implement the Web Serial API.
// ============================================================================

(function () {
  "use strict";

  const MAX_LINES = 2000;   // ring buffer: a chatty board fills the DOM otherwise

  class SerialMonitor {
    constructor(outputEl) {
      this.outputEl = outputEl;
      this.port = null;
      this.reader = null;
      this.writer = null;
      this.keepReading = false;
      this.lines = [];
      this.partial = "";
      this.autoscroll = true;
      this.onState = () => {};
    }

    get supported() {
      return "serial" in navigator;
    }

    get connected() {
      return this.port !== null;
    }

    append(text, className) {
      this.lines.push({ text, className });
      if (this.lines.length > MAX_LINES) this.lines.splice(0, this.lines.length - MAX_LINES);
      this.render();
    }

    render() {
      if (!this.outputEl) return;
      const fragment = document.createDocumentFragment();
      for (const line of this.lines) {
        const div = document.createElement("div");
        div.textContent = line.text;
        if (line.className) div.className = line.className;
        fragment.appendChild(div);
      }
      this.outputEl.replaceChildren(fragment);
      if (this.autoscroll) this.outputEl.scrollTop = this.outputEl.scrollHeight;
    }

    clear() {
      this.lines = [];
      this.partial = "";
      this.render();
    }

    async connect(baudRate) {
      if (!this.supported) {
        this.append("This browser has no Web Serial API. Use Chrome, Edge or another Chromium browser.", "ws-err");
        return false;
      }
      try {
        this.port = await navigator.serial.requestPort();
        await this.port.open({ baudRate, bufferSize: 4096 });
      } catch (err) {
        // A user dismissing the port chooser is a cancel, not a failure.
        if (err && err.name === "NotFoundError") {
          this.append("No port selected.", "ws-warn");
        } else {
          this.append(`Could not open the port: ${err.message}`, "ws-err");
        }
        this.port = null;
        this.onState();
        return false;
      }

      const info = this.port.getInfo ? this.port.getInfo() : {};
      const vid = info.usbVendorId ? info.usbVendorId.toString(16).padStart(4, "0") : "?";
      const pid = info.usbProductId ? info.usbProductId.toString(16).padStart(4, "0") : "?";
      this.append(`— connected at ${baudRate} baud (USB ${vid}:${pid}) —`, "ws-ok");

      this.keepReading = true;
      this.onState();
      this._readLoop();
      return true;
    }

    async _readLoop() {
      const decoder = new TextDecoder();
      while (this.port && this.keepReading) {
        try {
          this.reader = this.port.readable.getReader();
          while (true) {
            const { value, done } = await this.reader.read();
            if (done) break;
            this._ingest(decoder.decode(value, { stream: true }));
          }
        } catch (err) {
          if (this.keepReading) this.append(`Read error: ${err.message}`, "ws-err");
        } finally {
          try { this.reader?.releaseLock(); } catch { /* already released */ }
          this.reader = null;
        }
      }
    }

    _ingest(chunk) {
      // Serial arrives in arbitrary chunks, not lines; hold the tail until its
      // newline turns up, so a split line is not rendered as two.
      this.partial += chunk;
      const parts = this.partial.split(/\r?\n/);
      this.partial = parts.pop();
      for (const part of parts) this.append(part, null);
      if (this.partial.length > 400) {   // a board sending no newlines at all
        this.append(this.partial, null);
        this.partial = "";
      }
    }

    async send(text, lineEnding) {
      if (!this.port || !this.port.writable) return false;
      const payload = text + (lineEnding || "");
      try {
        this.writer = this.port.writable.getWriter();
        await this.writer.write(new TextEncoder().encode(payload));
        this.append(`> ${text}`, "ws-sent");
        return true;
      } catch (err) {
        this.append(`Write error: ${err.message}`, "ws-err");
        return false;
      } finally {
        try { this.writer?.releaseLock(); } catch { /* already released */ }
        this.writer = null;
      }
    }

    async disconnect() {
      this.keepReading = false;
      try { await this.reader?.cancel(); } catch { /* reader already gone */ }
      try { await this.port?.close(); } catch (err) {
        this.append(`Close error: ${err.message}`, "ws-warn");
      }
      this.port = null;
      this.append("— disconnected —", "ws-warn");
      this.onState();
    }
  }

  function init() {
    const outputEl = document.getElementById("webserial-output");
    if (!outputEl) return;   // monitor not present in this build

    const monitor = new SerialMonitor(outputEl);
    const el = (id) => document.getElementById(id);

    const connectBtn = el("btn-webserial-connect");
    const disconnectBtn = el("btn-webserial-disconnect");
    const baudSelect = el("webserial-baud");
    const inputEl = el("webserial-input");
    const statusEl = el("webserial-status");
    const supportEl = el("webserial-support");

    if (!monitor.supported && supportEl) {
      // Say WHICH of the two reasons applies. navigator.serial is undefined in
      // a browser that has no Web Serial, and equally undefined in Chrome on an
      // insecure origin -- and the second is the ordinary case here, because
      // the README tells everyone to open http://<robot-computer>:8000. Blaming
      // the browser there is simply wrong, and sends a Chrome user off to
      // install Chrome.
      const insecure = !window.isSecureContext;
      supportEl.innerHTML = insecure
        ? "\u26a0\ufe0f The Web Serial API is only available on a <b>secure origin</b>, and this page " +
          "is served over plain HTTP from <code>" + location.host + "</code>. Your browser is " +
          "not the problem. Open the cockpit as <code>http://localhost:8000</code> on the robot " +
          "computer itself, or put it behind HTTPS, to use this monitor. Everything else in the " +
          "cockpit works over plain HTTP \u2014 only this panel needs the secure origin."
        : "\u26a0\ufe0f This browser does not implement the Web Serial API. Use Chrome or Edge " +
          "(Firefox and Safari do not support it).";
      supportEl.hidden = false;
      if (connectBtn) connectBtn.disabled = true;
    }

    function refresh() {
      const on = monitor.connected;
      if (connectBtn) connectBtn.disabled = on || !monitor.supported;
      if (disconnectBtn) disconnectBtn.disabled = !on;
      if (inputEl) inputEl.disabled = !on;
      if (baudSelect) baudSelect.disabled = on;
      if (statusEl) {
        statusEl.textContent = on ? "Connected" : "Not connected";
        statusEl.className = "badge-pill " + (on ? "badge-ok" : "badge-idle");
      }
    }
    monitor.onState = refresh;

    connectBtn?.addEventListener("click", async () => {
      await monitor.connect(parseInt(baudSelect?.value || "115200", 10));
      refresh();
    });
    disconnectBtn?.addEventListener("click", () => monitor.disconnect());
    el("btn-webserial-clear")?.addEventListener("click", () => monitor.clear());

    el("webserial-autoscroll")?.addEventListener("change", (event) => {
      monitor.autoscroll = event.target.checked;
    });

    inputEl?.addEventListener("keydown", (event) => {
      if (event.key !== "Enter") return;
      const lineEnding = el("webserial-lineending")?.value ?? "\n";
      monitor.send(inputEl.value, lineEnding.replace(/\\r/g, "\r").replace(/\\n/g, "\n"));
      inputEl.value = "";
    });

    window.webserialMonitor = monitor;
    refresh();
  }

  if (document.readyState === "loading") {
    document.addEventListener("DOMContentLoaded", init);
  } else {
    init();
  }
})();
