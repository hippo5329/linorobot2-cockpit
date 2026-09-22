// ============================================================================
// rosviz.js — Zero-RViz browser visualizer
//
// A headless robot computer has no display for RViz to open on, so the map is
// drawn here instead: an HTML5 canvas fed straight from rosbridge on port 9090.
//
//   /map        nav_msgs/msg/OccupancyGrid                 occupancy raster
//   /scan       sensor_msgs/msg/LaserScan   (SensorDataQoS) laser points
//   /odom       nav_msgs/msg/Odometry                      robot pose
//   /amcl_pose  geometry_msgs/msg/PoseWithCovarianceStamped localized pose
//
// and publishes back:
//
//   /initialpose  geometry_msgs/msg/PoseWithCovarianceStamped  (2D Pose Estimate)
//   /goal_pose    geometry_msgs/msg/PoseStamped                (Nav Goal)
//
// The rosbridge JSON protocol is spoken directly rather than through roslibjs:
// the robot LAN is frequently offline, and a CDN <script> tag is the one thing
// that would make the whole viewer fail to load. The protocol is a handful of
// JSON ops -- vendoring a library to send them is not worth the failure mode.
// ============================================================================

(function () {
  "use strict";

  // ---------------------------------------------------------------- geometry
  function yawFromQuaternion(q) {
    if (!q) return 0;
    const { x = 0, y = 0, z = 0, w = 1 } = q;
    return Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
  }

  function quaternionFromYaw(yaw) {
    return { x: 0, y: 0, z: Math.sin(yaw / 2), w: Math.cos(yaw / 2) };
  }

  // ------------------------------------------------------------ rosbridge IO
  class RosBridge {
    constructor() {
      this.socket = null;
      this.url = "";
      this.handlers = new Map();   // topic -> callback
      this.advertised = new Set();
      this.onStatus = () => {};
      this.reconnectTimer = null;
      this.shouldReconnect = false;
    }

    get connected() {
      return this.socket && this.socket.readyState === WebSocket.OPEN;
    }

    connect(url) {
      this.disconnect();
      this.url = url;
      this.shouldReconnect = true;
      this.onStatus("connecting", `Connecting to ${url}…`);

      let socket;
      try {
        socket = new WebSocket(url);
      } catch (err) {
        this.onStatus("error", `Invalid rosbridge URL: ${err.message}`);
        return;
      }
      this.socket = socket;

      socket.onopen = () => {
        this.onStatus("ok", `Connected to ${url}`);
        // Re-issue every subscription: on a reconnect the server remembers nothing.
        for (const [topic, entry] of this.handlers) this._sendSubscribe(topic, entry);
        for (const topic of this.advertised) this._sendAdvertise(topic);
      };

      socket.onmessage = (event) => {
        let msg;
        try {
          msg = JSON.parse(event.data);
        } catch {
          return;
        }
        if (msg.op === "publish") {
          const entry = this.handlers.get(msg.topic);
          if (entry) {
            try {
              entry.callback(msg.msg);
            } catch (err) {
              console.error(`[rosviz] handler for ${msg.topic} threw`, err);
            }
          }
        } else if (msg.op === "status" && msg.level === "error") {
          this.onStatus("warn", `rosbridge: ${msg.msg}`);
        }
      };

      socket.onclose = () => {
        this.onStatus("err", "Disconnected from rosbridge");
        if (this.shouldReconnect) {
          clearTimeout(this.reconnectTimer);
          this.reconnectTimer = setTimeout(() => this.connect(this.url), 3000);
        }
      };

      socket.onerror = () => {
        // onclose always follows, and carries the message the user needs.
        this.onStatus("err", `Cannot reach ${url} — is rosbridge_server running?`);
      };
    }

    disconnect() {
      this.shouldReconnect = false;
      clearTimeout(this.reconnectTimer);
      if (this.socket) {
        try { this.socket.close(); } catch { /* already gone */ }
      }
      this.socket = null;
    }

    _send(payload) {
      if (!this.connected) return false;
      this.socket.send(JSON.stringify(payload));
      return true;
    }

    _sendSubscribe(topic, entry) {
      this._send({
        op: "subscribe",
        topic,
        type: entry.type,
        // queue_length 1 keeps the canvas on the newest frame instead of
        // working through a backlog after a stall.
        queue_length: 1,
        throttle_rate: entry.throttleMs,
      });
    }

    _sendAdvertise(topic) {
      const type = ADVERTISED_TYPES[topic];
      if (type) this._send({ op: "advertise", topic, type });
    }

    subscribe(topic, type, callback, throttleMs = 0) {
      const entry = { type, callback, throttleMs };
      this.handlers.set(topic, entry);
      this._sendSubscribe(topic, entry);
    }

    unsubscribe(topic) {
      this.handlers.delete(topic);
      this._send({ op: "unsubscribe", topic });
    }

    advertise(topic) {
      this.advertised.add(topic);
      this._sendAdvertise(topic);
    }

    publish(topic, msg) {
      if (!this.advertised.has(topic)) this.advertise(topic);
      return this._send({ op: "publish", topic, msg });
    }
  }

  const ADVERTISED_TYPES = {
    "/initialpose": "geometry_msgs/msg/PoseWithCovarianceStamped",
    "/goal_pose": "geometry_msgs/msg/PoseStamped",
  };

  // -------------------------------------------------------------- the viewer
  class MapViewer {
    constructor(canvas) {
      this.canvas = canvas;
      this.ctx = canvas.getContext("2d");

      // View transform: metres -> pixels, centred on a world point.
      this.scale = 40;          // pixels per metre
      this.center = { x: 0, y: 0 };

      this.map = null;          // { canvas, info }
      this.scan = null;
      this.pose = null;         // { x, y, yaw } -- ALWAYS in the map frame
      this.odomPose = null;     // the raw /odom pose, in the odom frame
      // map -> odom: what SLAM (or AMCL) has corrected the dead reckoning by.
      // Identity until /tf says otherwise, which is the right assumption before
      // a map exists.
      this.mapOdom = { x: 0, y: 0, yaw: 0 };
      this.poseSource = "";

      this.tool = null;         // null | "initialpose" | "goal_pose"
      this.drag = null;         // { start:{x,y}, current:{x,y} }
      this.panning = null;
      this.onPublish = () => {};
      this.onToolChange = () => {};

      this._bindInput();
      this._resize();
      window.addEventListener("resize", () => this._resize());
    }

    // ---- transforms ----
    worldToScreen(wx, wy) {
      return {
        x: (wx - this.center.x) * this.scale + this.canvas.width / 2,
        // Canvas y grows downward; REP-103 world y grows left/up. Flip it, or
        // the map renders mirrored and every goal lands on the wrong side.
        y: this.canvas.height / 2 - (wy - this.center.y) * this.scale,
      };
    }

    screenToWorld(sx, sy) {
      return {
        x: (sx - this.canvas.width / 2) / this.scale + this.center.x,
        y: (this.canvas.height / 2 - sy) / this.scale + this.center.y,
      };
    }

    _resize() {
      const rect = this.canvas.getBoundingClientRect();
      const dpr = window.devicePixelRatio || 1;
      if (rect.width && rect.height) {
        this.canvas.width = Math.round(rect.width * dpr);
        this.canvas.height = Math.round(rect.height * dpr);
      }
      this.draw();
    }

    // ---- data in ----
    setMap(msg) {
      const info = msg.info;
      const { width, height, resolution } = info;
      if (!width || !height) return;

      // Rasterize once per map message. Re-tinting 4M cells on every animation
      // frame would drop the canvas to single-digit FPS on a real map.
      const off = document.createElement("canvas");
      off.width = width;
      off.height = height;
      const offCtx = off.getContext("2d");
      const image = offCtx.createImageData(width, height);
      const data = msg.data;

      for (let row = 0; row < height; row++) {
        for (let col = 0; col < width; col++) {
          const value = data[row * width + col];
          // The grid's row 0 is the bottom of the map; image row 0 is the top.
          const pixel = ((height - 1 - row) * width + col) * 4;
          let r, g, b, a = 255;
          if (value < 0) {
            r = g = b = 130; a = 90;          // unknown
          } else if (value >= 65) {
            r = g = b = 20;                    // occupied
          } else {
            const shade = 255 - Math.round((value / 100) * 120);
            r = g = b = shade;                 // free, darkening with cost
          }
          image.data[pixel] = r;
          image.data[pixel + 1] = g;
          image.data[pixel + 2] = b;
          image.data[pixel + 3] = a;
        }
      }
      offCtx.putImageData(image, 0, 0);

      const first = !this.map;
      this.map = { canvas: off, info, width, height, resolution };
      if (first) this.fitToMap();
      this.draw();
    }

    setScan(msg) {
      this.scan = msg;
      this.draw();
    }

    // map -> odom, straight off /tf. Without it everything drawn from /odom is
    // in the WRONG FRAME: the map is published in `map`, the odometry is in
    // `odom`, and the difference between them is precisely the drift SLAM is
    // correcting. Measured on the bench 2026-09-23 while a scan visibly sat at
    // an angle to the mapped walls: map->odom was yaw +5.20 deg, (+0.099,
    // -0.121) m. This EKF fuses velocities only, so it has nothing to correct
    // against and that angle grows over a run.
    setMapOdom(transform) {
      this.mapOdom = {
        x: transform.translation.x,
        y: transform.translation.y,
        yaw: yawFromQuaternion(transform.rotation),
      };
      if (this.odomPose && this.poseSource !== "amcl") this._recomputePose();
      this.draw();
    }

    _recomputePose() {
      const m = this.mapOdom, o = this.odomPose;
      const c = Math.cos(m.yaw), s = Math.sin(m.yaw);
      this.pose = {
        x: m.x + c * o.x - s * o.y,
        y: m.y + s * o.x + c * o.y,
        yaw: m.yaw + o.yaw,
      };
    }

    setPose(pose, source) {
      // /amcl_pose is the localized truth; do not let raw /odom overwrite it.
      if (source === "odom" && this.poseSource === "amcl") return;
      const p = {
        x: pose.position.x,
        y: pose.position.y,
        yaw: yawFromQuaternion(pose.orientation),
      };
      if (source === "odom") {
        // /odom is in the odom frame and everything here is drawn in map.
        this.odomPose = p;
        this._recomputePose();
      } else {
        // /amcl_pose is already in map.
        this.pose = p;
      }
      this.poseSource = source;
      this.draw();
    }

    fitToMap() {
      if (!this.map) return;
      const { info, width, height, resolution } = this.map;
      this.center = {
        x: info.origin.position.x + (width * resolution) / 2,
        y: info.origin.position.y + (height * resolution) / 2,
      };
      const fit = Math.min(
        this.canvas.width / (width * resolution),
        this.canvas.height / (height * resolution)
      );
      this.scale = fit * 0.92;
      this.draw();
    }

    followRobot() {
      if (!this.pose) return;
      this.center = { x: this.pose.x, y: this.pose.y };
      this.draw();
    }

    // ---- input ----
    _bindInput() {
      const canvas = this.canvas;

      canvas.addEventListener("wheel", (event) => {
        event.preventDefault();
        const rect = canvas.getBoundingClientRect();
        const dpr = window.devicePixelRatio || 1;
        const sx = (event.clientX - rect.left) * dpr;
        const sy = (event.clientY - rect.top) * dpr;
        // Zoom about the cursor: the world point under it must not move.
        const before = this.screenToWorld(sx, sy);
        const factor = event.deltaY < 0 ? 1.15 : 1 / 1.15;
        this.scale = Math.max(2, Math.min(400, this.scale * factor));
        const after = this.screenToWorld(sx, sy);
        this.center.x += before.x - after.x;
        this.center.y += before.y - after.y;
        this.draw();
      }, { passive: false });

      const localPoint = (event) => {
        const rect = canvas.getBoundingClientRect();
        const dpr = window.devicePixelRatio || 1;
        return {
          x: (event.clientX - rect.left) * dpr,
          y: (event.clientY - rect.top) * dpr,
        };
      };

      canvas.addEventListener("mousedown", (event) => {
        const point = localPoint(event);
        if (this.tool) {
          const world = this.screenToWorld(point.x, point.y);
          this.drag = { start: world, current: world };
        } else {
          this.panning = { screen: point, center: { ...this.center } };
        }
      });

      canvas.addEventListener("mousemove", (event) => {
        const point = localPoint(event);
        if (this.drag) {
          this.drag.current = this.screenToWorld(point.x, point.y);
          this.draw();
        } else if (this.panning) {
          this.center.x = this.panning.center.x - (point.x - this.panning.screen.x) / this.scale;
          this.center.y = this.panning.center.y + (point.y - this.panning.screen.y) / this.scale;
          this.draw();
        }
      });

      const finishDrag = () => {
        if (this.drag && this.tool) {
          const { start, current } = this.drag;
          // Drag direction sets the heading; a bare click keeps the current one.
          const dx = current.x - start.x;
          const dy = current.y - start.y;
          const yaw = Math.hypot(dx, dy) > 0.05 ? Math.atan2(dy, dx) : (this.pose ? this.pose.yaw : 0);
          this.onPublish(this.tool, { x: start.x, y: start.y, yaw });
          this.setTool(null);
        }
        this.drag = null;
        this.panning = null;
        this.draw();
      };

      canvas.addEventListener("mouseup", finishDrag);
      canvas.addEventListener("mouseleave", () => {
        // Abandon an in-flight drag rather than publishing a goal the user
        // dragged off the canvas.
        this.drag = null;
        this.panning = null;
        this.draw();
      });
    }

    setTool(tool) {
      this.tool = tool;
      this.canvas.style.cursor = tool ? "crosshair" : "grab";
      this.onToolChange(tool);
      this.draw();
    }

    // ---- rendering ----
    draw() {
      const ctx = this.ctx;
      const { width: W, height: H } = this.canvas;
      ctx.clearRect(0, 0, W, H);
      ctx.fillStyle = "#0b1220";
      ctx.fillRect(0, 0, W, H);

      this._drawGrid();
      this._drawMap();
      this._drawScan();
      this._drawRobot();
      this._drawDrag();
    }

    _drawGrid() {
      const ctx = this.ctx;
      const step = this.scale >= 25 ? 1 : this.scale >= 8 ? 5 : 10;  // metres
      const topLeft = this.screenToWorld(0, 0);
      const bottomRight = this.screenToWorld(this.canvas.width, this.canvas.height);

      ctx.strokeStyle = "rgba(148,163,184,0.12)";
      ctx.lineWidth = 1;
      ctx.beginPath();
      for (let x = Math.floor(topLeft.x / step) * step; x <= bottomRight.x; x += step) {
        const s = this.worldToScreen(x, 0);
        ctx.moveTo(s.x, 0);
        ctx.lineTo(s.x, this.canvas.height);
      }
      for (let y = Math.floor(bottomRight.y / step) * step; y <= topLeft.y; y += step) {
        const s = this.worldToScreen(0, y);
        ctx.moveTo(0, s.y);
        ctx.lineTo(this.canvas.width, s.y);
      }
      ctx.stroke();
    }

    _drawMap() {
      if (!this.map) return;
      const ctx = this.ctx;
      const { canvas: img, info, width, height, resolution } = this.map;
      const originYaw = yawFromQuaternion(info.origin.orientation);

      // The image's top-left corner is the map's (0, height) cell corner.
      const corner = this.worldToScreen(
        info.origin.position.x,
        info.origin.position.y + height * resolution
      );

      ctx.save();
      if (Math.abs(originYaw) > 1e-6) {
        // Rotated map origins are rare but real; rotate about the grid origin.
        const pivot = this.worldToScreen(info.origin.position.x, info.origin.position.y);
        ctx.translate(pivot.x, pivot.y);
        ctx.rotate(-originYaw);
        ctx.translate(-pivot.x, -pivot.y);
      }
      // Nearest-neighbour: a smoothed occupancy grid invents free space
      // between an occupied cell and an unknown one.
      ctx.imageSmoothingEnabled = false;
      ctx.drawImage(
        img, corner.x, corner.y,
        width * resolution * this.scale,
        height * resolution * this.scale
      );
      ctx.restore();
    }

    _drawScan() {
      if (!this.scan || !this.pose) return;
      const ctx = this.ctx;
      const scan = this.scan;
      const ranges = scan.ranges || [];
      if (!ranges.length) return;

      // Projected from the robot pose in the MAP frame (see setMapOdom): the
      // browser has no full transform listener, and base_link -> laser really is
      // a small fixed offset. map -> odom is not -- skipping that one drew the
      // scan at an angle to the walls it had just built.
      const { x: rx, y: ry, yaw } = this.pose;
      ctx.fillStyle = "#f43f5e";
      for (let i = 0; i < ranges.length; i++) {
        const r = ranges[i];
        if (!isFinite(r) || r <= scan.range_min || r >= scan.range_max) continue;
        const angle = yaw + scan.angle_min + i * scan.angle_increment;
        const point = this.worldToScreen(rx + r * Math.cos(angle), ry + r * Math.sin(angle));
        ctx.fillRect(point.x - 1, point.y - 1, 2.5, 2.5);
      }
    }

    _drawRobot() {
      if (!this.pose) return;
      const ctx = this.ctx;
      const { x, y, yaw } = this.pose;
      const screen = this.worldToScreen(x, y);
      const radius = Math.max(6, 0.18 * this.scale);

      ctx.save();
      ctx.translate(screen.x, screen.y);
      ctx.rotate(-yaw);

      ctx.fillStyle = this.poseSource === "amcl" ? "rgba(34,197,94,0.85)" : "rgba(99,102,241,0.85)";
      ctx.strokeStyle = "#e2e8f0";
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      ctx.arc(0, 0, radius, 0, Math.PI * 2);
      ctx.fill();
      ctx.stroke();

      ctx.beginPath();          // heading
      ctx.moveTo(0, 0);
      ctx.lineTo(radius * 2.1, 0);
      ctx.strokeStyle = "#fbbf24";
      ctx.lineWidth = 2.5;
      ctx.stroke();
      ctx.restore();
    }

    _drawDrag() {
      if (!this.drag) return;
      const ctx = this.ctx;
      const from = this.worldToScreen(this.drag.start.x, this.drag.start.y);
      const to = this.worldToScreen(this.drag.current.x, this.drag.current.y);
      const color = this.tool === "goal_pose" ? "#22c55e" : "#38bdf8";

      ctx.strokeStyle = color;
      ctx.fillStyle = color;
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.moveTo(from.x, from.y);
      ctx.lineTo(to.x, to.y);
      ctx.stroke();

      const angle = Math.atan2(to.y - from.y, to.x - from.x);
      ctx.beginPath();          // arrow head
      ctx.moveTo(to.x, to.y);
      ctx.lineTo(to.x - 12 * Math.cos(angle - 0.4), to.y - 12 * Math.sin(angle - 0.4));
      ctx.lineTo(to.x - 12 * Math.cos(angle + 0.4), to.y - 12 * Math.sin(angle + 0.4));
      ctx.closePath();
      ctx.fill();

      ctx.beginPath();
      ctx.arc(from.x, from.y, 4, 0, Math.PI * 2);
      ctx.fill();
    }
  }

  // ------------------------------------------------------------------ wiring
  function init() {
    const canvas = document.getElementById("rosviz-canvas");
    if (!canvas) return;   // viewer tab not present in this build

    const bridge = new RosBridge();
    const viewer = new MapViewer(canvas);

    const el = (id) => document.getElementById(id);
    const statusEl = el("rosviz-status");
    const urlInput = el("rosviz-url");
    const topicsEl = el("rosviz-topics");

    const counts = { map: 0, scan: 0, odom: 0, amcl: 0, tf: 0 };

    function setStatus(level, message) {
      if (!statusEl) return;
      statusEl.textContent = message;
      statusEl.className = "hint rosviz-status rosviz-status-" + level;
    }

    function updateTopicCounts() {
      if (!topicsEl) return;
      topicsEl.textContent =
        `/map ${counts.map} · /scan ${counts.scan} · /odom ${counts.odom} · `
      + `/amcl_pose ${counts.amcl} · map→odom ${counts.tf}`;
    }

    bridge.onStatus = setStatus;

    function defaultUrl() {
      // Default to the machine serving this page: on the robot that is the robot,
      // and from a laptop it is whatever host the operator typed in the bar.
      const host = window.location.hostname || "localhost";
      return `ws://${host}:9090`;
    }

    if (urlInput && !urlInput.value) urlInput.value = defaultUrl();

    function connect() {
      const url = (urlInput && urlInput.value.trim()) || defaultUrl();
      bridge.connect(url);

      // The map is latched and large; 2 s of throttle is plenty and keeps a
      // 4000x4000 grid from saturating the socket on every republish.
      bridge.subscribe("/map", "nav_msgs/msg/OccupancyGrid", (msg) => {
        counts.map++; updateTopicCounts(); viewer.setMap(msg);
      }, 2000);

      bridge.subscribe("/scan", "sensor_msgs/msg/LaserScan", (msg) => {
        counts.scan++; updateTopicCounts(); viewer.setScan(msg);
      }, 100);

      // /tf is busy, and only one transform on it matters here. Throttled,
      // because the correction moves at SLAM's update rate, not the base's.
      bridge.subscribe("/tf", "tf2_msgs/msg/TFMessage", (msg) => {
        for (const t of msg.transforms || []) {
          if (t.header.frame_id === "map" && t.child_frame_id === "odom") {
            counts.tf++; updateTopicCounts(); viewer.setMapOdom(t.transform);
          }
        }
      }, 200);
      bridge.subscribe("/odom", "nav_msgs/msg/Odometry", (msg) => {
        counts.odom++; updateTopicCounts(); viewer.setPose(msg.pose.pose, "odom");
      }, 100);

      bridge.subscribe("/amcl_pose", "geometry_msgs/msg/PoseWithCovarianceStamped", (msg) => {
        counts.amcl++; updateTopicCounts(); viewer.setPose(msg.pose.pose, "amcl");
      }, 200);

      bridge.advertise("/initialpose");
      bridge.advertise("/goal_pose");
    }

    function nowStamp() {
      const ms = Date.now();
      return { sec: Math.floor(ms / 1000), nanosec: (ms % 1000) * 1e6 };
    }

    viewer.onPublish = (tool, pose) => {
      const header = { stamp: nowStamp(), frame_id: "map" };
      const orientation = quaternionFromYaw(pose.yaw);
      const position = { x: pose.x, y: pose.y, z: 0 };

      if (tool === "goal_pose") {
        const ok = bridge.publish("/goal_pose", {
          header, pose: { position, orientation },
        });
        setStatus(ok ? "ok" : "err", ok
          ? `Goal sent: x=${pose.x.toFixed(2)} y=${pose.y.toFixed(2)} yaw=${(pose.yaw * 180 / Math.PI).toFixed(0)}°`
          : "Not connected — goal not sent");
      } else {
        const covariance = new Array(36).fill(0);
        covariance[0] = 0.25;    // x
        covariance[7] = 0.25;    // y
        covariance[35] = 0.068;  // yaw — the AMCL default spread
        const ok = bridge.publish("/initialpose", {
          header, pose: { pose: { position, orientation }, covariance },
        });
        setStatus(ok ? "ok" : "err", ok
          ? `Initial pose set: x=${pose.x.toFixed(2)} y=${pose.y.toFixed(2)} yaw=${(pose.yaw * 180 / Math.PI).toFixed(0)}°`
          : "Not connected — pose not set");
      }
    };

    viewer.onToolChange = (tool) => {
      const initialBtn = el("btn-rosviz-initialpose");
      const goalBtn = el("btn-rosviz-goal");
      if (initialBtn) initialBtn.classList.toggle("active", tool === "initialpose");
      if (goalBtn) goalBtn.classList.toggle("active", tool === "goal_pose");
      if (tool) setStatus("info", tool === "goal_pose"
        ? "Click the goal, drag to aim the heading, release to send."
        : "Click the robot's true position, drag to aim its heading, release.");
    };

    el("btn-rosviz-connect")?.addEventListener("click", connect);
    el("btn-rosviz-disconnect")?.addEventListener("click", () => {
      bridge.disconnect();
      setStatus("info", "Disconnected.");
    });
    el("btn-rosviz-initialpose")?.addEventListener("click", () =>
      viewer.setTool(viewer.tool === "initialpose" ? null : "initialpose"));
    el("btn-rosviz-goal")?.addEventListener("click", () =>
      viewer.setTool(viewer.tool === "goal_pose" ? null : "goal_pose"));
    el("btn-rosviz-fit")?.addEventListener("click", () => viewer.fitToMap());
    el("btn-rosviz-follow")?.addEventListener("click", () => viewer.followRobot());

    // Escape cancels an armed tool -- otherwise the next click on the map
    // publishes a goal the user no longer wants.
    document.addEventListener("keydown", (event) => {
      if (event.key === "Escape" && viewer.tool) viewer.setTool(null);
    });

    // The canvas has no size until its tab is shown, so a viewer laid out while
    // hidden would measure 0x0 and stay blank.
    document.querySelectorAll(".tab-btn").forEach((btn) => {
      btn.addEventListener("click", () => {
        if (btn.dataset.tab === "rosviz") setTimeout(() => viewer._resize(), 50);
      });
    });

    window.rosviz = { bridge, viewer, connect };
    setStatus("info", "Not connected. Press Connect to subscribe to the robot's rosbridge.");
  }

  if (document.readyState === "loading") {
    document.addEventListener("DOMContentLoaded", init);
  } else {
    init();
  }
})();
