/* Scope UI: telemetry tree + signal quality (PlotJuggler / Foxglove style). */
const modeNames = ["stop", "servo", "cal", "current"];
const MAX_POINTS = 2000;

// Live scope mirrors CtrlReply fields only (free-running Telemetry is retired).
// Phase currents I1/I2/I3 and PWM duties are Snapshot-only, not Live.
const CHANNELS = [
  { key: "id_a", label: "Id", path: "current/Id", color: "#5ec4a0", unit: "A", on: true, group: "current" },
  { key: "iq_a", label: "Iq", path: "current/Iq", color: "#e0b35a", unit: "A", on: true, group: "current" },
  { key: "idref_a", label: "Idref", path: "current/Idref", color: "#5a6a88", unit: "A", on: true, group: "current" },
  { key: "iqref_a", label: "Iqref", path: "current/Iqref", color: "#7a5a88", unit: "A", on: true, group: "current" },
  { key: "vd_v", label: "Vd", path: "voltage/Vd", color: "#c47ad0", unit: "V", on: false, group: "voltage" },
  { key: "vq_v", label: "Vq", path: "voltage/Vq", color: "#d08a6a", unit: "V", on: false, group: "voltage" },
  { key: "bus_v", label: "Vbus", path: "voltage/Vbus", color: "#e08a8a", unit: "V", on: false, group: "voltage" },
  { key: "voltage_headroom_v", label: "Vheadroom", path: "voltage/headroom", color: "#80c0c8", unit: "V", on: false, group: "voltage" },
  { key: "fet_temp_c", label: "FetTemp", path: "thermal/fet", color: "#e0a060", unit: "C", on: true, group: "thermal" },
  { key: "theta_rad", label: "thetaElec", path: "motion/theta_elec", color: "#a0a8b8", unit: "rad", on: false, group: "motion" },
  { key: "omega_rad_s", label: "omegaMeas", path: "motion/omega_mech", color: "#88b0d0", unit: "rad/s", on: true, group: "motion" },
  { key: "omega_cmd_rad_s", label: "omegaCmd", path: "motion/omega_cmd", color: "#d0a088", unit: "rad/s", on: true, group: "motion" },
  { key: "omega_elec_rad_s", label: "omegaElec", path: "motion/omega_elec", color: "#7098c0", unit: "rad/s", on: false, group: "motion" },
  { key: "enc_raw", label: "encRaw", path: "encoder/raw", color: "#c0d088", unit: "", on: true, group: "encoder" },
  { key: "enc_spike", label: "encSpike", path: "encoder/spike", color: "#ff7a7a", unit: "", on: true, group: "encoder" },
  { key: "enc_theta_mech_rad", label: "encMech", path: "encoder/theta_mech", color: "#a8c070", unit: "rad", on: true, group: "encoder" },
  { key: "enc_theta_elec_rad", label: "encElec", path: "encoder/theta_elec", color: "#88b060", unit: "rad", on: true, group: "encoder" },
];

const GROUPS = [
  { id: "current", label: "Current" },
  { id: "voltage", label: "Voltage" },
  { id: "thermal", label: "Thermal" },
  { id: "motion", label: "Motion" },
  { id: "encoder", label: "Encoder" },
];

const hist = { t: [] };
for (const ch of CHANNELS) hist[ch.key] = [];

const statusEl = document.getElementById("status");
const portStatusEl = document.getElementById("portStatus");
const motorStatusEl = document.getElementById("motorStatus");
const motorInfoEl = document.getElementById("motorInfo"); // optional legacy
const portSelect = document.getElementById("portSelect");
const portCustom = document.getElementById("portCustom");
const portHints = document.getElementById("portHints");
const telemTree = document.getElementById("telemTree");
const chSearch = document.getElementById("chSearch");
const sigStatsBody = document.getElementById("sigStatsBody");
const msgLogEl = document.getElementById("msgLog");
const canvas = document.getElementById("plot");
const ctx = canvas.getContext("2d");

let view = {
  yMin: -2,
  yMax: 2,
  timeWindowMs: 5000,
  autoY: true,
  pause: false,
  dragging: false,
  lastY: 0,
  showMean: true,
};
let lastMsgId = 0;
let lastTelemSid = 0;
let telemInflight = false;
let plotDirty = false;
let latestVals = {};
let lastMotorInfo = null;
let focusKey = "iq_a";
let searchQuery = "";
const groupOpen = Object.fromEntries(GROUPS.map((g) => [g.id, true]));
let treeDirty = true;
let lastStatsAt = 0;

async function postJson(url, body) {
  const r = await fetch(url, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(body || {}),
  });
  return r.json();
}

function selectedIface() {
  const typed = (portCustom.value || "").trim();
  if (typed) return typed;
  return (portSelect.value || "").trim();
}

function fillPorts(ports, selected) {
  const prev = selected || portSelect.value || portCustom.value;
  portSelect.innerHTML = "";
  portHints.innerHTML = "";
  if (!ports || ports.length === 0) {
    const opt = document.createElement("option");
    opt.value = "";
    opt.textContent = "(no SocketCAN ports)";
    portSelect.appendChild(opt);
  } else {
    for (const p of ports) {
      const opt = document.createElement("option");
      opt.value = p.name;
      opt.textContent = `${p.name} [${p.state}]`;
      portSelect.appendChild(opt);
      const hint = document.createElement("option");
      hint.value = p.name;
      portHints.appendChild(hint);
    }
  }
  if (prev && [...portSelect.options].some((o) => o.value === prev)) {
    portSelect.value = prev;
  }
  if (prev && !portCustom.value) portCustom.value = prev;
}

function showMotor(j) {
  motorStatusEl.classList.remove("ok", "fail");
  if (motorInfoEl) motorInfoEl.textContent = "";
  if (j.motor_ok && j.info) {
    const i = j.info;
    motorStatusEl.classList.add("ok");
    // Motor identity / limits live in Config (Flash). Keep only connection + fw.
    lastMotorInfo = i;
    motorStatusEl.textContent = `电机连接成功 · fw ${i.fw_version}`;
  } else if (j.motor_error || (j.ok && j.motor_ok === false)) {
    motorStatusEl.classList.add("fail");
    motorStatusEl.textContent = j.motor_error || "电机连接失败";
  } else if (!j.ok && j.error) {
    motorStatusEl.classList.add("fail");
    motorStatusEl.textContent = `CAN 连接失败：${j.error}`;
    if (motorInfoEl && j.hint) motorInfoEl.textContent = j.hint;
  } else {
    motorStatusEl.textContent = "电机未探测";
  }
}

function fmtVal(v) {
  if (typeof v !== "number" || !Number.isFinite(v)) return "—";
  const a = Math.abs(v);
  if (a >= 1000) return v.toFixed(0);
  if (a >= 100) return v.toFixed(1);
  if (a >= 10) return v.toFixed(2);
  return v.toFixed(3);
}

function channelByKey(key) {
  return CHANNELS.find((c) => c.key === key);
}

function matchesSearch(ch, q) {
  if (!q) return true;
  const hay = `${ch.path} ${ch.label} ${ch.key} ${ch.unit}`.toLowerCase();
  return hay.includes(q);
}

function renderTree(force = false) {
  if (!force && !treeDirty) {
    // cheap live value refresh only
    for (const ch of CHANNELS) {
      const el = telemTree.querySelector(`[data-key="${ch.key}"] .val`);
      if (el) el.textContent = fmtVal(latestVals[ch.key]);
    }
    return;
  }
  treeDirty = false;
  const q = searchQuery.trim().toLowerCase();
  telemTree.innerHTML = "";

  for (const g of GROUPS) {
    const kids = CHANNELS.filter((c) => c.group === g.id && matchesSearch(c, q));
    if (!kids.length) continue;
    const open = q ? true : groupOpen[g.id];
    const onCount = kids.filter((c) => c.on).length;

    const group = document.createElement("div");
    group.className = "tree-group";

    const head = document.createElement("div");
    head.className = "tree-group-head";
    head.innerHTML =
      `<span class="tree-caret">${open ? "▼" : "▶"}</span>` +
      `<span class="tree-group-title">${g.label}</span>` +
      `<span class="tree-group-count">${onCount}/${kids.length}</span>`;
    head.onclick = () => {
      if (q) return;
      groupOpen[g.id] = !groupOpen[g.id];
      treeDirty = true;
      renderTree(true);
    };
    group.appendChild(head);

    if (open) {
      const children = document.createElement("div");
      children.className = "tree-children";
      for (const ch of kids) {
        const leaf = document.createElement("div");
        leaf.className =
          "tree-leaf" + (ch.on ? " on" : "") + (ch.key === focusKey ? " focus" : "");
        leaf.dataset.key = ch.key;
        leaf.innerHTML =
          `<input type="checkbox" ${ch.on ? "checked" : ""} />` +
          `<span class="swatch" style="background:${ch.color};opacity:${ch.on ? 1 : 0.35}"></span>` +
          `<span class="name">${ch.label}</span>` +
          `<span class="val">${fmtVal(latestVals[ch.key])}</span>`;
        const box = leaf.querySelector("input");
        box.onclick = (e) => e.stopPropagation();
        box.onchange = () => {
          ch.on = box.checked;
          treeDirty = true;
          renderTree(true);
          if (view.autoY) autoscaleY(false);
          draw();
          updateSigStats(true);
        };
        leaf.onclick = () => {
          focusKey = ch.key;
          treeDirty = true;
          renderTree(true);
          updateSigStats(true);
        };
        children.appendChild(leaf);
      }
      group.appendChild(children);
    }
    telemTree.appendChild(group);
  }
  if (!telemTree.childElementCount) {
    telemTree.innerHTML =
      `<div style="padding:0.5rem;color:#6b7588">无匹配「${escapeHtml(searchQuery)}」</div>`;
  }
}

chSearch.addEventListener("input", () => {
  searchQuery = chSearch.value || "";
  treeDirty = true;
  renderTree(true);
});

function windowSamples(key) {
  if (hist.t.length < 2) return { xs: [], ys: [] };
  const tEnd = hist.t[hist.t.length - 1];
  const tStart = tEnd - view.timeWindowMs;
  const xs = [];
  const ys = [];
  const arr = hist[key];
  for (let i = 0; i < hist.t.length; i++) {
    if (hist.t[i] < tStart) continue;
    xs.push(hist.t[i]);
    ys.push(arr[i]);
  }
  return { xs, ys };
}

function computeStats(key) {
  const { xs, ys } = windowSamples(key);
  const n = ys.length;
  if (n < 2) return null;
  let sum = 0;
  let min = ys[0];
  let max = ys[0];
  for (const v of ys) {
    sum += v;
    if (v < min) min = v;
    if (v > max) max = v;
  }
  const mean = sum / n;
  let varAcc = 0;
  for (const v of ys) {
    const d = v - mean;
    varAcc += d * d;
  }
  const std = Math.sqrt(varAcc / (n - 1));
  // high-frequency noise proxy: RMS of first difference
  let d2 = 0;
  for (let i = 1; i < n; i++) {
    const d = ys[i] - ys[i - 1];
    d2 += d * d;
  }
  const noiseRms = Math.sqrt(d2 / (n - 1));
  const rms = Math.sqrt(ys.reduce((a, v) => a + v * v, 0) / n);
  const dt = (xs[n - 1] - xs[0]) / 1000;
  const rate = dt > 0 ? (n - 1) / dt : 0;
  const snr = std > 1e-12 ? Math.abs(mean) / std : Infinity;
  return { n, mean, std, noiseRms, rms, min, max, pp: max - min, rate, snr, dt };
}

function qualityLabel(st, ch) {
  // Heuristic for current-like signals; still useful as relative cue.
  const unit = (ch && ch.unit) || "";
  let thrQuiet = 0.02;
  let thrNoisy = 0.1;
  if (unit === "V") {
    thrQuiet = 0.05;
    thrNoisy = 0.3;
  } else if (unit === "rad" || unit === "rad/s") {
    thrQuiet = 0.05;
    thrNoisy = 0.5;
  } else if (!unit) {
    thrQuiet = 20;
    thrNoisy = 100;
  }
  if (st.noiseRms <= thrQuiet) return { text: "quiet", cls: "q-good" };
  if (st.noiseRms <= thrNoisy) return { text: "moderate", cls: "q-warn" };
  return { text: "noisy", cls: "q-bad" };
}

function updateSigStats(force = false) {
  const now = performance.now();
  if (!force && now - lastStatsAt < 200) return;
  lastStatsAt = now;
  const ch = channelByKey(focusKey);
  if (!ch) {
    sigStatsBody.textContent = "选中通道查看均值 / 噪声";
    return;
  }
  const st = computeStats(ch.key);
  if (!st) {
    sigStatsBody.textContent = `${ch.path}\n等待窗口数据…`;
    return;
  }
  const q = qualityLabel(st, ch);
  const u = ch.unit ? ` ${ch.unit}` : "";
  sigStatsBody.innerHTML =
    `<b>${escapeHtml(ch.path)}</b>\n` +
    `mean   ${fmtVal(st.mean)}${u}\n` +
    `std    ${fmtVal(st.std)}${u}\n` +
    `noise  ${fmtVal(st.noiseRms)}${u}  (Δrms)\n` +
    `rms    ${fmtVal(st.rms)}${u}\n` +
    `min    ${fmtVal(st.min)}${u}\n` +
    `max    ${fmtVal(st.max)}${u}\n` +
    `p-p    ${fmtVal(st.pp)}${u}\n` +
    `SNR    ${st.snr === Infinity ? "∞" : st.snr.toFixed(1)}  (|mean|/std)\n` +
    `rate   ${st.rate.toFixed(1)} Hz  n=${st.n}\n` +
    `quality <span class="${q.cls}">${q.text}</span>`;
}

function clearHist() {
  hist.t = [];
  for (const ch of CHANNELS) hist[ch.key] = [];
  draw();
  updateSigStats(true);
}

function pushSample(t, tHostMs = null) {
  if (view.pause) return;
  hist.t.push(tHostMs == null ? performance.now() : tHostMs);
  for (const ch of CHANNELS) {
    const v = Number(t[ch.key]);
    hist[ch.key].push(Number.isFinite(v) ? v : 0);
    latestVals[ch.key] = v;
  }
  while (hist.t.length > MAX_POINTS) {
    hist.t.shift();
    for (const ch of CHANNELS) hist[ch.key].shift();
  }
  plotDirty = true;
}

function activeSeries() {
  return CHANNELS.filter((c) => c.on);
}

function autoscaleY(force = true) {
  const series = activeSeries();
  if (!series.length || hist.t.length < 2) {
    if (force) {
      view.yMin = -1;
      view.yMax = 1;
    }
    return;
  }
  const tEnd = hist.t[hist.t.length - 1];
  const tStart = tEnd - view.timeWindowMs;
  let lo = Infinity;
  let hi = -Infinity;
  for (let i = 0; i < hist.t.length; i++) {
    if (hist.t[i] < tStart) continue;
    for (const ch of series) {
      const v = hist[ch.key][i];
      if (v < lo) lo = v;
      if (v > hi) hi = v;
    }
  }
  if (!Number.isFinite(lo) || !Number.isFinite(hi)) return;
  if (lo === hi) {
    lo -= 1;
    hi += 1;
  }
  const pad = (hi - lo) * 0.12;
  view.yMin = lo - pad;
  view.yMax = hi + pad;
  view.autoY = true;
}

function draw() {
  const dpr = window.devicePixelRatio || 1;
  const cssW = canvas.clientWidth || 900;
  const cssH = 360;
  if (canvas.width !== Math.floor(cssW * dpr) || canvas.height !== Math.floor(cssH * dpr)) {
    canvas.width = Math.floor(cssW * dpr);
    canvas.height = Math.floor(cssH * dpr);
  }
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  const w = cssW;
  const h = cssH;
  ctx.clearRect(0, 0, w, h);
  ctx.fillStyle = "#0d1118";
  ctx.fillRect(0, 0, w, h);

  const padL = 48;
  const padR = 10;
  const padT = 10;
  const padB = 22;
  const plotW = w - padL - padR;
  const plotH = h - padT - padB;

  ctx.strokeStyle = "#1b2434";
  ctx.lineWidth = 1;
  for (let i = 0; i <= 4; i++) {
    const y = padT + (plotH * i) / 4;
    ctx.beginPath();
    ctx.moveTo(padL, y);
    ctx.lineTo(padL + plotW, y);
    ctx.stroke();
  }
  for (let i = 0; i <= 5; i++) {
    const x = padL + (plotW * i) / 5;
    ctx.beginPath();
    ctx.moveTo(x, padT);
    ctx.lineTo(x, padT + plotH);
    ctx.stroke();
  }

  if (view.yMin < 0 && view.yMax > 0) {
    const zy =
      padT + plotH - ((0 - view.yMin) / (view.yMax - view.yMin || 1)) * plotH;
    ctx.strokeStyle = "#2a3348";
    ctx.beginPath();
    ctx.moveTo(padL, zy);
    ctx.lineTo(padL + plotW, zy);
    ctx.stroke();
  }

  ctx.fillStyle = "#6b7588";
  ctx.font = "11px IBM Plex Sans, sans-serif";
  ctx.textAlign = "right";
  ctx.textBaseline = "middle";
  for (let i = 0; i <= 4; i++) {
    const v = view.yMax - ((view.yMax - view.yMin) * i) / 4;
    const y = padT + (plotH * i) / 4;
    ctx.fillText(fmtAxis(v), padL - 6, y);
  }
  ctx.textAlign = "left";
  ctx.fillText(`${(view.timeWindowMs / 1000).toFixed(1)}s`, padL, h - 8);

  if (hist.t.length < 2) return;
  const tEnd = hist.t[hist.t.length - 1];
  const tStart = tEnd - view.timeWindowMs;

  function mapX(t) {
    return padL + ((t - tStart) / view.timeWindowMs) * plotW;
  }
  function mapY(v) {
    return padT + plotH - ((v - view.yMin) / (view.yMax - view.yMin || 1)) * plotH;
  }

  ctx.save();
  ctx.beginPath();
  ctx.rect(padL, padT, plotW, plotH);
  ctx.clip();

  for (const ch of activeSeries()) {
    const arr = hist[ch.key];
    ctx.strokeStyle = ch.color;
    ctx.lineWidth = ch.key === focusKey ? 2.0 : 1.4;
    ctx.globalAlpha = ch.key === focusKey ? 1 : 0.75;
    ctx.beginPath();
    let started = false;
    for (let i = 0; i < hist.t.length; i++) {
      if (hist.t[i] < tStart) continue;
      const x = mapX(hist.t[i]);
      const y = mapY(arr[i]);
      if (!started) {
        ctx.moveTo(x, y);
        started = true;
      } else {
        ctx.lineTo(x, y);
      }
    }
    if (started) ctx.stroke();

    // mean overlay for focused channel
    if (view.showMean && ch.key === focusKey) {
      const st = computeStats(ch.key);
      if (st) {
        const y = mapY(st.mean);
        ctx.setLineDash([4, 4]);
        ctx.strokeStyle = ch.color;
        ctx.globalAlpha = 0.55;
        ctx.lineWidth = 1;
        ctx.beginPath();
        ctx.moveTo(padL, y);
        ctx.lineTo(padL + plotW, y);
        ctx.stroke();
        ctx.setLineDash([]);
      }
    }
  }
  ctx.globalAlpha = 1;
  ctx.restore();
}

function fmtAxis(v) {
  const a = Math.abs(v);
  if (a >= 100) return v.toFixed(0);
  if (a >= 10) return v.toFixed(1);
  return v.toFixed(2);
}

function zoomY(factor, pivotRatio = 0.5) {
  const span = view.yMax - view.yMin;
  const center = view.yMin + span * pivotRatio;
  const newSpan = Math.max(1e-3, span * factor);
  view.yMin = center - newSpan * pivotRatio;
  view.yMax = center + newSpan * (1 - pivotRatio);
  view.autoY = false;
  draw();
  updateSigStats(true);
}

function zoomTime(factor) {
  view.timeWindowMs = Math.min(60000, Math.max(200, view.timeWindowMs * factor));
  if (view.autoY) autoscaleY(false);
  draw();
  updateSigStats(true);
}

canvas.addEventListener(
  "wheel",
  (e) => {
    e.preventDefault();
    const rect = canvas.getBoundingClientRect();
    const y = e.clientY - rect.top;
    const pivot = Math.min(1, Math.max(0, 1 - y / rect.height));
    if (e.shiftKey) {
      zoomTime(e.deltaY > 0 ? 1.15 : 1 / 1.15);
    } else {
      zoomY(e.deltaY > 0 ? 1.15 : 1 / 1.15, pivot);
    }
  },
  { passive: false }
);

canvas.addEventListener("pointerdown", (e) => {
  view.dragging = true;
  view.lastY = e.clientY;
  canvas.setPointerCapture(e.pointerId);
});
canvas.addEventListener("pointermove", (e) => {
  if (!view.dragging) return;
  const dy = e.clientY - view.lastY;
  view.lastY = e.clientY;
  const span = view.yMax - view.yMin;
  const delta = (dy / canvas.clientHeight) * span;
  view.yMin += delta;
  view.yMax += delta;
  view.autoY = false;
  draw();
});
canvas.addEventListener("pointerup", () => {
  view.dragging = false;
});
canvas.addEventListener("dblclick", () => {
  autoscaleY(true);
  draw();
});

document.getElementById("btnZoomIn").onclick = () => zoomY(1 / 1.25);
document.getElementById("btnZoomOut").onclick = () => zoomY(1.25);
document.getElementById("btnTimeIn").onclick = () => zoomTime(1 / 1.25);
document.getElementById("btnTimeOut").onclick = () => zoomTime(1.25);
document.getElementById("btnAuto").onclick = () => {
  autoscaleY(true);
  draw();
};
document.getElementById("btnClearPlot").onclick = () => clearHist();
document.getElementById("chkPause").onchange = (e) => {
  view.pause = e.target.checked;
};

function appendMsgs(items) {
  if (!items || !items.length) return;
  const showTelem = document.getElementById("chkLogTelem").checked;
  const follow = document.getElementById("chkLogFollow").checked;
  const nearBottom =
    msgLogEl.scrollHeight - msgLogEl.scrollTop - msgLogEl.clientHeight < 40;
  for (const m of items) {
    if (
      m.kind === "RX" &&
      m.text &&
      (m.text.startsWith("TEL ") || m.text.startsWith("REPLY ")) &&
      !showTelem
    ) {
      continue;
    }
    const div = document.createElement("div");
    div.className = `msg kind-${m.kind || "SYS"}`;
    const ts = new Date(m.ts * 1000);
    const hh = String(ts.getHours()).padStart(2, "0");
    const mm = String(ts.getMinutes()).padStart(2, "0");
    const ss = String(ts.getSeconds()).padStart(2, "0");
    const ms = String(ts.getMilliseconds()).padStart(3, "0");
    div.innerHTML =
      `<span class="ts">${hh}:${mm}:${ss}.${ms}</span>` +
      `[${m.kind}] ${escapeHtml(m.text)}`;
    msgLogEl.appendChild(div);
    lastMsgId = Math.max(lastMsgId, m.id);
  }
  while (msgLogEl.childElementCount > 500) {
    msgLogEl.removeChild(msgLogEl.firstChild);
  }
  if (follow && nearBottom) {
    msgLogEl.scrollTop = msgLogEl.scrollHeight;
  }
}

function escapeHtml(s) {
  return String(s)
    .replaceAll("&", "&amp;")
    .replaceAll("<", "&lt;")
    .replaceAll(">", "&gt;");
}

document.getElementById("btnClearLog").onclick = async () => {
  await postJson("/api/messages/clear", {});
  msgLogEl.innerHTML = "";
  lastMsgId = 0;
};

portSelect.onchange = () => {
  if (portSelect.value) portCustom.value = portSelect.value;
};

async function refreshPorts() {
  const j = await fetch("/api/ports").then((r) => r.json());
  fillPorts(j.ports || [], j.interface);
  if (j.connected) {
    portStatusEl.textContent = `connected: ${j.interface}`;
    showMotor(j);
  } else {
    portStatusEl.textContent =
      j.ports && j.ports.length
        ? "not connected — select or type a port"
        : "no SocketCAN iface yet — type name (e.g. can0) after bringing link up";
    showMotor({ motor_ok: false });
  }
  return j;
}

document.getElementById("btnScan").onclick = () => refreshPorts();
document.getElementById("btnConnect").onclick = async () => {
  const iface = selectedIface();
  if (!iface) {
    portStatusEl.textContent = "先扫描或输入接口名（如 can0）";
    return;
  }
  portStatusEl.textContent = `正在打开 CAN ${iface}（ip link up + 连接）…`;
  motorStatusEl.textContent = "正在探测电机…";
  motorStatusEl.classList.remove("ok", "fail");
  if (motorInfoEl) motorInfoEl.textContent = "";
  const j = await postJson("/api/can/open", { interface: iface });
  fillPorts(j.ports || [], iface);
  if (!j.ok) {
    portStatusEl.textContent = `打开失败: ${j.error || ""}`;
    if (motorInfoEl && j.hint) motorInfoEl.textContent = j.hint;
  } else {
    portStatusEl.textContent = `CAN 已打开: ${j.interface}`;
  }
  showMotor(j);
};
document.getElementById("btnDisconnect").onclick = async () => {
  const iface = selectedIface();
  portStatusEl.textContent = `正在关闭 CAN ${iface || ""}…`;
  const j = await postJson("/api/can/close", { interface: iface });
  fillPorts(j.ports || []);
  if (!j.ok) {
    portStatusEl.textContent = `关闭失败: ${j.error || ""}`;
  } else {
    portStatusEl.textContent = "CAN 已关闭";
  }
  statusEl.textContent = "idle";
  showMotor({ motor_ok: false });
};

async function postCmd(body) {
  const r = await fetch("/api/cmd", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(body),
  });
  const j = await r.json();
  if (!j.ok) {
    statusEl.textContent = `error: ${j.error || JSON.stringify(j)}`;
    return j;
  }
  if (j.stream) {
    statusEl.textContent = `stream ${j.stream} @ 50Hz`;
  } else {
    statusEl.textContent = `ACK ok cmd=${j.cmd} status=${j.status}`;
  }
  return j;
}

document.getElementById("btnStop").onclick = () => postCmd({ op: "stop" });
function wrapTwoPi(value) {
  const twoPi = Math.PI * 2;
  let x = Number(value) % twoPi;
  if (x < 0) x += twoPi;
  return x;
}
function wrapNegPiToPi(value) {
  const twoPi = Math.PI * 2;
  let x = ((Number(value) + Math.PI) % twoPi + twoPi) % twoPi - Math.PI;
  return x;
}
function optionalNumber(id) {
  const el = document.getElementById(id);
  if (!el) return null;
  const raw = (el.value || "").trim();
  if (raw === "" || !Number.isFinite(Number(raw))) return null;
  return Number(raw);
}
document.getElementById("btnVel").onclick = () => {
  const input = document.getElementById("omegaMech");
  const omega = Number(input.value);
  if (!Number.isFinite(omega)) {
    statusEl.textContent = "ω 无效";
    return;
  }
  input.value = String(omega);
  const body = {
    op: "servo",
    omega_mech: omega,
    id: Number(document.getElementById("idA").value),
    feedforward: Number(document.getElementById("feedforwardNm").value || 0),
    kp_scale: Number(document.getElementById("kpScale").value || 1),
    kd_scale: Number(document.getElementById("kdScale").value || 1),
    ilimit_scale: Number(document.getElementById("ilimitScale").value || 1),
  };
  const posEl = document.getElementById("positionMech");
  const posRaw = (posEl.value || "").trim();
  const hasPos = posRaw !== "" && Number.isFinite(Number(posRaw));
  if (hasPos) {
    // Follow commanded ω until the target, then stop (moteus velocity+stop_position).
    // This is NOT "position target + velocity_limit"; that felt like a teleport on short moves.
    const target = wrapTwoPi(posRaw);
    posEl.value = String(target);
    const speed = Math.abs(omega);
    if (speed < 1e-6) {
      body.position = target;
      body.omega_mech = 0;
    } else {
      const current = Number(latestVals.enc_theta_mech_rad);
      const err = Number.isFinite(current) ? wrapNegPiToPi(target - current) : 0;
      const dir = err === 0 ? (Math.sign(omega) || 1) : Math.sign(err);
      body.omega_mech = dir * speed;
      body.stop_position = target;
    }
  }
  const stopEl = document.getElementById("stopPositionMech");
  const stopRaw = (stopEl.value || "").trim();
  if (!hasPos && stopRaw !== "" && Number.isFinite(Number(stopRaw))) {
    const stop = wrapTwoPi(stopRaw);
    stopEl.value = String(stop);
    body.stop_position = stop;
  }
  const maxTorque = optionalNumber("maxTorqueNm");
  if (maxTorque !== null) body.max_torque = maxTorque;
  const velLimit = optionalNumber("velocityLimit");
  if (velLimit !== null) body.velocity_limit = velLimit;
  const accelLimit = optionalNumber("accelLimit");
  if (accelLimit !== null) body.accel_limit = accelLimit;
  postCmd(body);
};
document.getElementById("btnCurrent").onclick = () => {
  postCmd({
    op: "current",
    id: Number(document.getElementById("idA").value),
    iq: Number(document.getElementById("iqA").value),
  });
};
document.getElementById("btnMit").onclick = () => {
  const body = {
    op: "mit",
    position: Number(document.getElementById("mitPos").value),
    velocity: Number(document.getElementById("mitVel").value),
    kp: Number(document.getElementById("mitKp").value),
    kd: Number(document.getElementById("mitKd").value),
    feedforward: Number(document.getElementById("feedforwardNm").value || 0),
  };
  const maxTorque = optionalNumber("maxTorqueNm");
  if (maxTorque !== null) body.max_torque = maxTorque;
  if (![body.position, body.velocity, body.kp, body.kd].every(Number.isFinite)) {
    statusEl.textContent = "MIT 参数无效";
    return;
  }
  postCmd(body);
};

const calMethodEl = document.getElementById("calMethod");
const calParamsEncEl = document.getElementById("calParamsEnc");
const calParamsBemfEl = document.getElementById("calParamsBemf");
const calParamsREl = document.getElementById("calParamsR");
const calParamsLEl = document.getElementById("calParamsL");
const calParamsCoggingEl = document.getElementById("calParamsCogging");
const calParamsEncCompEl = document.getElementById("calParamsEncComp");
const btnEncCompFromSnap = document.getElementById("btnEncCompFromSnap");
const btnEncCompClear = document.getElementById("btnEncCompClear");
function updateCalParamsVisibility() {
  const m = calMethodEl.value;
  calParamsEncEl.hidden =
      m === "bemf" || m === "r" || m === "l" || m === "cogging" || m === "enc_comp";
  calParamsBemfEl.hidden = m !== "bemf";
  calParamsREl.hidden = m !== "r";
  calParamsLEl.hidden = m !== "l";
  if (calParamsCoggingEl) calParamsCoggingEl.hidden = m !== "cogging";
  if (calParamsEncCompEl) calParamsEncCompEl.hidden = m !== "enc_comp";
  if (btnEncCompFromSnap) btnEncCompFromSnap.hidden = m !== "enc_comp";
  if (btnEncCompClear) btnEncCompClear.hidden = m !== "enc_comp";
}
calMethodEl.addEventListener("change", updateCalParamsVisibility);
updateCalParamsVisibility();

document.getElementById("btnCalStart").onclick = () => {
  const m = calMethodEl.value;
  if (m === "lock") {
    postCmd({
      op: "cal_lock",
      current: Number(document.getElementById("calCurrent").value),
    });
  } else if (m === "spin") {
    postCmd({
      op: "cal_enc",
      current: Number(document.getElementById("calCurrent").value),
      omega_elec: Number(document.getElementById("calOmega").value),
    });
  } else if (m === "bemf") {
    postCmd({
      op: "cal_bemf",
      max_speed: Number(document.getElementById("calBemfMaxSpeed").value),
      n_points: Number(document.getElementById("calBemfPoints").value),
    });
  } else if (m === "r") {
    postCmd({
      op: "cal_r",
      max_current: Number(document.getElementById("calRMaxCurrent").value),
      n_points: Number(document.getElementById("calRPoints").value),
    });
  } else if (m === "l") {
    postCmd({
      op: "cal_l",
      step_voltage: Number(document.getElementById("calLStepVoltage").value),
      n_trials: Number(document.getElementById("calLTrials").value),
    });
  } else if (m === "cogging") {
    postCmd({
      op: "cal_cogging",
      velocity: Number(document.getElementById("calCoggingVelocity").value),
      record_revs: Number(document.getElementById("calCoggingRevs").value),
    });
  } else if (m === "enc_comp") {
    runEncComp({ action: "run" });
  }
};

async function runEncComp(opts) {
  const status = document.getElementById("calStatus");
  const verdict = document.getElementById("calVerdict");
  const box = document.getElementById("calResult");
  const bar = document.getElementById("calProgressBar");
  const action = opts.action || "run";
  if (status) {
    status.textContent =
      action === "clear"
        ? "清除几何补偿…"
        : action === "from_snap"
          ? "从上次 Snap 生成并写入…"
          : "恒速采集中（请勿操作 CAN）…";
  }
  if (verdict) {
    verdict.textContent = "运行中…";
    verdict.className = "status";
  }
  if (bar) bar.style.width = action === "clear" ? "30%" : "15%";
  try {
    const body =
      action === "clear"
        ? { action: "clear" }
        : action === "from_snap"
          ? { action: "from_snap" }
          : {
              action: "run",
              omega_mech: Number(document.getElementById("calEncCompOmega").value),
              seconds: Number(document.getElementById("calEncCompSeconds").value),
              rate: Number(document.getElementById("calEncCompRate").value),
            };
    const j = await postJson("/api/enc_comp", body);
    if (bar) bar.style.width = "100%";
    if (!j.ok) {
      if (status) status.textContent = `失败: ${j.error || JSON.stringify(j)}`;
      if (verdict) {
        verdict.textContent = "不合格 — 几何补偿未写入";
        verdict.className = "status fail";
      }
      if (box) box.textContent = JSON.stringify(j, null, 2);
      return;
    }
    if (action === "clear") {
      if (status) status.textContent = "几何补偿已清除";
      if (verdict) {
        verdict.textContent = "已清除 — 表已禁用并写 Flash";
        verdict.className = "status";
      }
      if (box) box.textContent = "encoder geometric compensation cleared";
      return;
    }
    const pct = 100 * Number(j.vel_dev_std || 0);
    if (status) {
      status.textContent = `完成 · source=${j.source} · samples=${j.n_samples}`;
    }
    if (verdict) {
      const grade = pct < 2 ? "good" : pct < 5 ? "warn" : "bad";
      verdict.textContent =
        grade === "good"
          ? "良好 — 2/rev 偏差不大，表已写入"
          : grade === "warn"
            ? "已写入 — 仍建议复查磁环同心/平行"
            : "已写入 — 偏差偏大，优先检查磁环安装后再重做";
      verdict.className = "status";
    }
    if (box) {
      box.textContent = [
        `source=${j.source}`,
        `mean_ω=${Number(j.mean_omega).toFixed(3)} rad/s`,
        `采集前速度偏差 std=${pct.toFixed(2)}%`,
        `peak|Δθ|=${Number(j.peak_rad).toFixed(4)} rad (${Number(j.peak_deg).toFixed(2)}°)`,
        `bins=${j.filled_bins}/256  samples=${j.n_samples}`,
        "",
        "已写入固件 NVS。请重新 vel=200 Capture，看 ω harmonics 的 2 是否下降。",
      ].join("\n");
    }
  } catch (e) {
    if (status) status.textContent = `error: ${e}`;
    if (verdict) {
      verdict.textContent = "失败";
      verdict.className = "status fail";
    }
  }
}

if (btnEncCompFromSnap) {
  btnEncCompFromSnap.onclick = () => runEncComp({ action: "from_snap" });
}
if (btnEncCompClear) {
  btnEncCompClear.onclick = () => runEncComp({ action: "clear" });
}

document.getElementById("btnCalAbort").onclick = () =>
  postCmd({ op: "cal_abort" });

function judgeCalResult(t) {
  const resid = Number(t.cal_residual_rad) || 0;
  const samples = Number(t.cal_samples) || 0;
  const residDeg = (resid * 180) / Math.PI;
  const mapped = Number(t.cal_kind) === CAL_KIND_ENCODER_MAP;
  let grade = "bad";
  let title = "不合格";
  const goodLimit = mapped ? 0.12 : 0.2;
  const warnLimit = mapped ? 0.25 : 0.4;
  if (resid < goodLimit) {
    grade = "good";
    title = mapped ? "良好 — 64 点换相表可用" : "良好 — 仅全局 offset";
  } else if (resid < warnLimit) {
    grade = "warn";
    title = mapped ? "勉强 — 仅建议低速试闭环" : "勉强 — 请改做转圈映射";
  } else {
    grade = "bad";
    title = "不合格 — 不要进入高速编码器换相";
  }
  const tips = [];
  if (!t.cal_ok) tips.push("固件已拒绝该结果：未应用，也未写入 Flash NVS");
  else if (t.cal_persisted) tips.push("已自动写入 Flash NVS，掉电后仍会加载");
  else tips.push("运行时已应用；Flash 写入未确认，可重做一次标定");
  if (mapped) tips.push("残差已扣除正反转的恒定扭矩角，只评价周期映射一致性");
  if (!mapped) tips.push("锁定法没有周期补偿表；高速运行前必须完成转圈映射");
  if (samples < 200) tips.push("有效采样偏少，请降低转速或略增大校准电流后重做");
  if (resid >= warnLimit) {
    tips.push("确认轴空载、enc_raw 连续且正反转都能跟随");
    tips.push("校准电流可从 1.0 A 小步提高，最高 2.0 A");
  }
  return { grade, title, resid, residDeg, samples, mapped, tips };
}

const CAL_KIND_ENCODER_MAP = 1;
const CAL_KIND_BEMF = 3;
const CAL_KIND_RESISTANCE = 4;
const CAL_KIND_INDUCTANCE = 5;
const CAL_KIND_COGGING = 6;

function judgeBemfResult(t) {
  const ke = Number(t.cal_offset_rad) || 0; // V*s/rad
  const r2 = Number(t.cal_residual_rad) || 0; // 0..1
  const points = Number(t.cal_samples) || 0;
  const bemfVPerKrpm = (Math.sqrt(3) * ke * 2 * Math.PI * 1000) / 60;
  const torqueConstant = 1.5 * ke;
  let grade = "bad";
  let title = "不合格";
  if (r2 >= 0.98 && points >= 4) {
    grade = "good";
    title = "良好 — 可用于前馈";
  } else if (r2 >= 0.9 && points >= 3) {
    grade = "warn";
    title = "勉强 — 建议增加扫描点/转速范围";
  } else {
    grade = "bad";
    title = "不合格 — 检查负载是否为空载、转速是否达到目标";
  }
  return { grade, title, ke, r2, points, bemfVPerKrpm, torqueConstant };
}

function judgeRResult(t) {
  const r = Number(t.cal_offset_rad) || 0; // Ohm
  const r2 = Number(t.cal_residual_rad) || 0; // 0..1
  const points = Number(t.cal_samples) || 0;
  let grade = "bad";
  let title = "不合格";
  if (r2 >= 0.98 && points >= 4) {
    grade = "good";
    title = "良好 — 可用于电流环带宽计算";
  } else if (r2 >= 0.9 && points >= 3) {
    grade = "warn";
    title = "勉强 — 建议增加扫描点/最大电流";
  } else {
    grade = "bad";
    title = "不合格 — 检查电流是否达到目标、接线是否可靠";
  }
  return { grade, title, r, r2, points };
}

function judgeLResult(t) {
  const ld = Number(t.cal_offset_rad) || 0; // Henry
  const lq = Number(t.cal_residual_rad) || 0; // Henry
  const trials = Number(t.cal_samples) || 0; // both axes combined
  const ratio = ld > 0 ? lq / ld : 0;
  const plausible = ld >= 1e-6 && ld <= 0.02 && lq >= 1e-6 && lq <= 0.02;
  let grade = "bad";
  let title = "不合格";
  if (plausible && trials >= 8) {
    grade = "good";
    title = "良好 — Ld/Lq 可用于双轴电流环";
  } else if (plausible && trials >= 4) {
    grade = "warn";
    title = "勉强 — 建议增加每轴重复次数";
  } else {
    grade = "bad";
    title = plausible ? "不合格 — 有效试验次数太少" : "不合格 — Ld/Lq 超出合理范围";
  }
  return { grade, title, ld, lq, ratio, trials };
}

function updateCalPanel(t) {
  const pct = Math.round((t.cal_progress || 0) * 100);
  const bar = document.getElementById("calProgressBar");
  const status = document.getElementById("calStatus");
  const verdict = document.getElementById("calVerdict");
  const box = document.getElementById("calResult");
  if (!bar || !status || !verdict || !box) return;
  bar.style.width = `${pct}%`;
  status.textContent = `${t.cal_state_name || "idle"} · ${pct}% · samples=${t.cal_samples || 0}`;
  void maybeSyncMotorConfAfterCal(t);

  if (t.cal_kind === CAL_KIND_BEMF) {
    if (t.cal_state_name === "done" && t.cal_ok) {
      const j = judgeBemfResult(t);
      verdict.textContent = `判定：${j.title}（r² = ${j.r2.toFixed(4)}）`;
      box.classList.remove("cal-good", "cal-warn", "cal-bad");
      box.classList.add(`cal-${j.grade}`);
      box.textContent =
        `判定：${j.title}\n` +
        `Ke_dq = ${j.ke.toFixed(6)} V*s/rad\n` +
        `Kt = 1.5*Ke_dq = ${j.torqueConstant.toFixed(6)} Nm/A\n` +
        `vendor line-line Vpeak/krpm ≈ ${j.bemfVPerKrpm.toFixed(3)}\n` +
        `r² = ${j.r2.toFixed(4)}\n` +
        `points_used = ${j.points}\n\n` +
        `已写入 Config → Motor.bemf，并已生效\n` +
        `掉电保留：到 Config 页点 Save（标定 Flash 只记状态灯）\n` +
        `cal_flash = ${t.cal_persisted ? "OK" : "pending/fail"}`;
    } else if (t.cal_state_name === "failed") {
      verdict.textContent = "判定：失败 — 拟合点不足或电机未跟上目标转速";
      box.classList.remove("cal-good", "cal-warn", "cal-bad");
      box.classList.add("cal-bad");
      box.textContent =
        "FAILED — 确认已完成编码器标定、电机空载、扫描转速在可达范围内后重试";
    } else if (t.cal_state_name === "bemf_run") {
      verdict.textContent = "Ke 辨识中… 转轴会自行分级加速，保持空载";
      box.classList.remove("cal-good", "cal-warn", "cal-bad");
    }
    return;
  }
  if (t.cal_kind === CAL_KIND_RESISTANCE) {
    if (t.cal_state_name === "done" && t.cal_ok) {
      const j = judgeRResult(t);
      verdict.textContent = `判定：${j.title}（r² = ${j.r2.toFixed(4)}）`;
      box.classList.remove("cal-good", "cal-warn", "cal-bad");
      box.classList.add(`cal-${j.grade}`);
      box.textContent =
        `判定：${j.title}\n` +
        `R = ${j.r.toFixed(4)} Ohm\n` +
        `r² = ${j.r2.toFixed(4)}\n` +
        `points_used = ${j.points}\n\n` +
        `已写入 Config → Motor.R，并已生效\n` +
        `掉电保留：到 Config 页点 Save（标定 Flash 只记状态灯）\n` +
        `cal_flash = ${t.cal_persisted ? "OK" : "pending/fail"}\n\n` +
        `建议接下来做 L 辨识（会自动用这个 R 值）。`;
    } else if (t.cal_state_name === "failed") {
      verdict.textContent = "判定：失败 — 拟合点不足";
      box.classList.remove("cal-good", "cal-warn", "cal-bad");
      box.classList.add("cal-bad");
      box.textContent = "FAILED — 确认接线可靠、电流能达到目标后重试";
    } else if (t.cal_state_name === "r_run") {
      verdict.textContent = "R 辨识中… 转子会被电流拉住对齐，勿用手转";
      box.classList.remove("cal-good", "cal-warn", "cal-bad");
    }
    return;
  }
  if (t.cal_kind === CAL_KIND_INDUCTANCE) {
    if (t.cal_state_name === "done" && t.cal_ok) {
      const j = judgeLResult(t);
      verdict.textContent = `判定：${j.title}（两轴有效次数 ${j.trials}）`;
      box.classList.remove("cal-good", "cal-warn", "cal-bad");
      box.classList.add(`cal-${j.grade}`);
      box.textContent =
        `判定：${j.title}\n` +
        `Ld = ${(j.ld * 1e6).toFixed(3)} uH\n` +
        `Lq = ${(j.lq * 1e6).toFixed(3)} uH\n` +
        `Lq/Ld = ${j.ratio.toFixed(3)}\n` +
        `trials_used_total = ${j.trials}\n\n` +
        `已写入 Config → Motor.L = (Ld+Lq)/2，FOC 仍用独立 Ld/Lq\n` +
        `掉电保留：到 Config 页点 Save（标定 Flash 只记状态灯）\n` +
        `cal_flash = ${t.cal_persisted ? "OK" : "pending/fail"}`;
    } else if (t.cal_state_name === "failed") {
      verdict.textContent = "判定：失败 — D/Q 至少一轴有效试验不足";
      box.classList.remove("cal-good", "cal-warn", "cal-bad");
      box.classList.add("cal-bad");
      box.textContent =
        "FAILED — 先做 R；确认转子被 d 轴锁住，再小步调整阶跃电压";
    } else if (t.cal_state_name === "l_run") {
      verdict.textContent = "L 辨识中… 电压阶跃测试，勿用手转";
      box.classList.remove("cal-good", "cal-warn", "cal-bad");
    }
    return;
  }
  if (t.cal_kind === CAL_KIND_COGGING) {
    if (t.cal_state_name === "done" && t.cal_ok) {
      const scale = Number(t.cal_offset_rad) || 0;  // A per LSB
      const peak = Number(t.cal_residual_rad) || 0;  // peak A
      verdict.textContent = `判定：完成（峰值补偿电流 ${peak.toFixed(3)} A）`;
      box.classList.remove("cal-good", "cal-warn", "cal-bad");
      box.classList.add(peak > 1e-4 ? "cal-good" : "cal-warn");
      box.textContent =
        `判定：${peak > 1e-4 ? "已生成 1024 点齿槽补偿表" : "未测到明显齿槽（表为空）"}\n` +
        `peak_comp = ${peak.toFixed(4)} A\n` +
        `scale = ${scale.toExponential(3)} A/LSB\n` +
        `cogging_table = 1024 bins\n\n` +
        `已运行时生效（速度模式按转子位置前馈 q 电流）\n` +
        `flash_nvs = ${t.cal_persisted ? "OK，掉电后仍会加载" : "pending/fail — 可重做一次"}`;
    } else if (t.cal_state_name === "failed") {
      verdict.textContent = "判定：失败 — 某些位置采样不足";
      box.classList.remove("cal-good", "cal-warn", "cal-bad");
      box.classList.add("cal-bad");
      box.textContent =
        "FAILED — 先完成编码器标定、电机空载、降低扫描速度或增加记录圈数后重试";
    } else if (t.cal_state_name === "cogging_run") {
      verdict.textContent = "齿槽标定中… 会慢速正反各转设定圈数，保持空载";
      box.classList.remove("cal-good", "cal-warn", "cal-bad");
    }
    return;
  }
  if (t.cal_state_name === "done" && t.cal_ok) {
    const j = judgeCalResult(t);
    const offF = Number(t.cal_offset_rad).toFixed(6);
    verdict.textContent = `判定：${j.title}（residual ${j.resid.toFixed(3)} rad ≈ ${j.residDeg.toFixed(1)}°）`;
    box.classList.remove("cal-good", "cal-warn", "cal-bad");
    box.classList.add(`cal-${j.grade}`);
    box.textContent =
      `判定：${j.title}\n` +
      `sign = ${t.cal_sign}\n` +
      `offset_rad = ${offF}\n` +
      `residual_rms_elec = ${j.resid.toFixed(4)} rad\n` +
      `commutation_table = ${j.mapped ? "64 bins" : "none (global offset only)"}\n` +
      `samples = ${j.samples}\n` +
      `flash_nvs = ${t.cal_persisted ? "OK" : "pending/fail"}\n\n` +
      j.tips.map((s, i) => `${i + 1}. ${s}`).join("\n");
  } else if (t.cal_state_name === "failed") {
    verdict.textContent = "判定：失败 — 未完成有效拟合";
    box.classList.remove("cal-good", "cal-warn", "cal-bad");
    box.classList.add("cal-bad");
    box.textContent = "FAILED — 检查编码器/电压后重试";
  } else if (
    t.cal_state_name === "sense" ||
    t.cal_state_name === "fwd" ||
    t.cal_state_name === "rev" ||
    t.cal_state_name === "locking"
  ) {
    verdict.textContent =
      t.cal_state_name === "locking"
        ? "锁定中… 转子会被磁场拉住，勿用手转"
        : "转圈校准中… 保持轴空载";
    box.classList.remove("cal-good", "cal-warn", "cal-bad");
  }
}

async function tickTelem() {
  if (telemInflight) return;
  telemInflight = true;
  try {
    const t = await fetch(`/api/telem?after=${lastTelemSid}`).then((r) => r.json());
    if (!t.connected) {
      if (!statusEl.textContent.startsWith("ACK") && !statusEl.textContent.startsWith("stream")) {
        statusEl.textContent = "CAN disconnected";
      }
      return;
    }
    if (t && t.ok !== false && t.id_a !== undefined) {
      if (t.cal_state_name !== undefined) updateCalPanel(t);
      document.getElementById("sId").textContent =
        `${t.id_a.toFixed(3)} / ${t.idref_a.toFixed(3)} A`;
      document.getElementById("sIq").textContent =
        `${t.iq_a.toFixed(3)} / ${t.iqref_a.toFixed(3)} A`;
      document.getElementById("sMode").textContent =
        `${modeNames[t.mode] || t.mode} / ${t.cisr ? 1 : 0}`;
      // Empty samples = no new points. Do NOT re-push latest (creates stairs).
      const samples = Array.isArray(t.samples) ? t.samples : [];
      if (samples.length) {
        const t0 = performance.now();
        const n = samples.length;
        const hasMono = samples.every((s) => typeof s._t_mono === "number");
        for (let i = 0; i < n; i++) {
          const s = samples[i];
          const hostTs = hasMono
            ? t0 - (samples[n - 1]._t_mono - s._t_mono) * 1000
            : t0 - (n - 1 - i) * 20;
          pushSample(s, hostTs);
        }
      }
      if (typeof t.sid === "number" && t.sid > lastTelemSid) lastTelemSid = t.sid;
      const hz = Number(t.rx_hz) || 0;
      if (!statusEl.textContent.startsWith("ACK") && !statusEl.textContent.startsWith("stream")) {
        statusEl.textContent = `live ${hz.toFixed(0)} Hz @ ${t.interface || "?"}`;
      } else if (statusEl.textContent.startsWith("stream")) {
        statusEl.textContent = `stream ${hz.toFixed(0)} Hz @ ${t.interface || "?"}`;
      }
    }
  } catch (e) {
    statusEl.textContent = "telem offline";
  } finally {
    telemInflight = false;
  }
}

function pumpPlot() {
  if (plotDirty) {
    plotDirty = false;
    if (view.autoY) autoscaleY(false);
    renderTree(false);
    draw();
    updateSigStats(false);
  }
  requestAnimationFrame(pumpPlot);
}

async function tickMessages() {
  try {
    const j = await fetch(`/api/messages?after=${lastMsgId}`).then((r) =>
      r.json()
    );
    appendMsgs(j.messages || []);
  } catch (e) {
    /* ignore */
  }
}

treeDirty = true;
renderTree(true);
updateSigStats(true);
draw();
refreshPorts();
// Match firmware live telem (~50 Hz).
setInterval(tickTelem, 20);
setInterval(tickMessages, 200);
requestAnimationFrame(pumpPlot);
window.addEventListener("resize", () => {
  draw();
  drawSnap();
});

/* ---- Mode bar: Live / Snapshot ---- */
const SNAP_CHS = [
  { key: "id_a", label: "Id", color: "#5ec4a0", on: true },
  { key: "iq_a", label: "Iq", color: "#e0b35a", on: true },
  { key: "i1_a", label: "I1", color: "#6aa6ff", on: true },
  { key: "i2_a", label: "I2", color: "#4ec3e0", on: true },
  { key: "i3_a", label: "I3", color: "#3db8a0", on: true },
  { key: "theta_mech_rad", label: "θmech", color: "#c8a0d0", on: false, unit: "rad" },
  { key: "theta_elec_rad", label: "θelec", color: "#d0c070", on: false, unit: "rad" },
];
let uiMode = "live";
let snapData = null;
let snapView = { yMin: -2, yMax: 2, autoY: true };
const snapCanvas = document.getElementById("snapPlot");
const snapCtx = snapCanvas.getContext("2d");
const snapStatus = document.getElementById("snapStatus");
const snapInfo = document.getElementById("snapInfo");
const snapTree = document.getElementById("snapTree");

function setMode(mode) {
  uiMode = mode;
  document.getElementById("panelLive").hidden = mode !== "live";
  document.getElementById("panelSnap").hidden = mode !== "snap";
  const panelCal = document.getElementById("panelCal");
  if (panelCal) panelCal.hidden = mode !== "cal";
  const panelConfig = document.getElementById("panelConfig");
  if (panelConfig) panelConfig.hidden = mode !== "config";
  for (const btn of document.querySelectorAll(".modeBtn")) {
    btn.classList.toggle("active", btn.dataset.mode === mode);
  }
  if (mode === "live") draw();
  else if (mode === "snap") drawSnap();
  else if (mode === "config") {
    // Keep current form values; user can Refresh/Load explicitly.
  }
}

for (const btn of document.querySelectorAll(".modeBtn")) {
  btn.onclick = () => setMode(btn.dataset.mode);
}

function renderSnapTree() {
  snapTree.innerHTML = "";
  for (const ch of SNAP_CHS) {
    const leaf = document.createElement("div");
    leaf.className = "tree-leaf" + (ch.on ? " on" : "");
    leaf.innerHTML =
      `<input type="checkbox" ${ch.on ? "checked" : ""} />` +
      `<span class="swatch" style="background:${ch.color}"></span>` +
      `<span class="name">${ch.label}</span>`;
    const box = leaf.querySelector("input");
    box.onclick = (e) => e.stopPropagation();
    box.onchange = () => {
      ch.on = box.checked;
      renderSnapTree();
      drawSnap();
    };
    leaf.onclick = () => {
      ch.on = !ch.on;
      renderSnapTree();
      drawSnap();
    };
    snapTree.appendChild(leaf);
  }
}

function snapAutoscale() {
  if (!snapData) return;
  let lo = Infinity;
  let hi = -Infinity;
  for (const ch of SNAP_CHS) {
    if (!ch.on) continue;
    const arr = snapData.series[ch.key] || [];
    for (const v of arr) {
      if (v < lo) lo = v;
      if (v > hi) hi = v;
    }
  }
  if (!Number.isFinite(lo) || !Number.isFinite(hi)) return;
  if (lo === hi) {
    lo -= 1;
    hi += 1;
  }
  const pad = (hi - lo) * 0.12;
  snapView.yMin = lo - pad;
  snapView.yMax = hi + pad;
  snapView.autoY = true;
}

function drawSnap() {
  const dpr = window.devicePixelRatio || 1;
  const cssW = snapCanvas.clientWidth || 900;
  const cssH = 360;
  if (
    snapCanvas.width !== Math.floor(cssW * dpr) ||
    snapCanvas.height !== Math.floor(cssH * dpr)
  ) {
    snapCanvas.width = Math.floor(cssW * dpr);
    snapCanvas.height = Math.floor(cssH * dpr);
  }
  snapCtx.setTransform(dpr, 0, 0, dpr, 0, 0);
  const w = cssW;
  const h = cssH;
  snapCtx.clearRect(0, 0, w, h);
  snapCtx.fillStyle = "#0d1118";
  snapCtx.fillRect(0, 0, w, h);

  const padL = 48;
  const padR = 10;
  const padT = 10;
  const padB = 22;
  const plotW = w - padL - padR;
  const plotH = h - padT - padB;

  snapCtx.strokeStyle = "#1b2434";
  for (let i = 0; i <= 4; i++) {
    const y = padT + (plotH * i) / 4;
    snapCtx.beginPath();
    snapCtx.moveTo(padL, y);
    snapCtx.lineTo(padL + plotW, y);
    snapCtx.stroke();
  }

  snapCtx.fillStyle = "#6b7588";
  snapCtx.font = "11px IBM Plex Sans, sans-serif";
  snapCtx.textAlign = "right";
  snapCtx.textBaseline = "middle";
  for (let i = 0; i <= 4; i++) {
    const v = snapView.yMax - ((snapView.yMax - snapView.yMin) * i) / 4;
    const y = padT + (plotH * i) / 4;
    snapCtx.fillText(fmtAxis(v), padL - 6, y);
  }

  if (!snapData) {
    snapCtx.textAlign = "left";
    snapCtx.fillText("no snapshot", padL, h - 8);
    return;
  }

  const n = snapData.n_samples;
  const hz = snapData.sample_hz || 1;
  snapCtx.textAlign = "left";
  snapCtx.fillText(`${n} pts @ ${hz} Hz (${((n / hz) * 1000).toFixed(1)} ms)`, padL, h - 8);

  function mapX(i) {
    return padL + (i / Math.max(1, n - 1)) * plotW;
  }
  function mapY(v) {
    return (
      padT +
      plotH -
      ((v - snapView.yMin) / (snapView.yMax - snapView.yMin || 1)) * plotH
    );
  }

  snapCtx.save();
  snapCtx.beginPath();
  snapCtx.rect(padL, padT, plotW, plotH);
  snapCtx.clip();
  for (const ch of SNAP_CHS) {
    if (!ch.on) continue;
    const arr = snapData.series[ch.key] || [];
    snapCtx.strokeStyle = ch.color;
    snapCtx.lineWidth = 1.5;
    snapCtx.beginPath();
    for (let i = 0; i < arr.length; i++) {
      const x = mapX(i);
      const y = mapY(arr[i]);
      if (i === 0) snapCtx.moveTo(x, y);
      else snapCtx.lineTo(x, y);
    }
    snapCtx.stroke();
  }
  snapCtx.restore();
}

document.getElementById("btnSnapAuto").onclick = () => {
  snapAutoscale();
  drawSnap();
};

document.getElementById("btnSnap").onclick = async () => {
  snapStatus.textContent = "capturing…";
  const n = Number(document.getElementById("snapN").value) || 512;
  const dec = Number(document.getElementById("snapDec").value) || 1;
  try {
    const j = await postJson("/api/snap", { n_samples: n, decimate: dec });
    if (!j.ok) {
      snapStatus.textContent = `失败: ${j.error || JSON.stringify(j)}`;
      snapStatus.classList.add("fail");
      return;
    }
    snapStatus.classList.remove("fail");
    snapData = j;
    snapStatus.textContent = `ok · ${j.n_samples} pts · ${j.sample_hz} Hz · ${(j.duration_us / 1000).toFixed(2)} ms`;
    snapInfo.textContent =
      `n=${j.n_samples}\nhz=${j.sample_hz}\ndec=${j.decimate}\n` +
      `dur=${(j.duration_us / 1000).toFixed(2)} ms\n` +
      `ch=${(j.channels || []).join(",")}`;
    snapAutoscale();
    drawSnap();
  } catch (e) {
    snapStatus.textContent = `error: ${e}`;
    snapStatus.classList.add("fail");
  }
};


function downloadSnapJson(data, filename) {
  const blob = new Blob([JSON.stringify(data, null, 2)], {
    type: "application/json",
  });
  const url = URL.createObjectURL(blob);
  const a = document.createElement("a");
  a.href = url;
  a.download = filename;
  document.body.appendChild(a);
  a.click();
  a.remove();
  URL.revokeObjectURL(url);
}

document.getElementById("btnSnapExport").onclick = async () => {
  if (!snapData) {
    snapStatus.textContent = "先 Capture 一次快照再 Export";
    snapStatus.classList.add("fail");
    return;
  }
  const w = Number(document.getElementById("snapOmega").value) || 0;
  const stamp = new Date().toISOString().replace(/[:.]/g, "-").slice(0, 19);
  const name = w > 0 ? `snap_${w}.json` : `snap_${stamp}.json`;
  const payload = {
    ok: true,
    n_samples: snapData.n_samples,
    sample_hz: snapData.sample_hz,
    decimate: snapData.decimate,
    duration_us: snapData.duration_us,
    channels: snapData.channels,
    series: snapData.series,
    omega_mech_rad_s: w,
    exported_at: new Date().toISOString(),
  };
  downloadSnapJson(payload, name);
  // Also write under tools/host/ so snap_analysis.py can open it directly.
  let serverNote = "";
  try {
    const j = await postJson("/api/snap/export", { omega_mech: w, filename: name });
    if (j.ok) serverNote = ` · saved ${j.path}`;
    else serverNote = ` · server save failed: ${j.error || "?"}`;
  } catch (e) {
    serverNote = ` · server save failed: ${e}`;
  }
  snapStatus.classList.remove("fail");
  snapStatus.textContent = `exported ${name}${serverNote}`;
};

const snapAnalysisEl = document.getElementById("snapAnalysis");
document.getElementById("btnSnapAnalyze").onclick = async () => {
  if (!snapData) {
    snapAnalysisEl.textContent = "先 Capture 一次快照";
    return;
  }
  try {
    const w = Number(document.getElementById("snapOmega").value) || 0;
    const j = await postJson("/api/snap/analyze", {
      pole_pairs: Number(lastMotorInfo && lastMotorInfo.pole_pairs) || 14,
      omega_mech: w,
    });
    let encRatio = null;
    try {
      encRatio = await postJson("/api/enc/ratio");
    } catch (e) { encRatio = null; }
    if (!j.ok) {
      snapAnalysisEl.textContent = `失败: ${j.error || JSON.stringify(j)}`;
      return;
    }
    snapAnalysisEl.textContent = [
      `freq_hz: id=${j.freq_hz.id_a} iq=${j.freq_hz.iq_a} i1=${j.freq_hz.i1_a} i2=${j.freq_hz.i2_a} i3=${j.freq_hz.i3_a}`,
      `amp_A: id=${j.amp_A.id_a} iq=${j.amp_A.iq_a} i1=${j.amp_A.i1_a} i2=${j.amp_A.i2_a} i3=${j.amp_A.i3_a}`,
      `balance(min/max)=${j.balance_minmax_ratio}`,
      `i1+i2+i3 pkpk=${j.sum_pkpk_A} A @ ${j.sum_freq_hz} Hz`,
      `f(id/iq)/f(phase)=${j.dq_over_phase_ratio}`,
      j.f_elec_expected_hz
        ? `f_phase=${j.freq_hz.i1_a} Hz vs 期望电频 ${j.f_elec_expected_hz} Hz (比 ${j.f_phase_over_f_expected})`
        : "未对比期望电频率（ω 输入为 0）",
      encRatio && encRatio.samples > 4
        ? `极对数实测: Δθelec/Δθmech = ${encRatio.pp_ratio}（样本 ${encRatio.samples}，期望 14）`
        : "极对数实测: 样本不足（等电机稳定转一会再分析）",
      j.theta_dbg
        ? [
            `Δθmech=${j.theta_dbg.d_mech_rad}rad Δθelec=${j.theta_dbg.d_elec_rad}rad`,
            `f_mech(θ)=${j.f_mech_enc_hz}Hz f_elec(θ)=${j.f_elec_snap_hz}Hz`,
            `θmech[0:16]=${j.theta_dbg.mech_first16.join(",")}`,
            `θelec[0:16]=${j.theta_dbg.elec_first16.join(",")}`,
          ].join("\n")
        : "",
      j.omega_from_theta
        ? `ω(from θ): mean=${j.omega_from_theta.mean_rad_s} amp=±${j.omega_from_theta.amp_rad_s} rel=${(100 * j.omega_from_theta.rel_amp).toFixed(1)}% f=${j.omega_from_theta.ripple_hz}Hz cyc/rev=${j.omega_from_theta.cycles_per_rev} bestN=${j.omega_from_theta.best_harmonic}`
        : "",
      j.iq_ripple
        ? `Iq ripple: mean=${j.iq_ripple.mean_A}A amp=±${j.iq_ripple.amp_A}A f=${j.iq_ripple.freq_hz}Hz`
        : "",
      "解读: dq/相≈1→offset/1x · ≈2→增益失配/2x · 相频/期望≈1→极对数对 · 极对数实测≠14→角度换算 bug",
    ].join("\n");
  } catch (e) {
    snapAnalysisEl.textContent = `error: ${e}`;
  }
};



/* ---- Config panel ---- */
const CONF_GROUPS = {
  motor: [
    "pole_pairs",
    "R",
    "L",
    "bemf_Vpeak_per_krpm",
    "max_phase_current_A",
    "fw_speed_rad_s",
    "bus_V",
  ],
  foc: [
    "bandwidth_hz",
    "bemf_ff",
    "current_ff",
    "cross_ff",
    "max_current_rate",
  ],
  servo: [
    "kp",
    "ki",
    "kd",
    "ilimit",
    "sign",
    "max_iq",
    "vel_threshold",
    "slip",
    "vel_err",
    "default_vel_limit",
    "default_accel_limit",
  ],
  encoder: ["pll_filter_hz", "spike_error_rad", "filter_us"],
};

const confStatus = document.getElementById("confStatus");

function confInput(group, key) {
  return document.querySelector(`[data-conf="${group}.${key}"]`);
}

function setConfStatus(ok, msg) {
  if (!confStatus) return;
  confStatus.classList.toggle("ok", !!ok);
  confStatus.classList.toggle("fail", ok === false);
  confStatus.textContent = msg;
}

function readConfGroup(group) {
  const keys = CONF_GROUPS[group] || [];
  const fields = {};
  for (const key of keys) {
    const el = confInput(group, key);
    if (!el) continue;
    const raw = String(el.value ?? "").trim();
    if (raw === "") {
      // Servo accel empty => NaN (null over the wire).
      fields[key] = group === "servo" && key === "default_accel_limit" ? null : 0;
      continue;
    }
    const n = Number(raw);
    fields[key] = Number.isFinite(n) ? n : null;
  }
  return fields;
}

function writeConfGroup(group, fields) {
  if (!fields || typeof fields !== "object") return;
  const keys = CONF_GROUPS[group] || [];
  for (const key of keys) {
    const el = confInput(group, key);
    if (!el || !(key in fields)) continue;
    const v = fields[key];
    if (v === null || v === undefined || Number.isNaN(v)) {
      el.value = "";
    } else {
      el.value = String(v);
    }
  }
}

function writeConfAll(fieldsByGroup) {
  if (!fieldsByGroup) return;
  // get(all) returns {motor:{...}, foc:{...}, ...}
  for (const group of Object.keys(CONF_GROUPS)) {
    if (fieldsByGroup[group]) writeConfGroup(group, fieldsByGroup[group]);
  }
  if (fieldsByGroup.cal) writeCalStatus(fieldsByGroup.cal);
}

function writeCalStatus(cal) {
  const box = document.getElementById("confCalValues");
  const keys = ["encoder", "resistance", "inductance", "bemf", "cogging", "enc_comp"];
  for (const key of keys) {
    const el = document.querySelector(`[data-cal="${key}"]`);
    if (!el) continue;
    const on = !!(cal && cal[key]);
    el.classList.toggle("on", on);
    el.classList.toggle("off", !on);
  }
  if (!box) return;
  if (!cal) {
    box.textContent = "尚未读取";
    return;
  }
  const fmt = (v, unit) =>
    v === null || v === undefined || Number.isNaN(v)
      ? "—"
      : `${Number(v).toPrecision(4)}${unit}`;
  box.textContent =
    `R=${fmt(cal.R, " Ω")}  Ld=${fmt(cal.Ld, " H")}  Lq=${fmt(cal.Lq, " H")}\n` +
    `Ke(dq)=${fmt(cal.Ke, " V·s/rad")}  flags=0x${(cal.flags >>> 0).toString(16)}`;
}

async function postConf(body) {
  const j = await postJson("/api/conf", body);
  return j;
}

let lastMotorConfSyncKey = null;

async function maybeSyncMotorConfAfterCal(t) {
  if (!t || t.cal_state_name !== "done" || !t.cal_ok) return;
  const kind = Number(t.cal_kind);
  if (![CAL_KIND_RESISTANCE, CAL_KIND_INDUCTANCE, CAL_KIND_BEMF].includes(kind)) return;
  const key = `${kind}:${t.cal_samples || 0}:${Number(t.cal_offset_rad) || 0}`;
  if (key === lastMotorConfSyncKey) return;
  lastMotorConfSyncKey = key;
  try { await confGetAll(); } catch (_) {}
}

async function confGetAll() {
  setConfStatus(null, "getting config…");
  try {
    const j = await postConf({ op: "get", group: "all" });
    if (!j.ok) {
      setConfStatus(false, j.error || "get failed");
      return;
    }
    writeConfAll(j.fields || {});
    const flash = j.flash_valid ? "flash=valid" : "flash=empty";
    setConfStatus(true, `get ok (${flash})`);
  } catch (e) {
    setConfStatus(false, String(e.message || e));
  }
}

async function confApply(group) {
  setConfStatus(null, `applying ${group}…`);
  try {
    const fields = readConfGroup(group);
    const j = await postConf({ op: "set", group, fields });
    if (!j.ok) {
      setConfStatus(false, j.error || `set ${group} failed`);
      return;
    }
    if (j.fields) writeConfGroup(group, j.fields);
    setConfStatus(true, `set ${group} ok`);
  } catch (e) {
    setConfStatus(false, String(e.message || e));
  }
}

async function confSimple(op) {
  setConfStatus(null, `${op}…`);
  try {
    const j = await postConf({ op, group: "all" });
    if (!j.ok) {
      setConfStatus(false, j.error || `${op} failed`);
      return;
    }
    // After load/defaults, refresh RAM view.
    if (op === "load" || op === "defaults") {
      const g = await postConf({ op: "get", group: "all" });
      if (g.ok) writeConfAll(g.fields || {});
    }
    setConfStatus(true, `${op} ok`);
  } catch (e) {
    setConfStatus(false, String(e.message || e));
  }
}

const btnConfGet = document.getElementById("btnConfGet");
const btnConfLoad = document.getElementById("btnConfLoad");
const btnConfSave = document.getElementById("btnConfSave");
const btnConfDefaults = document.getElementById("btnConfDefaults");
if (btnConfGet) btnConfGet.onclick = () => confGetAll();
if (btnConfLoad) btnConfLoad.onclick = () => confSimple("load");
if (btnConfSave) btnConfSave.onclick = () => confSimple("save");
if (btnConfDefaults) btnConfDefaults.onclick = () => confSimple("defaults");
for (const btn of document.querySelectorAll("[data-conf-apply]")) {
  btn.onclick = () => confApply(btn.getAttribute("data-conf-apply"));
}


renderSnapTree();
setMode("live");


