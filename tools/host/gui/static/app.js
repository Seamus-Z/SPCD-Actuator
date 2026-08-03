/* Scope UI: telemetry tree + signal quality (PlotJuggler / Foxglove style). */
const modeNames = ["stop", "raw", "vfoc", "dq"];
const MAX_POINTS = 2000;

const CHANNELS = [
  { key: "id_a", label: "Id", path: "current/Id", color: "#5ec4a0", unit: "A", on: true, group: "current" },
  { key: "iq_a", label: "Iq", path: "current/Iq", color: "#e0b35a", unit: "A", on: true, group: "current" },
  { key: "idref_a", label: "Idref", path: "current/Idref", color: "#5a6a88", unit: "A", on: true, group: "current" },
  { key: "iqref_a", label: "Iqref", path: "current/Iqref", color: "#7a5a88", unit: "A", on: true, group: "current" },
  { key: "i1_a", label: "I1", path: "current/I1", color: "#6aa6ff", unit: "A", on: false, group: "current" },
  { key: "i2_a", label: "I2", path: "current/I2", color: "#4ec3e0", unit: "A", on: false, group: "current" },
  { key: "i3_a", label: "I3", path: "current/I3", color: "#3db8a0", unit: "A", on: false, group: "current" },
  { key: "vd_v", label: "Vd", path: "voltage/Vd", color: "#c47ad0", unit: "V", on: false, group: "voltage" },
  { key: "vq_v", label: "Vq", path: "voltage/Vq", color: "#d08a6a", unit: "V", on: false, group: "voltage" },
  { key: "bus_v", label: "Vbus", path: "voltage/Vbus", color: "#e08a8a", unit: "V", on: false, group: "voltage" },
  { key: "theta_rad", label: "theta", path: "motion/theta", color: "#a0a8b8", unit: "rad", on: false, group: "motion" },
  { key: "omega_rad_s", label: "omega", path: "motion/omega", color: "#88b0d0", unit: "rad/s", on: false, group: "motion" },
  { key: "enc_raw", label: "encRaw", path: "encoder/raw", color: "#c0d088", unit: "", on: true, group: "encoder" },
  { key: "enc_theta_mech_rad", label: "encMech", path: "encoder/theta_mech", color: "#a8c070", unit: "rad", on: true, group: "encoder" },
  { key: "enc_theta_elec_rad", label: "encElec", path: "encoder/theta_elec", color: "#88b060", unit: "rad", on: true, group: "encoder" },
  { key: "duty_a", label: "dutyA", path: "pwm/dutyA", color: "#b8c0d0", unit: "", on: false, group: "pwm" },
  { key: "duty_b", label: "dutyB", path: "pwm/dutyB", color: "#98a0b0", unit: "", on: false, group: "pwm" },
  { key: "duty_c", label: "dutyC", path: "pwm/dutyC", color: "#788090", unit: "", on: false, group: "pwm" },
];

const GROUPS = [
  { id: "current", label: "Current" },
  { id: "voltage", label: "Voltage" },
  { id: "motion", label: "Motion" },
  { id: "encoder", label: "Encoder" },
  { id: "pwm", label: "PWM" },
];

const hist = { t: [] };
for (const ch of CHANNELS) hist[ch.key] = [];

const statusEl = document.getElementById("status");
const portStatusEl = document.getElementById("portStatus");
const motorStatusEl = document.getElementById("motorStatus");
const motorInfoEl = document.getElementById("motorInfo");
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
let latestVals = {};
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
  if (j.motor_ok && j.info) {
    const i = j.info;
    motorStatusEl.classList.add("ok");
    motorStatusEl.textContent = `电机连接成功 · ${i.motor} · fw ${i.fw_version}`;
    motorInfoEl.textContent =
      `node=${i.node_id}  family=${i.family}\n` +
      `PWM ${i.pwm_hz} Hz  bus ${i.bus_v} V  Imax ${i.i_max_a} A\n` +
      `poles ${i.pole_pairs}  R ${i.r_ohm} Ω  L ${Number(i.l_uH).toFixed(0)} µH`;
  } else if (j.motor_error || (j.ok && j.motor_ok === false)) {
    motorStatusEl.classList.add("fail");
    motorStatusEl.textContent = j.motor_error || "电机连接失败";
    motorInfoEl.textContent = "";
  } else if (!j.ok && j.error) {
    motorStatusEl.classList.add("fail");
    motorStatusEl.textContent = `CAN 连接失败：${j.error}`;
    motorInfoEl.textContent = j.hint || "";
  } else {
    motorStatusEl.textContent = "电机未探测";
    motorInfoEl.textContent = "";
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

function pushSample(t) {
  if (view.pause) return;
  hist.t.push(performance.now());
  for (const ch of CHANNELS) {
    const v = Number(t[ch.key]);
    hist[ch.key].push(Number.isFinite(v) ? v : 0);
    latestVals[ch.key] = v;
  }
  while (hist.t.length > MAX_POINTS) {
    hist.t.shift();
    for (const ch of CHANNELS) hist[ch.key].shift();
  }
  if (view.autoY) autoscaleY(false);
  renderTree(false);
  draw();
  updateSigStats(false);
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
    if (m.kind === "RX" && m.text && m.text.startsWith("TEL ") && !showTelem) {
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
  motorInfoEl.textContent = "";
  const j = await postJson("/api/can/open", { interface: iface });
  fillPorts(j.ports || [], iface);
  if (!j.ok) {
    portStatusEl.textContent = `打开失败: ${j.error || ""}`;
    if (j.hint) motorInfoEl.textContent = j.hint;
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
  statusEl.textContent = j.ok
    ? `ACK ok cmd=${j.cmd} status=${j.status}`
    : `error: ${j.error || JSON.stringify(j)}`;
  return j;
}

document.getElementById("btnStop").onclick = () => postCmd({ op: "stop" });
document.getElementById("btnDq").onclick = () =>
  postCmd({
    op: "dq",
    id: Number(document.getElementById("idA").value),
    iq: Number(document.getElementById("iqA").value),
    omega: Number(document.getElementById("omega").value),
  });
document.getElementById("btnVfoc").onclick = () =>
  postCmd({
    op: "vfoc",
    theta: 0,
    v: Number(document.getElementById("vfocV").value),
    omega: Number(document.getElementById("omega").value),
  });

async function tickTelem() {
  try {
    const t = await fetch("/api/telem").then((r) => r.json());
    if (!t.connected) {
      if (!statusEl.textContent.startsWith("ACK")) {
        statusEl.textContent = "CAN disconnected";
      }
      return;
    }
    if (t && t.ok !== false && t.id_a !== undefined) {
      document.getElementById("sId").textContent =
        `${t.id_a.toFixed(3)} / ${t.idref_a.toFixed(3)} A`;
      document.getElementById("sIq").textContent =
        `${t.iq_a.toFixed(3)} / ${t.iqref_a.toFixed(3)} A`;
      document.getElementById("sMode").textContent =
        `${modeNames[t.mode] || t.mode} / ${t.cisr ? 1 : 0}`;
      pushSample(t);
      if (!statusEl.textContent.startsWith("ACK")) {
        statusEl.textContent = `live @ ${t.interface || "?"}`;
      }
    }
  } catch (e) {
    statusEl.textContent = "telem offline";
  }
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
setInterval(tickTelem, 10);
setInterval(tickMessages, 200);
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
  for (const btn of document.querySelectorAll(".modeBtn")) {
    btn.classList.toggle("active", btn.dataset.mode === mode);
  }
  if (mode === "live") draw();
  else drawSnap();
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

renderSnapTree();
setMode("live");

