// ============================================================================
// [W2] Web Dashboard client — SSE 4Hz + polling fallback, canvas map/series
// Phong cách "Tactical Instrument" (palette khớp desktop/src/ui_theme.hpp)
// ============================================================================
"use strict";

const C = {
  bg: "#0E100F", line: "#2C302A", lineStrong: "#4A5046", grid: "#262A25",
  text: "#D9D6CC", muted: "#7D8075", dim: "#54584E",
  amber: "#E8A33D", olive: "#8A9A5B", oliveBright: "#B0C276",
  red: "#D9493A", yellow: "#E8D03D", blue: "#4F8FD6", blueSoft: "#78AAE1", terra: "#CC7040",
};
const MONO = '"JetBrains Mono", "Cascadia Mono", Consolas, monospace';
const CLUSTER_COLORS = [C.amber, C.oliveBright, C.text, C.terra];

const state = {
  view: { center_e: 1000, center_n: 1000, ppm: 0.30 },  // giống MapView desktop
  dragging: false,
  last_x: 0,
  last_y: 0,
  fitted: false,
  last_event_ms: 0,
  last: null,          // snapshot gần nhất (vẽ lại khi pan/zoom)
};

const ACTIVITY = [
  ["IDLE", "ĐỨNG YÊN"], ["WALK", "ĐI BỘ"], ["RUN", "CHẠY"], ["STAIR UP", "LÊN THANG"],
  ["STAIR DN", "XUỐNG THANG"], ["LANDING", "CHIẾU NGHỈ"], ["ESCALATOR", "THANG CUỐN"],
  ["CYCLE", "ĐẠP XE"], ["VEHICLE", "XE CƠ"], ["ELEVATOR", "THANG MÁY"], ["CABLE CAR", "CÁP TREO"],
];
const SAFETY = [
  { text: "STATUS: NOMINAL — AN TOÀN BÌNH THƯỜNG", cls: "nominal" },
  { text: "PRE-ALERT — PHÁT HIỆN NGÃ · ĐẾM NGƯỢC 45s", cls: "prealert" },
  { text: "DISTRESS — KHÓA CỨU NẠN SOS TỰ ĐỘNG", cls: "distress" },
  { text: "CIRCLING — ĐI VÒNG QUANH, MẤT PHƯƠNG HƯỚNG", cls: "panic" },
];
const SIM_EPOCH_MS = 1700000000000;  // gốc current_time_ns của AppState

const $ = (id) => document.getElementById(id);

// Thước 10 vạch
const meterEl = $("conf-meter");
meterEl.innerHTML = "<i></i>".repeat(10);
const meterTicks = [...meterEl.children];
const gnsBars = [...$("gns-bars").children];

// ---------- Kết nối: SSE với fallback polling ----------
function setConn(text, cls) {
  const conn = $("conn");
  conn.className = "conn " + cls;
  conn.querySelector("b").textContent = text;
}

function connectSSE() {
  const es = new EventSource("/api/events");
  es.onopen = () => setConn("LINK · SSE 4Hz", "on");
  es.onmessage = (ev) => {
    state.last_event_ms = Date.now();
    render(JSON.parse(ev.data));
  };
  es.onerror = () => {
    es.close();
    setConn("SSE LOST · POLLING", "warn");
    setTimeout(pollLoop, 1000);
  };
}

function pollLoop() {
  fetch("/api/state").then(r => r.json()).then(data => {
    setConn("LINK · POLL 1Hz", "on");
    state.last_event_ms = Date.now();
    render(data);
    setTimeout(pollLoop, 1000);
  }).catch(() => {
    setConn("NO LINK · RETRY", "off");
    setTimeout(pollLoop, 2000);
  });
}

// ---------- Tiện ích ----------
const tri = (v, good, warn) => (v > good ? C.oliveBright : v > warn ? C.amber : C.red);
const pad = (n, w) => String(Math.max(0, Math.floor(n))).padStart(w, "0");
function groupHex4(s) {
  const raw = (s || "").replace(/\s+/g, "");
  return raw.match(/.{1,4}/g)?.join(" ") || "—";
}

// ---------- Render ----------
function render(d) {
  state.last = d;

  // Đồng hồ nhiệm vụ
  if (d.timestamp_ms) {
    const el = Math.max(0, (d.timestamp_ms - SIM_EPOCH_MS) / 1000);
    $("clock").textContent = `T+${pad(el / 3600, 2)}:${pad((el / 60) % 60, 2)}:${pad(el % 60, 2)}`;
  }

  // Dải an toàn
  const banner = $("safety-banner");
  const safety = SAFETY[d.safety_state] || SAFETY[0];
  banner.className = "strip " + safety.cls;
  banner.querySelector(".safety-text").textContent = safety.text;

  // Số đo
  const gns = d.gns ?? 0;
  const gnsEl = $("gns");
  gnsEl.textContent = Math.round(gns * 100) + "%";
  const gCol = tri(gns, 0.7, 0.4);
  gnsEl.style.color = gCol;
  const lit = Math.round(gns * gnsBars.length);
  gnsBars.forEach((b, i) => { b.style.background = i < lit ? gCol : ""; });
  $("ce95").textContent = "±" + (d.ce95_m ?? 0).toFixed(1);
  $("steps").textContent = pad(d.steps ?? 0, 4);
  $("dist").textContent = (d.distance ?? 0).toFixed(1).padStart(5, "0");
  $("alt").textContent = (d.altitude ?? 0).toFixed(1);
  const bat = $("battery");
  bat.textContent = Math.round(d.battery ?? 0) + "%";
  bat.style.color = tri(d.battery ?? 0, 40, 15);
  $("fps").textContent = "FPS " + (d.fps ?? 0).toFixed(0);
  $("age").textContent = "AGE " + ((Date.now() - state.last_event_ms) / 1000).toFixed(1) + "s";

  // Hoạt động
  const [code, vi] = ACTIVITY[d.activity] || ["UNKNOWN", "KHÔNG RÕ"];
  $("activity").textContent = code;
  $("activity-vi").textContent = "· " + vi;
  const conf = d.activity_conf ?? 0;
  const on = Math.round(conf * 10);
  meterTicks.forEach((t, i) => t.classList.toggle("on", i < on));
  $("conf-pct").textContent = Math.round(conf * 100) + "%";
  const stair = d.stair || {};
  $("stair").textContent = stair.steps > 0
    ? `THANG · ${stair.steps} bậc · riser ${stair.riser.toFixed(2)}m · Δz ${stair.vertical >= 0 ? "+" : ""}${stair.vertical.toFixed(2)}m · landing ${stair.landings}`
    : "THANG · chưa có dữ liệu";

  // Ma trận sức khỏe
  const f = d.flags || {};
  const cells = [
    ["IMU", f.imu ? "ok" : "bad"], ["BARO", f.baro ? "ok" : "bad"],
    ["MAG", !f.mag_anomaly ? "ok" : "bad"], ["CEL", f.celestial ? "info" : ""],
    ["ZUPT", f.zupt ? "info" : ""], ["DIV", !f.diverged ? "ok" : "bad"],
    ["STALE", !f.stale ? "ok" : "bad"], ["IMMOB", !f.immobile ? "ok" : "bad"],
  ];
  $("flags").innerHTML = cells.map(([l, c]) => `<div class="cell ${c}">${l}</div>`).join("");
  const cc = d.calibrated_containment ?? 0;
  $("mode").textContent = Math.round(cc <= 1 ? cc * 100 : cc) + "%";

  // Cứu hộ
  $("packet-hex").textContent = groupHex4(d.packet_hex);
  $("packet-sms").textContent = d.packet_sms || "— (bấm LƯU LỊCH SỬ trên desktop để tạo gói)";

  drawMap(d);
  drawSeries(d);
}

// ---------- Canvas HiDPI ----------
function fitCanvas(cv) {
  const dpr = window.devicePixelRatio || 1;
  const w = Math.max(1, Math.round(cv.clientWidth * dpr));
  const h = Math.max(1, Math.round(cv.clientHeight * dpr));
  if (cv.width !== w || cv.height !== h) { cv.width = w; cv.height = h; }
  const ctx = cv.getContext("2d");
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  return { ctx, W: cv.clientWidth, H: cv.clientHeight };
}

function niceStep(ppm, minPx) {
  for (const s of [1, 2, 5, 10, 25, 50, 100, 250, 500, 1000, 2500, 5000, 10000]) {
    if (s * ppm >= minPx) return s;
  }
  return 10000;
}

// ---------- Bản đồ tactical (cùng quy ước MapView desktop) ----------
const mapCanvas = $("map");
let mapW = 800, mapH = 520;
const w2sX = (e) => (e - state.view.center_e) * state.view.ppm + mapW / 2;
const w2sY = (n) => -(n - state.view.center_n) * state.view.ppm + mapH / 2;

function drawMap(d) {
  if (!d) return;
  const { ctx, W, H } = fitCanvas(mapCanvas);
  mapW = W; mapH = H;
  const v = state.view;

  if (!state.fitted && d.trail && d.trail.length >= 2) {
    fitAll(d);
    state.fitted = true;
  }

  ctx.fillStyle = C.bg;
  ctx.fillRect(0, 0, W, H);

  const eMin = v.center_e - W / 2 / v.ppm, eMax = v.center_e + W / 2 / v.ppm;
  const nMin = v.center_n - H / 2 / v.ppm, nMax = v.center_n + H / 2 / v.ppm;
  const gridM = niceStep(v.ppm, 90);
  const minorM = gridM / 5;

  // Lưới chấm phụ
  if (minorM * v.ppm >= 12) {
    ctx.fillStyle = C.line;
    for (let e = Math.floor(eMin / minorM) * minorM; e <= eMax; e += minorM) {
      const x = Math.round(w2sX(e));
      for (let n = Math.floor(nMin / minorM) * minorM; n <= nMax; n += minorM) {
        ctx.fillRect(x, Math.round(w2sY(n)), 1, 1);
      }
    }
  }

  // Lưới chính + nhãn tọa độ mép
  ctx.strokeStyle = C.grid;
  ctx.lineWidth = 1;
  ctx.beginPath();
  for (let e = Math.floor(eMin / gridM) * gridM; e <= eMax; e += gridM) {
    const x = Math.round(w2sX(e)) + 0.5;
    ctx.moveTo(x, 0); ctx.lineTo(x, H);
  }
  for (let n = Math.floor(nMin / gridM) * gridM; n <= nMax; n += gridM) {
    const y = Math.round(w2sY(n)) + 0.5;
    ctx.moveTo(0, y); ctx.lineTo(W, y);
  }
  ctx.stroke();
  ctx.font = `11px ${MONO}`;
  ctx.fillStyle = C.muted;
  for (let e = Math.floor(eMin / gridM) * gridM; e <= eMax; e += gridM) {
    ctx.fillText("E " + pad(e, 4), w2sX(e) + 4, 34);
  }
  for (let n = Math.floor(nMin / gridM) * gridM; n <= nMax; n += gridM) {
    ctx.fillText("N " + pad(n, 4), 6, w2sY(n) - 5);
  }
  $("grid-label").textContent = `UTM GRID ${gridM} m`;

  // Đường đồng mức XANH DƯƠNG — núi cone đỉnh (1000,1000), chân r=500m
  for (let k = 1; k <= 10; k++) {
    const r = 50 * k;
    const index = k % 2 === 0;
    ctx.strokeStyle = index ? "rgba(79,143,214,0.68)" : "rgba(79,143,214,0.32)";
    ctx.lineWidth = index ? 1.3 : 1;
    ctx.beginPath();
    ctx.arc(w2sX(1000), w2sY(1000), r * v.ppm, 0, 2 * Math.PI);
    ctx.stroke();
  }
  {
    const px = w2sX(1000), py = w2sY(1000);
    ctx.fillStyle = C.blueSoft;
    ctx.beginPath();
    ctx.moveTo(px, py - 6); ctx.lineTo(px - 5, py + 4); ctx.lineTo(px + 5, py + 4);
    ctx.closePath(); ctx.fill();
    ctx.fillText("PK", px + 8, py + 3);
  }

  // Trail
  if (d.trail && d.trail.length >= 2) {
    ctx.strokeStyle = "rgba(232,163,61,0.82)";
    ctx.lineWidth = 1.6;
    ctx.lineJoin = "round";
    ctx.beginPath();
    d.trail.forEach(([e, n], i) => (i === 0 ? ctx.moveTo(w2sX(e), w2sY(n)) : ctx.lineTo(w2sX(e), w2sY(n))));
    ctx.stroke();
  }

  // PF cloud
  ctx.fillStyle = "rgba(232,163,61,0.45)";
  for (const p of (d.particles || [])) {
    const x = w2sX(p[0]), y = w2sY(p[1]);
    if (x < -4 || x > W + 4 || y < -4 || y > H + 4) continue;
    ctx.fillRect(x - 1, y - 1, 2, 2);
  }

  // Cụm + ellipse CE95
  (d.clusters || []).forEach((c, ci) => {
    const col = CLUSTER_COLORS[ci % 4];
    const cosH = Math.cos(c.h), sinH = Math.sin(c.h);
    const pt = (t) => {
      const a = c.a * Math.cos(t), b = c.b * Math.sin(t);
      return [w2sX(c.e + a * sinH + b * cosH), w2sY(c.n + a * cosH - b * sinH)];
    };
    ctx.strokeStyle = col;
    ctx.globalAlpha = 0.88;
    ctx.lineWidth = 1.3;
    ctx.beginPath();
    for (let i = 0; i <= 72; i++) {
      const [x, y] = pt((i / 72) * 2 * Math.PI);
      i === 0 ? ctx.moveTo(x, y) : ctx.lineTo(x, y);
    }
    ctx.stroke();
    ctx.globalAlpha = 1;
    const [tx, ty] = pt(0);
    ctx.fillStyle = col;
    ctx.fillText("H" + ci, tx + 6, ty - 6);
    if (ci > 0) {
      const cx = w2sX(c.e), cy = w2sY(c.n);
      ctx.beginPath();
      ctx.moveTo(cx - 4, cy); ctx.lineTo(cx + 4, cy);
      ctx.moveTo(cx, cy - 4); ctx.lineTo(cx, cy + 4);
      ctx.stroke();
    }
  });

  // Tâm ngắm vị trí ước lượng
  if (d.est) {
    const cx = w2sX(d.est.e), cy = w2sY(d.est.n);
    ctx.strokeStyle = C.amber;
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.moveTo(cx - 13, cy); ctx.lineTo(cx - 4, cy);
    ctx.moveTo(cx + 4, cy); ctx.lineTo(cx + 13, cy);
    ctx.moveTo(cx, cy - 13); ctx.lineTo(cx, cy - 4);
    ctx.moveTo(cx, cy + 4); ctx.lineTo(cx, cy + 13);
    ctx.stroke();
    ctx.globalAlpha = 0.6;
    ctx.lineWidth = 1;
    const b = 9, l = 4;
    ctx.beginPath();
    for (const [sx, sy] of [[-1, -1], [1, -1], [-1, 1], [1, 1]]) {
      const x0 = cx + sx * b, y0 = cy + sy * b;
      ctx.moveTo(x0, y0 - sy * l); ctx.lineTo(x0, y0); ctx.lineTo(x0 - sx * l, y0);
    }
    ctx.stroke();
    ctx.globalAlpha = 1;
  }

  // Vị trí thật: thoi rỗng + vạch hướng
  if (d.true) {
    const tx = w2sX(d.true.e), ty = w2sY(d.true.n), r = 5.5;
    ctx.strokeStyle = C.text;
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.moveTo(tx, ty - r); ctx.lineTo(tx + r, ty); ctx.lineTo(tx, ty + r); ctx.lineTo(tx - r, ty);
    ctx.closePath();
    ctx.moveTo(tx, ty);
    ctx.lineTo(tx + 22 * Math.sin(d.true.heading), ty - 22 * Math.cos(d.true.heading));
    ctx.stroke();
  }

  // Overlay: nhãn lưới góc trên trái
  ctx.fillStyle = "rgba(14,16,15,0.86)";
  ctx.fillRect(0, 0, 150, 20);
  ctx.fillStyle = C.text;
  ctx.fillText(`UTM GRID ${gridM} m`, 7, 14);

  // Mũi tên Bắc + thước tỉ lệ
  {
    const bx = 22, by = H - 22;
    ctx.fillStyle = C.text;
    ctx.beginPath();
    ctx.moveTo(bx, by - 26); ctx.lineTo(bx - 6, by - 12); ctx.lineTo(bx + 6, by - 12);
    ctx.closePath(); ctx.fill();
    ctx.fillText("N", bx - 3.5, by + 1);
    const sm = niceStep(v.ppm, 70);
    const sw = sm * v.ppm, sx0 = bx + 22;
    ctx.strokeStyle = C.text;
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.moveTo(sx0, by); ctx.lineTo(sx0 + sw, by);
    ctx.moveTo(sx0, by - 5); ctx.lineTo(sx0, by + 1);
    ctx.moveTo(sx0 + sw, by - 5); ctx.lineTo(sx0 + sw, by + 1);
    ctx.moveTo(sx0 + sw / 2, by - 3); ctx.lineTo(sx0 + sw / 2, by);
    ctx.stroke();
    ctx.fillText(`${sm} m`, sx0, by - 9);
  }

  $("zoom-label").textContent =
    `E ${v.center_e.toFixed(1).padStart(7, "0")}  N ${v.center_n.toFixed(1).padStart(7, "0")}  ·  ${v.ppm.toFixed(2)} px/m`;
}

function fitAll(d) {
  const pts = (d.trail || []).concat(d.true ? [[d.true.e, d.true.n]] : []);
  if (pts.length < 1) return;
  let minE = pts[0][0], maxE = minE, minN = pts[0][1], maxN = minN;
  pts.forEach(([e, n]) => {
    minE = Math.min(minE, e); maxE = Math.max(maxE, e);
    minN = Math.min(minN, n); maxN = Math.max(maxN, n);
  });
  const W = mapCanvas.clientWidth || mapW, H = mapCanvas.clientHeight || mapH;
  state.view.ppm = Math.min((W - 120) / Math.max(1, maxE - minE), (H - 120) / Math.max(1, maxN - minN));
  state.view.ppm = Math.max(0.02, Math.min(8, state.view.ppm));
  state.view.center_e = (minE + maxE) / 2;
  state.view.center_n = (minN + maxN) / 2;
}

// ---------- Tương tác bản đồ (chuột + cảm ứng) ----------
function zoomAt(sx, sy, factor) {
  const v = state.view;
  const eB = (sx - mapW / 2) / v.ppm + v.center_e;
  const nB = -(sy - mapH / 2) / v.ppm + v.center_n;
  v.ppm = Math.max(0.02, Math.min(8, v.ppm * factor));
  const eA = (sx - mapW / 2) / v.ppm + v.center_e;
  const nA = -(sy - mapH / 2) / v.ppm + v.center_n;
  v.center_e += eB - eA;
  v.center_n += nB - nA;
  drawMap(state.last);
}
$("zoom-in").onclick = () => zoomAt(mapW / 2, mapH / 2, 1.25);
$("zoom-out").onclick = () => zoomAt(mapW / 2, mapH / 2, 1 / 1.25);
$("fit").onclick = () => { state.fitted = false; drawMap(state.last); };
mapCanvas.addEventListener("wheel", (e) => {
  e.preventDefault();
  zoomAt(e.offsetX, e.offsetY, e.deltaY < 0 ? 1.15 : 1 / 1.15);
}, { passive: false });
mapCanvas.addEventListener("pointerdown", (e) => {
  state.dragging = true; state.last_x = e.clientX; state.last_y = e.clientY;
  mapCanvas.setPointerCapture(e.pointerId);
  mapCanvas.classList.add("dragging");
});
mapCanvas.addEventListener("pointerup", (e) => {
  state.dragging = false;
  mapCanvas.releasePointerCapture(e.pointerId);
  mapCanvas.classList.remove("dragging");
});
mapCanvas.addEventListener("pointermove", (e) => {
  if (!state.dragging) return;
  const dx = e.clientX - state.last_x, dy = e.clientY - state.last_y;
  state.last_x = e.clientX; state.last_y = e.clientY;
  // Kéo nội dung: trung tâm dịch ngược (như MapView::panPixels desktop)
  state.view.center_e -= dx / state.view.ppm;
  state.view.center_n += dy / state.view.ppm;
  drawMap(state.last);
});
window.addEventListener("resize", () => { drawMap(state.last); drawSeries(state.last); });

// ---------- Chuỗi tín hiệu — kiểu máy hiện sóng ----------
const seriesCanvas = $("series");
function drawSeries(d) {
  if (!d) return;
  const { ctx, W, H } = fitCanvas(seriesCanvas);
  ctx.fillStyle = C.bg;
  ctx.fillRect(0, 0, W, H);
  const sets = [
    { data: d.series_gns || [], color: C.amber, w: 1.6 },
    { data: d.series_speed || [], color: C.text, w: 1.2 },
    { data: d.series_vertical || [], color: C.oliveBright, w: 1.2 },
    { data: d.series_conf || [], color: C.blueSoft, w: 1.2 },
  ];
  let maxV = 1.0, minV = 0.0, n = 0;
  sets.forEach(s => s.data.forEach(v => { maxV = Math.max(maxV, v); minV = Math.min(minV, v); }));
  sets.forEach(s => { n = Math.max(n, s.data.length); });
  const span = (maxV - minV) * 1.1 || 1;
  const L = 34, R = 6, T = 8, B = 18;
  const pw = W - L - R, ph = H - T - B;

  // Lưới + trục
  ctx.strokeStyle = C.line;
  ctx.lineWidth = 1;
  ctx.font = `10px ${MONO}`;
  ctx.fillStyle = C.dim;
  ctx.beginPath();
  for (let i = 0; i <= 4; i++) {
    const y = Math.round(T + (ph * i) / 4) + 0.5;
    ctx.moveTo(L, y); ctx.lineTo(W - R, y);
    ctx.fillText((minV + span * (1 - i / 4)).toFixed(1).padStart(4, " "), 2, y + 3);
  }
  for (let i = 0; i <= 6; i++) {
    const x = Math.round(L + (pw * i) / 6) + 0.5;
    ctx.moveTo(x, T); ctx.lineTo(x, T + ph);
    ctx.fillText(`${Math.round(((n / 10) * i) / 6)}s`, x - 6, H - 4);
  }
  ctx.stroke();

  sets.forEach(s => {
    if (s.data.length < 2) return;
    ctx.strokeStyle = s.color;
    ctx.lineWidth = s.w;
    ctx.lineJoin = "round";
    ctx.beginPath();
    s.data.forEach((v, i) => {
      const x = L + (i / (s.data.length - 1)) * pw;
      const y = T + ph - ((v - minV) / span) * ph;
      i === 0 ? ctx.moveTo(x, y) : ctx.lineTo(x, y);
    });
    ctx.stroke();
  });
}

// ---------- Copy SMS ----------
$("copy-sms").onclick = () => {
  const txt = $("packet-sms").textContent;
  const btn = $("copy-sms");
  navigator.clipboard.writeText(txt).then(() => {
    const old = btn.textContent;
    btn.textContent = "[COPIED ✓]";
    setTimeout(() => { btn.textContent = old; }, 1200);
  }).catch(() => { btn.textContent = "[COPY FAILED]"; });
};

// Khởi động: thử SSE, lỗi thì polling
connectSSE();
