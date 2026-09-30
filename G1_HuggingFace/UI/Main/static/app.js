// 警備システム UI の画面側。
//
// ⚠️ **ここに ROS の語彙は出てこない。** サーバの /api/state が返す JSON だけを見る。
//    航法側がモックでも実機でも、このファイルは変わらない。
'use strict';

const POLL_MS = 200;          // 状態の取得周期。5Hz あれば人の目には十分
const canvas = document.getElementById('map');
const ctx = canvas.getContext('2d');

let mapInfo = null;           // {resolution, origin:[x,y], width, height}
let mapImg = null;            // 地図の PNG
let latest = null;            // 直近の /api/state
let busy = false;             // 操作の二重押し防止

// --- 地図の読み込み（起動時に1回だけ。地図は静的） ---------------------------
async function loadMap() {
  try {
    const res = await fetch('/api/map.json');
    if (!res.ok) return;
    mapInfo = await res.json();
    mapImg = new Image();
    mapImg.onload = draw;
    mapImg.src = '/api/map.png';
  } catch (e) {
    console.warn('地図を読めませんでした', e);
  }
}

// --- 世界座標(m) -> 地図ピクセル。ROS の map_server と同じ規約 ---------------
function worldToPx(x, y) {
  const r = mapInfo.resolution;
  return {
    col: (x - mapInfo.origin[0]) / r,
    row: (mapInfo.height - 1) - (y - mapInfo.origin[1]) / r,
  };
}

// --- 地図の描画 --------------------------------------------------------------
function draw() {
  const dpr = window.devicePixelRatio || 1;
  const w = canvas.clientWidth, h = canvas.clientHeight;
  if (canvas.width !== w * dpr || canvas.height !== h * dpr) {
    canvas.width = w * dpr; canvas.height = h * dpr;
  }
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, w, h);
  if (!mapInfo || !mapImg || !mapImg.complete) return;

  const pose = latest && latest.pose;
  const follow = document.getElementById('follow').checked && pose;

  // ⚠️ 地図全体(23.8 x 37.5m)を収めると機体が点にしか見えない。既定は
  //    **巡回路と機体が入る範囲**に合わせ、「機体に寄る」でさらに寄る。
  const box = contentBox();
  const scale = follow
    ? Math.min(w, h) / (8 / mapInfo.resolution)      // 機体の周り 8m 四方
    : Math.min(w / box.wpx, h / box.hpx);
  let cx, cy;
  if (follow) {
    const p = worldToPx(pose.x, pose.y);
    cx = w / 2 - p.col * scale;
    cy = h / 2 - p.row * scale;
  } else {
    cx = (w - box.wpx * scale) / 2 - box.col0 * scale;
    cy = (h - box.hpx * scale) / 2 - box.row0 * scale;
  }

  ctx.save();
  ctx.translate(cx, cy);
  ctx.scale(scale, scale);
  ctx.imageSmoothingEnabled = false;
  ctx.drawImage(mapImg, 0, 0);
  ctx.restore();

  if (!latest) return;
  const T = (x, y) => {
    const p = worldToPx(x, y);
    return [cx + p.col * scale, cy + p.row * scale];
  };

  // 巡回路（点線）と各点
  drawPath(latest.route, T, '#4a8fd4', 2, [6, 5]);
  (latest.route || []).forEach(([x, y], i) => {
    const [px, py] = T(x, y);
    ctx.beginPath(); ctx.arc(px, py, 4, 0, Math.PI * 2);
    ctx.fillStyle = (i === latest.patrol.index) ? '#e0a33a' : '#2b4a6b';
    ctx.fill();
    ctx.strokeStyle = '#0d1117'; ctx.lineWidth = 1; ctx.stroke();
  });

  // Nav2 が立てた経路（実線）
  drawPath(latest.plan, T, '#35c46b', 2.5, []);

  // 目的地
  if (latest.goal) {
    const [px, py] = T(latest.goal.x, latest.goal.y);
    ctx.beginPath(); ctx.arc(px, py, 7, 0, Math.PI * 2);
    ctx.strokeStyle = '#e0a33a'; ctx.lineWidth = 2; ctx.stroke();
  }

  // 機体（向きの分かる三角）
  if (pose) {
    const [px, py] = T(pose.x, pose.y);
    const a = -pose.yaw_deg * Math.PI / 180;   // 画面は y 下向きなので符号を反転
    ctx.save();
    ctx.translate(px, py); ctx.rotate(a);
    ctx.beginPath();
    ctx.moveTo(11, 0); ctx.lineTo(-7, 7); ctx.lineTo(-3, 0); ctx.lineTo(-7, -7);
    ctx.closePath();
    ctx.fillStyle = '#35c46b'; ctx.fill();
    ctx.strokeStyle = '#0d1117'; ctx.lineWidth = 1.5; ctx.stroke();
    ctx.restore();
  }
}

// 巡回路・経路・機体が収まるピクセル範囲。何も無ければ地図全体
function contentBox() {
  const pts = [];
  if (latest) {
    (latest.route || []).forEach((p) => pts.push(p));
    (latest.plan || []).forEach((p) => pts.push(p));
    if (latest.pose) pts.push([latest.pose.x, latest.pose.y]);
    if (latest.goal) pts.push([latest.goal.x, latest.goal.y]);
  }
  const whole = { col0: 0, row0: 0, wpx: mapInfo.width, hpx: mapInfo.height };
  if (pts.length < 2) return whole;

  const margin = 4 / mapInfo.resolution;      // 周囲 4m ぶんの余白
  let c0 = Infinity, c1 = -Infinity, r0 = Infinity, r1 = -Infinity;
  pts.forEach(([x, y]) => {
    const p = worldToPx(x, y);
    c0 = Math.min(c0, p.col); c1 = Math.max(c1, p.col);
    r0 = Math.min(r0, p.row); r1 = Math.max(r1, p.row);
  });
  // ⚠️ **地図の範囲で切り詰めないこと。** 現在地や経路が地図の外に出ている場合
  //    （地図が古い・自己位置が飛んだ・別の地図を読んでいる）に、切り詰めると
  //    範囲が負の大きさになって**何も描画されなくなる**。外に出ていることこそ
  //    見えてほしい情報なので、地図の外まで含めて表示する。
  const box = {
    col0: c0 - margin, row0: r0 - margin,
    wpx: (c1 - c0) + 2 * margin, hpx: (r1 - r0) + 2 * margin,
  };
  // 念のための保険。計算が壊れたら地図全体に戻す
  if (!(box.wpx > 0) || !(box.hpx > 0)) return whole;
  return box;
}

function drawPath(pts, T, color, width, dash) {
  if (!pts || pts.length < 2) return;
  ctx.save();
  ctx.setLineDash(dash);
  ctx.strokeStyle = color; ctx.lineWidth = width;
  ctx.lineJoin = 'round'; ctx.lineCap = 'round';
  ctx.beginPath();
  pts.forEach(([x, y], i) => {
    const [px, py] = T(x, y);
    i ? ctx.lineTo(px, py) : ctx.moveTo(px, py);
  });
  ctx.stroke();
  ctx.restore();
}

// --- 状態の反映 --------------------------------------------------------------
const BRIDGE_CLASS = {
  NAVIGATING: 'ok', READY: 'warn', STANDBY: 'warn',
  FAULT: 'bad', E_STOP: 'bad', DISCONNECTED: 'bad',
};

function setChip(id, text, cls) {
  const el = document.getElementById(id);
  el.querySelector('b').textContent = text;
  el.className = 'chip' + (cls ? ' ' + cls : '');
}

function apply(state) {
  latest = state;
  const v = state.vision || {};

  setChip('chip-bridge', state.bridge_state, BRIDGE_CLASS[state.bridge_state] || '');
  const p = state.patrol || {};
  const patrolText = p.total ? `${p.state} ${p.index + 1}/${p.total}` : p.state;
  setChip('chip-patrol', patrolText, p.state === 'RUNNING' ? 'ok' : '');
  setChip('chip-vision', v.alive ? `${v.fps} fps` : '途絶', v.alive ? 'ok' : 'bad');
  setChip('chip-link', state.connected ? '正常' : '切断', state.connected ? 'ok' : 'bad');

  document.getElementById('cam-note').textContent =
    (v.error ? v.error : `${v.device || '-'} / 検知 ${v.count || 0} 人`);
  const alert = document.getElementById('alert');
  alert.hidden = !v.present;              // 生の count はちらつくので使わない
  if (v.present) alert.textContent = `人を検知 ${v.count}（信頼度 ${v.max_confidence}）`;

  // 押せないボタンは押せなくする。断られる操作を押させない方が親切
  const nav = state.bridge_state;
  const en = (cmd) => {
    switch (cmd) {
      case 'enable_navigation':  return nav === 'READY';
      case 'disable_navigation': return nav === 'NAVIGATING';
      case 'clear_fault':        return nav === 'FAULT';
      case 'patrol_start':       return nav === 'NAVIGATING' && p.state !== 'RUNNING';
      case 'patrol_pause':       return p.state === 'RUNNING';
      // ⚠️ 一時停止後の状態は実機では 'HOLD'
      case 'patrol_stop':        return p.state === 'RUNNING' || p.state === 'HOLD';
      case 'patrol_skip':        return p.state === 'RUNNING' || p.state === 'HOLD';
      default:                   return true;   // 停止はいつでも押せる
    }
  };
  document.querySelectorAll('button[data-cmd]').forEach((b) => {
    b.disabled = busy || !state.connected || !en(b.dataset.cmd);
  });

  document.getElementById('mockbar').hidden = !state.mock;

  draw();
}

// --- 検知ログ ----------------------------------------------------------------
function renderEvents(events) {
  const ul = document.getElementById('events');
  if (!events.length) return;
  ul.innerHTML = events.map((e) => {
    const t = new Date(e.at * 1000).toLocaleTimeString('ja-JP');
    const d = e.kind === '消失'
      ? `${e.duration_s} 秒間`
      : `${e.count} 人 / 信頼度 ${e.confidence}`;
    return `<li><span class="t">${t}</span><span class="k ${e.kind}">${e.kind}</span><span class="d">${d}</span></li>`;
  }).join('');
}

// --- 操作 --------------------------------------------------------------------
async function send(cmd) {
  busy = true;
  const msg = document.getElementById('msg');
  try {
    const res = await fetch('/api/command/' + cmd, { method: 'POST' });
    const r = await res.json();
    msg.textContent = r.message || (r.ok ? '受理しました' : '断られました');
    msg.className = 'msg ' + (r.ok ? 'ok' : 'bad');
  } catch (e) {
    msg.textContent = 'サーバに届きませんでした: ' + e;
    msg.className = 'msg bad';
  } finally {
    busy = false;
  }
}

document.querySelectorAll('button[data-cmd]').forEach((b) => {
  b.addEventListener('click', () => send(b.dataset.cmd));
});

// --- 検出枠の表示 ------------------------------------------------------------
// ⚠️ **切り替わるのは表示だけ。** YOLO は回り続け、検知ログも残る。
//    枠は焼き込みなので、切り替えは MJPEG の張り直しになる（一瞬だけ絵が途切れる）。
const boxesToggle = document.getElementById('boxes');

function applyBoxes() {
  const on = boxesToggle.checked;
  try { localStorage.setItem('g1ui.boxes', on ? '1' : '0'); } catch (e) { /* 使えなくても困らない */ }
  document.getElementById('video').src = '/video.mjpg?boxes=' + (on ? '1' : '0');
}

try {
  const saved = localStorage.getItem('g1ui.boxes');
  if (saved !== null) boxesToggle.checked = (saved === '1');
} catch (e) { /* 既定(枠あり)のまま */ }

boxesToggle.addEventListener('change', applyBoxes);
document.getElementById('follow').addEventListener('change', draw);
window.addEventListener('resize', draw);

// --- 周期処理 ----------------------------------------------------------------
async function poll() {
  try {
    const res = await fetch('/api/state');
    apply(await res.json());
  } catch (e) {
    setChip('chip-link', '切断', 'bad');
  }
}

async function pollEvents() {
  try {
    const res = await fetch('/api/events');
    renderEvents((await res.json()).events || []);
  } catch (e) { /* 次の周期で取り直す */ }
}

function tick() {
  document.getElementById('clock').textContent = new Date().toLocaleTimeString('ja-JP');
}

applyBoxes();   // 映像の接続はここで始まる
loadMap();
poll();
pollEvents();
tick();
setInterval(poll, POLL_MS);
setInterval(pollEvents, 1000);
setInterval(tick, 1000);
