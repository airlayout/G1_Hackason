// G1 撮影のページ（iPhone の Safari）。phone_server.py が配る。
// カメラの絵に PC2 の走らせ役の状態を重ね、録った動画を 1 秒ずつの塊で Mac へ送る。
// 時計: /api/time を数回聞き、往復の一番短い 1 回で「Mac − 電話」を取る。動画の 0 秒は MediaRecorder の start の時刻。
// 自動: 腕を借りたら（ARMS_IN・READY・DOOR）録り始め、止めて（POST）6 秒たつか次の a で 1 本にする。1 回の a につき 1 本。
'use strict';

const PHASE_NAMES = {
  WAIT: '待ち', ARMS_IN: '腕を借りる', READY: '構え', DOOR: '扉', POST: '止めた',
  ARMS_OUT: '腕を返す', DAMP: '脱力', PROBE: '試し',
};
const ARMED = ['ARMS_IN', 'READY', 'DOOR'];
const RECORDING_PHASES = ['ARMS_IN', 'READY', 'DOOR', 'POST', 'ARMS_OUT'];
const AUTO_STOP_AFTER_POST_MS = 6000;
const AUTO_STOP_STALE_S = 60;      // 走らせ役の出力がこれだけ止まったら（落ちた）自動の録画を閉じる
const MAX_AUTO_MS = 10 * 60 * 1000;
const API_TIMEOUT_MS = 8000;       // iOS は Wi-Fi が切り替わると、つなぎっぱなしの接続で fetch が返らないことがある
const CHUNK_TIMEOUT_MS = 60000;
const MIN_UPLOAD_BYTES_PER_MS = 12.5;  // 0.1 Mbps。塊が大きい時は時間切れを延ばす（遅い Wi-Fi で打ち切って送り直すと帯を食うだけ）
const BEHIND_S = 10;                   // 送り待ちがこの秒数ぶんを超えたら「追いつかない」と出す
const LAST_DATA_QUIET_MS = 500;    // 止めた後、最後の塊がこれだけ来なければ閉じる
const IDLE_PING_MS = 20000;
const PING_N = 8;
const RESYNC_MS = 20000;
const POLL_MS = 500;
const STALE_S = 3;
const TEMP_WARN = 80;
const TEMP_HOT = 88;
const TIMESLICE_MS = 1000;
const NO_DATA_MS = 6000;           // 録り始めてこれだけ塊が出なければ、その 1 本は空（Chrome の偽のカメラは最初の塊まで 3.5 秒かかった）
const MIME_CANDIDATES = [
  'video/mp4;codecs=avc1.640028,mp4a.40.2', 'video/mp4;codecs=avc1', 'video/mp4',
  'video/webm;codecs=vp9,opus', 'video/webm',
];

const $ = (id) => document.getElementById(id);

function safeGet(k, dflt) {
  try { const v = localStorage.getItem(k); return v === null ? dflt : v; } catch (e) { return dflt; }
}
function safeSet(k, v) {
  try { localStorage.setItem(k, v); } catch (e) { /* 保存できない時は覚えないだけ */ }
}

const KEY = (() => {
  const k = new URLSearchParams(location.search).get('k');
  if (k) { safeSet('g1_key', k); return k; }
  return safeGet('g1_key', '');
})();

const timeOrigin = (typeof performance.timeOrigin === 'number' && performance.timeOrigin > 0)
  ? performance.timeOrigin : Date.now() - performance.now();
const phoneNow = () => timeOrigin + performance.now();

const app = {
  stream: null, streamUsed: false, recorder: null, rec: null, starting: false,
  queue: [], uploading: false, sentBytes: 0, queuedBytes: 0,
  clock: null,            // {offset, rtt, at}（ミリ秒）
  state: {}, polling: false, postSince: null, recordedCycle: null,
  lastAnaT: null, wakeLock: null, resyncTimer: null,
};

// ── 通信 ──
async function api(path, opts = {}) {
  const headers = { 'X-Key': KEY };
  let body = opts.body || null;
  if (opts.json !== undefined) { headers['Content-Type'] = 'application/json'; body = JSON.stringify(opts.json); }
  if (opts.blob) { headers['Content-Type'] = 'application/octet-stream'; body = opts.blob; }
  const ctl = new AbortController();
  const timer = setTimeout(() => ctl.abort(), opts.timeout || API_TIMEOUT_MS);
  let r;
  try {
    r = await fetch(path, { method: opts.method || (body ? 'POST' : 'GET'), headers, body, cache: 'no-store', signal: ctl.signal });
  } finally {
    clearTimeout(timer);
  }
  const data = await r.json().catch(() => ({}));
  if (!r.ok) {
    const e = new Error(data.error || `HTTP ${r.status}`);
    e.status = r.status; e.data = data;
    throw e;
  }
  return data;
}

async function ping(n) {
  const out = [];
  for (let i = 0; i < n; i += 1) {
    const s = phoneNow();
    const d = await api('/api/time');
    const e = phoneNow();
    out.push([s, d.t, e]);
    const rtt = e - s;
    const offset = d.t - (s + e) / 2;
    const old = app.clock;
    if (!old || rtt <= old.rtt || performance.now() - old.at > 60000) app.clock = { offset, rtt, at: performance.now() };
  }
  return out;
}

// ── 表示 ──
let toastTimer = null;
function toast(msg, level) {
  const t = $('toast');
  t.textContent = msg;
  t.dataset.level = level || '';
  t.dataset.show = 'true';
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => { t.dataset.show = 'false'; }, level === 'hot' ? 5000 : 1600);
}
function showError(e) { toast(String(e && e.message ? e.message : e), 'hot'); }

const fmt = (v, d) => (v === null || v === undefined || Number.isNaN(v) ? '—' : Number(v).toFixed(d));

function renderState(st) {
  const phase = st.phase || '';
  $('phase').textContent = phase ? `${PHASE_NAMES[phase] || phase}` : '—';
  $('sub').textContent = phase === 'DOOR' || phase === 'POST' ? (st.sub_label || '') : '';
  $('door').textContent = st.door_deg === null || st.door_deg === undefined ? '—' : `${fmt(st.door_deg, 1)}°`;
  const temp = $('temp');
  temp.textContent = st.temp_c === null || st.temp_c === undefined ? '—' : `${fmt(st.temp_c, 0)} °C`;
  temp.dataset.level = st.temp_c >= TEMP_HOT ? 'hot' : st.temp_c >= TEMP_WARN ? 'warn' : '';
  $('period').textContent = st.period_ms === null || st.period_ms === undefined ? '—' : `${fmt(st.period_ms, 0)} ms`;
  $('tries').textContent = String(st.tries || 0);
  const conn = $('conn');
  const stale = st.age_s !== undefined && st.age_s > STALE_S;
  conn.dataset.ok = String(Boolean(st.connected) && !stale);
  conn.textContent = !st.connected ? `PC2 ✕ ${st.error || ''}` : stale ? `PC2 ○（出力が ${fmt(st.age_s, 0)} 秒止まっている）` : 'PC2 ○';
  renderEvents(st.events || []);
  renderAna(st);
}

function renderEvents(events) {
  const ol = $('events');
  const items = events.slice(-3).reverse();
  ol.replaceChildren(...items.map(([, text]) => {
    const li = document.createElement('li');
    li.textContent = text.replace(/^\[(script|staged|door_stock)\]\s*/, '');
    if (/受け付けない|待っている|⚠️|起動しない/.test(text)) li.dataset.kind = 'refuse';
    return li;
  }));
}

function renderAna(st) {
  if (!st.ana || !st.ana.length) return;
  $('anaText').textContent = st.ana.join('\n');
  if (st.ana_t !== app.lastAnaT) {
    const first = app.lastAnaT === null;
    app.lastAnaT = st.ana_t;
    if (!first) $('ana').hidden = false;  // 新しい要約が来たら開く（ページを開いた時の古い要約は開かない）
  }
}

function renderRec() {
  const on = Boolean(app.recorder);
  const rs = $('recstate');
  rs.dataset.on = String(on);
  const btn = $('recBtn');
  btn.setAttribute('aria-pressed', String(on));
  btn.textContent = on ? '止める' : '録る';
  if (on && app.rec) {
    const s = Math.floor((performance.now() - app.rec.t0) / 1000);
    rs.textContent = `${app.rec.auto ? '自動 ' : ''}${String(Math.floor(s / 60)).padStart(2, '0')}:${String(s % 60).padStart(2, '0')}`;
  } else {
    rs.textContent = $('auto').checked ? '自動で待つ' : '録っていない';
  }
  const up = $('upload');
  const behind = app.queuedBytes > (Number($('rate').value) / 8) * BEHIND_S;
  up.textContent = `送った ${(app.sentBytes / 1e6).toFixed(1)} MB` + (app.queuedBytes > 0 ? `・待ち ${(app.queuedBytes / 1e6).toFixed(1)} MB` : '')
    + (behind ? '・⚠️ 送りが追いつかない（Wi-Fi が遅い。画質と速さを下げる）' : '');
  up.dataset.level = behind ? 'hot' : '';
  $('clockq').textContent = app.clock ? `時計の往復 ${app.clock.rtt.toFixed(0)} ms` : '時計 —';
}

// ── カメラ ──
async function acquireStream() {
  if (app.stream) app.stream.getTracks().forEach((t) => { t.onended = null; t.stop(); });  // 前のカメラは閉じてから
  app.stream = null;
  app.streamUsed = false;
  const h = Number($('res').value);
  const w = h === 1080 ? 1920 : 1280;
  const stream = await navigator.mediaDevices.getUserMedia({
    video: { facingMode: { ideal: 'environment' }, width: { ideal: w }, height: { ideal: h }, frameRate: { ideal: 30 } },
    audio: true,
  });
  // 着信・他のアプリがカメラを取った等でトラックが切れたら、そこまでで 1 本にして開き直しを促す
  stream.getTracks().forEach((t) => { t.onended = () => cameraLost('カメラが切れた'); });
  app.stream = stream;
  const v = $('preview');
  v.srcObject = stream;
  await v.play().catch(() => {});
}

async function openCamera() {
  if (!window.isSecureContext) throw new Error('https で開いていない（証明書を入れて Mac の QR から開き直す）');
  if (!navigator.mediaDevices || !window.MediaRecorder) throw new Error('この端末のブラウザは録画できない（iOS 14.3 以降の Safari）');
  await acquireStream();
  await requestWakeLock();
  await ping(PING_N);
}

function trackInfo(stream) {
  if (!stream) return 'カメラなし';
  return stream.getTracks().map((t) => `${t.kind}:${t.readyState}${t.muted ? '・muted' : ''}${t.enabled ? '' : '・disabled'}`).join(' ');
}

async function requestWakeLock() {
  const hint = '画面が消えないよう「設定 → 画面表示と明るさ → 自動ロック → なし」にする（消えると録画が止まる）';
  try {
    if ('wakeLock' in navigator) app.wakeLock = await navigator.wakeLock.request('screen');
    else toast(hint, 'hot');  // iOS 16.4 より前の Safari
  } catch (e) {
    toast(hint, 'hot');
  }
}

function pickMime() {
  return MIME_CANDIDATES.find((m) => MediaRecorder.isTypeSupported(m)) || '';
}

function eventPhoneMs(e) {
  const ts = e && e.timeStamp;
  if (typeof ts === 'number' && ts > 0 && ts <= performance.now() + 1000) return timeOrigin + ts;
  return phoneNow();
}

function streamLive() {
  return Boolean(app.stream) && app.stream.getVideoTracks().some((t) => t.readyState === 'live');
}

function cameraLost(why) {
  if (app.recorder) stopRec();
  app.recordedCycle = null;  // 戻ったら、腕を借りている回なら新しい 1 本を始める
  $('gate').hidden = false;
  $('gateMsg').textContent = `${why}。そこまでの動画は Mac へ送る。「カメラを開く」で続ける`;
  $('gateMsg').dataset.level = 'hot';
}

// ── 録画 ──
async function startRec(isAuto, cycle, retried = false) {
  if (!streamLive() || app.recorder || app.starting) throw new Error(streamLive() ? '録画を始められない' : 'カメラが開いていない');
  app.starting = true;
  try {
    // 11-13 の iPhone（Safari 26）: カメラを開いて 1 本目は録れたが、同じカメラで録った 2 本目から塊が 1 つも出なかった。
    // 1 本ごとにカメラを開き直す（0.5 秒ほどかかる。自動は a で腕を借りた時に始まるので、扉の動きには間に合う）
    if (app.streamUsed) await acquireStream();
    const mime = pickMime();
    const bitrate = Number($('rate').value);
    const s = app.stream.getVideoTracks()[0].getSettings();
    const pings = await ping(4);
    const r = await api('/api/rec/start', {
      json: { ua: navigator.userAgent, mime, width: s.width, height: s.height, fps: s.frameRate, bitrate, auto: isAuto,
        pings, call_phone_ms: phoneNow() },
    });
    const rec = {
      id: r.rec, seq: 0, acked: 0, broken: '', pings, auto: isAuto, cycle: cycle === undefined ? app.state.arms : cycle,
      t0: performance.now(), lastDataAt: performance.now(), callPhoneMs: phoneNow(), startPhoneMs: null, stopPhoneMs: null,
      // 電話の中の様子（閉じる時に Mac へ送る。塊が出ない時に原因を見分ける）
      diag: { events: 0, empty: 0, bytes: 0, firstDataMs: null, errors: [], hidden: 0, retried, noDataStop: false,
        recorderMime: '', trackStart: trackInfo(app.stream), trackStop: '' },
    };
    const stream = app.stream;
    const opts = { videoBitsPerSecond: bitrate, audioBitsPerSecond: 128000 };
    if (mime) opts.mimeType = mime;
    const recorder = new MediaRecorder(stream, opts);
    rec.diag.recorderMime = recorder.mimeType || '';
    recorder.onstart = (e) => { rec.startPhoneMs = eventPhoneMs(e); };
    recorder.ondataavailable = (e) => {
      rec.lastDataAt = performance.now();
      rec.diag.events += 1;
      if (!e.data || !e.data.size) { rec.diag.empty += 1; return; }
      rec.diag.bytes += e.data.size;
      if (rec.diag.firstDataMs === null) rec.diag.firstDataMs = Math.round(performance.now() - rec.t0);
      enqueue(rec, e.data);
    };
    recorder.onstop = () => {
      // iOS が自分で止めた時（画面ロック・着信など）も、ここで片付ける（片付けないと「録画中」のまま次を録れない）
      if (app.recorder === recorder) {
        app.recorder = null;
        clearInterval(app.resyncTimer);
      }
      if (rec.stopPhoneMs === null) rec.stopPhoneMs = phoneNow();
      rec.diag.trackStop = trackInfo(stream);
      renderRec();
      finishRec(rec).catch(showError);
    };
    recorder.onerror = (e) => {
      const msg = e.error ? `${e.error.name}: ${e.error.message}` : String(e);
      rec.diag.errors.push(msg);
      showError(`録画の誤り: ${msg}`);
    };
    app.rec = rec;
    app.recorder = recorder;
    recorder.start(TIMESLICE_MS);
    app.streamUsed = true;
    app.resyncTimer = setInterval(() => { ping(3).then((p) => rec.pings.push(...p)).catch(() => {}); }, RESYNC_MS);
    setTimeout(() => watchNoData(rec, recorder), NO_DATA_MS);
    if (retried) toast(`録画が ${NO_DATA_MS / 1000} 秒空だった → カメラを開き直して撮り直した`, 'hot');
    else toast(isAuto ? '自動で録り始めた' : '録り始めた');
  } finally {
    app.starting = false;
    renderRec();
  }
}

function watchNoData(rec, recorder) {
  // 塊は 1 秒おきに出るはず。出なければ、その 1 本は空なので閉じ、カメラを開き直して 1 回だけ録り直す
  // 止めた後に遅れて来た塊は捨てずに送る（broken にしない）
  if (rec.diag.firstDataMs !== null || app.recorder !== recorder) return;
  rec.diag.noDataStop = true;
  stopRec();
  if (rec.diag.retried) {
    showError('録画が空のまま（カメラを開き直しても塊が出ない）。「カメラを開く」から開き直すか、Safari を開き直す');
    return;
  }
  setTimeout(() => { startRec(rec.auto, rec.cycle, true).catch(showError); }, 300);
}

function stopRec() {
  const r = app.recorder;
  if (!r) return;
  clearInterval(app.resyncTimer);
  app.recorder = null;
  if (app.rec && app.rec.stopPhoneMs === null) app.rec.stopPhoneMs = phoneNow();
  if (r.state !== 'inactive') r.stop();
  renderRec();
}

function enqueue(rec, blob) {
  if (rec.broken) return;  // 送れなくなった録画の後ろの塊は捨てる（先頭で詰まると次の録画も送れない）
  app.queue.push({ rec, seq: rec.seq, blob });
  rec.seq += 1;
  app.queuedBytes += blob.size;
  pump();
}

function dropRec(rec, why) {
  rec.broken = why;
  const keep = app.queue.filter((it) => it.rec !== rec);
  app.queue.filter((it) => it.rec === rec).forEach((it) => { app.queuedBytes -= it.blob.size; });
  app.queue = keep;
  showError(`${rec.id} の残りを送れない: ${why}`);
}

const sleep = (ms) => new Promise((res) => { setTimeout(res, ms); });

async function pump() {
  if (app.uploading) return;
  app.uploading = true;
  let backoff = 500;
  while (app.queue.length) {
    const it = app.queue[0];
    try {
      await api(`/api/rec/chunk?rec=${encodeURIComponent(it.rec.id)}&seq=${it.seq}`,
        { method: 'POST', blob: it.blob, timeout: Math.max(CHUNK_TIMEOUT_MS, it.blob.size / MIN_UPLOAD_BYTES_PER_MS) });
      app.sentBytes += it.blob.size;
      it.rec.acked = it.seq + 1;
      done(it);
      backoff = 500;
    } catch (e) {
      if (e.status === 409 && e.data && e.data.next > it.seq) { it.rec.acked = e.data.next; done(it); continue; }  // もう届いていた
      if (e.status === 409 || e.status === 404 || e.status === 413 || e.status === 400) {
        dropRec(it.rec, e.message);  // Mac が前の塊を持っていない・受け口を起こし直した等: その録画は打ち切る
        continue;
      }
      await sleep(backoff);   // 網が切れた・Mac が止まった・書けない: 待って送り直す（塊は電話に持ったまま）
      backoff = Math.min(backoff * 2, 5000);
    }
    renderRec();
  }
  app.uploading = false;
  renderRec();
}

function done(it) {
  app.queue.shift();
  app.queuedBytes -= it.blob.size;
}

async function finishRec(rec) {
  // 最後の塊が来なくなるまで待ってから（古い WebKit は stop の後に最後の dataavailable を出すことがある）、
  // この 1 本の塊が全部届くまで待つ（送っている途中の塊も queue の先頭に残っている。次の 1 本の塊は待たない）
  while (performance.now() - rec.lastDataAt < LAST_DATA_QUIET_MS) await sleep(100);
  while (app.queue.some((it) => it.rec === rec)) await sleep(200);
  try { rec.pings.push(...(await ping(4))); } catch (e) { /* 終わりの往復が無くても始めので合わせる */ }
  const body = {
    rec: rec.id, chunks: rec.acked, start_phone_ms: rec.startPhoneMs === null ? rec.callPhoneMs : rec.startPhoneMs,
    call_phone_ms: rec.callPhoneMs, stop_phone_ms: rec.stopPhoneMs, pings: rec.pings, broken: rec.broken || null,
    diag: rec.diag,
  };
  for (let i = 0; i < 30; i += 1) {
    try {
      const r = await api('/api/rec/stop', { json: body });
      if (r.bytes > 0) toast(`送った: ${r.rec}（${(r.bytes / 1e6).toFixed(1)} MB）`);
      else toast(`${r.rec} は空だった（録画が塊を出さなかった）。「カメラを開く」から開き直す`, 'hot');
      return;
    } catch (e) {
      if (e.status === 404) throw e;
      await sleep(1000);
    }
  }
  throw new Error(`${rec.id} を閉じられない（Mac の受け口を確かめる）`);
}

// ── 状態を聞く・自動で録る ──
async function poll() {
  if (app.polling) return;
  app.polling = true;
  try {
    const st = await api('/api/state');
    app.state = st;
    renderState(st);
    autoTick(st);
  } catch (e) {
    $('conn').dataset.ok = 'false';
    $('conn').textContent = `Mac ✕ ${e.message}`;
  } finally {
    app.polling = false;
    renderRec();
  }
}

function autoTick(st) {
  const phase = st.phase || '';
  if (phase === 'POST') { if (app.postSince === null) app.postSince = performance.now(); } else app.postSince = null;
  if (!$('auto').checked || !streamLive() || !st.connected) return;
  // 開いた時に腕を借りていれば、その回から録る（-1 はどの回とも違う）。借りていなければ次の a から
  if (app.recordedCycle === null) app.recordedCycle = ARMED.includes(phase) ? -1 : st.arms;
  if (!app.recorder && !app.starting && ARMED.includes(phase) && st.arms !== app.recordedCycle) {
    const prev = app.recordedCycle;
    const cycle = st.arms;
    app.recordedCycle = cycle;
    startRec(true, cycle).catch((e) => { app.recordedCycle = prev; showError(e); });  // 失敗したら次の問い合わせで録り直す
    return;
  }
  const rec = app.rec;
  if (app.recorder && rec && rec.auto) {
    const now = performance.now();
    const afterPost = app.postSince !== null && now - app.postSince > AUTO_STOP_AFTER_POST_MS;
    const left = !RECORDING_PHASES.includes(phase);
    const stale = st.age_s !== undefined && st.age_s !== null && st.age_s > AUTO_STOP_STALE_S;
    if (afterPost || left || stale || st.arms !== rec.cycle || now - rec.t0 > MAX_AUTO_MS) stopRec();
  }
}

// ── 印・メモ ──
async function sendMark(label, memo, phoneMs) {
  const c = app.clock;
  await api('/api/mark', {
    json: { rec: app.recorder && app.rec ? app.rec.id : null, phone_ms: phoneMs, label, memo,
      offset_ms: c ? c.offset : null, rtt_ms: c ? c.rtt : null },
  });
}

function bindUi() {
  let timers = false;
  $('openBtn').addEventListener('click', async () => {
    try {
      if (app.stream) app.stream.getTracks().forEach((t) => t.stop());  // 開き直す時は前の（死んだ）ストリームを閉じる
      await openCamera();
      $('gate').hidden = true;
      $('gateMsg').dataset.level = '';
      if (!timers) {
        timers = true;
        setInterval(poll, POLL_MS);
        setInterval(renderRec, 1000);
        setInterval(() => { if (!app.recorder) ping(2).catch(() => {}); }, IDLE_PING_MS);  // 印の時刻用に、録っていない間も合わせ直す
      }
      poll();
    } catch (e) {
      $('gateMsg').textContent = String(e.message || e);
      $('gateMsg').dataset.level = 'hot';
    }
  });
  $('recBtn').addEventListener('click', () => {
    if (app.recorder) stopRec(); else startRec(false).catch(showError);
  });
  document.querySelectorAll('.mark[data-label]').forEach((b) => {
    b.addEventListener('click', () => {
      const t = phoneNow();
      sendMark(b.dataset.label, '', t)
        .then(() => { b.dataset.flash = 'true'; setTimeout(() => { b.dataset.flash = 'false'; }, 400); })
        .catch(showError);
    });
  });
  let memoT = null;
  $('memoBtn').addEventListener('click', () => { memoT = phoneNow(); $('memoText').value = ''; $('memoDlg').showModal(); });
  $('memoDlg').addEventListener('close', () => {
    const text = $('memoText').value.trim();
    if ($('memoDlg').returnValue === 'send' && text) sendMark('メモ', text, memoT).then(() => toast('メモを残した')).catch(showError);
  });
  $('anaClose').addEventListener('click', () => { $('ana').hidden = true; });
  $('anaBtn').addEventListener('click', () => { $('ana').hidden = !$('ana').hidden; });
  $('auto').checked = safeGet('g1_auto', '1') === '1';
  $('auto').addEventListener('change', () => { safeSet('g1_auto', $('auto').checked ? '1' : '0'); renderRec(); });
  $('res').value = safeGet('g1_res', '720');
  $('res').addEventListener('change', () => safeSet('g1_res', $('res').value));
  $('rate').value = safeGet('g1_rate', '1500000');
  $('rate').addEventListener('change', () => safeSet('g1_rate', $('rate').value));
  document.addEventListener('visibilitychange', () => {
    if (document.visibilityState === 'hidden' && app.recorder) {
      if (app.rec) app.rec.diag.hidden += 1;
      stopRec();                 // 裏に回るとカメラが止まる: そこまでで 1 本にする
      app.recordedCycle = null;  // 戻ったら、腕を借りている回なら新しい 1 本を始める
    }
    if (document.visibilityState === 'visible' && app.stream) {
      if (streamLive()) requestWakeLock();
      else cameraLost('裏に回ってカメラが止まった');
    }
  });
  if (!KEY) { $('gateMsg').textContent = '鍵が無い。Mac の画面の QR から開き直す'; $('gateMsg').dataset.level = 'hot'; }
}

bindUi();
