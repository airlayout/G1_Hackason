// /sim/camera: カメラの前の手の動きで、シミュレーションの腕を操作する（左: カメラ映像、右: シミュレーション）。
// シミュレーション側の表示・キー操作・状態の取得は sim.js（window.simControl で操作の入口を受け取る）。
// 認識はすべてブラウザの中。映像はどこへも送らない（送るのは、1 フレームごとの数値の記録だけ → /sim/log）。計算の部分は sim_hand.js。
import { handFeatures, featureUnits, unitsToCommand, UnitSmoother, FALLBACK_FINGER, retryBox, mapFromBox, RETRY_FRESH_MS } from "/sim_hand.js";

const CAMERA = { video: { width: { ideal: 640 }, height: { ideal: 480 } }, audio: false };
const MOSAIC_BLOCKS_ACROSS = 36;       // 背景のモザイクの横方向のブロック数（多いほど細かい）
const MASK_BLUR_PX = 4;                // 人の切り抜きの縁をぼかす幅（動いたとき、手や体の縁が背景に食われないよう少し広げる）
const POSE_EVERY = 3;                  // 人の切り抜きは、手の認識 N 回につき 1 回だけ更新する（負荷を抑える）
const HAND_LOST_MS = 500;              // 手が見つからないまま、この時間が過ぎたら「映っていません」にする
const HAND_CONFIDENCE = 0.3;           // 手を見つける・手がある・追っている、各判定のしきい値（既定 0.5 だと小さい手や横向きの手がよく外れた）
const RETRY_PX = 256;                  // 外れたときの再検出で、切り出した画像を拡大する一辺（px）
const MAX_DT = 0.2;                    // フィルタへ渡す経過時間の上限（秒。止まっていたあとの飛びを避ける）
const LOG_FLUSH_MS = 1000;             // 操作ログを、この間隔でまとめて送る
const LOG_BUFFER_MAX = 600;            // 送れないまま溜めておく記録の最大数（超えたら古いものを捨てる）
const PERF_SHOW_MS = 500;              // 認識の時間・fps の表示を更新する間隔
const HAND_LINES = [[0, 1], [1, 2], [2, 3], [3, 4], [0, 5], [5, 6], [6, 7], [7, 8], [5, 9], [9, 10], [10, 11], [11, 12],
                    [9, 13], [13, 14], [14, 15], [15, 16], [13, 17], [17, 18], [18, 19], [19, 20], [0, 17]];
const METERS = [["pan", "左右", -1], ["height", "上下", -1], ["reach", "前後", -1], ["roll", "手首", -1], ["opening", "開き", 0]];

const $ = (id) => document.getElementById(id);
const video = $("video"), canvas = $("cam"), ctx = canvas.getContext("2d");
const part = (w, h) => Object.assign(document.createElement("canvas"), { width: w, height: h });
const small = part(1, 1), crop = part(RETRY_PX, RETRY_PX), person = part(1, 1), maskCanvas = part(1, 1);
const cropCtx = crop.getContext("2d"), sctx = small.getContext("2d"), pctx = person.getContext("2d"), mctx = maskCanvas.getContext("2d");

let stream = null, hands = null, pose = null, haveMask = false, poseFailed = false, delegate = "CPU";
let finger = FALLBACK_FINGER, fingerNote = "既定値";
let lastPoints = null, lastPointsAt = -1e9;
let lastHandAt = -1e9, lastTime = -1, lastNow = 0, frames = 0, openedAt = 0;
const perf = { ms: 0, count: 0, shown: 0, text: "" };
const smoother = new UnitSmoother();
const session = Array.from(crypto.getRandomValues(new Uint8Array(8)), (b) => (b % 36).toString(36)).join("");
let logBuffer = [], logSaved = 0, logFailed = 0, logTimer = null;

function setHint(text, warn = false) { const hint = $("hint"); hint.hidden = !text; hint.textContent = text; hint.className = "hint" + (warn ? " warn" : ""); }

// --- 操作ログ（1 フレームごとの数値。原因調査用。サーバーの debug/sim-camera-*.jsonl へ）
function record(entry) {
  logBuffer.push({ t: Math.round(performance.now() - openedAt), ...entry });
  if (logBuffer.length > LOG_BUFFER_MAX) logBuffer = logBuffer.slice(-LOG_BUFFER_MAX);
}
async function flushLog() {
  if (!logBuffer.length) return;
  const records = logBuffer.slice(0, 200);
  logBuffer = logBuffer.slice(records.length);
  try {
    const response = await fetch("/sim/log", { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify({ session, records }) });
    if (!response.ok) throw new Error((await response.json()).error ?? response.status);
    logSaved += records.length;
  } catch (error) { logFailed += records.length; console.warn("操作ログを送れません", error); }
}

async function loadFinger() {
  try {      // 実機の操作画面で測った指の校正（親指と人差し指の距離比の、開き・閉じ）。測っていなければ既定値
    const cal = await (await fetch("/finger")).json();
    if (Number.isFinite(cal.open) && Number.isFinite(cal.closed) && cal.open > cal.closed) { finger = { open: cal.open, closed: cal.closed }; fingerNote = "校正済み"; }
  } catch (error) { console.warn("指の校正を読めません", error); }
}

async function loadTrackers() {
  if (hands) return;
  const { FilesetResolver, HandLandmarker, PoseLandmarker } = await import("/assets/vision_bundle.mjs");
  const files = await FilesetResolver.forVisionTasks("/assets");
  // GPU が使えれば GPU（CPU だと人の切り抜きで 1 フレームが遅く、手の追従が遅れる）。だめなら CPU
  const make = async (type, options, model) => {
    for (const d of ["GPU", "CPU"]) {
      try { const t = await type.createFromOptions(files, { baseOptions: { modelAssetPath: model, delegate: d }, ...options }); return [t, d]; }
      catch (error) { console.warn(`${model} を ${d} で読めません`, error); }
    }
    throw new Error(`${model} を読めません`);
  };
  // IMAGE モード: 毎フレーム手を探し直す（VIDEO モードは手が速く動くと指先がずれた。app.js と同じ理由）
  [hands, delegate] = await make(HandLandmarker, { runningMode: "IMAGE", numHands: 1, minHandDetectionConfidence: HAND_CONFIDENCE, minHandPresenceConfidence: HAND_CONFIDENCE, minTrackingConfidence: HAND_CONFIDENCE }, "/assets/hand_landmarker.task");
  try {      // 人の切り抜き（背景のモザイク用）。読めなくても操作はできる（そのときは映像の全体にモザイクをかける）
    [pose] = await make(PoseLandmarker, { runningMode: "VIDEO", numPoses: 1, outputSegmentationMasks: true }, "/assets/pose_landmarker_full.task");
  } catch (error) { poseFailed = true; console.warn("人の切り抜きを読めません", error); }
}

function updateMask(now) {
  const result = pose.detectForVideo(video, now);
  const mask = result.segmentationMasks?.[0];
  haveMask = Boolean(mask);      // 人が見つからないときは切り抜かず、全体をモザイクにする
  if (mask) {
    const { width: w, height: h } = mask, values = mask.getAsFloat32Array();
    if (maskCanvas.width !== w || maskCanvas.height !== h) { maskCanvas.width = w; maskCanvas.height = h; }
    const image = mctx.createImageData(w, h);
    for (let i = 0; i < values.length; i++) image.data[i * 4 + 3] = Math.round(values[i] * 255);
    mctx.putImageData(image, 0, 0);
  }
  result.close?.();
}

function compose() {
  const w = video.videoWidth, h = video.videoHeight;
  if (canvas.width !== w || canvas.height !== h) { canvas.width = w; canvas.height = h; person.width = w; person.height = h; }
  ctx.imageSmoothingEnabled = false;
  if (!$("mosaic").checked) { ctx.drawImage(video, 0, 0, w, h); return; }
  const bw = MOSAIC_BLOCKS_ACROSS, bh = Math.max(1, Math.round((bw * h) / w));
  small.width = bw; small.height = bh;
  sctx.drawImage(video, 0, 0, bw, bh);      // 縮小で平均になる
  ctx.drawImage(small, 0, 0, w, h);         // 補間なしで拡大 → 四角いブロック
  if (!haveMask) return;
  pctx.globalCompositeOperation = "source-over";
  pctx.drawImage(video, 0, 0, w, h);
  pctx.globalCompositeOperation = "destination-in";      // 人の形だけ残す
  pctx.filter = `blur(${MASK_BLUR_PX}px)`;
  pctx.drawImage(maskCanvas, 0, 0, w, h);
  pctx.filter = "none";
  ctx.drawImage(person, 0, 0);
}

function drawHand(points, closed) {
  const w = canvas.width, h = canvas.height;
  ctx.lineWidth = 3; ctx.strokeStyle = "#3ddc84"; ctx.beginPath();
  for (const [a, b] of HAND_LINES) { ctx.moveTo(points[a].x * w, points[a].y * h); ctx.lineTo(points[b].x * w, points[b].y * h); }
  ctx.stroke();
  ctx.strokeStyle = closed ? "#ff5a4f" : "#ffb020"; ctx.lineWidth = closed ? 6 : 3; ctx.beginPath();      // つまみ（閉じたと判定している間は赤く太く）
  ctx.moveTo(points[4].x * w, points[4].y * h); ctx.lineTo(points[8].x * w, points[8].y * h); ctx.stroke();
}

function buildMeters() {
  const bars = {};
  for (const [key, label] of METERS) {
    const row = document.createElement("div"); row.className = "meter";
    const bar = Object.assign(document.createElement("progress"), { max: 1, value: 0.5 });
    const text = document.createElement("span"); text.style.textAlign = "right";
    row.append(Object.assign(document.createElement("span"), { textContent: label }), bar, text);
    $("meters").append(row);
    bars[key] = (v, low) => { bar.value = low < 0 ? (v + 1) / 2 : v; text.textContent = v.toFixed(2); };
  }
  return (units) => { for (const [key, , low] of METERS) bars[key](units[key], low); };
}
const showMeters = buildMeters();

function showDebug(extra = "") {
  const sim = window.simControl?.state();
  const state = sim ? `　アシスト ${sim.assist ? "オン" : "オフ"}${sim.held ? "（持っている）" : sim.grabbable ? "（つかめる）" : ""}` : "";
  $("debug").textContent = `${perf.text}（${delegate}）　指の校正: ${fingerNote}　ログ ${logSaved} 件${logFailed ? `（送れず ${logFailed}）` : ""}${state}${extra}`;
}

// 全体で外れたら、前に手があった周り（なければ中央）を切り出して拡大し、もう一度探す。遠くの小さな手を拾うため
function detectHand(now) {
  const full = hands.detect(video);
  if (full.landmarks[0]) return { points: full.landmarks[0], world: full.worldLandmarks?.[0], via: "full" };
  const w = video.videoWidth, h = video.videoHeight;
  const box = retryBox(now - lastPointsAt < RETRY_FRESH_MS ? lastPoints : null, w, h);
  cropCtx.drawImage(video, box.x, box.y, box.size, box.size, 0, 0, RETRY_PX, RETRY_PX);
  const zoom = hands.detect(crop);
  if (!zoom.landmarks[0]) return { points: null, world: null, via: "none" };
  return { points: mapFromBox(zoom.landmarks[0], box, w, h), world: zoom.worldLandmarks?.[0], via: "crop" };
}

function control(found, features, now, dt, ms, via) {
  const sim = window.simControl;
  if (!features) {
    record({ hand: false, ms, via, why: found ? "小さい" : "なし" });
    if (now - lastHandAt > HAND_LOST_MS) { smoother.reset(); setHint("手が映っていません。カメラの前へ手を出してください", true); }
    return false;
  }
  lastHandAt = now;
  setHint(poseFailed ? "人の切り抜きを読めないため、映像の全体にモザイクをかけています" : "", poseFailed);
  const units = smoother.step(featureUnits(features, finger, $("roll").checked), dt);
  showMeters(units);
  const command = sim && $("drive").checked ? unitsToCommand(units, sim.ranges) : null;
  if (command) sim.send(command);
  const s = sim?.state();
  record({
    hand: true, ms, via, ratio: +features.ratio.toFixed(3), ratio2d: +features.ratio2d.toFixed(3), w3: features.have3d,
    f: { x: +features.x.toFixed(3), y: +features.y.toFixed(3), width: +features.width.toFixed(3), tilt: +features.tiltDeg.toFixed(1) },
    u: Object.fromEntries(Object.entries(units).map(([k, v]) => [k, +v.toFixed(3)])),
    cmd: command && { x: +command.x.toFixed(4), height: +command.height.toFixed(4), pan: +command.pan.toFixed(1), roll: +command.roll.toFixed(1), opening: +command.opening.toFixed(3) },
    sim: s && { goal: s.goal, tip: s.tip, held: s.held, grabbable: s.grabbable, assist: s.assist, clamped: s.clamped, lift: s.task?.lift, got: s.task?.got },
  });
  return units.opening <= 0;
}

function loop(now) {
  requestAnimationFrame(loop);
  if (!stream || video.readyState < 2 || video.currentTime === lastTime) return;
  lastTime = video.currentTime;
  const dt = Math.min(MAX_DT, (now - lastNow) / 1000);
  lastNow = now;
  try {
    if (pose && $("mosaic").checked && frames++ % POSE_EVERY === 0) updateMask(now);
    const began = performance.now();
    const found = detectHand(now);
    const ms = +(performance.now() - began).toFixed(1);
    perf.ms += ms; perf.count++;
    if (now - perf.shown >= PERF_SHOW_MS) {
      perf.text = `認識 ${(perf.ms / perf.count).toFixed(1)} ms／フレーム　${((perf.count * 1000) / (now - perf.shown)).toFixed(1)} fps`;
      perf.ms = 0; perf.count = 0; perf.shown = now;
    }
    const points = found.points;
    if (points) { lastPoints = points; lastPointsAt = now; }
    const features = points ? handFeatures(points, found.world, video.videoWidth, video.videoHeight) : null;
    compose();
    const closed = control(points, features, now, dt, ms, found.via);
    if (points) drawHand(points, closed);
    showDebug(features ? `　距離比 ${features.ratio.toFixed(2)}${features.have3d ? "(3D)" : "(2D)"} → 開き ${Math.round(featureUnits(features, finger).opening * 100)}%` : "");
  } catch (error) { setHint(`認識でエラー: ${error.message}`, true); console.error(error); record({ error: String(error.message) }); }
}

async function openCamera() {
  setHint("読み込み中…");
  await Promise.all([loadTrackers(), loadFinger()]);
  stream = await navigator.mediaDevices.getUserMedia(CAMERA);
  video.srcObject = stream;
  await video.play();
  haveMask = false; lastPoints = null; lastPointsAt = -1e9; lastTime = -1; openedAt = performance.now(); smoother.reset();
  record({ event: "open", delegate, finger, userAgent: navigator.userAgent, size: [video.videoWidth, video.videoHeight] });
  logTimer ??= setInterval(flushLog, LOG_FLUSH_MS);
  $("open").textContent = "カメラを閉じる";
  setHint("カメラの前へ手を出してください");
}

function closeCamera() {
  stream?.getTracks().forEach((track) => track.stop());
  stream = null; video.srcObject = null; haveMask = false;
  record({ event: "close" });
  flushLog();
  clearInterval(logTimer); logTimer = null;
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  $("open").textContent = "カメラを開く";
  setHint("「カメラを開く」を押してください");
}

$("open").addEventListener("click", async () => {
  try { if (stream) closeCamera(); else await openCamera(); }
  catch (error) { closeCamera(); setHint(`カメラを開けません: ${error.message}`, true); console.error(error); }
});
addEventListener("sim-assist", (e) => record({ event: "assist", on: e.detail.on }));
addEventListener("pagehide", () => {      // 閉じるとき、残りのログを送る（navigator.sendBeacon は Content-Type を指定できないので、ここは取りこぼしを許す）
  if (logBuffer.length) flushLog();
});
requestAnimationFrame(loop);
