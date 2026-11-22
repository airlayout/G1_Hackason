// 骨格版の画面。カメラ → MediaPipe Pose → 腕の角度（skeleton_math.js）→ サーバー（/command の skeleton_*）。
import {armFeatures, ARMS, smooth, steadyAngles, wrap} from "/skeleton_math.js";
import {armFigure} from "/skeleton_figure.js";

const $ = id => document.getElementById(id);
const JOINT_NAMES = {shoulder_pan: "台座", shoulder_lift: "肩", elbow_flex: "肘", wrist_flex: "手首の曲げ", wrist_roll: "手首の回転"};
const ANGLE_NAMES = {pan: "横向き", lift: "上げ下げ", elbow: "肘の角度", flex: "手首の曲げ", roll: "手首の回転"};
// 人がとる腕の形（度。skeleton_math.js の角度の定義）。ロボットの3姿勢（skeleton.py の POSE_OFFSETS）に対応させた暫定の形。
// 対応の向きは、肩の上げ下げだけ実機で確認済み。ほかの関節は、ロボットが動いた形を見て、図と食い違えばここを直す。
const HUMAN_POSES = [
  {pan: 0, lift: 40, elbow: 90, flex: 0, roll: 0},
  {pan: 30, lift: 75, elbow: 125, flex: 25, roll: 0},
  {pan: -30, lift: 25, elbow: 55, flex: -25, roll: 40},
];
function describePose(a) {
  const side = a.pan === 0 ? "正面" : `カメラから見て${a.pan > 0 ? "右" : "左"}へ${Math.abs(a.pan)}°`;
  const roll = a.roll === 0 ? "手のひらは下向き" : `手のひらを${a.roll}°ひねる（0°が下向き）`;
  return [`上腕: ${side}、真下から${a.lift}°上げる（90°で水平）`, `肘: ${a.elbow}°（90°で直角、180°で伸ばす）`,
    `手首: 肘と同じ向きへ${a.flex}°曲げる（0°で前腕と一直線）`, roll];
}
const SMOOTH_ALPHA = 0.4;
const SEND_INTERVAL_MS = 66;        // 追従の送信間隔（約15回/秒。前の送信が終わるまで次は送らない）
const POLL_MS = 300;
const FRESH_MS = 300;               // これより古い骨格は「見えていない」とする
const CAPTURE_FRAMES = 15, CAPTURE_TIMEOUT_MS = 3000;
const READY_DELAY_MS = 3000;        // ロボットが着いてから、人が姿勢をとるのを待つ時間
const ARRIVE_TIMEOUT_MS = 20000, STEADY_TIMEOUT_MS = 60000;
const CORE_ANGLES = ["lift", "elbow"];   // 撮影に必須の角度
const MOVE_KEYS = ["pan", "lift", "elbow"], MIN_CHANGE_DEG = 10;   // 前の姿勢と、これ未満しか違わないときは、まねできていない
const SNAPSHOT_QUALITY = 0.85;
const BONES = [["shoulder", "elbow"], ["elbow", "wrist"], ["wrist", "index"], ["wrist", "pinky"], ["index", "pinky"]];

let state = null, camera = null, tracker = null, token = null, sending = false, lastVideoTime = -1;
let latest = null, latestAt = 0, raw = null;         // latest=平滑化した角度、raw=平滑化前（撮影用）
let autoRun = 0, phaseText = "";
let collecting = null;                               // 撮影中は、フレームごとの角度をここへ集める
const thumbs = {};
const perf = {ms: 0, count: 0, shown: performance.now()};

async function command(op, fields = {}) {
  const res = await fetch("/command", {method: "POST", headers: {"Content-Type": "application/json"}, body: JSON.stringify({op, ...fields})});
  const data = await res.json();
  if (!res.ok) throw new Error(data.error);
  return data;
}
function message(text) { $("message").textContent = text || ""; }
function bind(button, action) {
  button.onclick = async () => { try { message(""); await action(); } catch (e) { message(e.message); } };
}
const sk = () => state?.skeleton;
const fresh = () => latest && performance.now() - latestAt <= FRESH_MS;

// ---- カメラと骨格推定 ----
// 画角: 多くのカメラは、16:9 で撮ると4:3の映像の上下を切り取る。4:3で撮ると、センサー全体（広い視野）が映る
// 縦長（3:4）は、iPhone を縦に置いて上下を広く映すとき。縦の映像が来るかは、開いたあとの「幅×高さ」で確かめる
const FRAME = {"4:3": {width: 1280, height: 960}, "16:9": {width: 1280, height: 720}, "3:4": {width: 960, height: 1280}};
const constraint = id => ({video: {...(id ? {deviceId: {exact: id}} : {}), width: {ideal: FRAME[$("fov").value].width}, height: {ideal: FRAME[$("fov").value].height}}, audio: false});
// 開いたカメラの実際の大きさに、映像と認識点の枠をそろえる。ズームできるカメラは、いちばん広い側にする
async function fitCamera() {
  const video = $("input"), track = camera.getVideoTracks()[0], caps = track.getCapabilities?.() ?? {};
  document.querySelector(".preview").style.setProperty("--ratio", video.videoWidth/video.videoHeight);
  let zoom = "ズーム調整なし";
  if (caps.zoom) {
    try { await track.applyConstraints({advanced: [{zoom: caps.zoom.min}]}); zoom = `ズーム ${caps.zoom.min}（最も広角）`; }
    catch (e) { zoom = "ズームを変えられません"; console.warn("zoom", e); }
  }
  $("cameraInfo").textContent = `${video.videoWidth}×${video.videoHeight}　${zoom}`;
}
async function listCameras() {
  const devices = (await navigator.mediaDevices.enumerateDevices()).filter(d => d.kind === "videoinput" && d.deviceId);
  const select = $("cam"), selected = select.value;
  while (select.options.length > 1) select.remove(1);
  devices.forEach((d, i) => select.add(new Option(d.label || "カメラ " + (i + 1), d.deviceId)));
  if ([...select.options].some(o => o.value === selected)) select.value = selected;
}
async function loadTracker() {
  if (tracker) return;
  const {FilesetResolver, PoseLandmarker} = await import("/assets/vision_bundle.mjs");
  tracker = await PoseLandmarker.createFromOptions(await FilesetResolver.forVisionTasks("/assets"), {
    baseOptions: {modelAssetPath: "/assets/pose_landmarker_full.task"}, runningMode: "VIDEO", numPoses: 1,
  });
}
async function stopFollow(why) {
  token = null; cancelAuto();
  state = await command("stop", why ? {why} : {});
  render();
}
async function closeCamera() {
  await stopFollow("映像を閉じた");
  camera?.getTracks().forEach(t => t.stop());
  camera = null; latest = raw = null; $("input").srcObject = null;
  const canvas = $("overlay"); canvas.getContext("2d").clearRect(0, 0, canvas.width, canvas.height);
  $("cameraInfo").textContent = ""; $("angles").textContent = ""; $("perf").textContent = "認識 — ms／フレーム";
}
async function openCamera() {
  await stopFollow("映像を開き直した");
  await loadTracker();
  camera = await navigator.mediaDevices.getUserMedia(constraint($("cam").value));
  $("input").srcObject = camera; await $("input").play(); await fitCamera(); await listCameras();
}
bind($("devices"), async () => {
  if (!camera) (await navigator.mediaDevices.getUserMedia(constraint(""))).getTracks().forEach(t => t.stop());
  await listCameras();
});
bind($("enable"), async () => { camera ? await closeCamera() : await openCamera(); render(); });

function drawArm(points, side) {
  const video = $("input"), canvas = $("overlay"), ctx = canvas.getContext("2d");
  canvas.width = video.videoWidth; canvas.height = video.videoHeight;
  if (!points) return;
  const ids = ARMS[side], at = name => points[ids[name]];
  ctx.lineWidth = 4; ctx.strokeStyle = "#ffe081"; ctx.fillStyle = "#ff6b4a";
  for (const [a, b] of BONES) {
    ctx.beginPath(); ctx.moveTo(at(a).x*canvas.width, at(a).y*canvas.height); ctx.lineTo(at(b).x*canvas.width, at(b).y*canvas.height); ctx.stroke();
  }
  for (const name of Object.keys(ids)) { ctx.beginPath(); ctx.arc(at(name).x*canvas.width, at(name).y*canvas.height, 6, 0, 7); ctx.fill(); }
}
function showAngles() {
  const box = $("angles"); box.textContent = "";
  if (!camera) return;
  if (!fresh()) { box.textContent = "骨格が認識できません（肩・肘・手首を映してください）"; return; }
  for (const key of Object.keys(ANGLE_NAMES)) {
    const span = document.createElement("span");
    span.textContent = ANGLE_NAMES[key] + " " + (key in latest ? latest[key].toFixed(0) + "°" : "—");
    box.append(span);
  }
}
function detect() {
  const video = $("input");
  if (!camera || !tracker || video.readyState < 2 || video.currentTime === lastVideoTime) return;
  lastVideoTime = video.currentTime;
  const began = performance.now(), side = $("side").value;
  const result = tracker.detectForVideo(video, began);
  const points = result.landmarks?.[0], world = result.worldLandmarks?.[0];
  raw = points && world ? armFeatures(world, points.map(p => p.visibility ?? 0), side) : {};
  if (Object.keys(raw).length) {
    const next = {};
    for (const [key, value] of Object.entries(raw)) next[key] = smooth(latest?.[key], value, SMOOTH_ALPHA);
    latest = next; latestAt = began;
    collecting?.push({...raw});
  }
  drawArm(points, side); showAngles();
  perf.ms += performance.now() - began; perf.count++;
  if (began - perf.shown >= 500) {
    $("perf").textContent = "認識 " + (perf.ms/perf.count).toFixed(1) + " ms／フレーム";
    perf.ms = perf.count = 0; perf.shown = began;
  }
}
(function loop() { try { detect(); } catch (e) { message("骨格の認識でエラー: " + e.message); } requestAnimationFrame(loop); })();

// ---- 較正 ----
function composeSnapshot() {
  const video = $("input"), canvas = document.createElement("canvas");
  canvas.width = video.videoWidth; canvas.height = video.videoHeight;
  const ctx = canvas.getContext("2d");
  ctx.drawImage(video, 0, 0); ctx.drawImage($("overlay"), 0, 0);
  return canvas.toDataURL("image/jpeg", SNAPSHOT_QUALITY);
}
function collectFrames() {
  return new Promise(resolve => {
    const frames = collecting = [], began = performance.now();
    const timer = setInterval(() => {
      if (frames.length >= CAPTURE_FRAMES || performance.now() - began > CAPTURE_TIMEOUT_MS) {
        clearInterval(timer); collecting = null; resolve(frames);
      }
    }, 50);
  });
}
const earlier = {};                 // 撮影済みの姿勢の番号 → そのときの人の角度（前の姿勢から動いたかの確認用）
function sameAsEarlier(index, angles) {
  return Object.entries(earlier).some(([other, before]) => Number(other) !== index && MOVE_KEYS.every(k => !(k in angles && k in before) || Math.abs(wrap(angles[k] - before[k])) < MIN_CHANGE_DEG));
}
async function submitCapture(index, frames, angles, unstable) {
  const image = composeSnapshot();
  thumbs[index] = image; earlier[index] = angles;
  try {      // 診断用の画像。保存に失敗しても較正は続ける
    await fetch("/snapshot", {method: "POST", headers: {"Content-Type": "application/json"},
      body: JSON.stringify({image, info: {kind: "skeleton_calibration", pose: index, side: $("side").value, angles, unstable, frames: frames.length}})});
  } catch (e) { console.warn("snapshot", e); }
  state = await command("skeleton_capture", {index, features: angles});
  render();
  message(unstable.length ? "ぶれたため除いた角度: " + unstable.map(k => ANGLE_NAMES[k]).join("、") : "");
}
// ---- 自動較正: ロボットが姿勢をとる → 人がまねる → じっとしたら自動で撮影（カメラが近く、ボタンを押せないため）----
const cancelled = () => Object.assign(new Error("中止"), {cancelled: true});
const sleep = ms => new Promise(resolve => setTimeout(resolve, ms));
function say(text) { try { speechSynthesis.speak(new SpeechSynthesisUtterance(text)); } catch (e) { console.warn("speech", e); } }
function cancelAuto() { autoRun++; phaseText = ""; }
async function waitUntil(test, limitMs, run, timeoutText) {
  const began = performance.now();
  while (!test()) {
    if (run !== autoRun) throw cancelled();
    if (performance.now() - began > limitMs) throw new Error(timeoutText);
    await sleep(100);
  }
}
async function captureWhenSteady(index, run) {
  const began = performance.now();
  while (performance.now() - began < STEADY_TIMEOUT_MS) {
    const frames = await collectFrames();
    if (run !== autoRun) throw cancelled();
    const {angles, unstable} = steadyAngles(frames);
    if (!CORE_ANGLES.every(k => k in angles)) { phaseText = sk().names[index] + "：骨格が認識できません。肩・肘・手首を映してください"; continue; }
    if (sameAsEarlier(index, angles)) { phaseText = sk().names[index] + "：前の姿勢から動いていません。ロボットと同じ姿勢をとってください"; continue; }
    if (unstable.some(k => CORE_ANGLES.includes(k) || k === "pan")) { phaseText = sk().names[index] + "：じっとしていてください"; continue; }
    await submitCapture(index, frames, angles, unstable);
    return;
  }
  throw new Error(sk().names[index] + "を撮影できませんでした（姿勢が定まりませんでした）");
}
async function autoCalibrate(indices) {
  if (!camera) throw new Error("先に「映像を開く」でカメラを開いてください");
  const run = ++autoRun;
  try {
    for (const index of indices) {
      const name = sk().names[index];
      phaseText = name + "：ロボットが動いています"; render();
      state = await command("skeleton_pose", {index}); render();
      await waitUntil(() => sk().at === index && !sk().moving, ARRIVE_TIMEOUT_MS, run, "ロボットが" + name + "に着きませんでした");
      say(name + "。同じ姿勢をとってください");
      for (let left = Math.round(READY_DELAY_MS/1000); left > 0; left--) {
        phaseText = `${name}：ロボットと同じ姿勢をとってください（${left}）`; await sleep(1000);
        if (run !== autoRun) throw cancelled();
      }
      phaseText = name + "：じっとしていてください";
      await captureWhenSteady(index, run);
      say("撮影しました");
    }
    say(indices.length > 1 ? "較正が終わりました" : "撮影しました");
  } catch (e) {
    if (!e.cancelled) throw e;
  } finally {
    if (run === autoRun) phaseText = "";
    render();
  }
}
function buildPoses() {
  const box = $("poses"); box.textContent = "";
  sk().names.forEach((name, index) => {
    const card = document.createElement("div"); card.className = "pose"; card.id = "pose" + index;
    const title = document.createElement("h3"), img = document.createElement("img"), status = document.createElement("p"), row = document.createElement("div");
    title.textContent = name; status.className = "help";
    const pose = HUMAN_POSES[index], about = document.createElement("ul"); about.className = "help";
    if (pose) for (const line of describePose(pose)) about.append(Object.assign(document.createElement("li"), {textContent: line})); row.className = "row"; img.alt = name + "の撮影画像";
    const move = document.createElement("button"), go = document.createElement("button");
    move.textContent = "ロボットをこの姿勢へ"; move.dataset.role = "move";        // 動かすだけ（撮影しない。姿勢の確認用）
    go.textContent = "この姿勢だけ撮り直す"; go.dataset.role = "go";
    bind(move, async () => { cancelAuto(); state = await command("skeleton_pose", {index}); render(); });
    bind(go, () => autoCalibrate([index]));
    row.append(move, go); card.append(title, pose ? armFigure(pose) : "", about, img, status, row); box.append(card);
  });
}
function renderPoses() {
  const s = sk();
  if ($("poses").children.length !== s.names.length) buildPoses();
  const idle = !s.following && !s.moving && !phaseText;
  $("auto").disabled = !idle || state.drive !== "on" || !camera;
  s.names.forEach((_, i) => {
    const card = $("pose" + i), img = card.querySelector("img"), status = card.querySelector("p");
    card.querySelector('[data-role="go"]').disabled = $("auto").disabled;
    card.querySelector('[data-role="move"]').disabled = s.following || s.moving || !!phaseText || state.drive !== "on";
    img.hidden = !thumbs[i]; if (thumbs[i] && img.src !== thumbs[i]) img.src = thumbs[i];
    status.textContent = s.captured[i] ? "撮影済み" : "未撮影";
    status.className = s.captured[i] ? "done" : "help";
  });
}
function renderFits() {
  const table = $("fits"); table.textContent = "";
  const head = table.insertRow();
  for (const text of ["関節", "人の角度", "判定", "詳細"]) head.append(Object.assign(document.createElement("th"), {textContent: text}));
  for (const [joint, key] of Object.entries(sk().joints)) {
    const fit = sk().fits[joint], row = table.insertRow();
    const detail = !fit ? "未撮影" : fit.ok ? `傾き ${fit.a.toFixed(2)}／度、幅 ${fit.span.toFixed(0)}°、ずれ ${fit.resid.toFixed(1)}` : fit.reason;
    row.insertCell().textContent = JOINT_NAMES[joint]; row.insertCell().textContent = ANGLE_NAMES[key];
    const verdict = row.insertCell(); verdict.textContent = fit?.ok ? "使える" : "使えない"; verdict.className = fit?.ok ? "ok" : "ng";
    row.insertCell().textContent = detail;
  }
}

// ---- 追従 ----
function buildFollowJoints() {
  const box = $("followJoints"); box.textContent = "";
  for (const [joint, label] of Object.entries(JOINT_NAMES)) {
    const wrap = document.createElement("label"), check = document.createElement("input");
    check.type = "checkbox"; check.checked = true; check.dataset.joint = joint;
    wrap.append(check, label); box.append(wrap);
  }
}
const followJoints = () => [...document.querySelectorAll("#followJoints input")].filter(c => c.checked).map(c => c.dataset.joint);
bind($("start"), async () => {
  if (!camera) throw new Error("先に「映像を開く」でカメラを開いてください");
  const result = await command("skeleton_start");
  token = result.token; state = result; render();
});
setInterval(async () => {
  if (!token || sending) return;
  sending = true;
  try { state = await command("skeleton_sample", {token, features: fresh() ? latest : {}, follow: followJoints()}); }
  catch (e) { token = null; message(e.message); }
  finally { sending = false; }
}, SEND_INTERVAL_MS);

// ---- 表示 ----
function nextStep() {
  const s = sk();
  if (phaseText) return phaseText;
  if (!camera) return "「映像を開く」を押してください";
  if (state.drive !== "on") return "「モーターON」を押してください";
  if (s.following) return "追従中です。止めるときは「追従停止」（Space）";
  if (s.moving) return "ロボットが動いています。着くまで待ってください";
  if (s.captured.includes(false)) return "「自動で較正」を押してください（押したら、カメラから離れて姿勢をとれます）";
  return s.ready ? "「追従開始」を押してください" : "使える関節がありません。「較正をやり直す」で撮り直してください";
}
function render() {
  if (!state) return;
  const s = sk();
  $("mode").textContent = s.following ? "追従中" : s.moving ? "ロボット移動中" : "待機";
  $("next").textContent = nextStep();
  $("status").textContent = state.note || "待機中";
  $("driveState").textContent = "モーター: " + {on: "ON", soft: "やわらか", off: "OFF"}[state.drive];
  $("drive").disabled = state.drive === "on" || s.following;
  $("enable").textContent = camera ? "映像を閉じる" : "映像を開く";
  $("start").disabled = !s.ready || s.following || state.drive !== "on" || !camera;
  $("reset").disabled = s.following;
  $("side").disabled = s.following || !!phaseText;      // 追従・較正の途中で腕を替えない
  renderPoses(); renderFits();
}
bind($("drive"), async () => { state = await command("drive", {mode: "on"}); render(); });
bind($("auto"), () => autoCalibrate(sk().names.map((_, i) => i)));
bind($("reset"), async () => { cancelAuto(); for (const k of Object.keys(earlier)) delete earlier[k]; for (const k of Object.keys(thumbs)) delete thumbs[k]; state = await command("skeleton_reset"); render(); });
bind($("stop"), () => stopFollow("停止ボタン／スペースキー"));
document.addEventListener("keydown", e => {
  if (e.code === "Space" && !["INPUT", "SELECT", "BUTTON"].includes(e.target.tagName)) { e.preventDefault(); $("stop").click(); }
});
async function poll() {
  try {
    state = await (await fetch("/state")).json();
    if (token && !sk().following) token = null;     // サーバー側で止まった（故障・入力途絶など）
    render();
  } catch (e) { message(e.message); }
}
// 追従する腕（左右）。選んだ腕は覚えておく。左右で角度の符号が逆になるものがあるため、腕を替えたら較正は無効（やり直す）
const SIDE_KEY = "skeleton.side";
try { const saved = localStorage.getItem(SIDE_KEY); if (saved in ARMS) $("side").value = saved; } catch (e) { console.warn("storage", e); }
$("side").addEventListener("change", async () => {
  try { localStorage.setItem(SIDE_KEY, $("side").value); } catch (e) { console.warn("storage", e); }
  latest = raw = null;
  try {
    if (sk()?.captured.some(Boolean)) {
      for (const k of Object.keys(thumbs)) delete thumbs[k];
      for (const k of Object.keys(earlier)) delete earlier[k];
      state = await command("skeleton_reset"); render();
      message("腕を切り替えたので、較正をやり直してください");
    }
  } catch (e) { message(e.message); }
});
buildFollowJoints(); poll(); setInterval(poll, POLL_MS);
