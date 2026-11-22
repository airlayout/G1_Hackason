import {initLimits, renderLimits} from "/limits.js";
import {renderSync} from "/sync.js";
const $ = id => document.getElementById(id);
const joints = {shoulder_pan:"台座", shoulder_lift:"肩", elbow_flex:"肘", wrist_flex:"手首の曲げ", wrist_roll:"手首の回転", gripper:"グリッパー"};
const labels = {low:"つかむ位置", high:"持ち上げた位置"};
// 指の開き＝親指の先と人差し指の先の距離 ÷ 手のひらの幅（比）。最大に開いた比とくっつけた比を、1回だけ測って
// （指の校正）ブラウザに保存する。角度で測ると、つまんだときも角度が大きいままになり「閉じても開き」と判定された
// （2026-11-22）。校正前の表示用の仮の値は PROVISIONAL。校正が済むまで「追従開始」は使えない
const PROVISIONAL={open:1,closed:.15};
const CAL_DELAY_MS=3000, CAL_SAMPLE_MS=1000, CAL_STEADY=.12, CAL_MIN_GAP=.3, EDGE=.05;
// 手首の回転＝手の傾き。手首(0)→中指の付け根(9)の線が画面の真上から何度傾いたか（画面上で時計回りが＋）。
// ROLL_FULL_DEG 傾けると手首の可動範囲いっぱい。向きが逆なら ROLL_SIGN を -1 にする（1 のままで実機の向きが合うことを 2026-11-22 に確認済み）
const ROLL_FULL_DEG=45, ROLL_SIGN=1;
// 手首の曲げ＝手を奥へ倒したときの手のひらの縮み。縮み比＝(手首〜中指の付け根の長さ)÷(人差し指〜小指の付け根の幅)。
// 真っ直ぐ約1.6、奥に倒すと約1.0（2026-11-22 の実画像）。FLEX_STRAIGHT 以上は「曲げなし」（可動範囲の中央）、
// FLEX_FULL で端。手前に倒すと認識が外れるので片方向だけ。向きが逆なら FLEX_SIGN を -1 にする（1 のままで実機の向きが合うことを 2026-11-22 に確認済み）
const FLEX_STRAIGHT=1.55, FLEX_FULL=1.05, FLEX_SIGN=1;
// 台座＝手の左右位置。手の画面内の横位置（中指の付け根の x。0〜1）が 0.5±PAN_HALF_RANGE で台座の可動範囲の両端。
// 映像は鏡像にしていない。実機で 1 だと左右が逆だったので -1（2026-11-22）。手が画面の右（本人から見て左）へ行くと台座は負の側へ
const PAN_HALF_RANGE=.3, PAN_SIGN=-1;
// 肩は手の高さ（上げると＋）、肘は手の大きさ＝カメラへの近さ（人差し指の付け根〜小指の付け根が画面幅に占める割合。
// 手前ほど大きい）。向きは実機で確かめる値。逆なら符号を反転する。
// 肩は実機で 1 だと逆（手を上げても肩が上がらない）だったので -1（2026-11-22）。肩の値が小さいほど、実機の腕は上がる
const SHOULDER_SIGN=-1;
const REACH_FAR=.09, REACH_NEAR=.16, REACH_SIGN=1;     // 実測の範囲（遠い小さな手 0.082〜近い大きな手 0.177）の内側
let fingerCal={}, lastRatio=null, lastHand=null;
// 校正はサーバーのファイル（finger_calibration.json）に保存する。ブラウザを替えても、消しても、測り直しは要らない
async function loadFinger(){
  try{fingerCal=await (await fetch("/finger")).json(); $("fingerState").textContent=fingerText();}catch(e){info(e.message);}
}
async function saveFinger(update){
  const res=await fetch("/finger",{method:"POST",headers:{"Content-Type":"application/json"},body:JSON.stringify(update)});
  const data=await res.json();
  if(!res.ok) throw new Error(data.error);
  fingerCal=data;
}
const fingerReady=()=>Number.isFinite(fingerCal.open)&&Number.isFinite(fingerCal.closed);
const fingerRange=()=>fingerReady()?fingerCal:PROVISIONAL;
function fingerText(){
  const f=a=>Number.isFinite(a)?a.toFixed(2):"未測定";
  return (fingerReady()?"校正済み":"未校正")+"　開 "+f(fingerCal.open)+" ／ 閉 "+f(fingerCal.closed);
}
// 距離比から指の開き（0〜1）。両端の EDGE 以内は 0・1 に寄せる
function openingFromRatio(ratio){
  const {open,closed}=fingerRange(), v=(ratio-closed)/(open-closed);
  return v<EDGE?0:v>1-EDGE?1:v;
}
let state, token, tracker, camera, recorder, recordingURL;
let pending = false, starting = false, generation = 0, frame = -1, lastSample = 0, lastSeen = 0, latest;
let followed = false, historyKey = "", moving = false;
const targets = {}, stalls = {};
const MOVE_INTERVAL_MS = 35;   // スライダー操作の指令間隔。server.py の MOVE_STEP と合わせて、細かい刻みを速く送る
const MOVE_STALL_LIMIT = 15;   // この回数つづけて動かなければ、その目標を諦める（約0.5秒）
const SOFT_SEND_MS = 150;      // やわらかさのスライダーを動かしてから、実機へ送るまでの待ち
const DRIVE_POLL_MS = 300;    // OFF・やわらかのあいだ、手で動かした姿勢を画面へ映す間隔
const DRIVES = [
  {mode:"off", name:"OFF", sub:"力を抜く（手で動かせる）"},
  {mode:"soft", name:"やわらか", sub:"弱い力で保持（押すと動く）"},
  {mode:"on", name:"ON", sub:"通常の力で保持"},
];
const DRIVE_HELP = {
  off:"モーターの力が抜けています。腕が自重で下がることがあるので、手で支えてください。手で作った姿勢は「位置の登録」で記録できます。",
  soft:"弱い力で保持しています。重い関節（肩）は少し下がることがあります。スライダーと追従は、ON に戻すと使えます。",
  on:"通常の力で保持しています。",
};
const pct = (j,v) => j==="gripper" ? v : (v+100)/2;
const rawOf = (j,p) => j==="gripper" ? p : p*2-100;
const pctText = (j,v) => pct(j,v).toFixed(1)+"%";
const info = message => { $("error").textContent = message; };
const limit = value => Math.max(0, Math.min(1,value));
const poseText = pose => "肩 "+pctText("shoulder_lift",pose.shoulder_lift)+" ／ 肘 "+pctText("elbow_flex",pose.elbow_flex)+" ／ グリッパー "+pctText("gripper",pose.gripper);
function buildMarkTable() {
  const head=document.createElement("tr"); head.append(document.createElement("th"));
  for (const name of Object.values(joints)) { const th=document.createElement("th"); th.textContent=name; head.append(th); }
  $("markTable").append(head);
  for (const [key,name] of Object.entries(labels)) {
    const row=document.createElement("tr"), th=document.createElement("th");
    th.textContent=name; row.append(th);
    for (const j of Object.keys(joints)) {
      const td=document.createElement("td"); td.id="m-"+key+"-"+j;
      // つかむ位置・持ち上げた位置は腕の列が使われる（グリッパーは登録なし）
      if (j!=="gripper") td.className="used";
      row.append(td);
    }
    $("markTable").append(row);
  }
}
function renderMarks(data) {
  for (const key of Object.keys(labels))
    for (const j of Object.keys(joints)) $("m-"+key+"-"+j).textContent = data.marks[key] ? pct(j,data.marks[key][j]).toFixed(1) : "—";
  $("problem").textContent = data.problem || "";
  const nextKey = JSON.stringify(data.history);
  if (nextKey === historyKey) return;
  historyKey = nextKey;
  $("history").replaceChildren(...[...data.history].reverse().map(h => {
    const li=document.createElement("li"); li.textContent=h.time+"　"+labels[h.label]+"　"+poseText(h.pose); return li;
  }));
  $("noHistory").hidden = data.history.length > 0;
}
function render(data) {
  state = data;
  $("mode").textContent = data.live ? "実機接続" : "模擬モード";
  $("status").textContent = data.note;
  $("sim").hidden = data.live;
  for (const j of Object.keys(joints)) {
    $("pos-"+j).value = pctText(j, data.pose[j]);
    if (!(j in targets)) $("slide-"+j).value = pct(j, data.pose[j]);   // 操作中の目標は上書きしない
  }
  renderCalibration(data.calibration);
  for (const [key,name] of Object.entries(labels)) {
    const b=$("mark-"+key), done=key in data.marks;
    b.firstChild.textContent = (done ? "✓ " : "") + name+"を登録";
    b.lastChild.textContent = markSummary(key, data.marks[key]);
    b.classList.toggle("done", done);
  }
  renderMarks(data);
  renderDrive(data);
  renderLimits(data);
  renderSync(data);
  const blocked = data.active || data.fault || !data.calibration.done;
  const driven = data.drive === "on";   // OFF・やわらかのときは、スライダーと追従を使えない（姿勢の登録は使える）
  $("start").disabled = blocked || !driven || Object.keys(data.marks).length !== 2;
  $("startGripper").disabled = blocked || !driven;
  $("homeSet").disabled = blocked;      // モーター OFF（手で姿勢を作る）のときも記録できる
  $("homeGo").disabled = blocked || !driven || !data.home;
  $("homeState").textContent = data.home ? "初期姿勢: 登録済み" : "初期姿勢: 未登録（追従は、動かせる範囲の真ん中が基準になる）";
  for (const b of document.querySelectorAll("#joints input")) b.disabled = blocked || !driven;
  for (const b of document.querySelectorAll("#marks button")) b.disabled = blocked;
  for (const b of document.querySelectorAll("#drive button")) b.disabled = data.fault || !data.calibration.done;
}
function renderDrive(data) {
  for (const b of document.querySelectorAll("#drive button")) b.setAttribute("aria-pressed", String(b.dataset.mode === data.drive));
  $("driveHelp").textContent = DRIVE_HELP[data.drive] ?? "";
  if (document.activeElement !== $("softLevel")) {          // 操作中のスライダーは上書きしない
    $("softLevel").value = data.soft_level;
    $("softValue").textContent = Math.round(data.soft_level)+"%";
  }
}
function renderCalibration(c) {
  $("calibState").textContent = c.done
    ? (c.required ? "✓ 校正済み（"+c.detail+"）" : "✓ "+c.detail)
    : "未校正: "+c.detail;
  $("calibTodo").hidden = c.done;
  $("calibCmd").textContent = c.command;
}
// 手順の表示。上から順に、最初に終わっていない手順が「次にやること」
const recording = () => recorder?.state === "recording";
const steps = [
  {text:"モーターを校正", help:"画面右の「モーターの校正」の案内に従い、校正を終えて「校正を確認」を押す", done:() => state?.calibration.done},
  {text:"つかむ位置を登録", help:"現在位置のスライダーで爪を袋の左右に合わせ、「つかむ位置を登録」を押す", done:() => state && "low" in state.marks},
  {text:"持ち上げた位置を登録", help:"肩か肘を少し動かして袋が浮く姿勢にし、「持ち上げた位置を登録」を押す", done:() => state && "high" in state.marks},
  {text:"操作用の映像を開く", help:"「カメラ一覧を更新」→ カメラを選ぶ →「映像を開く」。模擬モードではスライダー入力でも可", done:() => camera || isManual()},
  {text:"指の開閉を校正（1回だけ）", help:"「指を最大に開く」を押し、3秒後に測る間、指を大きく開いたままにする。「指をくっつける」も同じ。結果はブラウザに保存される", done:() => fingerReady() || isManual()},
  {text:"追従を試す（空運転）", help:"手を下の線に合わせ、指を開いて「追従開始」。手を上下し、つまむ・開くを試す。止めるのはスペースキー", done:() => followed},
  {text:"録画を開始する", help:"カメラに手・爪・袋が一緒に映っているのを確かめ、「録画開始」を押す", done:() => recording() || Boolean(recordingURL)},
  {text:"録画を終えて保存する", help:"追従を止めて「録画終了」→「動画を保存」。保存した動画を再生して確認する", done:() => Boolean(recordingURL) && !recording()},
];
function buildSteps() {
  for (const step of steps) { const li=document.createElement("li"); li.textContent=step.text; $("steps").append(li); }
}
function updateGuide() {
  if (!state) return;
  const index = steps.findIndex(s => !s.done());
  [...$("steps").children].forEach((li, i) => {
    const done = steps[i].done();
    li.className = done ? "done" : i === index ? "now" : "";
    li.textContent = (done ? "✓ " : "") + steps[i].text;
  });
  let next;
  if (state.fault) next = "通信異常です。実機と配線を確認し、サーバーを再起動してください";
  else if (!state.calibration.done) next = "先にモーターの校正が必要です。画面右の案内に従ってください";
  else if (state.active) next = state.gripper_only ? "追従中（全関節）です。指を開閉し、手を上下・左右・前後へ動かし、傾け、手首を奥へ曲げて動きを確認する。止めるにはスペースキー" : "追従中です。手を上下し、指を開閉して動きを確認する。止めるにはスペースキー";
  else if (state.problem) next = "直す必要があります: " + state.problem;
  else if (index < 0) next = "すべて完了です。保存した動画を再生して、袋が机から離れたことを確認してください";
  else next = "手順 " + (index + 1) + "／" + steps.length + "：" + steps[index].text + "　" + steps[index].help;
  if ($("next").textContent !== next) $("next").textContent = next;
  const nextMark = index >= 1 && index <= 2 ? Object.keys(labels)[index-1] : null;
  for (const key of Object.keys(labels)) $("mark-"+key).classList.toggle("next", key === nextMark);
}
function markSummary(key, pose) {
  if (!pose) return "未登録";
  return "肩 "+pctText("shoulder_lift",pose.shoulder_lift)+" ／ 肘 "+pctText("elbow_flex",pose.elbow_flex);
}

// 仮想アーム（横と正面から見た模式図）。角度への換算は見た目用。向きは実機で確かめた分だけ合わせてある:
// 肩は、値が小さいほど腕が上がる（確認済み 2026-11-22）。肘・台座・手首の回転の向きは未確認（仮）
const ARM = {side:{base:[95,205], tag:"arm-"}, front:{base:[150,205], tag:"af-"}, lengths:[110,100,50], jaw:45};
const ARM_SHOULDER_SIGN = -1, ARM_PAN_DEG = 0.45, ARM_PAN_SIGN = 1, ARM_ROLL_DEG = 0.9;   // 正面図: 台座＋は画面の右、手首の回転＋は時計回り
const PREVIEW_RANGE = {shoulder_lift:20, elbow_flex:10};   // 登録前の予定姿勢で上げ下げする幅
const toRad = deg => deg*Math.PI/180;
function setAttrs(id, attrs) { const el=$(id); for (const [k,v] of Object.entries(attrs)) el.setAttribute(k, v.toFixed(1)); }
// 腕の面の中の点 (r, z)（r: 台座の軸からの水平の距離、z: 肩からの高さ）を、各リンクの角度から求める
function armPoints(pose) {
  const a1 = 45 + ARM_SHOULDER_SIGN*0.45*pose.shoulder_lift, a2 = a1 - 70 + 0.5*pose.elbow_flex, a3 = a2 - 30 + 0.5*pose.wrist_flex;
  const pts = [[0,0]];
  [a1,a2,a3].forEach((a,i) => {
    const [r,z] = pts[i];
    pts.push([r + ARM.lengths[i]*Math.cos(toRad(a)), z + ARM.lengths[i]*Math.sin(toRad(a))]);
  });
  return {pts, tipAngle:a3};
}
function drawLinks(tag, screen) {          // screen: 腕の4点の画面上の位置
  setAttrs(tag+"l1", {x2:screen[1][0], y2:screen[1][1]});
  setAttrs(tag+"l2", {x1:screen[1][0], y1:screen[1][1], x2:screen[2][0], y2:screen[2][1]});
  setAttrs(tag+"l3", {x1:screen[2][0], y1:screen[2][1], x2:screen[3][0], y2:screen[3][1]});
  for (const i of [1,2,3]) setAttrs(tag+"j"+i, {cx:screen[i][0], cy:screen[i][1]});
}
function drawArm(pose) {
  const {pts, tipAngle} = armPoints(pose), grip = Math.max(0, Math.min(100, pose.gripper));
  const pan = toRad(ARM_PAN_SIGN*ARM_PAN_DEG*pose.shoulder_pan), roll = toRad(ARM_ROLL_DEG*pose.wrist_roll);
  // 横から: 台座の回転で、腕は奥行き（r·cos）へ縮む
  const [sx, sy] = ARM.side.base, side = pts.map(([r,z]) => [sx + r*Math.cos(pan), sy - z]);
  drawLinks(ARM.side.tag, side);
  const spread = 3 + 0.4*grip;
  for (const [id,sign] of [["arm-ja",1],["arm-jb",-1]]) {
    const a = toRad(tipAngle + sign*spread);
    setAttrs(id, {x1:side[3][0], y1:side[3][1], x2:side[3][0] + ARM.jaw*Math.cos(a), y2:side[3][1] - ARM.jaw*Math.sin(a)});
  }
  // 正面から: 台座の回転で、腕は左右（r·sin）へ振れる。手首の回転は、指の開く向きの傾きに出る
  const [fx, fy] = ARM.front.base, front = pts.map(([r,z]) => [fx + r*Math.sin(pan), fy - z]);
  drawLinks(ARM.front.tag, front);
  const k = 8 + 0.35*grip, [ux, uy] = [Math.sin(roll), -Math.cos(roll)];
  for (const [id,sign] of [["af-ja",1],["af-jb",-1]])
    setAttrs(id, {x1:front[3][0], y1:front[3][1], x2:front[3][0] + sign*k*ux, y2:front[3][1] + sign*k*uy});
}
function handInput(now) {
  if (isManual()) return {height:$("height").value/100, opening:$("opening").value/100};
  return latest && now - lastSeen < 300 ? latest : null;
}
// 手の入力から求めた予定の姿勢。グリッパーは固定範囲。腕は、登録が揃っていれば登録した2姿勢の間、なければ現在の姿勢を基準にした仮の範囲
function targetPose({height, opening}) {
  const m = state.marks, ready = Object.keys(labels).every(k => k in m), out = {};
  for (const j of Object.keys(joints)) {
    if (j === "gripper") {
      const {closed, open} = state.gripper;
      out[j] = closed + (open-closed)*opening;
    } else {
      const a = ready ? m.low[j] : state.pose[j], b = ready ? m.high[j] : a + (PREVIEW_RANGE[j] ?? 0);
      out[j] = a + (b-a)*height;
    }
  }
  return out;
}
function updateArm() {
  if (!state) return;
  const input = handInput(performance.now());
  let pose = state.pose, label = "現在の姿勢", preview = false;
  if (state.active) label = "実際の姿勢（追従中）";
  else if (input) { pose = targetPose(input); label = "予定の姿勢（アームは動いていません）"; preview = true; }
  drawArm(pose);
  const mode = $("armMode");
  if (mode.textContent !== label) mode.textContent = label;
  mode.classList.toggle("preview-mode", preview);
  for (const k of ["height","opening"]) {
    $("bar-"+k).style.width = input ? Math.round(input[k]*100)+"%" : "0%";
    $("val-"+k).textContent = input ? Math.round(input[k]*100)+"%" : "—";
  }
}
buildMarkTable(); buildSteps(); setInterval(updateGuide, 250); setInterval(updateArm, 50);
async function command(op, fields={}) {
  const res = await fetch("/command",{method:"POST",headers:{"Content-Type":"application/json"},body:JSON.stringify({op,...fields})});
  const data = await res.json();
  if (!res.ok) throw new Error(data.error);
  return data;
}
function bind(button, action) {
  button.onclick = async () => { try { info(""); await action(); } catch(e) { info(e.message); } };
}
// why は停止の原因。サーバーのデバッグログ（debug/follow-*.jsonl）の stop に残る
async function stop(why) {
  generation++; token = null; for (const k of Object.keys(targets)) delete targets[k];
  render(await command("stop", why ? {why} : {}));
}
bind($("stop"),()=>stop("停止ボタン／スペースキー"));
initLimits({joints, command, info, render});
window.addEventListener("keydown", e => {
  if(e.code === "Space"){e.preventDefault(); if(!e.repeat) $("stop").click();}
});
document.addEventListener("visibilitychange", () => {
  if(document.hidden && (token || starting)) stop("タブが隠れた").catch(e=>info(e.message));
});
// 現在位置: スライダーは可動範囲に対する 0〜100%。離した後も、目標へ少しずつ近づける（1回の指令で動く量はサーバーが制限）
for (const [j,name] of Object.entries(joints)){
  const row=document.createElement("div"), text=document.createElement("span"), value=document.createElement("output"), slider=document.createElement("input");
  text.textContent=name; value.id="pos-"+j; value.textContent="—";
  slider.type="range"; slider.min=0; slider.max=100; slider.step=0.5; slider.value=50; slider.id="slide-"+j; slider.setAttribute("aria-label",name);
  slider.addEventListener("input", () => { targets[j]=Number(slider.value); stalls[j]=0; });
  row.append(text,slider,value); $("joints").append(row);
}
// 初期姿勢: 今の姿勢を記録する（サーバーが home.json に保存）／記録した姿勢へ戻す。戻すのはスライダーと同じ仕組みで、少しずつ動く
bind($("homeSet"),async()=>{
  if(!state || state.active) throw new Error("追従中は記録できません。先に停止してください");
  render(await command("home"));
});
bind($("homeGo"),async()=>{
  if(!state || state.active) throw new Error("追従中は動かせません。先に停止してください");
  if(!state.home) throw new Error("初期姿勢が未登録です。机にグリッパーを置いた姿勢で「今の姿勢を初期姿勢にする」を押してください");
  for(const [j,v] of Object.entries(state.home)){
    targets[j]=pct(j,v); stalls[j]=0; $("slide-"+j).value=targets[j];
  }
});
async function tickMoves(){
  if (moving || !state || state.active || state.fault) return;
  const names=Object.keys(targets);
  if (!names.length) return;
  moving=true;
  try {
    for (const j of names) {
      const before=pct(j,state.pose[j]);
      if (Math.abs(targets[j]-before)<0.5) { delete targets[j]; delete stalls[j]; continue; }
      const data=await command("move",{joint:j,value:rawOf(j,targets[j])});
      stalls[j]=Math.abs(pct(j,data.pose[j])-before)<0.05 ? (stalls[j]??0)+1 : 0;
      if (stalls[j]>MOVE_STALL_LIMIT) {   // 動かない（限界・負荷）なら諦め、押し続けないよう今の位置で保持する
        delete targets[j]; delete stalls[j];
        render(await command("move",{joint:j,value:data.pose[j]}));
        continue;
      }
      render(data);
    }
  } catch(e) {
    for (const k of Object.keys(targets)) delete targets[k];
    info(e.message);
  } finally { moving=false; }
}
setInterval(tickMoves, MOVE_INTERVAL_MS);
// モーターの切り替え。切り替えると、スライダーの目標と追従は止まる（サーバー側でも止める）
for (const {mode,name,sub} of DRIVES) {
  const b=document.createElement("button"), title=document.createElement("strong"), small=document.createElement("small");
  b.dataset.mode=mode; title.textContent=name; small.textContent=sub; b.append(title,small);
  bind(b, async()=>{
    generation++; token=null; for (const k of Object.keys(targets)) delete targets[k];
    render(await command("drive",{mode, level:Number($("softLevel").value)}));
  });
  $("drive").append(b);
}
// やわらかさ: やわらかモード中に動かすと、その場で保持の強さを変える（間引いて送る）
let softTimer;
$("softLevel").addEventListener("input", () => {
  $("softValue").textContent = $("softLevel").value+"%";
  if (state?.drive !== "soft") return;
  clearTimeout(softTimer);
  softTimer = setTimeout(async () => {
    try { info(""); render(await command("drive",{mode:"soft", level:Number($("softLevel").value)})); }
    catch(e) { info(e.message); }
  }, SOFT_SEND_MS);
});
let polling=false;
async function pollPose(){
  if (polling || !state || state.drive==="on" || state.active) return;
  polling=true;
  try { render(await (await fetch("/state")).json()); } catch(e) { info(e.message); } finally { polling=false; }
}
setInterval(pollPose, DRIVE_POLL_MS);
bind($("calibCheck"), async()=>render(await command("calibration_check")));
for (const key of Object.keys(labels)){
  const b=document.createElement("button"), name=document.createElement("strong"), sub=document.createElement("small");
  b.id="mark-"+key; name.textContent=labels[key]+"を登録"; sub.textContent=markSummary(key); b.append(name,sub);
  bind(b, async()=>render(await command("mark",{label:key}))); $("marks").append(b);
}
const isManual = () => state && !state.live && $("manual").checked;
const sleep = ms => new Promise(resolve => setTimeout(resolve, ms));
// ボタンを押してから CAL_DELAY_MS 待ち（手を構える時間）、CAL_SAMPLE_MS の間の角度の中央値を返す
async function measureAngle() {
  if (!camera) throw new Error("先に「映像を開く」でカメラを開いてください");
  for (let s = CAL_DELAY_MS/1000; s > 0; s--) { $("fingerState").textContent = s+" 秒後に測ります。指の形を作ってください"; await sleep(1000); }
  $("fingerState").textContent = "測定中…";
  const values = [], end = performance.now() + CAL_SAMPLE_MS;
  while (performance.now() < end) {
    if (lastRatio !== null && performance.now()-lastSeen < 150) values.push(lastRatio);
    await sleep(30);
  }
  if (values.length < 5) throw new Error("手が映っていません。手を映してやり直してください");
  values.sort((a, b) => a-b);
  if (values.at(-1) - values[0] > CAL_STEADY) throw new Error("測定中に指が動きました。形を保ってやり直してください");
  return values[values.length >> 1];
}
function bindFingerCal(button, key) {
  bind(button, async () => {
    try {
      const angle = await measureAngle(), next = {...fingerCal, [key]: angle};
      if (Number.isFinite(next.open) && Number.isFinite(next.closed) && next.open - next.closed < CAL_MIN_GAP)
        throw new Error("開いた比（"+next.open.toFixed(2)+"）とくっつけた比（"+next.closed.toFixed(2)+"）の差が "+CAL_MIN_GAP+" 未満です。指を大きく開き直してください");
      await saveFinger({[key]: angle});
    } finally { $("fingerState").textContent = fingerText(); }
  });
}
// 診断の保存: 押した瞬間の映像（認識点つき）と数値を、サーバーの debug フォルダへ送る（原因調査用）
bind($("snap"), async () => {
  const video = $("input");
  if (!camera || video.readyState < 2) throw new Error("先に「映像を開く」でカメラを開いてください");
  const canvas = document.createElement("canvas");
  canvas.width = video.videoWidth; canvas.height = video.videoHeight;
  const ctx = canvas.getContext("2d");
  ctx.drawImage(video, 0, 0); ctx.drawImage($("overlay"), 0, 0);
  const info = {time: new Date().toISOString(), hand: $("hand").textContent, finger: {...fingerCal},
    latest, ...lastHand, size: [canvas.width, canvas.height], drive: state?.drive, active: state?.active};
  const res = await fetch("/snapshot", {method: "POST", headers: {"Content-Type": "application/json"},
    body: JSON.stringify({image: canvas.toDataURL("image/jpeg", .85), info})});
  const data = await res.json();
  if (!res.ok) throw new Error(data.error);
  $("snapState").textContent = "保存: " + data.saved;
});
bindFingerCal($("calOpen"), "open"); bindFingerCal($("calClosed"), "closed");
$("fingerState").textContent = fingerText(); loadFinger();
bind($("start"), () => startFollow(false));
bind($("startGripper"), () => startFollow(true));
// 追従する関節のチェック（「全関節を追従」用）。サンプルごとにサーバーへ送る。サーバーは、チェックの無い関節へ指令を出さない
const followJoints = () => [...document.querySelectorAll("#followBox input[data-joint]")].filter(c => c.checked).map(c => c.dataset.joint);
for (const [id, on] of [["followAll", true], ["followNone", false]])
  $(id).addEventListener("click", () => document.querySelectorAll("#followBox input[data-joint]").forEach(c => { c.checked = on; }));
async function startFollow(gripperOnly){
  if(starting) return;
  if(!isManual() && !fingerReady()) throw new Error("先に指の開閉を校正してください（「指を最大に開く」「指をくっつける」）");
  if(!isManual() && (!latest || performance.now()-lastSeen>300)) throw new Error("操作カメラに片手を映してください");
  starting=true;
  const sequence=++generation;
  try {
    for (const k of Object.keys(targets)) delete targets[k];
    const result=await command("start", gripperOnly ? {gripper_only: true} : {});
    if(sequence!==generation){await stop("開始中に別の操作が入った"); return;}
    token=result.token; followed=true; render(result);
  } finally {starting=false;}
}
function constraint(id){return {video:id?{deviceId:{exact:id}}:true,audio:false};}
async function listCameras(){
  const devices=(await navigator.mediaDevices.enumerateDevices()).filter(d=>d.kind==="videoinput" && d.deviceId);
  for(const id of ["cam"]){
    const select=$(id), selected=select.value;
    while(select.options.length>1) select.remove(1);
    devices.forEach((d,i)=>select.add(new Option(d.label || "カメラ "+(i+1),d.deviceId)));
    if([...select.options].some(o=>o.value===selected)) select.value=selected;
  }
}
bind($("devices"),async()=>{
  if(!camera){const temp=await navigator.mediaDevices.getUserMedia(constraint(""));temp.getTracks().forEach(t=>t.stop());}
  await listCameras();
});
async function loadTracker(){
  if(tracker) return;
  const {FilesetResolver,HandLandmarker} = await import("/assets/vision_bundle.mjs");
  tracker=await HandLandmarker.createFromOptions(await FilesetResolver.forVisionTasks("/assets"),{
    baseOptions:{modelAssetPath:"/assets/hand_landmarker.task"}, runningMode:"VIDEO", numHands:1
  });
}
function closeStream(stream){stream?.getTracks().forEach(t=>t.stop());}
// 映像ボタンは開閉の切り替え。開いている間は「映像を閉じる」、閉じている間は「映像を開く」
function showCameraButton(){$("enable").textContent=camera?"映像を閉じる":"映像を開く";}
async function closeCamera(){
  await stop("映像を閉じた");
  closeStream(camera); camera=null; latest=null; lastRatio=null; lastHand=null; frame=-1;
  $("input").srcObject=null;
  const canvas=$("overlay"); canvas.getContext("2d").clearRect(0,0,canvas.width,canvas.height);
  $("hand").textContent="停止中"; $("perf").textContent="認識 — ms／フレーム　— fps";
}
async function openCamera(){
  await stop("映像を開き直した");
  await loadTracker();
  camera=await navigator.mediaDevices.getUserMedia(constraint($("cam").value));
  $("input").srcObject=camera; await $("input").play(); await listCameras();
}
bind($("enable"),async()=>{
  if(recorder?.state==="recording") throw new Error("録画終了後に切り替えてください");
  try{ await (camera?closeCamera():openCamera()); }
  finally{ showCameraButton(); }
});
bind($("record"),async()=>{
  const stream=camera;
  if(!stream || stream.getVideoTracks()[0]?.readyState!=="live") throw new Error("先に「映像を開く」でカメラを開いてください");
  if(!window.MediaRecorder) throw new Error("録画にはデスクトップ版Chromeを使ってください");
  const mime=["video/webm;codecs=vp8","video/webm","video/mp4"].find(t=>MediaRecorder.isTypeSupported(t));
  recorder=new MediaRecorder(stream,mime?{mimeType:mime}:{});
  const chunks=[];
  recorder.ondataavailable=e=>{if(e.data.size) chunks.push(e.data);};
  recorder.onstop=()=>{
    $("finish").disabled=true; $("record").disabled=false;
    if(!chunks.length){info("空の動画です。カメラ接続を確認してください"); return;}
    if(recordingURL) URL.revokeObjectURL(recordingURL);
    const type=recorder.mimeType, ext=type.includes("mp4")?"mp4":"webm";
    recordingURL=URL.createObjectURL(new Blob(chunks,{type}));
    const a=$("save"); a.href=recordingURL; a.download="so101-"+Date.now()+"."+ext; a.hidden=false;
    $("filmState").textContent="録画終了。動画を保存してください";
  };
  recorder.start(500); $("record").disabled=true; $("finish").disabled=false;
  $("filmState").textContent="● 録画中";
});
bind($("finish"),()=>recorder.stop());

const perf={ms:0,count:0,shown:0};      // 認識の処理時間と1秒あたりの処理数（500ms ごとに更新）
function track(now){
  const video=$("input");
  if(!tracker || !camera || video.readyState<2 || video.currentTime===frame) return;
  frame=video.currentTime;
  const began=performance.now();
  const pts=tracker.detectForVideo(video,now).landmarks[0];
  perf.ms+=performance.now()-began; perf.count++;
  if(now-perf.shown>=500){
    $("perf").textContent="認識 "+(perf.ms/perf.count).toFixed(1)+" ms／フレーム　"+(perf.count*1000/(now-perf.shown)).toFixed(1)+" fps";
    perf.ms=0; perf.count=0; perf.shown=now;
  }
  const canvas=$("overlay"), ctx=canvas.getContext("2d");
  canvas.width=video.videoWidth; canvas.height=video.videoHeight;
  const w=canvas.width,h=canvas.height;
  ctx.strokeStyle="#ffe081";
  for(const y of [.28,.72]){ctx.beginPath();ctx.moveTo(0,y*h);ctx.lineTo(w,y*h);ctx.stroke();}
  // latest は消さない。認識が1フレーム抜けても追従は止めず、loop が lastSeen（0.26秒）で「手を見失った」を判断する
  lastRatio=null; lastHand=null;
  if(!pts){$("hand").textContent="手が見えません";return;}
  const distance=(a,b)=>Math.hypot((pts[a].x-pts[b].x)*w,(pts[a].y-pts[b].y)*h);
  const palm=distance(0,9);                     // 手首〜中指の付け根。横向きでも手のひら幅ほど縮まない
  if(palm<12){$("hand").textContent="手をもう少し大きく映してください";return;}
  const vec=(a,b)=>[(pts[b].x-pts[a].x)*w,(pts[b].y-pts[a].y)*h];
  const [tx,ty]=vec(2,4), [ix,iy]=vec(5,8);
  const angle=Math.acos(Math.max(-1,Math.min(1,(tx*ix+ty*iy)/(Math.hypot(tx,ty)*Math.hypot(ix,iy)||1))))*180/Math.PI;
  const ratio=distance(4,8)/palm;
  lastRatio=ratio;
  const [rx,ry]=vec(0,9), tilt=Math.atan2(rx,-ry)*180/Math.PI;   // 真上が 0°、時計回りが＋
  const roll=Math.max(-1,Math.min(1,ROLL_SIGN*tilt/ROLL_FULL_DEG));
  const shrink=palm/distance(5,17)||0;
  const flex=FLEX_SIGN*limit((FLEX_STRAIGHT-shrink)/(FLEX_STRAIGHT-FLEX_FULL));
  const pan=PAN_SIGN*Math.max(-1,Math.min(1,(pts[9].x-.5)/PAN_HALF_RANGE));
  const height=limit((.72-pts[0].y)/.44);
  const width=distance(5,17)/w;
  const lift=SHOULDER_SIGN*(2*height-1);
  const reach=REACH_SIGN*(2*limit((width-REACH_FAR)/(REACH_NEAR-REACH_FAR))-1);
  lastHand={ratio,angle,tilt,shrink,pts:pts.map(({x,y,z})=>[+x.toFixed(4),+y.toFixed(4),+z.toFixed(4)])};
  latest={height,opening:limit(openingFromRatio(ratio)),roll,flex,pan,lift,reach,
    // 画面側の生の値。サーバーのデバッグログ（debug/follow-*.jsonl）に、入力の元の値として残る
    dbg:{ratio,tilt,shrink,palm,width,x:pts[9].x,y:pts[0].y,angle}};
  for(const i of [0,4,8]){ctx.fillStyle="#60ffb1";ctx.beginPath();ctx.arc(pts[i].x*w,pts[i].y*h,5,0,2*Math.PI);ctx.fill();}
  lastSeen=now; $("hand").textContent="手を認識　高さ "+Math.round(latest.height*100)+"% ／ 開き "+Math.round(latest.opening*100)+"%（距離比 "+ratio.toFixed(2)+"、角度 "+Math.round(angle)+"°）　傾き "+Math.round(tilt)+"°　縮み "+shrink.toFixed(2);
}
async function loop(now){
  try{
    track(now);
    if(token && !pending && now-lastSample>65){
      const sample=isManual()?{height:$("height").value/100,opening:$("opening").value/100}:latest;
      pending=true; lastSample=now;
      try{
        if(!sample || (!isManual() && now-lastSeen>260)){await stop(sample?"手を見失った（0.26秒以上）":"手の入力なし"); return;}
        const seq=generation;
        const result=await command("sample",{token,...sample,...(state?.gripper_only?{follow:followJoints()}:{})});
        if(seq===generation) render(result);
      }finally{pending=false;}
    }
  }catch(e){
    const wasActive=Boolean(token); token=null;
    if(wasActive){try{await stop("画面側のエラー: "+e.message);}catch{}}
    info(e.message);
  }finally{requestAnimationFrame(loop);}
}
try{render(await (await fetch("/state")).json());}catch(e){info(e.message);}
requestAnimationFrame(loop);
