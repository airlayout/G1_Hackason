// 手と腕の姿勢のずれの表示。サーバーが測った、関節ごとの手の入力（hand）と腕の姿勢（robot）を、同じ尺度（-1〜1）で並べる。
// 同期待ちの間は、ずれの大きい関節に「手をどちらへ動かすと合うか」を出す（腕は、すべて合うまで動かさない）。
const ROWS = [
  ["shoulder_pan", "pan", "台座"], ["shoulder_lift", "lift", "肩"], ["elbow_flex", "reach", "肘"],
  ["wrist_flex", "flex", "手首の曲げ"], ["wrist_roll", "roll", "手首の回転"],
];
const rowOf = {};
const pos = v => (Math.max(-1, Math.min(1, v)) + 1) * 50 + "%";

// 入力を need（＋なら入力を増やす向き）へ動かすために、手をどうするか。signs は入力の符号（app.js の定数）
function hint(key, need, signs) {
  const up = need * signs[key] > 0;
  return {
    lift: up ? "手を上げる" : "手を下げる",
    reach: up ? "手をカメラに近づける" : "手をカメラから遠ざける",
    pan: up ? "画面の右へ" : "画面の左へ",
    flex: up ? "手首を奥へ倒す" : "手首をまっすぐに戻す",
    roll: up ? "時計回りに傾ける" : "反時計回りに傾ける",
  }[key];
}

function build(box) {
  for (const [joint, key, name] of ROWS) {
    const row = document.createElement("div"), label = document.createElement("span"), track = document.createElement("div");
    const zone = document.createElement("i"), robot = document.createElement("b"), hand = document.createElement("u");
    const text = document.createElement("output");
    row.className = "sync-row"; label.textContent = name; track.className = "sync-track";
    zone.className = "sync-zone"; robot.className = "sync-robot"; hand.className = "sync-hand";
    robot.title = "腕の今の姿勢"; hand.title = "手の入力";
    track.append(zone, robot, hand); row.append(label, track, text); box.append(row);
    rowOf[joint] = {row, key, zone, robot, hand, text};
  }
}

export function renderSync(data, signs) {
  const card = document.getElementById("syncCard"), box = document.getElementById("syncRows");
  const sync = data.sync;
  card.hidden = !sync;
  if (!sync) return;
  if (!box.firstChild) build(box);
  const off = [];
  for (const [joint, {row, key, zone, robot, hand, text}] of Object.entries(rowOf)) {
    const gap = sync.gaps[joint];
    row.hidden = !gap;
    if (!gap) continue;
    const diff = gap.hand - gap.robot, bad = Math.abs(diff) > sync.tolerance;
    if (bad) off.push(joint);
    robot.style.left = pos(gap.robot); hand.style.left = pos(gap.hand);
    const from = Math.max(-1, gap.robot - sync.tolerance), to = Math.min(1, gap.robot + sync.tolerance);
    zone.style.left = pos(from); zone.style.width = (to - from) * 50 + "%";    // 許容の範囲（腕の姿勢の前後）
    row.classList.toggle("ng", bad);
    text.textContent = bad ? hint(key, -diff, signs) : "合っている";
  }
  document.getElementById("syncState").textContent = sync.waiting
    ? (off.length ? "同期待ち: 赤い行を合わせると、腕が動き出します（それまで腕は動きません）"
                  : "同期待ち: そのまま保つと、腕が動き出します")
    : "同期済み（追従中）。ずれは、腕が手に追いつくまでの遅れです";
}
