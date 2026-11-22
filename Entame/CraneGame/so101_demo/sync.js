// 追従の基準と、手と腕の動きの差の表示。追従を始めた直後は、サーバーが基準（手の構えと腕の姿勢）を取るまで「手を止める」案内を出す。
// 基準ができたら、関節ごとに、手が基準から動いた量（hand）と、腕が基準から動いた量（robot）を、同じ尺度（-1〜1）で並べる。
// 差は、腕が手に追いつくまでの遅れ（速度の制限による）。
const ROWS = [
  ["shoulder_pan", "台座"], ["shoulder_lift", "肩"], ["elbow_flex", "肘"],
  ["wrist_flex", "手首の曲げ"], ["wrist_roll", "手首の回転"],
];
const rowOf = {};
const pos = v => (Math.max(-1, Math.min(1, v)) + 1) * 50 + "%";

function build(box) {
  for (const [joint, name] of ROWS) {
    const row = document.createElement("div"), label = document.createElement("span"), track = document.createElement("div");
    const zone = document.createElement("i"), robot = document.createElement("b"), hand = document.createElement("u");
    const text = document.createElement("output");
    row.className = "sync-row"; label.textContent = name; track.className = "sync-track";
    zone.className = "sync-zone"; robot.className = "sync-robot"; hand.className = "sync-hand";
    robot.title = "腕が基準から動いた量"; hand.title = "手が基準から動いた量";
    track.append(zone, robot, hand); row.append(label, track, text); box.append(row);
    rowOf[joint] = {row, zone, robot, hand, text};
  }
}

export function renderSync(data) {
  const card = document.getElementById("syncCard"), box = document.getElementById("syncRows");
  const sync = data.sync;
  card.hidden = !sync;
  if (!sync) return;
  if (!box.firstChild) build(box);
  let late = 0;
  for (const [joint, {row, zone, robot, hand, text}] of Object.entries(rowOf)) {
    const gap = sync.gaps[joint];
    row.hidden = !gap;
    if (!gap) continue;
    const slow = Math.abs(gap.hand - gap.robot) > sync.tolerance;
    if (slow) late++;
    robot.style.left = pos(gap.robot); hand.style.left = pos(gap.hand);
    const from = Math.max(-1, gap.hand - sync.tolerance), to = Math.min(1, gap.hand + sync.tolerance);
    zone.style.left = pos(from); zone.style.width = (to - from) * 50 + "%";    // 手の位置の前後。腕がここに入れば追いついている
    row.classList.toggle("ng", slow);
    text.textContent = slow ? "追いつき中" : "合っている";
  }
  document.getElementById("syncState").textContent = sync.waiting
    ? "基準を取っています: 手を止めてください（腕は動きません）。構えは何でもかまいません"
    : (late ? "追従中。赤い行は、腕が手に追いついている途中です" : "追従中。中央が、始めたときの手と腕の姿勢です");
}
