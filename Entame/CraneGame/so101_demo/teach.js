// 教示ページ: 人の姿勢 H を選び、ロボットを手で動かして、その関節値を保存する（サーバーの teach_save）。
import {armFigure} from "/skeleton_figure.js";

const $ = id => document.getElementById(id);
const JOINT_NAMES = {shoulder_pan: "台座 pan", shoulder_lift: "肩 lift", elbow_flex: "肘 elbow", wrist_flex: "手首の曲げ", wrist_roll: "手首の回転", gripper: "グリッパー"};
const DRIVE_NAMES = {on: "ON", soft: "やわらか", off: "OFF"};
const POLL_MS = 400;
let state = null, current = null;
const skipped = new Set();      // 今回の画面で「やらない」にした H（保存はしない。画面を開き直すと戻る）

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
const poses = () => state?.teach.poses ?? [];
const saved = () => state?.teach.saved ?? {};
const fmt = v => (Math.round(v * 10) / 10).toFixed(1);
const anglesText = p => `横向き pan ${p.pan === null ? "不問" : p.pan + "°"}　上げ lift ${p.lift}°　肘 elbow ${p.elbow}°`;
const figureOf = p => armFigure({pan: p.pan ?? 0, lift: p.lift, elbow: p.elbow});
const valuesText = pose => ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"].map(j => `${JOINT_NAMES[j]} ${fmt(pose[j])}`).join("　");

function nextUnsaved(after) {
  const list = poses(), start = after ? list.findIndex(p => p.id === after) + 1 : 0;
  const order = [...list.slice(start), ...list.slice(0, start)];
  return (order.find(p => !saved()[p.id] && !skipped.has(p.id)) ?? list.find(p => p.id === after) ?? list[0])?.id;
}

function renderNow() {
  const p = poses().find(x => x.id === current);
  if (!p) return;
  $("nowTitle").textContent = `${p.id}　${p.name}`;
  $("nowFigure").replaceChildren(figureOf(p));
  $("nowAngles").textContent = anglesText(p);
  const skips = [...skipped].filter(id => !saved()[id]).length;
  $("progress").textContent = `保存済み ${Object.keys(saved()).length} / ${poses().length}` + (skips ? `（やらない ${skips}）` : "");
}
function renderCards() {
  $("cards").replaceChildren(...poses().map(p => {
    const entry = saved()[p.id], card = document.createElement("div");
    card.className = "card" + (p.id === current ? " current" : "");
    card.innerHTML = `<h3>${p.id}　${p.name}</h3><p class="vals">${anglesText(p)}</p>`;
    card.append(figureOf(p));
    const status = document.createElement("p");
    status.className = entry ? "done" : "todo";
    status.textContent = entry ? `保存済み ${entry.time}` : skipped.has(p.id) ? "やらない（スキップ中）" : "未保存";
    card.append(status);
    if (entry) {
      const vals = document.createElement("p");
      vals.className = "vals"; vals.textContent = valuesText(entry.robot); card.append(vals);
    }
    const row = document.createElement("div"); row.className = "row";
    const pick = document.createElement("button"); pick.textContent = entry ? "選び直して上書き" : skipped.has(p.id) ? "やっぱり教える" : "この姿勢を教える";
    bind(pick, async () => { skipped.delete(p.id); current = p.id; render(); window.scrollTo({top: 0, behavior: "smooth"}); });
    row.append(pick);
    if (entry) {
      const del = document.createElement("button"); del.textContent = "保存を消す";
      bind(del, async () => { state = await command("teach_delete", {id: p.id}); render(); });
      row.append(del);
    }
    card.append(row);
    return card;
  }));
}
function renderLive() {
  const pose = state.pose;
  $("live").replaceChildren(...Object.entries(JOINT_NAMES).flatMap(([j, name]) => {
    const a = document.createElement("span"), b = document.createElement("span");
    a.textContent = name; b.textContent = pose[j] === undefined ? "—" : fmt(pose[j]);
    return [a, b];
  }));
}
function renderControl() {
  $("mode").textContent = state.live ? "実機" : "模擬";
  $("driveState").textContent = "いまのモーター: " + (DRIVE_NAMES[state.drive] ?? state.drive);
  $("status").textContent = state.note || "待機中";
  for (const b of document.querySelectorAll("[data-drive]")) b.setAttribute("aria-pressed", String(b.dataset.drive === state.drive));
  $("save").disabled = !current;
  $("skip").disabled = !current;
}
let shownKey = "";
function render() {
  if (!state) return;
  if (!current) current = nextUnsaved(null);
  renderControl(); renderLive();
  const key = JSON.stringify([current, state.teach.saved, [...skipped]]);    // 姿勢の図は、選択か保存が変わったときだけ描き直す
  if (key !== shownKey) { shownKey = key; renderNow(); renderCards(); }
}

for (const b of document.querySelectorAll("[data-drive]")) {
  bind(b, async () => { state = await command("drive", {mode: b.dataset.drive}); render(); });
}
bind($("skip"), async () => {
  skipped.add(current);
  current = nextUnsaved(current);
  render();
});
bind($("save"), async () => {
  const id = current;
  state = await command("teach_save", {id});
  skipped.delete(id);
  current = nextUnsaved(id);
  render();
});

async function poll() {
  try {
    const res = await fetch("/state");
    if (res.ok) { state = await res.json(); render(); }
  } catch (e) { message("サーバーに接続できません"); }
  setTimeout(poll, POLL_MS);
}
poll();
