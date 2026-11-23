// 詳細設定: モーター別の制限。値はすべて「可動範囲に対する%」。保存・読み込みはサーバー（limits.json）が行う。
// スライダーの初期値は、保存が無ければ既定値（サーバーが state.limit_defaults で返す）。
const FIELDS = [
  {key: "speed", label: "追従の最大速度", unit: "%/秒", step: 1},
  {key: "step", label: "1回の最大移動量", unit: "%", step: .25},
  {key: "min", label: "動かせる下限", unit: "%", step: 1},
  {key: "max", label: "動かせる上限", unit: "%", step: 1},
];
const COMMAND_INTERVAL_S = .065;   // 追従の指令間隔（app.js の送信間隔と同じ）。1回の最大移動量 ÷ これ が速度の天井になる
const OPEN_KEY = "so101.limitsOpen";
const $ = id => document.getElementById(id);
let joints, dirty = false, built = false;

const slider = (j, key) => $("lim-" + j + "-" + key);
const fmt = (v, key) => (key === "step" ? v.toFixed(2) : String(Math.round(v)));

function build(state) {
  const body = $("limitsList");
  for (const [j, name] of Object.entries(joints)) {
    const box = document.createElement("fieldset"), legend = document.createElement("legend");
    legend.textContent = name; box.append(legend);
    for (const f of FIELDS) {
      const [low, high] = state.limit_ranges[f.key];
      const row = document.createElement("div"), text = document.createElement("span"),
        input = document.createElement("input"), out = document.createElement("output"), def = document.createElement("small");
      row.className = "limit-row"; text.textContent = f.label;
      input.type = "range"; input.min = low; input.max = high; input.step = f.step; input.id = "lim-" + j + "-" + f.key;
      input.setAttribute("aria-label", name + " " + f.label); out.id = "limv-" + j + "-" + f.key;
      def.textContent = "既定 " + fmt(state.limit_defaults[j][f.key], f.key);
      input.addEventListener("input", () => onInput(j, f.key));
      row.append(text, input, out, def); box.append(row);
    }
    const note = document.createElement("p"); note.className = "help"; note.id = "limeff-" + j; box.append(note);
    body.append(box);
  }
  built = true;
}

function refresh(j) {
  for (const f of FIELDS) $("limv-" + j + "-" + f.key).textContent = fmt(Number(slider(j, f.key).value), f.key) + " " + f.unit;
  const speed = Number(slider(j, "speed").value), cap = Number(slider(j, "step").value) / COMMAND_INTERVAL_S;
  $("limeff-" + j).textContent = "追従の実際の上限: 毎秒 " + Math.round(Math.min(speed, cap)) + "%" +
    (cap < speed ? "（1回の最大移動量で頭打ち。速くするには移動量も上げる）" : "");
}

function onInput(j, key) {
  const lo = slider(j, "min"), hi = slider(j, "max"), span = state0.limit_min_span;
  if (key === "min" && Number(lo.value) > Number(hi.value) - span) lo.value = Number(hi.value) - span;   // 幅を確保する
  if (key === "max" && Number(hi.value) < Number(lo.value) + span) hi.value = Number(lo.value) + span;
  dirty = true; refresh(j); status();
}

let state0;
function status() {
  $("limitsState").textContent = dirty ? "未保存の変更があります。「保存して適用」を押すと、実機に反映され、次回起動でも使われます"
    : "適用中（保存済み）。変更したらスライダーを動かして、保存して適用";
}

function read() {
  return Object.fromEntries(Object.keys(joints).map(j =>
    [j, Object.fromEntries(FIELDS.map(f => [f.key, Number(slider(j, f.key).value)]))]));
}

function show(limits) {
  for (const j of Object.keys(joints)) { for (const f of FIELDS) slider(j, f.key).value = limits[j][f.key]; refresh(j); }
  dirty = false; status();
}

export function renderLimits(state) {
  if (!state.limits) return;
  state0 = state;
  if (!built) { build(state); show(state.limits); }
  else if (!dirty) show(state.limits);          // 操作中・未保存の値は上書きしない
  const blocked = state.active || state.fault || !state.calibration.done;   // 追従中はサーバーも変更を拒否する
  for (const el of document.querySelectorAll("#limitsBody input, #limitsBody button")) el.disabled = blocked;
}

function setOpen(open) {
  $("limitsBody").hidden = !open;
  $("limitsToggle").textContent = open ? "詳細設定を隠す" : "詳細設定を表示";
  $("limitsToggle").setAttribute("aria-expanded", String(open));
  try { localStorage.setItem(OPEN_KEY, open ? "1" : "0"); } catch { /* 保存できなくても動く */ }
}

export function initLimits({joints: names, command, info, render}) {
  joints = names;
  let open = false;
  try { open = localStorage.getItem(OPEN_KEY) === "1"; } catch { /* 既定は非表示 */ }
  setOpen(open);
  $("limitsToggle").onclick = () => setOpen($("limitsBody").hidden);
  const run = action => async () => { try { info(""); render(await action()); dirty = false; show(state0.limits); } catch (e) { info(e.message); } };
  $("limitsSave").onclick = run(() => command("limits", {limits: read()}));
  $("limitsReset").onclick = run(() => command("limits", {reset: true}));
}
