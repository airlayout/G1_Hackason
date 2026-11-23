// /sim の操作・状態表示。3D 表示は sim_scene.js、物理はサーバー（pickplace_sim.py）。
import { createScene } from "/sim_scene.js";

const POLL_MS = 33, SEND_MS = 50, RETRY_MS = 1000, SLOW = 0.35;
const KEY_RATES = {      // キー → [操作量の名前, 向き]（pan の＋は左）
  KeyW: ["reach", 1], KeyS: ["reach", -1], KeyA: ["pan", 1], KeyD: ["pan", -1],
  ArrowUp: ["height", 1], ArrowDown: ["height", -1], KeyQ: ["roll", 1], KeyE: ["roll", -1],
};
const SLIDERS = [
  ["x", "前後", (v) => `${(v * 100).toFixed(0)} cm`], ["height", "高さ", (v) => `${(v * 100).toFixed(1)} cm`],
  ["pan", "向き", (v) => `${v.toFixed(0)}`], ["roll", "手首", (v) => `${v.toFixed(0)}`], ["opening", "開き", (v) => `${(v * 100).toFixed(0)} %`],
];
const $ = (id) => document.getElementById(id);

async function post(body) {
  const response = await fetch("/sim/command", { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify(body) });
  if (!response.ok) throw new Error((await response.json()).error ?? `HTTP ${response.status}`);
}

function showBanner(text, kind = "") {
  const banner = $("banner");
  banner.hidden = !text;
  banner.textContent = text;
  banner.className = "banner " + kind;
}

function buildSliders(ranges, send) {
  const dragging = new Set(), outputs = {}, inputs = {};
  for (const [key, label, format] of SLIDERS) {
    const [low, high] = ranges[key];
    const row = document.createElement("label");
    row.className = "slider";
    const input = Object.assign(document.createElement("input"), { type: "range", min: low, max: high, step: (high - low) / 400 });
    const output = document.createElement("output");
    row.append(Object.assign(document.createElement("span"), { textContent: label }), input, output);
    $("sliders").append(row);
    input.addEventListener("pointerdown", () => dragging.add(key));
    input.addEventListener("pointerup", () => { dragging.delete(key); input.blur(); });
    input.addEventListener("input", () => send({ op: "set", [key]: Number(input.value) }));
    inputs[key] = input;
    outputs[key] = (v) => { output.textContent = format(v); };
  }
  return (goal) => {
    for (const [key] of SLIDERS) {
      if (dragging.has(key)) continue;
      inputs[key].value = goal[key];
      outputs[key](goal[key]);
    }
  };
}

function renderTask(state) {
  const { task } = state;
  $("step-lift").classList.toggle("done", task.lifted);
  $("step-got").classList.toggle("done", task.got);
  $("prize").hidden = !task.got;
  $("timer").textContent = `${task.elapsed.toFixed(1)} 秒`;
  const result = $("result");
  if (task.got) { result.textContent = `景品ゲット！ ${task.elapsed.toFixed(1)} 秒`; result.className = "result ok"; }
  else if (task.fallen) { result.textContent = "お菓子が机から落ちました。「リスタート」で最初からやり直せます"; result.className = "result bad"; }
  else { result.textContent = ""; result.className = "result"; }
  $("start").disabled = state.running;
  $("stop").disabled = !state.running;
  if (!state.running) showBanner(task.started ? "ストップ中です。スタートで続きから動かせます" : "「スタート」を押してください", "warn");
  else if (state.clamped) showBanner("ここから先は腕が届きません", "warn");
  else showBanner("");
}

async function main() {
  const model = await (await fetch("/sim/model")).json();
  const fetchMesh = async (index) => (await fetch(`/sim/mesh/${index}`)).arrayBuffer();
  const scene = await createScene($("view"), model, fetchMesh);
  let latest = null;

  // 送信は一定間隔にまとめる（スライダーを速く動かしても、サーバーへは 20 Hz まで）
  let pending = null, sending = false;
  const send = (body) => { pending = body.op === "set" && pending?.op === "set" ? { ...pending, ...body } : body; };
  setInterval(async () => {
    if (!pending || sending) return;
    const body = pending; pending = null; sending = true;
    try { await post(body); } catch (error) { showBanner(`操作できません: ${error.message}`, "bad"); } finally { sending = false; }
  }, SEND_MS);

  const syncSliders = buildSliders(model.ranges, send);

  const held = new Set();
  let wasMoving = false;
  setInterval(() => {
    const rates = {};
    for (const code of held) if (KEY_RATES[code]) { const [name, sign] = KEY_RATES[code]; rates[name] = (rates[name] ?? 0) + sign; }
    const scale = held.has("ShiftLeft") || held.has("ShiftRight") ? SLOW : 1;
    for (const name in rates) rates[name] = Math.max(-1, Math.min(1, rates[name])) * scale;
    const moving = Object.values(rates).some((v) => v !== 0);
    if (moving || wasMoving) send({ op: "move", ...rates });
    wasMoving = moving;
  }, SEND_MS);

  // 進行（スタート・ストップ・リスタート）。set とは別に、すぐ送る（まとめ送りで落とさない）
  const control = async (op) => {
    pending = null;
    try { await post({ op }); } catch (error) { showBanner(`操作できません: ${error.message}`, "bad"); }
  };
  const restart = () => control("restart");
  addEventListener("keydown", (e) => {
    if (e.metaKey || e.ctrlKey || e.altKey) return;
    if (e.code === "Space") { e.preventDefault(); if (!e.repeat && latest) send({ op: "set", opening: latest.goal.opening > 0.5 ? 0 : 1 }); }
    else if (e.code === "KeyR") { if (!e.repeat) restart(); }
    else if (KEY_RATES[e.code] || e.code.startsWith("Shift")) { if (e.code.startsWith("Arrow")) e.preventDefault(); held.add(e.code); }
  });
  addEventListener("keyup", (e) => held.delete(e.code));
  addEventListener("blur", () => held.clear());
  // ボタンに焦点が残ると、Space（開閉キー）でもう一度押されるので、押したら外す
  for (const [id, run] of [["start", () => control("start")], ["stop", () => control("stop")], ["restart", restart]]) {
    $(id).addEventListener("click", (e) => { run(); e.currentTarget.blur(); });
  }

  // アシスト（近くでグリッパーを閉じると自動でつかみ、穴の上で止まると自動で離す）。set とは別に、すぐ送る（まとめ送りで落とさない）
  const assist = $("assist");
  let assistBusy = false;
  assist.addEventListener("change", async () => {
    assistBusy = true;
    try { await post({ op: "assist", on: assist.checked }); window.dispatchEvent(new CustomEvent("sim-assist", { detail: { on: assist.checked } })); }
    catch (error) { showBanner(`操作できません: ${error.message}`, "bad"); }
    finally { assistBusy = false; assist.blur(); }
  });
  document.querySelectorAll("[data-view]").forEach((button) => button.addEventListener("click", () => {
    scene.setView(button.dataset.view);
    document.querySelectorAll("[data-view]").forEach((b) => b.classList.toggle("on", b === button));
  }));

  // /sim/camera（sim_camera.js）が、同じ操作の入口と状態を使えるよう渡す
  window.simControl = { send, state: () => latest, ranges: model.ranges };
  window.dispatchEvent(new Event("sim-ready"));

  (function draw() { if (latest) scene.update(latest); scene.render(); requestAnimationFrame(draw); })();
  for (;;) {
    try {
      const response = await fetch("/sim/state");
      if (!response.ok) throw new Error((await response.json()).error ?? `HTTP ${response.status}`);
      latest = await response.json();
      syncSliders(latest.goal);
      if (!assistBusy) assist.checked = latest.assist;
      renderTask(latest);
      await new Promise((resolve) => setTimeout(resolve, POLL_MS));
    } catch (error) {
      showBanner(`サーバーと繋がりません（${error.message}）`, "bad");
      await new Promise((resolve) => setTimeout(resolve, RETRY_MS));
    }
  }
}

main().catch((error) => { showBanner(`起動できません: ${error.message}`, "bad"); console.error(error); });
