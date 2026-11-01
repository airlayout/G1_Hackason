"use strict";
const $ = (id) => document.getElementById(id);
const phaseLabels = {
  idle: "待機",
  loading: "モデル読込中",
  running: "実行中",
  stopping: "停止中",
  error: "停止",
};
const jointNames = [
  "肩 pitch",
  "肩 roll",
  "肩 yaw",
  "肘",
  "手首 roll",
  "手首 pitch",
  "手首 yaw",
];
for (let i = 0; i < 7; i++) {
  const row = document.createElement("div");
  row.className = "joint-row";
  for (const [label, id] of [
    [jointNames[i], null],
    ["—", `left-${i}`],
    ["—", `right-${i}`],
  ]) {
    const cell = document.createElement("span");
    cell.textContent = label;
    if (id) {
      cell.id = id;
      cell.className = "joint-value";
    }
    row.append(cell);
  }
  $("joints").append(row);
}
let sending = false;
let latestState = null;
let cameraLoading = false;
let cameraUrl = null;
async function api(path, body) {
  const response = await fetch(path, {
    method: body === undefined ? "GET" : "POST",
    headers: body === undefined ? {} : { "Content-Type": "application/json" },
    body: body === undefined ? undefined : JSON.stringify(body),
    signal: AbortSignal.timeout(6000),
  });
  const result = await response.json();
  if (!response.ok) throw new Error(result.error || `HTTP ${response.status}`);
  return result;
}
function notice(message) {
  $("notice").textContent = message || "";
  $("notice").hidden = !message;
}
function render(state) {
  latestState = state;
  $("model-name").textContent = state.model_label;
  $("model-name").title = state.model;
  $("connection").classList.toggle("online", state.connected);
  $("connection").lastChild.textContent = state.connected ? "接続" : "未接続";
  $("mode").textContent = state.demo
    ? "DEMO"
    : state.robot?.execute
      ? "実送信"
      : "送信なし";
  $("demo-label").hidden = !state.demo;
  $("phase").textContent = phaseLabels[state.phase] || state.phase;
  $("latency").replaceChildren(
    document.createTextNode(state.inference_ms ?? "—"),
    unit("ms"),
  );
  $("weight").replaceChildren(
    document.createTextNode(Math.round((state.robot?.weight || 0) * 100)),
    unit("%"),
  );
  $("count").textContent = `${state.steps} 推論`;
  $("time-limit").textContent = state.max_task_s;
  $("camera-age").textContent =
    state.connected && state.robot?.camera_age_s != null
      ? `${Math.round(state.robot.camera_age_s * 1000)} ms`
      : "—";
  $("camera-state").classList.toggle("online", state.connected);
  $("run").disabled =
    !state.connected ||
    state.busy ||
    sending ||
    state.robot?.phase === "stopping";
  $("instruction").disabled = state.busy;
  const q = state.robot?.q;
  for (let i = 0; i < 7; i++) {
    $(`left-${i}`).textContent = q ? q[15 + i].toFixed(2) : "—";
    $(`right-${i}`).textContent = q ? q[22 + i].toFixed(2) : "—";
  }
  const events = state.events || [];
  if (events.length) {
    const nodes = events
      .slice()
      .reverse()
      .map((event) => {
        const item = document.createElement("li");
        const time = document.createElement("time");
        time.textContent = event.time;
        const text = document.createElement("span");
        text.textContent = event.text;
        item.append(time, text);
        return item;
      });
    $("events").replaceChildren(...nodes);
  }
  notice(
    state.error ||
      state.robot_error ||
      (state.demo ? "通信確認用。実機・VLAは使用しません。" : ""),
  );
}
function unit(text) {
  const element = document.createElement("small");
  element.textContent = text;
  return element;
}
$("command-form").addEventListener("submit", async (event) => {
  event.preventDefault();
  if (sending || $("run").disabled) return;
  sending = true;
  $("run").disabled = true;
  try {
    render(await api("/api/task", { instruction: $("instruction").value }));
  } catch (error) {
    notice(error.message);
  } finally {
    sending = false;
  }
});
async function stop() {
  try {
    render(await api("/api/stop", {}));
  } catch (error) {
    notice(`停止を確認できません: ${error.message}`);
  }
}
$("stop").addEventListener("click", stop);
document.addEventListener("keydown", (event) => {
  if (event.key === "Escape") {
    event.preventDefault();
    stop();
  }
  if (event.key === "Enter" && (event.ctrlKey || event.metaKey)) {
    event.preventDefault();
    $("command-form").requestSubmit();
  }
});
for (const button of document.querySelectorAll("[data-prompt]")) {
  button.addEventListener("click", () => {
    if (latestState?.busy) return;
    $("instruction").value = button.dataset.prompt;
    $("instruction").focus();
  });
}
async function poll() {
  try {
    render(await api("/api/status"));
  } catch (error) {
    $("connection").classList.remove("online");
    $("connection").lastChild.textContent = "未接続";
    $("run").disabled = true;
    $("camera").hidden = true;
    $("camera-placeholder").hidden = false;
    notice(error.message);
  }
  setTimeout(poll, 300);
}
async function camera() {
  if (!cameraLoading && latestState?.connected && !document.hidden) {
    cameraLoading = true;
    try {
      const response = await fetch("/api/camera.jpg", {
        signal: AbortSignal.timeout(3000),
      });
      if (!response.ok) throw new Error("camera");
      const next = URL.createObjectURL(await response.blob());
      $("camera").src = next;
      $("camera").hidden = false;
      $("camera-placeholder").hidden = true;
      if (cameraUrl) URL.revokeObjectURL(cameraUrl);
      cameraUrl = next;
    } catch (_) {
      $("camera").hidden = true;
      $("camera-placeholder").hidden = false;
    } finally {
      cameraLoading = false;
    }
  } else if (!latestState?.connected) {
    $("camera").hidden = true;
    $("camera-placeholder").hidden = false;
  }
  setTimeout(camera, 125);
}
poll();
camera();
