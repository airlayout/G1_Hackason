// ステージエディタのエントリポイント。DOMイベントを状態更新とcanvas再描画に繋ぐ。

import { loadStages } from "./stagesLoader.js";
import { createInitialState, loadStage } from "./editorState.js";
import { validateStage } from "./editorValidate.js";
import { renderEditor } from "./editorRender.js";
import { exportStageJson } from "./editorExport.js";
import { createPlaySession, stepPlaySession, renderPlaySession } from "./editorPlay.js";

const canvas = document.getElementById("editor-canvas");
const ctx = canvas.getContext("2d");
const modeButtons = document.querySelectorAll(".mode-button");
const stageSelect = document.getElementById("stage-select");
const fieldName = document.getElementById("field-name");
const fieldId = document.getElementById("field-id");
const fieldWidthM = document.getElementById("field-width-m");
const fieldHeightM = document.getElementById("field-height-m");
const fieldMoveSpeed = document.getElementById("field-move-speed");
const fieldVisionAngle = document.getElementById("field-vision-angle");
const fieldVisionDistance = document.getElementById("field-vision-distance");
const validationEl = document.getElementById("validation-messages");
const exportArea = document.getElementById("export-output");
const exportButton = document.getElementById("btn-export");
const undoWaypointButton = document.getElementById("btn-undo-waypoint");
const clearWaypointsButton = document.getElementById("btn-clear-waypoints");
const newStageButton = document.getElementById("btn-new-stage");
const editorPanel = document.getElementById("editor-panel");
const testPlayButton = document.getElementById("btn-test-play");
const saveButton = document.getElementById("btn-save");
const playOverlay = document.getElementById("play-overlay");
const playOverlayTitle = document.getElementById("play-overlay-title");
const stopPlayButton = document.getElementById("btn-stop-play");

// スケール: 1m = 80px。ステージのエリアサイズは5m〜25mの範囲で調整できる。
const SCALE_PX_PER_M = 80;
const MIN_AREA_M = 5;
const MAX_AREA_M = 25;

let stage = createInitialState();
let mode = "start";
let wallDraft = null;
let wallDragOrigin = null;
let baseStages = [];

function resizeCanvasToStage() {
  canvas.width = stage.width;
  canvas.height = stage.height;
}

function setMode(nextMode) {
  mode = nextMode;
  modeButtons.forEach((btn) => btn.classList.toggle("active", btn.dataset.mode === mode));
}

modeButtons.forEach((btn) => {
  btn.addEventListener("click", () => setMode(btn.dataset.mode));
});

function populateStageSelect(stages) {
  for (const s of stages) {
    const option = document.createElement("option");
    option.value = String(s.id);
    option.textContent = s.name;
    stageSelect.appendChild(option);
  }
}

stageSelect.addEventListener("change", () => {
  const value = stageSelect.value;
  if (value === "new") {
    stage = createInitialState();
  } else {
    const found = baseStages.find((s) => String(s.id) === value);
    if (found) stage = loadStage(found);
  }
  syncFieldsFromStage();
  draw();
});

newStageButton.addEventListener("click", () => {
  stage = createInitialState();
  stageSelect.value = "new";
  syncFieldsFromStage();
  draw();
});

function syncFieldsFromStage() {
  fieldName.value = stage.name;
  fieldId.value = stage.id;
  fieldWidthM.value = stage.width / SCALE_PX_PER_M;
  fieldHeightM.value = stage.height / SCALE_PX_PER_M;
  fieldMoveSpeed.value = stage.guard.moveSpeed / SCALE_PX_PER_M;
  fieldVisionAngle.value = stage.guard.visionAngle;
  fieldVisionDistance.value = stage.guard.visionDistance / SCALE_PX_PER_M;
  resizeCanvasToStage();
}

function bindNumberField(el, apply) {
  el.addEventListener("input", () => {
    const value = Number(el.value);
    if (!Number.isFinite(value)) return;
    apply(value);
    draw();
  });
}

fieldName.addEventListener("input", () => {
  stage = { ...stage, name: fieldName.value };
});
bindNumberField(fieldId, (value) => {
  stage = { ...stage, id: value };
});
function bindAreaSizeField(el, apply) {
  el.addEventListener("input", () => {
    const meters = Number(el.value);
    if (!Number.isFinite(meters)) return;
    const clamped = Math.min(MAX_AREA_M, Math.max(MIN_AREA_M, meters));
    if (clamped !== meters) el.value = clamped;
    stage = apply(stage, Math.round(clamped * SCALE_PX_PER_M));
    resizeCanvasToStage();
    draw();
  });
}
bindAreaSizeField(fieldWidthM, (s, px) => ({ ...s, width: px }));
bindAreaSizeField(fieldHeightM, (s, px) => ({ ...s, height: px }));
bindNumberField(fieldMoveSpeed, (metersPerSec) => {
  stage = { ...stage, guard: { ...stage.guard, moveSpeed: Math.round(metersPerSec * SCALE_PX_PER_M) } };
});
bindNumberField(fieldVisionAngle, (value) => {
  stage = { ...stage, guard: { ...stage.guard, visionAngle: value } };
});
bindNumberField(fieldVisionDistance, (meters) => {
  stage = { ...stage, guard: { ...stage.guard, visionDistance: Math.round(meters * SCALE_PX_PER_M) } };
});

function getCanvasPos(event) {
  const rect = canvas.getBoundingClientRect();
  return {
    x: Math.round(((event.clientX - rect.left) / rect.width) * stage.width),
    y: Math.round(((event.clientY - rect.top) / rect.height) * stage.height),
  };
}

function isInsideWall(pos, wall) {
  return pos.x >= wall.x && pos.x <= wall.x + wall.w && pos.y >= wall.y && pos.y <= wall.y + wall.h;
}

canvas.addEventListener("mousedown", (event) => {
  if (isPlaying() || mode !== "wall") return;
  wallDragOrigin = getCanvasPos(event);
  wallDraft = { ...wallDragOrigin, w: 0, h: 0 };
});

canvas.addEventListener("mousemove", (event) => {
  if (isPlaying() || mode !== "wall" || !wallDragOrigin) return;
  const pos = getCanvasPos(event);
  wallDraft = {
    x: Math.min(wallDragOrigin.x, pos.x),
    y: Math.min(wallDragOrigin.y, pos.y),
    w: Math.abs(pos.x - wallDragOrigin.x),
    h: Math.abs(pos.y - wallDragOrigin.y),
  };
  draw();
});

canvas.addEventListener("mouseup", () => {
  if (isPlaying() || mode !== "wall" || !wallDraft) return;
  if (wallDraft.w > 4 && wallDraft.h > 4) {
    stage = { ...stage, walls: [...stage.walls, wallDraft] };
  }
  wallDraft = null;
  wallDragOrigin = null;
  draw();
});

canvas.addEventListener("click", (event) => {
  if (isPlaying()) return;
  const pos = getCanvasPos(event);

  if (mode === "start") {
    stage = { ...stage, start: pos };
  } else if (mode === "target") {
    stage = { ...stage, target: pos };
  } else if (mode === "guardStart") {
    stage = { ...stage, guard: { ...stage.guard, x: pos.x, y: pos.y } };
  } else if (mode === "waypoint") {
    stage = { ...stage, guard: { ...stage.guard, patrolPath: [...stage.guard.patrolPath, pos] } };
  } else if (mode === "deleteWall") {
    stage = { ...stage, walls: stage.walls.filter((wall) => !isInsideWall(pos, wall)) };
  }
  draw();
});

undoWaypointButton.addEventListener("click", () => {
  stage = { ...stage, guard: { ...stage.guard, patrolPath: stage.guard.patrolPath.slice(0, -1) } };
  draw();
});

clearWaypointsButton.addEventListener("click", () => {
  stage = { ...stage, guard: { ...stage.guard, patrolPath: [] } };
  draw();
});

exportButton.addEventListener("click", () => {
  exportArea.value = exportStageJson(stage);
  exportArea.select();
});

// --- 保存（JSONファイルのダウンロード） ---

saveButton.addEventListener("click", async () => {
  const validation = validateStage(stage);
  if (validation.blockedLegs.length > 0 || validation.stuckPoints.length > 0) {
    const proceed = window.confirm(
      "壁との衝突警告が出ています。このまま保存すると、警備員が壁にスタックする可能性があります。保存しますか？"
    );
    if (!proceed) return;
  }

  const fileName = `stage-${stage.id}.json`;
  const isNewStage = !baseStages.some((s) => s.id === stage.id);

  try {
    await saveStageViaApi(fileName, stage);
    baseStages = mergeStageIntoList(baseStages, stage);
    refreshStageSelectOptions();
    window.alert(`${fileName} を stages/ に保存しました。次回の読み込みから反映されます。`);
  } catch (error) {
    console.error("サーバー経由の保存に失敗しました:", error);
    downloadTextFile(fileName, exportStageJson(stage));
    baseStages = mergeStageIntoList(baseStages, stage);
    refreshStageSelectOptions();
    const manifestHint = isNewStage
      ? `\nstages/ フォルダに置いた上で、stages/manifest.json に "${fileName}" を追記してください（新規ステージのため）。`
      : `\nstages/ フォルダの同名ファイルを置き換えてください。`;
    window.alert(
      `サーバー経由の保存に失敗したため、${fileName} をダウンロードしました。` +
        manifestHint +
        `\n（devserver.py で起動している場合は自動保存されます: python3 devserver.py）`
    );
  }
});

async function saveStageViaApi(fileName, stageData) {
  const response = await fetch("/api/save-stage", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ fileName, stage: stageData }),
  });
  if (!response.ok) {
    const body = await response.json().catch(() => ({}));
    throw new Error(body.error || `保存APIがエラーを返しました (status: ${response.status})`);
  }
}

function mergeStageIntoList(list, edited) {
  const index = list.findIndex((s) => s.id === edited.id);
  if (index === -1) return [...list, edited];
  const next = [...list];
  next[index] = edited;
  return next;
}

function downloadTextFile(fileName, content) {
  const blob = new Blob([content], { type: "application/json" });
  const url = URL.createObjectURL(blob);
  const link = document.createElement("a");
  link.href = url;
  link.download = fileName;
  document.body.appendChild(link);
  link.click();
  document.body.removeChild(link);
  URL.revokeObjectURL(url);
}

function refreshStageSelectOptions() {
  const currentValue = stageSelect.value;
  stageSelect.innerHTML = '<option value="new">New</option>';
  populateStageSelect(baseStages);
  const stillExists = Array.from(stageSelect.options).some((opt) => opt.value === currentValue);
  stageSelect.value = stillExists ? currentValue : "new";
}

// --- テストプレイ ---

let playSession = null;
let playAnimationHandle = null;
let playLastTimestamp = 0;
const playInput = { up: false, down: false, left: false, right: false };
const PLAY_KEY_MAP = { ArrowUp: "up", ArrowDown: "down", ArrowLeft: "left", ArrowRight: "right" };

function isPlaying() {
  return playSession !== null;
}

window.addEventListener("keydown", (event) => {
  if (!isPlaying()) return;
  const dir = PLAY_KEY_MAP[event.key];
  if (dir) {
    playInput[dir] = true;
    event.preventDefault();
  }
});

window.addEventListener("keyup", (event) => {
  if (!isPlaying()) return;
  const dir = PLAY_KEY_MAP[event.key];
  if (dir) {
    playInput[dir] = false;
    event.preventDefault();
  }
});

testPlayButton.addEventListener("click", () => {
  startTestPlay();
});

stopPlayButton.addEventListener("click", () => {
  stopTestPlay();
});

function startTestPlay() {
  editorPanel.classList.add("hidden");
  playOverlay.classList.remove("hidden");
  playOverlayTitle.textContent = "";
  playInput.up = playInput.down = playInput.left = playInput.right = false;

  playSession = createPlaySession(loadStage(stage));
  playSession.input = playInput;
  playLastTimestamp = 0;
  playAnimationHandle = requestAnimationFrame(playLoop);
}

function stopTestPlay() {
  if (playAnimationHandle) cancelAnimationFrame(playAnimationHandle);
  playAnimationHandle = null;
  playSession = null;
  editorPanel.classList.remove("hidden");
  playOverlay.classList.add("hidden");
  draw();
}

function playLoop(timestamp) {
  if (!playSession) return;
  const dt = playLastTimestamp ? Math.min(0.05, (timestamp - playLastTimestamp) / 1000) : 0;
  playLastTimestamp = timestamp;

  stepPlaySession(playSession, dt);
  renderPlaySession(ctx, playSession);

  if (playSession.result === "CLEAR") {
    playOverlayTitle.textContent = "CLEAR!";
    return;
  }
  if (playSession.result === "GAME_OVER") {
    playOverlayTitle.textContent = "GAME OVER";
    return;
  }
  playAnimationHandle = requestAnimationFrame(playLoop);
}

// --- 描画・検証表示 ---

function draw() {
  const validation = validateStage(stage);
  renderEditor(ctx, stage, validation, wallDraft);
  renderValidationMessages(validation);
}

function renderValidationMessages(validation) {
  const messages = [];
  if (validation.stuckPoints.length > 0) {
    messages.push(`壁と重なっています: ${validation.stuckPoints.join(", ")}`);
  }
  if (validation.blockedLegs.length > 0) {
    messages.push(`巡回ルートが壁を通っています: leg ${validation.blockedLegs.join(", ")}`);
  }
  if (messages.length === 0) {
    validationEl.textContent = "OK: 壁・巡回ルートに問題はありません";
    validationEl.className = "validation-ok";
  } else {
    validationEl.textContent = messages.join(" / ");
    validationEl.className = "validation-error";
  }
}

async function init() {
  setMode("start");
  try {
    baseStages = await loadStages();
    populateStageSelect(baseStages);
  } catch (error) {
    console.error("ステージデータの読み込みに失敗しました:", error);
    validationEl.textContent = "既存ステージの読み込みに失敗しました（新規ステージの作成のみ可能です）";
    validationEl.className = "validation-error";
  }
  syncFieldsFromStage();
  draw();
}

init();
