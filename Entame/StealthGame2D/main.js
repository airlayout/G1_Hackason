import { loadStages } from "./stagesLoader.js";
import { createPlayer, updatePlayer, isBackAtStart } from "./player.js";
import { createGuard, updateGuard, isCaught } from "./guard.js";
import { renderStage } from "./render.js";

const STATE = {
  LEVEL_SELECT: "LEVEL_SELECT",
  PLAYING: "PLAYING",
  CLEAR: "CLEAR",
  GAME_OVER: "GAME_OVER",
};

const canvas = document.getElementById("game-canvas");
const ctx = canvas.getContext("2d");
const levelSelectEl = document.getElementById("level-select");
const overlayEl = document.getElementById("overlay");
const overlayTitleEl = document.getElementById("overlay-title");
const overlayHintEl = document.getElementById("overlay-hint");
const playControlsEl = document.getElementById("play-controls");
const playRulesEl = document.getElementById("play-rules");
const retryButtonEl = document.getElementById("btn-retry");
const backButtonEl = document.getElementById("btn-back");

let state = STATE.LEVEL_SELECT;
let currentStage = null;
let player = null;
let guard = null;
let lastTimestamp = 0;

const input = { up: false, down: false, left: false, right: false };

const KEY_MAP = {
  ArrowUp: "up",
  ArrowDown: "down",
  ArrowLeft: "left",
  ArrowRight: "right",
};

window.addEventListener("keydown", (event) => {
  const dir = KEY_MAP[event.key];
  if (dir) {
    input[dir] = true;
    event.preventDefault();
  }
});

window.addEventListener("keyup", (event) => {
  const dir = KEY_MAP[event.key];
  if (dir) {
    input[dir] = false;
    event.preventDefault();
  }
});

function buildLevelSelect(stages) {
  levelSelectEl.innerHTML = "";
  for (const stage of stages) {
    const button = document.createElement("button");
    button.className = "level-button";
    button.textContent = `LEVEL ${stage.id}`;
    button.addEventListener("click", () => startStage(stage));
    levelSelectEl.appendChild(button);
  }
}

function startStage(stage) {
  currentStage = stage;
  canvas.width = stage.width;
  canvas.height = stage.height;
  player = createPlayer(stage.start);
  guard = createGuard(stage.guard);
  input.up = input.down = input.left = input.right = false;
  state = STATE.PLAYING;
  showScreen();
}

function showScreen() {
  levelSelectEl.classList.toggle("hidden", state !== STATE.LEVEL_SELECT);
  canvas.classList.toggle("hidden", state === STATE.LEVEL_SELECT);
  playControlsEl.classList.toggle("hidden", state === STATE.LEVEL_SELECT);
  playRulesEl.classList.toggle("hidden", state === STATE.LEVEL_SELECT);
  overlayEl.classList.toggle("hidden", state !== STATE.CLEAR && state !== STATE.GAME_OVER);

  if (state === STATE.CLEAR) {
    overlayTitleEl.textContent = "CLEAR!";
    overlayHintEl.textContent = "Click to return to LEVEL SELECT";
  } else if (state === STATE.GAME_OVER) {
    overlayTitleEl.textContent = "GAME OVER";
    overlayHintEl.textContent = "Click to return to LEVEL SELECT";
  }
}

overlayEl.addEventListener("click", () => {
  state = STATE.LEVEL_SELECT;
  showScreen();
});

retryButtonEl.addEventListener("click", () => {
  startStage(currentStage);
});

backButtonEl.addEventListener("click", () => {
  state = STATE.LEVEL_SELECT;
  showScreen();
});

function update(dt) {
  updatePlayer(player, input, dt, currentStage);
  updateGuard(guard, dt, currentStage, player);

  if (isCaught(guard)) {
    state = STATE.GAME_OVER;
  } else if (isBackAtStart(player, currentStage)) {
    state = STATE.CLEAR;
  }
}

function loop(timestamp) {
  const dt = lastTimestamp ? Math.min(0.05, (timestamp - lastTimestamp) / 1000) : 0;
  lastTimestamp = timestamp;

  if (state === STATE.PLAYING) {
    update(dt);
    // 検知（赤）確定でGAME_OVERに遷移した瞬間の状態も1フレーム描画してから
    // オーバーレイを出す。ここで描画をスキップすると赤い視野が一切見えないまま終わる。
    renderStage(ctx, currentStage, player, guard, currentStage.name);
    if (state !== STATE.PLAYING) {
      showScreen();
    }
  }

  requestAnimationFrame(loop);
}

async function init() {
  try {
    const stages = await loadStages();
    buildLevelSelect(stages);
  } catch (error) {
    console.error("ステージデータの読み込みに失敗しました:", error);
    levelSelectEl.innerHTML = "<p>ステージデータの読み込みに失敗しました。サーバー経由で開いているか確認してください。</p>";
  }
  showScreen();
  requestAnimationFrame(loop);
}

init();
