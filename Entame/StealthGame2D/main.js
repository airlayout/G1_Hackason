import { loadStages } from "./stagesLoader.js";
import { createPlayer, updatePlayer } from "./player.js";
import { createGuard, updateGuard, isCaught } from "./guard.js";
import { createSecurityCamera, updateSecurityCamera } from "./securityCamera.js";
import { updateMiniRobot, isStageCleared, detectionTargets } from "./miniRobot.js";
import { RULE, createPlayRobot, tryRescue } from "./rescueRule.js";
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
const levelOptionsEl = document.getElementById("level-options");
const miniRobotToggleEl = document.getElementById("play-mini-robot");
const rescueToggleEl = document.getElementById("play-rescue");
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
let securityCameras = [];
let miniRobot = null;
let lastTimestamp = 0;

const input = { up: false, down: false, left: false, right: false, rescue: false };

const KEY_MAP = {
  ArrowUp: "up",
  ArrowDown: "down",
  ArrowLeft: "left",
  ArrowRight: "right",
};

window.addEventListener("keydown", (event) => {
  // Space = 救出の操作（実機では LOVOT のセンサーホーンのモードスイッチ）。1回の押下を1フレームだけ伝える。
  if (event.key === " " && state === STATE.PLAYING) {
    if (!event.repeat) input.rescue = true;
    event.preventDefault();
    return;
  }
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

// 救出するときは LOVOT が TARGET の位置に必ずいるので、「一緒に行く」は選べなくする。
rescueToggleEl.addEventListener("change", () => {
  miniRobotToggleEl.disabled = rescueToggleEl.checked;
});

function buildLevelSelect(stages) {
  levelSelectEl.innerHTML = "";
  for (const stage of stages) {
    const button = document.createElement("button");
    button.className = "level-button";
    // エディタでつけたステージ名を出す（名前が空のときだけ ID で代用）。
    button.textContent = stage.name || `LEVEL ${stage.id}`;
    button.addEventListener("click", () => startStage(stage));
    levelSelectEl.appendChild(button);
  }
}

function startStage(stage) {
  currentStage = stage;
  canvas.width = stage.width;
  canvas.height = stage.height;
  // ルールと、ミニロボットを連れて行くかは、ステージではなくプレイする人がレベル選択で選ぶ。
  const rule = rescueToggleEl.checked ? RULE.RESCUE : RULE.STEAL;
  player = createPlayer(stage.start, rule === RULE.STEAL);
  guard = createGuard(stage.guard);
  securityCameras = stage.securityCameras.map(createSecurityCamera);
  miniRobot = createPlayRobot(stage, rule, miniRobotToggleEl.checked);
  input.up = input.down = input.left = input.right = input.rescue = false;
  state = STATE.PLAYING;
  showScreen();
}

function showScreen() {
  levelSelectEl.classList.toggle("hidden", state !== STATE.LEVEL_SELECT);
  levelOptionsEl.classList.toggle("hidden", state !== STATE.LEVEL_SELECT);
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
  if (miniRobot) tryRescue(miniRobot, player, input.rescue);
  input.rescue = false;
  if (miniRobot) updateMiniRobot(miniRobot, dt, player, currentStage);
  const targets = detectionTargets(player, miniRobot);
  updateGuard(guard, dt, currentStage, targets);
  for (const camera of securityCameras) updateSecurityCamera(camera, dt, currentStage, targets);

  if (isCaught(guard) || securityCameras.some((camera) => camera.caught)) {
    state = STATE.GAME_OVER;
  } else if (isStageCleared(player, miniRobot, currentStage)) {
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
    renderStage(ctx, currentStage, player, guard, currentStage.name, securityCameras, miniRobot);
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
