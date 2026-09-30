import * as THREE from "three";
import { loadStages } from "./stagesLoader";
import { createPlayer, updatePlayer, isBackAtStart } from "./player";
import { createGuard, updateGuard, isCaught, computeVisionPolygon } from "./guard";
import { buildStageScene, updateVisionMesh, toWorld, type StageScene } from "./scene";
import type { InputState, Stage } from "./types";
import type { Player } from "./player";
import type { Guard } from "./guard";
import type { GuardReaction } from "./types";
import {
  PLAYER_EYE_HEIGHT_M,
  KEY_LOOK_SPEED_RAD,
  PLAYER_HORIZONTAL_FOV_DEG,
  PLAYER_VERTICAL_FOV_DEG,
  PLAYER_MAX_PITCH_RAD,
} from "./constants";

// 人間の水平/垂直FOVを画面比率に関わらず再現するための固定アスペクト比。
// aspect = tan(H/2) / tan(V/2)（三角比から算出、画面のアスペクト比とは無関係）。
const HUMAN_FOV_ASPECT =
  Math.tan(THREE.MathUtils.degToRad(PLAYER_HORIZONTAL_FOV_DEG) / 2) /
  Math.tan(THREE.MathUtils.degToRad(PLAYER_VERTICAL_FOV_DEG) / 2);

type GameState = "LEVEL_SELECT" | "PLAYING" | "CLEAR" | "GAME_OVER";

const canvas = document.getElementById("game-canvas") as HTMLCanvasElement;
const levelSelectEl = document.getElementById("level-select") as HTMLDivElement;
const levelButtonsEl = document.getElementById("level-buttons") as HTMLDivElement;
const hudEl = document.getElementById("hud") as HTMLDivElement;
const hudTargetEl = document.getElementById("hud-target") as HTMLDivElement;
const hudTimerEl = document.getElementById("hud-timer") as HTMLDivElement;
const overlayEl = document.getElementById("overlay") as HTMLDivElement;
const overlayTitleEl = document.getElementById("overlay-title") as HTMLHeadingElement;
const overlayHintEl = document.getElementById("overlay-hint") as HTMLParagraphElement;
const retryButtonEl = document.getElementById("btn-retry") as HTMLButtonElement;
const backButtonEl = document.getElementById("btn-back") as HTMLButtonElement;
const overlayBackButtonEl = document.getElementById("btn-overlay-back") as HTMLButtonElement;
const reactionSelectEl = document.getElementById("reaction-select") as HTMLSelectElement;

const renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
renderer.setPixelRatio(window.devicePixelRatio);

let state: GameState = "LEVEL_SELECT";
let currentStage: Stage | null = null;
let stageScene: StageScene | null = null;
let player: Player | null = null;
let guard: Guard | null = null;
let elapsed = 0;
let lastTimestamp = 0;

const input: InputState = {
  up: false,
  down: false,
  left: false,
  right: false,
  lookLeft: false,
  lookRight: false,
  lookUp: false,
  lookDown: false,
};

// 移動: WASD、視点操作: 矢印キー(マウス不要)。
const KEY_MAP: Record<string, keyof InputState> = {
  w: "up",
  s: "down",
  a: "left",
  d: "right",
  ArrowUp: "lookUp",
  ArrowDown: "lookDown",
  ArrowLeft: "lookLeft",
  ArrowRight: "lookRight",
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

function resizeRenderer(): void {
  const width = window.innerWidth;
  const height = window.innerHeight;
  renderer.setSize(width, height);
  if (stageScene) {
    // 一人称カメラは画面比率に関わらず、人間の水平/垂直FOVをそのまま再現する
    // 固定aspectを使う(画面との差分はrenderSplitScreenでレターボックスして埋める)。
    stageScene.fpCamera.fov = PLAYER_VERTICAL_FOV_DEG;
    stageScene.fpCamera.aspect = HUMAN_FOV_ASPECT;
    stageScene.fpCamera.updateProjectionMatrix();

    const halfAspect = width / 2 / height;
    stageScene.topCamera.aspect = halfAspect;
    stageScene.topCamera.updateProjectionMatrix();
  }
}
window.addEventListener("resize", resizeRenderer);

function buildLevelSelect(stages: Stage[]): void {
  levelButtonsEl.innerHTML = "";
  for (const stage of stages) {
    const button = document.createElement("button");
    button.className = "level-button";
    button.textContent = `LEVEL ${stage.id}`;
    button.addEventListener("click", () => startStage(stage));
    levelButtonsEl.appendChild(button);
  }
}

function startStage(stage: Stage): void {
  currentStage = stage;
  stageScene = buildStageScene(stage);
  player = createPlayer(stage.start);
  guard = createGuard(stage.guard, reactionSelectEl.value as GuardReaction);
  input.up = input.down = input.left = input.right = false;
  input.lookLeft = input.lookRight = input.lookUp = input.lookDown = false;
  elapsed = 0;
  state = "PLAYING";
  resizeRenderer();
  showScreen();
}

function showScreen(): void {
  levelSelectEl.classList.toggle("hidden", state !== "LEVEL_SELECT");
  canvas.classList.toggle("hidden", state === "LEVEL_SELECT");
  hudEl.classList.toggle("hidden", state === "LEVEL_SELECT");
  overlayEl.classList.toggle("hidden", state !== "CLEAR" && state !== "GAME_OVER");

  if (state === "CLEAR") {
    overlayTitleEl.textContent = "CLEAR!";
    overlayHintEl.textContent = `TIME: ${elapsed.toFixed(1)}s`;
  } else if (state === "GAME_OVER") {
    overlayTitleEl.textContent = "GAME OVER";
    overlayHintEl.textContent = "G1に発見されました";
  }
}

retryButtonEl.addEventListener("click", () => {
  if (currentStage) startStage(currentStage);
});

backButtonEl.addEventListener("click", () => {
  state = "LEVEL_SELECT";
  showScreen();
});

overlayBackButtonEl.addEventListener("click", () => {
  state = "LEVEL_SELECT";
  showScreen();
});

function updateLook(dt: number): void {
  if (!player) return;
  if (input.lookLeft) player.yaw -= KEY_LOOK_SPEED_RAD * dt;
  if (input.lookRight) player.yaw += KEY_LOOK_SPEED_RAD * dt;
  if (input.lookUp || input.lookDown) {
    const delta = (input.lookUp ? 1 : 0) - (input.lookDown ? 1 : 0);
    const nextPitch = player.pitch + delta * KEY_LOOK_SPEED_RAD * dt;
    player.pitch = Math.max(-PLAYER_MAX_PITCH_RAD, Math.min(PLAYER_MAX_PITCH_RAD, nextPitch));
  }
}

function update(dt: number): void {
  if (!player || !guard || !currentStage) return;
  elapsed += dt;
  updateLook(dt);
  updatePlayer(player, input, dt, currentStage);
  updateGuard(guard, dt, currentStage, player);

  if (isCaught(guard)) {
    state = "GAME_OVER";
  } else if (isBackAtStart(player, currentStage)) {
    state = "CLEAR";
  }
}

function syncScene(): void {
  if (!stageScene || !player || !guard || !currentStage) return;
  const playerPos = toWorld(player.x, player.y);
  stageScene.playerMesh.position.x = playerPos.x;
  stageScene.playerMesh.position.z = playerPos.y;
  stageScene.footRing.position.x = playerPos.x;
  stageScene.footRing.position.z = playerPos.y;

  const guardPos = toWorld(guard.x, guard.y);
  stageScene.guardMesh.position.x = guardPos.x;
  stageScene.guardMesh.position.z = guardPos.y;
  stageScene.guardMesh.rotation.y = -guard.facingAngle;
  (stageScene.guardMesh.material as THREE.MeshStandardMaterial).color.set(guard.alert ? 0xff1744 : 0xef5350);

  const visionPoints = computeVisionPolygon(guard, currentStage.walls);
  updateVisionMesh(stageScene.visionMesh, visionPoints);

  stageScene.fpCamera.position.set(playerPos.x, PLAYER_EYE_HEIGHT_M, playerPos.y);
  stageScene.fpCamera.rotation.set(player.pitch, -player.yaw - Math.PI / 2, 0);

  hudTargetEl.textContent = `TARGET: ${player.hasTarget ? "GET" : "-"}`;
  hudTimerEl.textContent = `TIME: ${elapsed.toFixed(1)}`;
}

function loop(timestamp: number): void {
  const dt = lastTimestamp ? Math.min(0.05, (timestamp - lastTimestamp) / 1000) : 0;
  lastTimestamp = timestamp;

  if (state === "PLAYING") {
    update(dt);
    syncScene();
    if (state !== "PLAYING") showScreen();
  }

  if (stageScene) renderSplitScreen(stageScene);
  requestAnimationFrame(loop);
}

// 左半分: 一人称視点(人間のFOV比率をレターボックスで維持) / 右半分: 見下ろし(2Dマップ相当)視点。
function renderSplitScreen(stageScene: StageScene): void {
  const width = renderer.domElement.width;
  const height = renderer.domElement.height;
  const halfWidth = Math.floor(width / 2);

  renderer.setScissorTest(true);

  // レターボックス部分を含む左半分全体を先に黒で塗る。
  renderer.setViewport(0, 0, halfWidth, height);
  renderer.setScissor(0, 0, halfWidth, height);
  renderer.setClearColor(0x000000, 1);
  renderer.clear();

  let fpWidth = halfWidth;
  let fpHeight = Math.round(halfWidth / HUMAN_FOV_ASPECT);
  if (fpHeight > height) {
    fpHeight = height;
    fpWidth = Math.round(height * HUMAN_FOV_ASPECT);
  }
  const fpX = Math.floor((halfWidth - fpWidth) / 2);
  const fpY = Math.floor((height - fpHeight) / 2);

  renderer.setViewport(fpX, fpY, fpWidth, fpHeight);
  renderer.setScissor(fpX, fpY, fpWidth, fpHeight);
  renderer.render(stageScene.scene, stageScene.fpCamera);

  renderer.setViewport(halfWidth, 0, width - halfWidth, height);
  renderer.setScissor(halfWidth, 0, width - halfWidth, height);
  renderer.render(stageScene.scene, stageScene.topCamera);

  renderer.setScissorTest(false);
}

async function init(): Promise<void> {
  try {
    const stages = await loadStages();
    buildLevelSelect(stages);
  } catch (error) {
    console.error("ステージデータの読み込みに失敗しました:", error);
    levelButtonsEl.innerHTML = "<p>ステージデータの読み込みに失敗しました。開発サーバー経由で開いているか確認してください。</p>";
  }
  showScreen();
  requestAnimationFrame(loop);
}

init();
