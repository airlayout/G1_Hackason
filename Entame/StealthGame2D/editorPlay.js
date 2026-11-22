// エディタ内でのテストプレイ。ゲーム本体のロジック（player.js/guard.js/render.js）を
// そのまま再利用し、保存前でも編集中のステージ設定をその場で確認できるようにする。

import { createPlayer, updatePlayer } from "./player.js";
import { createGuard, updateGuard, isCaught } from "./guard.js";
import { createSecurityCamera, updateSecurityCamera } from "./securityCamera.js";
import { createMiniRobot, defaultMiniRobot, updateMiniRobot, isStageCleared, detectionTargets } from "./miniRobot.js";
import { renderStage } from "./render.js";

// withMiniRobot: テストプレイでミニロボットを連れて行くか（ステージには保存しない）。
export function createPlaySession(stage, withMiniRobot = false) {
  return {
    stage,
    player: createPlayer(stage.start),
    guard: createGuard(stage.guard),
    securityCameras: stage.securityCameras.map(createSecurityCamera),
    miniRobot: withMiniRobot ? createMiniRobot(defaultMiniRobot(), stage.start) : null,
    input: { up: false, down: false, left: false, right: false },
    result: null,
  };
}

export function stepPlaySession(session, dt) {
  if (session.result) return;

  updatePlayer(session.player, session.input, dt, session.stage);
  if (session.miniRobot) updateMiniRobot(session.miniRobot, dt, session.player, session.stage);
  const targets = detectionTargets(session.player, session.miniRobot);
  updateGuard(session.guard, dt, session.stage, targets);
  for (const camera of session.securityCameras) {
    updateSecurityCamera(camera, dt, session.stage, targets);
  }

  if (isCaught(session.guard) || session.securityCameras.some((camera) => camera.caught)) {
    session.result = "GAME_OVER";
    return;
  }
  if (isStageCleared(session.player, session.miniRobot, session.stage)) {
    session.result = "CLEAR";
  }
}

export function renderPlaySession(ctx, session) {
  renderStage(ctx, session.stage, session.player, session.guard, `TEST PLAY: ${session.stage.name}`, session.securityCameras, session.miniRobot);
}
