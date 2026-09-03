// エディタ内でのテストプレイ。ゲーム本体のロジック（player.js/guard.js/render.js）を
// そのまま再利用し、保存前でも編集中のステージ設定をその場で確認できるようにする。

import { createPlayer, updatePlayer, isBackAtStart } from "./player.js";
import { createGuard, updateGuard, isCaught } from "./guard.js";
import { renderStage } from "./render.js";

export function createPlaySession(stage) {
  return {
    stage,
    player: createPlayer(stage.start),
    guard: createGuard(stage.guard),
    input: { up: false, down: false, left: false, right: false },
    result: null,
  };
}

export function stepPlaySession(session, dt) {
  if (session.result) return;

  updatePlayer(session.player, session.input, dt, session.stage);
  updateGuard(session.guard, dt, session.stage, session.player);

  if (isCaught(session.guard)) {
    session.result = "GAME_OVER";
    return;
  }
  if (isBackAtStart(session.player, session.stage)) {
    session.result = "CLEAR";
  }
}

export function renderPlaySession(ctx, session) {
  renderStage(ctx, session.stage, session.player, session.guard, `TEST PLAY: ${session.stage.name}`);
}
