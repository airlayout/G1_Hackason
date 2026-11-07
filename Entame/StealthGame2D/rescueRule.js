// 救出ルール: TARGET を盗む代わりに、囚われのミニロボット（LOVOT）を迎えに行き、START に連れて帰る。
// ストーリー: 監視員の目を掻い潜って、囚われの仲間を救出せよ。
//
// 流れ:
//   1. ミニロボットは TARGET の位置で「ステイモード」（動かない）。
//   2. 人がミニロボットのそばまで行き、救出の操作をする。実機では LOVOT の頭部のセンサーホーンの
//      モードスイッチでステイモードから通常モードに切り替える。ゲームでは Space キーがその代わり。
//   3. 通常モードになったミニロボットは人に付いてくる。人とミニロボットの両方が START に戻れば CLEAR。
// 見つかったら GAME OVER は従来どおり。ステイモード中のミニロボットは検知の対象にしない（仮置き）。
// 通常のルールとの違いはミニロボットの生成だけで、ステージJSONは変えない。

import { createMiniRobot, defaultMiniRobot, MODE } from "./miniRobot.js";

export const RULE = { STEAL: "steal", RESCUE: "rescue" };

// 救出の操作ができる距離。人とミニロボットの半径の和（16px）より少し広い。実機のホーンに手が届く距離の測定値ではない。
export const RESCUE_RANGE_PX = 48; // 0.6m

// ルールに応じたミニロボットを作る。steal: 連れて行く設定のときだけ START に。rescue: 必ず TARGET の位置にステイモードで。
export function createPlayRobot(stage, rule, withMiniRobot) {
  if (rule === RULE.RESCUE) {
    const robot = createMiniRobot(defaultMiniRobot(), stage.target);
    robot.mode = MODE.STAY;
    robot.rescueMission = true;
    robot.rescued = false;
    return robot;
  }
  return withMiniRobot ? createMiniRobot(defaultMiniRobot(), stage.start) : null;
}

export function canRescue(robot, player) {
  if (!robot?.rescueMission || robot.rescued) return false;
  return Math.hypot(player.x - robot.x, player.y - robot.y) <= RESCUE_RANGE_PX;
}

// 救出の操作が押されたら、届く距離にいれば通常モード（追従）に切り替える。切り替えたら true。
export function tryRescue(robot, player, pressed) {
  if (!pressed || !canRescue(robot, player)) return false;
  robot.rescued = true;
  robot.mode = MODE.FOLLOW;
  robot.modeSecLeft = 2;
  robot.lastSeen = { x: player.x, y: player.y };
  return true;
}
