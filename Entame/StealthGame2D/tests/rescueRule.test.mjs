// 救出ルールの動作確認（ブラウザ不要）。実行: node --test Entame/StealthGame2D/tests/
import test from "node:test";
import assert from "node:assert/strict";
import { createPlayer, updatePlayer } from "../player.js";
import { updateMiniRobot, isStageCleared, detectionTargets, MODE } from "../miniRobot.js";
import { RULE, RESCUE_RANGE_PX, createPlayRobot, tryRescue } from "../rescueRule.js";

const STAGE = {
  width: 800,
  height: 800,
  walls: [],
  start: { x: 40, y: 400 },
  target: { x: 760, y: 400 },
};
const STEP = 1 / 50;
const none = { up: false, down: false, left: false, right: false };

function place(player, x, y) {
  player.x = x;
  player.y = y;
}

test("救出ルールでは TARGET の位置にステイモードのミニロボットがいる", () => {
  const robot = createPlayRobot(STAGE, RULE.RESCUE, false);
  assert.equal(robot.mode, MODE.STAY);
  assert.deepEqual([robot.x, robot.y], [STAGE.target.x, STAGE.target.y]);
  assert.equal(robot.rescued, false);
});

test("盗むルールでは従来どおり（連れて行かなければ null、連れて行けば START）", () => {
  assert.equal(createPlayRobot(STAGE, RULE.STEAL, false), null);
  const robot = createPlayRobot(STAGE, RULE.STEAL, true);
  assert.deepEqual([robot.x, robot.y], [STAGE.start.x, STAGE.start.y]);
  assert.equal(robot.rescueMission, undefined);
});

test("ステイモード中は動かず、検知の対象にもならない", () => {
  const robot = createPlayRobot(STAGE, RULE.RESCUE, false);
  const player = createPlayer(STAGE.start, false);
  for (let i = 0; i < 500; i++) updateMiniRobot(robot, STEP, player, STAGE);
  assert.deepEqual([robot.x, robot.y], [STAGE.target.x, STAGE.target.y]);
  assert.equal(robot.mode, MODE.STAY);
  assert.deepEqual(detectionTargets(player, robot), [player]);
});

test("救出ルールの人は TARGET に触れても取得しない", () => {
  const player = createPlayer(STAGE.start, false);
  place(player, STAGE.target.x - 5, STAGE.target.y);
  updatePlayer(player, none, STEP, STAGE);
  assert.equal(player.hasTarget, false);
});

test("届かない距離や、操作なしでは救出できない", () => {
  const robot = createPlayRobot(STAGE, RULE.RESCUE, false);
  const player = createPlayer(STAGE.start, false);
  place(player, robot.x - RESCUE_RANGE_PX - 1, robot.y);
  assert.equal(tryRescue(robot, player, true), false);
  place(player, robot.x - 20, robot.y);
  assert.equal(tryRescue(robot, player, false), false);
  assert.equal(robot.mode, MODE.STAY);
});

test("届く距離で操作すると通常モード（追従）になり、検知の対象になる", () => {
  const robot = createPlayRobot(STAGE, RULE.RESCUE, false);
  const player = createPlayer(STAGE.start, false);
  place(player, robot.x - RESCUE_RANGE_PX, robot.y);
  assert.equal(tryRescue(robot, player, true), true);
  assert.equal(robot.rescued, true);
  assert.equal(robot.mode, MODE.FOLLOW);
  assert.deepEqual(detectionTargets(player, robot), [player, robot]);
});

test("救出前に人だけ START にいても CLEAR にならない", () => {
  const robot = createPlayRobot(STAGE, RULE.RESCUE, false);
  const player = createPlayer(STAGE.start, false);
  assert.equal(isStageCleared(player, robot, STAGE), false);
});

test("救出して人と一緒に START へ戻ると CLEAR（途中ではならない）", () => {
  const robot = createPlayRobot(STAGE, RULE.RESCUE, false);
  const player = createPlayer(STAGE.start, false);
  place(player, STAGE.target.x - 30, STAGE.target.y);
  assert.equal(tryRescue(robot, player, true), true);

  let cleared = false;
  let secondsAtClear = null;
  for (let i = 0; i < 50 * 120 && !cleared; i++) {
    // 人は START へ向かって歩く（左へ）。ミニロボットは遅れて付いてくる。
    updatePlayer(player, { ...none, left: player.x > STAGE.start.x }, STEP, STAGE);
    updateMiniRobot(robot, STEP, player, STAGE, () => 0.99);
    if (player.x <= STAGE.start.x + 1 && !isStageCleared(player, robot, STAGE)) {
      assert.ok(robot.x > STAGE.start.x + 40, "ミニロボットが遅れている間は CLEAR にならない");
    }
    cleared = isStageCleared(player, robot, STAGE);
    if (cleared) secondsAtClear = i * STEP;
  }
  assert.ok(cleared, "120 秒以内に CLEAR になる");
  assert.ok(secondsAtClear > 20, `ミニロボット（0.3 m/s）が遅いので人より後になる: ${secondsAtClear}s`);
  assert.equal(player.hasTarget, false);
});
