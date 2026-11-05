// ミニロボット（任意）。人に同行する小型の家庭用ロボット（LOVOT のようなもの）。
// 人が見えている間は、常に人のいる方向へ最短ルートで向かう（見通しが通れば直線、壁があれば回り込む）。
// 人に追いついたときだけ、立ち止まったり、人のすぐそばを小さく動き回ったりする。
// 人から離れていく挙動はしない。
// 壁で人が見えなくなったら探索モード: 人を最後に見た場所へ行き、そこから周りを探す
// （見えない人の今の位置は知らない）。人が見えたらすぐ追従に戻る。
// ルール: 人またはミニロボットのどちらかが G1 / Pepper に見つかったら GAME OVER。
//         CLEAR には、人とミニロボットの両方が START に戻る必要がある。
// 注: LOVOT の思考（行動の決め方）は非公開（README 参照）。挙動は推測。
// 速度: LOVOT の移動速度は時速約1〜2km（0.28〜0.56 m/s。第三者サイトの掲載情報で、メーカー公式は未確認）。
//       初期値はその下限に近い 0.3 m/s。ステージJSONの `miniRobot.speed`（px/s）で変えられる。

import { moveWithWallCollision, segmentIntersectsWall } from "./collision.js";
import { isBackAtStart } from "./player.js";
import { findPath, isLineClear, isPointFree } from "./pathfinding.js";

const PX_PER_M = 80;
export const MINI_ROBOT_RADIUS = 6;
export const MINI_ROBOT_COLOR = "#ff9f1c";
export const MINI_ROBOT_DEFAULT_SPEED_MPS = 0.3; // LOVOT の時速1〜2km（0.28〜0.56 m/s）に合わせた
export const MINI_ROBOT_MIN_SPEED_MPS = 0.05;
export const MINI_ROBOT_MAX_SPEED_MPS = 1.0;

// 人に近づく最短距離は「人とミニロボットの半径の和」（重ならずに接する距離）。
// CLEAR 判定のため、START からこの距離以内ならミニロボットも START にいるとみなす。
// 人の判定（半径 + 14 = 24px）に、人のすぐ横で止まる分（16px）を足した値。
export const MINI_ROBOT_START_ZONE_PX = 40;

const WAYPOINT_ARRIVAL_DIST = 1;
const REPATH_SEC = 0.3; // 壁の向こうに人がいるとき、経路を引き直す間隔

// 気ままな挙動。LOVOT の思考（行動の決め方）は非公開で、ここは公開情報からの推測・ゲーム上の仮置き。
// 立ち止まる・動き回るのは「人に追いついたとき」だけ。追いついていない間は常に追従する。
const CAUGHT_UP_DIST_PX = 32; // 人までこの距離以内（0.4m）を「追いついた」とみなす
const WANDER_LIMIT_PX = 48; // 動き回るのは人からこの距離（0.6m）まで。超えたら人の方へ戻る
const FOLLOW_SEC = [2, 4]; // 追いついてから、次に立ち止まる・動き回るかを決めるまでの時間
const PAUSE_SEC = [1, 3]; // 立ち止まる時間
const WANDER_SEC = [1, 2.5]; // 人のそばを動き回る時間
const PAUSE_PROBABILITY = 0.5; // 追いついたあと、立ち止まる確率（残りは動き回る）

// 探索モード（人を見失ったとき）。★ 探し方は LOVOT の実挙動ではなく、ゲーム上の仮置き。
// 人を最後に見た場所へ行き、そこを中心に探す範囲を少しずつ広げながら、見えるまで探し続ける。
const SEARCH_RADIUS_START_PX = 80; // 最初に探す範囲（最後に見た場所から 1m）
const SEARCH_RADIUS_STEP_PX = 40; // 探す地点を決めるたびに広げる量
const SEARCH_RADIUS_MAX_PX = 480; // 探す範囲の上限（6m）
const SEARCH_MIN_DIST_PX = 30; // 最後に見た場所から、これ以上離れた地点を探す
const SEARCH_CANDIDATES = 12; // 探す地点を決めるときの試行回数
const SEARCH_AHEAD_PROBABILITY = 0.5; // 探す地点の半分は、人が最後に向かっていた方向（±60°）に偏らせる
const SEARCH_AHEAD_SPREAD = (2 * Math.PI) / 3;
const SEARCH_ARRIVAL_DIST_PX = 4;
const HEADING_SAMPLE_MIN_PX = 2; // これ以上動いたときだけ、人の進行方向を更新する

export const MODE = { FOLLOW: "follow", PAUSE: "pause", WANDER: "wander", SEARCH: "search" };

// 画面に出すラベル用の名前（"Mini: follower" のように表示する）。
export const MODE_LABEL = {
  [MODE.FOLLOW]: "follower",
  [MODE.PAUSE]: "pause",
  [MODE.WANDER]: "wander",
  [MODE.SEARCH]: "search",
};

// エディタで有効にしたときの初期値（ステージJSONの `miniRobot` と同じ形。speed は px/s）。
export function defaultMiniRobot() {
  return { speed: Math.round(MINI_ROBOT_DEFAULT_SPEED_MPS * PX_PER_M) };
}

function randomBetween(range, rng) {
  return range[0] + (range[1] - range[0]) * rng();
}

export function createMiniRobot(config, start, rng = Math.random) {
  const full = { ...defaultMiniRobot(), ...config };
  return {
    x: start.x,
    y: start.y,
    radius: MINI_ROBOT_RADIUS,
    speed: full.speed,
    path: [], // 壁を回り込むときの経路（これから通る点。見通しが通る間は空）
    repathSecLeft: 0,
    mode: MODE.FOLLOW,
    modeSecLeft: randomBetween(FOLLOW_SEC, rng),
    wanderHeading: 0,
    lastSeen: { x: start.x, y: start.y }, // 人を最後に見た場所
    lastSeenHeading: null, // そのとき人が向かっていた方向（rad）。分からなければ null
    searchTarget: null, // 探索モードで向かっている地点
    searchRadius: SEARCH_RADIUS_START_PX,
  };
}

// rng は 0〜1 の乱数。テストで固定できるよう引数にしている。
export function updateMiniRobot(robot, dt, player, stage, rng = Math.random) {
  const visible = canSeePlayer(robot, player, stage.walls);
  if (visible) rememberPlayer(robot, player);

  if (!visible) {
    if (robot.mode !== MODE.SEARCH) startSearch(robot);
  } else if (robot.mode === MODE.SEARCH) {
    // 人が見えた: 探索をやめて追いかける。
    setMode(robot, MODE.FOLLOW, randomBetween(FOLLOW_SEC, rng));
    robot.path = [];
    robot.searchTarget = null;
  } else {
    updateModeNearPlayer(robot, dt, player, rng);
  }

  if (robot.mode === MODE.FOLLOW) follow(robot, dt, player, stage);
  else if (robot.mode === MODE.WANDER) wander(robot, dt, stage, player);
  else if (robot.mode === MODE.SEARCH) search(robot, dt, stage, rng);
  // PAUSE: その場で止まる
}

// 人が見えているときの、追従・立ち止まり・動き回りの切り替え。
function updateModeNearPlayer(robot, dt, player, rng) {
  const distance = distanceToPlayer(robot, player);
  if (robot.mode !== MODE.FOLLOW && distance > WANDER_LIMIT_PX) {
    // 人が離れていったら、止まる・動き回るをやめてすぐ追いかける。
    setMode(robot, MODE.FOLLOW, randomBetween(FOLLOW_SEC, rng));
    return;
  }
  // 追従中は、追いついている間だけ時間を数える（追いつくまでは追従を続ける）。
  if (robot.mode !== MODE.FOLLOW || distance <= CAUGHT_UP_DIST_PX) robot.modeSecLeft -= dt;
  if (robot.modeSecLeft <= 0) switchMode(robot, rng);
}

function setMode(robot, mode, seconds) {
  robot.mode = mode;
  robot.modeSecLeft = seconds;
}

function distanceToPlayer(robot, player) {
  return Math.hypot(player.x - robot.x, player.y - robot.y);
}

// 人が見える: ロボットから人まで、壁に遮られていない（距離の制限は設けていない）。
function canSeePlayer(robot, player, walls) {
  return !walls.some((wall) => segmentIntersectsWall(robot, player, wall));
}

// 見えた人の位置と進行方向を覚える（見失ったときの手がかり）。
function rememberPlayer(robot, player) {
  const dx = player.x - robot.lastSeen.x;
  const dy = player.y - robot.lastSeen.y;
  if (Math.hypot(dx, dy) >= HEADING_SAMPLE_MIN_PX) robot.lastSeenHeading = Math.atan2(dy, dx);
  robot.lastSeen = { x: player.x, y: player.y };
}

// 次の行動を決める。追いついたあとは「立ち止まる / 動き回る」、それ以外は追従に戻る。
function switchMode(robot, rng) {
  if (robot.mode !== MODE.FOLLOW) {
    robot.mode = MODE.FOLLOW;
    robot.modeSecLeft = randomBetween(FOLLOW_SEC, rng);
    return;
  }
  if (rng() < PAUSE_PROBABILITY) {
    robot.mode = MODE.PAUSE;
    robot.modeSecLeft = randomBetween(PAUSE_SEC, rng);
    return;
  }
  robot.mode = MODE.WANDER;
  robot.modeSecLeft = randomBetween(WANDER_SEC, rng);
  robot.wanderHeading = rng() * Math.PI * 2;
}

// 人のいる方向へ最短で向かう。見通しが通れば直線、壁があれば回り込む経路をたどる。
function follow(robot, dt, player, stage) {
  const stopDist = robot.radius + player.radius;
  if (isLineClear(robot, player, stage.walls, robot.radius)) {
    robot.path = [];
    moveToward(robot, player, robot.speed * dt, stopDist);
    return;
  }

  robot.repathSecLeft -= dt;
  if (robot.path.length === 0 || robot.repathSecLeft <= 0) {
    robot.path = findPath(robot, player, stage, robot.radius) ?? [];
    robot.repathSecLeft = REPATH_SEC;
  }
  followPath(robot, robot.speed * dt);
}

function followPath(robot, distance) {
  let remaining = distance;
  while (remaining > 0 && robot.path.length > 0) {
    const next = robot.path[0];
    const dist = Math.hypot(next.x - robot.x, next.y - robot.y);
    if (dist <= WAYPOINT_ARRIVAL_DIST || dist <= remaining) {
      robot.x = next.x;
      robot.y = next.y;
      robot.path.shift();
      remaining -= dist;
    } else {
      robot.x += ((next.x - robot.x) / dist) * remaining;
      robot.y += ((next.y - robot.y) / dist) * remaining;
      remaining = 0;
    }
  }
}

// target の方へ step だけ進む。target から stopDist より近くには寄らない。
function moveToward(robot, target, step, stopDist) {
  const dx = target.x - robot.x;
  const dy = target.y - robot.y;
  const dist = Math.hypot(dx, dy);
  const move = Math.min(step, dist - stopDist);
  if (move <= 0) return;
  robot.x += (dx / dist) * move;
  robot.y += (dy / dist) * move;
}

// 向きを決めて少し進む。壁・エリア端にはぶつかる（通り抜けない）。
function wander(robot, dt, stage, player) {
  const step = robot.speed * dt;
  // 人から離れすぎそうなら、向きを人の方へ変える（人から遠ざかる方へは進まない）。
  if (distanceToPlayer(robot, player) >= WANDER_LIMIT_PX * 0.8) {
    robot.wanderHeading = Math.atan2(player.y - robot.y, player.x - robot.x);
  }
  moveWithWallCollision(
    robot,
    Math.cos(robot.wanderHeading) * step,
    Math.sin(robot.wanderHeading) * step,
    robot.radius,
    stage.walls,
    stage
  );
}

// 人を見失った: まず人を最後に見た場所へ向かう。
function startSearch(robot) {
  robot.mode = MODE.SEARCH;
  robot.path = [];
  robot.searchTarget = { x: robot.lastSeen.x, y: robot.lastSeen.y };
  robot.searchRadius = SEARCH_RADIUS_START_PX;
}

// 探索モード: 目的地へ壁を避けて向かい、着いたら（人が見えなければ）次の地点を決める。
function search(robot, dt, stage, rng) {
  const target = robot.searchTarget;
  if (target && Math.hypot(target.x - robot.x, target.y - robot.y) <= SEARCH_ARRIVAL_DIST_PX) {
    robot.searchTarget = null;
    robot.path = [];
  }
  if (!robot.searchTarget) robot.searchTarget = pickSearchTarget(robot, stage, rng);
  if (!robot.searchTarget) return;

  if (robot.path.length === 0) {
    const path = findPath(robot, robot.searchTarget, stage, robot.radius);
    if (!path) {
      robot.searchTarget = null; // 行けない地点は捨てて、次のフレームで選び直す
      return;
    }
    robot.path = path;
  }
  followPath(robot, robot.speed * dt);
}

// 人を最後に見た場所の周りから、行ける地点を選ぶ。半分は人が最後に向かっていた方向に偏らせる。
// 選ぶたびに探す範囲を広げるので、見つからなければ少しずつ遠くまで探す。
function pickSearchTarget(robot, stage, rng) {
  const origin = robot.lastSeen;
  let found = null;
  for (let i = 0; i < SEARCH_CANDIDATES && !found; i++) {
    const ahead = robot.lastSeenHeading !== null && rng() < SEARCH_AHEAD_PROBABILITY;
    const angle = ahead ? robot.lastSeenHeading + (rng() - 0.5) * SEARCH_AHEAD_SPREAD : rng() * Math.PI * 2;
    const dist = SEARCH_MIN_DIST_PX + (robot.searchRadius - SEARCH_MIN_DIST_PX) * rng();
    const point = { x: origin.x + Math.cos(angle) * dist, y: origin.y + Math.sin(angle) * dist };
    if (isPointFree(point, stage, robot.radius)) found = point;
  }
  robot.searchRadius = Math.min(SEARCH_RADIUS_MAX_PX, robot.searchRadius + SEARCH_RADIUS_STEP_PX);
  return found;
}

export function isMiniRobotAtStart(robot, stage) {
  return Math.hypot(robot.x - stage.start.x, robot.y - stage.start.y) < MINI_ROBOT_START_ZONE_PX;
}

// CLEAR 条件: 人が TARGET を持って START に戻り、ミニロボットがいればそれも START に戻っている。
export function isStageCleared(player, miniRobot, stage) {
  if (!isBackAtStart(player, stage)) return false;
  return !miniRobot || isMiniRobotAtStart(miniRobot, stage);
}

// 見つかったらアウトの対象の一覧。
export function detectionTargets(player, miniRobot) {
  return miniRobot ? [player, miniRobot] : [player];
}
