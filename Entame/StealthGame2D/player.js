// プレイヤー：移動・壁衝突・ターゲット取得を管理する。

import { moveWithWallCollision } from "./collision.js";

// スケール: 1m = 80px（部屋10m×10m = 800px×800px）。
// プレイヤー(人)の歩行速度は実測1.0m/sを想定。
const RADIUS = 10;
const SPEED = 80; // px/sec (1.0 m/s)

export function createPlayer(start) {
  return {
    x: start.x,
    y: start.y,
    radius: RADIUS,
    hasTarget: false,
  };
}

export function updatePlayer(player, input, dt, stage) {
  let dx = 0;
  let dy = 0;
  if (input.up) dy -= 1;
  if (input.down) dy += 1;
  if (input.left) dx -= 1;
  if (input.right) dx += 1;

  if (dx !== 0 || dy !== 0) {
    const len = Math.hypot(dx, dy);
    dx = (dx / len) * SPEED * dt;
    dy = (dy / len) * SPEED * dt;
  }

  moveWithWallCollision(player, dx, dy, player.radius, stage.walls, stage);

  if (!player.hasTarget) {
    const distToTarget = Math.hypot(player.x - stage.target.x, player.y - stage.target.y);
    if (distToTarget < player.radius + 12) {
      player.hasTarget = true;
    }
  }
}

export function isBackAtStart(player, stage) {
  const distToStart = Math.hypot(player.x - stage.start.x, player.y - stage.start.y);
  return player.hasTarget && distToStart < player.radius + 14;
}
