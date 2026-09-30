import { moveWithWallCollision } from "./collision";
import { PLAYER_RADIUS_PX, PLAYER_SPEED_PX, TARGET_PICKUP_MARGIN_PX, START_RETURN_MARGIN_PX } from "./constants";
import type { InputState, Stage } from "./types";

export interface Player {
  x: number;
  y: number;
  radius: number;
  hasTarget: boolean;
  yaw: number; // 一人称視点の水平方向。atan2(dy,dx)と同じ規約(guardのfacingAngleと共通)。
  pitch: number; // 一人称視点の上下方向(ラジアン、上向き正)。
}

export function createPlayer(start: { x: number; y: number }): Player {
  return { x: start.x, y: start.y, radius: PLAYER_RADIUS_PX, hasTarget: false, yaw: 0, pitch: 0 };
}

// 移動はプレイヤーの向き(yaw)基準の相対移動(前後・左右ストレイフ)。
export function updatePlayer(player: Player, input: InputState, dt: number, stage: Stage): void {
  const forwardX = Math.cos(player.yaw);
  const forwardY = Math.sin(player.yaw);
  const rightX = -forwardY;
  const rightY = forwardX;

  let dx = 0;
  let dy = 0;
  if (input.up) {
    dx += forwardX;
    dy += forwardY;
  }
  if (input.down) {
    dx -= forwardX;
    dy -= forwardY;
  }
  if (input.right) {
    dx += rightX;
    dy += rightY;
  }
  if (input.left) {
    dx -= rightX;
    dy -= rightY;
  }

  if (dx !== 0 || dy !== 0) {
    const len = Math.hypot(dx, dy);
    dx = (dx / len) * PLAYER_SPEED_PX * dt;
    dy = (dy / len) * PLAYER_SPEED_PX * dt;
  }

  moveWithWallCollision(player, dx, dy, player.radius, stage.walls, stage);

  if (!player.hasTarget) {
    const distToTarget = Math.hypot(player.x - stage.target.x, player.y - stage.target.y);
    if (distToTarget < player.radius + TARGET_PICKUP_MARGIN_PX) {
      player.hasTarget = true;
    }
  }
}

export function isBackAtStart(player: Player, stage: Stage): boolean {
  const distToStart = Math.hypot(player.x - stage.start.x, player.y - stage.start.y);
  return player.hasTarget && distToStart < player.radius + START_RETURN_MARGIN_PX;
}
