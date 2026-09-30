// 警備員：巡回ルート移動と「距離＋視野角＋遮蔽物」の3条件による発見判定を担う。

import { moveWithWallCollision, intersectRayWithWall, segmentIntersectsWall } from "./collision.js";

const WAYPOINT_ARRIVAL_DIST = 4;
const RADIUS = 10;

// 方向転換時、G1はその場で足踏みしながら視界ごとゆっくり体を回す
// （直進しながら瞬時に向きを変えるのではなく、旋回中は移動しない）。
const TURN_SPEED = Math.PI / 4; // rad/sec（約45°/秒。従来の半分）
const TURN_ALIGN_THRESHOLD = (2 * Math.PI) / 180; // これ未満の角度差なら整列済みとみなす

// 視界に入った瞬間に即アウト（一発アウト）。

export function createGuard(config) {
  return {
    x: config.x,
    y: config.y,
    radius: RADIUS,
    patrolPath: config.patrolPath,
    waypointIndex: 0,
    moveSpeed: config.moveSpeed,
    visionAngle: config.visionAngle,
    visionDistance: config.visionDistance,
    facingAngle: 0,
    caught: false,
  };
}

export function updateGuard(guard, dt, stage, player) {
  if (guard.caught) return;

  if (canSeePlayer(guard, player, stage.walls)) {
    guard.caught = true;
    return;
  }

  updatePatrolMovement(guard, dt, stage);
}

function updatePatrolMovement(guard, dt, stage) {
  const target = guard.patrolPath[guard.waypointIndex];
  const dx = target.x - guard.x;
  const dy = target.y - guard.y;
  const dist = Math.hypot(dx, dy);

  if (dist < WAYPOINT_ARRIVAL_DIST) {
    guard.waypointIndex = (guard.waypointIndex + 1) % guard.patrolPath.length;
    return;
  }

  const desiredAngle = Math.atan2(dy, dx);
  const delta = signedAngleDelta(guard.facingAngle, desiredAngle);

  if (Math.abs(delta) > TURN_ALIGN_THRESHOLD) {
    const turnStep = Math.sign(delta) * Math.min(Math.abs(delta), TURN_SPEED * dt);
    guard.facingAngle = normalizeAngle(guard.facingAngle + turnStep);
    return;
  }

  guard.facingAngle = desiredAngle;
  const step = guard.moveSpeed * dt;
  const ratio = Math.min(1, step / dist);
  moveWithWallCollision(guard, dx * ratio, dy * ratio, guard.radius, stage.walls, stage);
}

function signedAngleDelta(from, to) {
  let diff = (to - from) % (Math.PI * 2);
  if (diff > Math.PI) diff -= Math.PI * 2;
  if (diff < -Math.PI) diff += Math.PI * 2;
  return diff;
}

function normalizeAngleDiff(a) {
  let diff = a % (Math.PI * 2);
  if (diff > Math.PI) diff -= Math.PI * 2;
  if (diff < -Math.PI) diff += Math.PI * 2;
  return Math.abs(diff);
}

function rayDistanceToWalls(originX, originY, angle, maxDist, walls) {
  const dirX = Math.cos(angle);
  const dirY = Math.sin(angle);
  let closest = maxDist;

  for (const wall of walls) {
    const tEnter = intersectRayWithWall(originX, originY, dirX, dirY, wall);
    if (tEnter !== null && tEnter > 0 && tEnter < closest) closest = tEnter;
  }

  return closest;
}

const CORNER_EPSILON = 0.0005; // ラジアン。壁の角ぎりぎりの視界を正確に拾うための微小オフセット。

function normalizeAngle(a) {
  let angle = a % (Math.PI * 2);
  if (angle > Math.PI) angle -= Math.PI * 2;
  if (angle < -Math.PI) angle += Math.PI * 2;
  return angle;
}

export function computeVisionPolygon(guard, walls, baseSegments = 24) {
  const halfAngle = (guard.visionAngle / 2) * (Math.PI / 180);
  const minAngle = -halfAngle;
  const maxAngle = halfAngle;

  const relativeAngles = new Set();
  for (let i = 0; i <= baseSegments; i++) {
    relativeAngles.add(minAngle + (maxAngle - minAngle) * (i / baseSegments));
  }

  for (const wall of walls) {
    const corners = [
      { x: wall.x, y: wall.y },
      { x: wall.x + wall.w, y: wall.y },
      { x: wall.x + wall.w, y: wall.y + wall.h },
      { x: wall.x, y: wall.y + wall.h },
    ];
    for (const corner of corners) {
      const angleToCorner = normalizeAngle(Math.atan2(corner.y - guard.y, corner.x - guard.x) - guard.facingAngle);
      for (const offset of [-CORNER_EPSILON, 0, CORNER_EPSILON]) {
        const angle = angleToCorner + offset;
        if (angle >= minAngle && angle <= maxAngle) relativeAngles.add(angle);
      }
    }
  }

  const sortedAngles = Array.from(relativeAngles).sort((a, b) => a - b);
  const points = [{ x: guard.x, y: guard.y }];
  for (const relativeAngle of sortedAngles) {
    const angle = guard.facingAngle + relativeAngle;
    const dist = rayDistanceToWalls(guard.x, guard.y, angle, guard.visionDistance, walls);
    points.push({ x: guard.x + Math.cos(angle) * dist, y: guard.y + Math.sin(angle) * dist });
  }

  return points;
}

export function isCaught(guard) {
  return guard.caught;
}

export function canSeePlayer(guard, player, walls) {
  const dx = player.x - guard.x;
  const dy = player.y - guard.y;
  const dist = Math.hypot(dx, dy);
  if (dist > guard.visionDistance) return false;

  const angleToPlayer = Math.atan2(dy, dx);
  const angleDiff = normalizeAngleDiff(angleToPlayer - guard.facingAngle);
  if (angleDiff > (guard.visionAngle / 2) * (Math.PI / 180)) return false;

  const guardPos = { x: guard.x, y: guard.y };
  const playerPos = { x: player.x, y: player.y };
  const occluded = walls.some((wall) => segmentIntersectsWall(guardPos, playerPos, wall));
  return !occluded;
}
