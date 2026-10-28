// 警備員(G1)：巡回・視認判定・発見後の反応パターンを担う。
// 視認判定(距離+視野角+遮蔽)は2D版 guard.js を踏襲、検知は時間しきい値ありに変更。

import { moveWithWallCollision, intersectRayWithWall, segmentIntersectsWall } from "./collision";
import {
  GUARD_RADIUS_PX,
  WAYPOINT_ARRIVAL_DIST_PX,
  GUARD_TURN_SPEED,
  GUARD_TURN_ALIGN_THRESHOLD,
  DETECTION_HOLD_TIME_SEC,
  REACTION_DURATION_SEC,
} from "./constants";
import type { GuardConfig, GuardReaction, Point2D, Stage } from "./types";
import type { Player } from "./player";

export interface Guard {
  x: number;
  y: number;
  radius: number;
  patrolPath: Point2D[];
  waypointIndex: number;
  moveSpeed: number;
  visionAngle: number;
  visionDistance: number;
  facingAngle: number;
  caught: boolean;
  reaction: GuardReaction;
  detectTimer: number;
  reactionTimer: number;
  lastSeenPos: Point2D | null;
  alert: boolean;
}

export function createGuard(config: GuardConfig, reaction: GuardReaction = "STOP_NONE"): Guard {
  return {
    x: config.x,
    y: config.y,
    radius: GUARD_RADIUS_PX,
    patrolPath: config.patrolPath,
    waypointIndex: 0,
    moveSpeed: config.moveSpeed,
    visionAngle: config.visionAngle,
    visionDistance: config.visionDistance,
    facingAngle: 0,
    caught: false,
    reaction,
    detectTimer: 0,
    reactionTimer: 0,
    lastSeenPos: null,
    alert: false,
  };
}

export function updateGuard(guard: Guard, dt: number, stage: Stage, player: Player): void {
  if (guard.caught) return;

  const seen = canSeePlayer(guard, player, stage.walls);

  if (seen) {
    guard.detectTimer += dt;
    guard.reactionTimer = REACTION_DURATION_SEC;
    guard.lastSeenPos = { x: player.x, y: player.y };
    guard.alert = true;
    if (guard.detectTimer >= DETECTION_HOLD_TIME_SEC) {
      guard.caught = true;
      return;
    }
  } else {
    guard.detectTimer = 0;
  }

  if (!seen && guard.reactionTimer > 0) {
    guard.reactionTimer -= dt;
    guard.alert = guard.reactionTimer > 0;
    applyReaction(guard, dt, stage);
    return;
  }

  guard.alert = seen;
  updatePatrolMovement(guard, dt, stage);
}

function applyReaction(guard: Guard, dt: number, stage: Stage): void {
  if (guard.reaction === "STOP_NONE") {
    updatePatrolMovement(guard, dt, stage);
    return;
  }
  if (guard.reaction === "STOP" || !guard.lastSeenPos) {
    return;
  }
  if (guard.reaction === "LOOK_AT_PLAYER") {
    faceTowards(guard, guard.lastSeenPos, dt);
    return;
  }
  if (guard.reaction === "CHASE") {
    chaseTowards(guard, guard.lastSeenPos, dt, stage);
  }
}

function faceTowards(guard: Guard, target: Point2D, dt: number): void {
  const desiredAngle = Math.atan2(target.y - guard.y, target.x - guard.x);
  const delta = signedAngleDelta(guard.facingAngle, desiredAngle);
  const turnStep = Math.sign(delta) * Math.min(Math.abs(delta), GUARD_TURN_SPEED * dt);
  guard.facingAngle = normalizeAngle(guard.facingAngle + turnStep);
}

function chaseTowards(guard: Guard, target: Point2D, dt: number, stage: Stage): void {
  const dx = target.x - guard.x;
  const dy = target.y - guard.y;
  const dist = Math.hypot(dx, dy);
  if (dist < WAYPOINT_ARRIVAL_DIST_PX) return;

  faceTowards(guard, target, dt);
  const desiredAngle = Math.atan2(dy, dx);
  if (Math.abs(signedAngleDelta(guard.facingAngle, desiredAngle)) > GUARD_TURN_ALIGN_THRESHOLD) return;

  const step = guard.moveSpeed * dt;
  const ratio = Math.min(1, step / dist);
  moveWithWallCollision(guard, dx * ratio, dy * ratio, guard.radius, stage.walls, stage);
}

function updatePatrolMovement(guard: Guard, dt: number, stage: Stage): void {
  const target = guard.patrolPath[guard.waypointIndex];
  const dx = target.x - guard.x;
  const dy = target.y - guard.y;
  const dist = Math.hypot(dx, dy);

  if (dist < WAYPOINT_ARRIVAL_DIST_PX) {
    guard.waypointIndex = (guard.waypointIndex + 1) % guard.patrolPath.length;
    return;
  }

  const desiredAngle = Math.atan2(dy, dx);
  const delta = signedAngleDelta(guard.facingAngle, desiredAngle);

  if (Math.abs(delta) > GUARD_TURN_ALIGN_THRESHOLD) {
    const turnStep = Math.sign(delta) * Math.min(Math.abs(delta), GUARD_TURN_SPEED * dt);
    guard.facingAngle = normalizeAngle(guard.facingAngle + turnStep);
    return;
  }

  guard.facingAngle = desiredAngle;
  const step = guard.moveSpeed * dt;
  const ratio = Math.min(1, step / dist);
  moveWithWallCollision(guard, dx * ratio, dy * ratio, guard.radius, stage.walls, stage);
}

function normalizeAngle(a: number): number {
  let angle = a % (Math.PI * 2);
  if (angle > Math.PI) angle -= Math.PI * 2;
  if (angle < -Math.PI) angle += Math.PI * 2;
  return angle;
}

function signedAngleDelta(from: number, to: number): number {
  return normalizeAngle(to - from);
}

function normalizeAngleDiff(a: number): number {
  return Math.abs(normalizeAngle(a));
}

export function canSeePlayer(guard: Guard, player: Player, walls: Stage["walls"]): boolean {
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

const CORNER_EPSILON = 0.0005;

export function computeVisionPolygon(guard: Guard, walls: Stage["walls"], baseSegments = 24): Point2D[] {
  const halfAngle = (guard.visionAngle / 2) * (Math.PI / 180);
  const minAngle = -halfAngle;
  const maxAngle = halfAngle;

  const relativeAngles = new Set<number>();
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
  const points: Point2D[] = [{ x: guard.x, y: guard.y }];
  for (const relativeAngle of sortedAngles) {
    const angle = guard.facingAngle + relativeAngle;
    const dist = rayDistanceToWalls(guard.x, guard.y, angle, guard.visionDistance, walls);
    points.push({ x: guard.x + Math.cos(angle) * dist, y: guard.y + Math.sin(angle) * dist });
  }

  return points;
}

function rayDistanceToWalls(originX: number, originY: number, angle: number, maxDist: number, walls: Stage["walls"]): number {
  const dirX = Math.cos(angle);
  const dirY = Math.sin(angle);
  let closest = maxDist;

  for (const wall of walls) {
    const tEnter = intersectRayWithWall(originX, originY, dirX, dirY, wall);
    if (tEnter !== null && tEnter > 0 && tEnter < closest) closest = tEnter;
  }

  return closest;
}

export function isCaught(guard: Guard): boolean {
  return guard.caught;
}
