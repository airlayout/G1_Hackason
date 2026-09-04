import type { Wall } from "./types";

// 2D版 collision.js のXZ平面移植。円(XZ平面上の半径)と矩形壁の当たり判定を共通利用する。

export interface Movable {
  x: number;
  y: number;
}

export function circleIntersectsWall(x: number, y: number, radius: number, wall: Wall): boolean {
  const closestX = Math.max(wall.x, Math.min(x, wall.x + wall.w));
  const closestY = Math.max(wall.y, Math.min(y, wall.y + wall.h));
  const dx = x - closestX;
  const dy = y - closestY;
  return dx * dx + dy * dy < radius * radius;
}

export function moveWithWallCollision(
  entity: Movable,
  dx: number,
  dy: number,
  radius: number,
  walls: Wall[],
  bounds: { width: number; height: number }
): void {
  const nextX = Math.max(radius, Math.min(bounds.width - radius, entity.x + dx));
  const blockedX = walls.some((wall) => circleIntersectsWall(nextX, entity.y, radius, wall));
  if (!blockedX) entity.x = nextX;

  const nextY = Math.max(radius, Math.min(bounds.height - radius, entity.y + dy));
  const blockedY = walls.some((wall) => circleIntersectsWall(entity.x, nextY, radius, wall));
  if (!blockedY) entity.y = nextY;
}

// 軸並行の矩形(壁)に対するスラブ法によるレイ交差判定。視認の遮蔽判定に使う。
export function intersectRayWithWall(originX: number, originY: number, dirX: number, dirY: number, wall: Wall): number | null {
  let tEnter = -Infinity;
  let tExit = Infinity;

  if (dirX === 0) {
    if (originX < wall.x || originX > wall.x + wall.w) return null;
  } else {
    const tx1 = (wall.x - originX) / dirX;
    const tx2 = (wall.x + wall.w - originX) / dirX;
    tEnter = Math.max(tEnter, Math.min(tx1, tx2));
    tExit = Math.min(tExit, Math.max(tx1, tx2));
  }

  if (dirY === 0) {
    if (originY < wall.y || originY > wall.y + wall.h) return null;
  } else {
    const ty1 = (wall.y - originY) / dirY;
    const ty2 = (wall.y + wall.h - originY) / dirY;
    tEnter = Math.max(tEnter, Math.min(ty1, ty2));
    tExit = Math.min(tExit, Math.max(ty1, ty2));
  }

  if (tEnter > tExit || tExit < 0) return null;
  return tEnter;
}

export function segmentIntersectsWall(p1: Movable, p2: Movable, wall: Wall): boolean {
  const dx = p2.x - p1.x;
  const dy = p2.y - p1.y;
  const length = Math.hypot(dx, dy);
  if (length === 0) return false;

  const tEnter = intersectRayWithWall(p1.x, p1.y, dx / length, dy / length, wall);
  if (tEnter === null) return false;

  const EPS = 1e-6;
  return tEnter > EPS && tEnter < length - EPS;
}
