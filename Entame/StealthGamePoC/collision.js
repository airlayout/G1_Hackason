// 円と矩形壁の当たり判定。プレイヤー・警備員で共通利用する。

export function circleIntersectsWall(x, y, radius, wall) {
  const closestX = Math.max(wall.x, Math.min(x, wall.x + wall.w));
  const closestY = Math.max(wall.y, Math.min(y, wall.y + wall.h));
  const dx = x - closestX;
  const dy = y - closestY;
  return dx * dx + dy * dy < radius * radius;
}

export function moveWithWallCollision(entity, dx, dy, radius, walls, bounds) {
  const nextX = Math.max(radius, Math.min(bounds.width - radius, entity.x + dx));
  const blockedX = walls.some((wall) => circleIntersectsWall(nextX, entity.y, radius, wall));
  if (!blockedX) entity.x = nextX;

  const nextY = Math.max(radius, Math.min(bounds.height - radius, entity.y + dy));
  const blockedY = walls.some((wall) => circleIntersectsWall(entity.x, nextY, radius, wall));
  if (!blockedY) entity.y = nextY;
}

// 軸並行の矩形(壁)に対するスラブ法によるレイ交差判定。
// 角をかすめるレイでも、辺ごとの線分交差判定のような丸め誤差で
// 交差を取りこぼすことがないため、こちらを正とする。
export function intersectRayWithWall(originX, originY, dirX, dirY, wall) {
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

export function segmentIntersectsWall(p1, p2, wall) {
  const dx = p2.x - p1.x;
  const dy = p2.y - p1.y;
  const length = Math.hypot(dx, dy);
  if (length === 0) return false;

  const tEnter = intersectRayWithWall(p1.x, p1.y, dx / length, dy / length, wall);
  if (tEnter === null) return false;

  const EPS = 1e-6;
  return tEnter > EPS && tEnter < length - EPS;
}

// 巡回ルートの1区間が、警備員半径ぶん壁を膨張させた領域と交差していないか確認する。
// エディタとステージ検証スクリプトの両方から使う。
export function segmentBlockedByWall(p1, p2, wall, radius) {
  const expanded = { x: wall.x - radius, y: wall.y - radius, w: wall.w + radius * 2, h: wall.h + radius * 2 };
  return segmentIntersectsWall(p1, p2, expanded);
}
