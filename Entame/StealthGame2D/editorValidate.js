// ステージデータの妥当性チェック。「警備員が壁にスタックする」種類のバグを
// エディタ上で作った時点で検出するために使う（stages.jsの検証スクリプトと同じロジック）。

import { segmentBlockedByWall, circleIntersectsWall } from "./collision.js";

const ENTITY_RADIUS = 10;

export function validateStage(stage) {
  const blockedLegs = [];
  const path = stage.guard.patrolPath;
  if (path.length >= 2) {
    for (let i = 0; i < path.length; i++) {
      const a = path[i];
      const b = path[(i + 1) % path.length];
      const blocked = stage.walls.some((wall) => segmentBlockedByWall(a, b, wall, ENTITY_RADIUS));
      if (blocked) blockedLegs.push(i);
    }
  }

  const stuckPoints = [];
  const checkPoint = (label, pos) => {
    const stuck = stage.walls.some((wall) => circleIntersectsWall(pos.x, pos.y, ENTITY_RADIUS, wall));
    if (stuck) stuckPoints.push(label);
  };
  checkPoint("START", stage.start);
  checkPoint("TARGET", stage.target);
  checkPoint("GUARD", { x: stage.guard.x, y: stage.guard.y });

  return { blockedLegs, stuckPoints };
}
