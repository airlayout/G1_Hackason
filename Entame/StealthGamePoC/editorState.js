// エディタの初期状態生成とステージデータの複製。

// スケール: 1m = 80px（部屋10m×10m = 800px×800px）。
// moveSpeed 40px/s は実機G1の歩行速度0.5m/sに対応する。
export function createInitialState() {
  return {
    id: 1,
    name: "STAGE 1",
    width: 800,
    height: 800,
    start: { x: 40, y: 400 },
    target: { x: 760, y: 400 },
    walls: [],
    guard: {
      x: 400,
      y: 400,
      patrolPath: [],
      moveSpeed: 40,
      visionAngle: 65,
      visionDistance: 400,
    },
  };
}

export function loadStage(stage) {
  return JSON.parse(JSON.stringify(stage));
}
