// エディタの初期状態生成とステージデータの複製。

// スケール: 1m = 80px（部屋10m×10m = 800px×800px）。
// moveSpeed 32px/s = G1 の歩行速度 0.4m/s（実測 0.38m/s。指令 0.5 に対し。docs/safety.md）。
// visionAngle 70° は docs/g1-detection-spec.md のカメラ水平画角。
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
      moveSpeed: 32,
      visionAngle: 70,
      visionDistance: 400,
    },
    // セキュリティカメラ（Pepper）。何台でも置ける。1台ごとに securityCamera.js の
    // defaultSecurityCamera() の形（direction / swingAngle は度）。
    securityCameras: [],
  };
}

export function loadStage(stage) {
  return JSON.parse(JSON.stringify(stage));
}
