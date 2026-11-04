// セキュリティカメラ（Pepper の首振り監視役）。位置は固定で、向きだけを往復して振る。
// 数値の根拠は Entame/docs/pepper-detection-spec.md。★の項目は未測定の仮置き。

import { canSeePlayer } from "./guard.js";

export const CAMERA_VISION_ANGLE_DEG = 57.2; // Pepper 上カメラの水平画角
export const CAMERA_VISION_DISTANCE_PX = 240; // ★ 3m（1m = 80px）。最大視認距離は未測定
export const CAMERA_SWING_SPEED_DEG = 28.6; // ★ 0.5rad/s。首の最大速度は未測定
export const CAMERA_DEFAULT_SWING_DEG = 90;
export const CAMERA_MAX_SWING_DEG = 114.6; // HeadYaw の使用範囲 ±1.0rad の全幅
export const CAMERA_DEFAULT_DIRECTION_DEG = 90; // 0 = 右、90 = 下（Canvas の座標系）

const DEG_TO_RAD = Math.PI / 180;

// エディタで置いたときの初期値（ステージJSONの `securityCamera` と同じ形。角度は度）。
export function defaultSecurityCamera(x, y) {
  return {
    x,
    y,
    direction: CAMERA_DEFAULT_DIRECTION_DEG,
    swingAngle: CAMERA_DEFAULT_SWING_DEG,
    swingSpeed: CAMERA_SWING_SPEED_DEG,
    visionAngle: CAMERA_VISION_ANGLE_DEG,
    visionDistance: CAMERA_VISION_DISTANCE_PX,
  };
}

export function createSecurityCamera(config) {
  const full = { ...defaultSecurityCamera(config.x, config.y), ...config };
  const baseAngle = full.direction * DEG_TO_RAD;
  return {
    x: full.x,
    y: full.y,
    baseAngle,
    halfSwing: (full.swingAngle * DEG_TO_RAD) / 2,
    swingSpeed: full.swingSpeed * DEG_TO_RAD,
    visionAngle: full.visionAngle,
    visionDistance: full.visionDistance,
    swingOffset: 0,
    swingDirection: 1,
    facingAngle: baseAngle,
    caught: false,
  };
}

export function updateSecurityCamera(camera, dt, stage, player) {
  if (camera.caught) return;

  if (canSeePlayer(camera, player, stage.walls)) {
    camera.caught = true;
    return;
  }

  if (camera.halfSwing <= 0) return;
  const next = camera.swingOffset + camera.swingDirection * camera.swingSpeed * dt;
  if (Math.abs(next) >= camera.halfSwing) {
    camera.swingOffset = Math.sign(next) * camera.halfSwing;
    camera.swingDirection = -camera.swingDirection;
  } else {
    camera.swingOffset = next;
  }
  camera.facingAngle = camera.baseAngle + camera.swingOffset;
}
