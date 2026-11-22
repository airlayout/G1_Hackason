// エディタ画面のCanvas描画。ゲーム本体のrender.jsとは別に、
// 巡回ルートの点・順序・壁との衝突警告を描く必要があるため専用に用意する。

import { CANVAS_FONT, labelY } from "./canvasText.js";
import { defaultSecurityCamera } from "./securityCamera.js";

const DEG_TO_RAD = Math.PI / 180;
const CAMERA_BODY_HALF = 9;

// selectedCamera: 編集中（Cam Direction / Swing の対象）のカメラの番号。無ければ -1。
export function renderEditor(ctx, stage, validation, wallDraft, selectedCamera = -1) {
  ctx.clearRect(0, 0, stage.width, stage.height);
  ctx.fillStyle = "#1b1f24";
  ctx.fillRect(0, 0, stage.width, stage.height);

  drawWalls(ctx, stage.walls);
  if (wallDraft) drawWallDraft(ctx, wallDraft);
  drawPatrolPath(ctx, stage.guard.patrolPath, validation.blockedLegs);
  drawMarker(ctx, stage.start, "#3ddc84", "START", validation.stuckPoints.includes("START"));
  drawMarker(ctx, stage.target, "#f4d35e", "TARGET", validation.stuckPoints.includes("TARGET"));
  drawGuard(ctx, stage.guard, validation.stuckPoints.includes("GUARD"));
  stage.securityCameras.forEach((camera, i) => {
    const label = `CAMERA ${i + 1}`;
    drawSecurityCamera(ctx, camera, validation.stuckPoints.includes(label), label, i === selectedCamera);
  });
}

// ミニロボットは START から人に同行して出発する。START の少し右下に置いて見せる。
function drawMiniRobotAtStart(ctx, start) {
  const x = start.x + 24;
  const y = start.y + 24;
  ctx.fillStyle = MINI_ROBOT_COLOR;
  ctx.fillRect(x - MINI_ROBOT_RADIUS, y - MINI_ROBOT_RADIUS, MINI_ROBOT_RADIUS * 2, MINI_ROBOT_RADIUS * 2);
  ctx.fillStyle = "#e8eaed";
  ctx.font = CANVAS_FONT;
  ctx.textAlign = "center";
  ctx.textBaseline = "middle";
  ctx.fillText("MINI", x, labelY(y, MINI_ROBOT_RADIUS));
}

// 首を振って見渡せる範囲（振り幅 + 視野角）を扇形で薄く塗り、中心の向きを線で示す。
function drawSecurityCamera(ctx, camera, stuck, label, selected) {
  const cam = { ...defaultSecurityCamera(camera.x, camera.y), ...camera };
  const direction = cam.direction * DEG_TO_RAD;
  const halfCoverage = ((cam.swingAngle + cam.visionAngle) / 2) * DEG_TO_RAD;

  ctx.fillStyle = "rgba(201, 168, 255, 0.18)";
  ctx.beginPath();
  ctx.moveTo(cam.x, cam.y);
  ctx.arc(cam.x, cam.y, cam.visionDistance, direction - halfCoverage, direction + halfCoverage);
  ctx.closePath();
  ctx.fill();

  ctx.strokeStyle = "#c9a8ff";
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.moveTo(cam.x, cam.y);
  ctx.lineTo(cam.x + Math.cos(direction) * cam.visionDistance, cam.y + Math.sin(direction) * cam.visionDistance);
  ctx.stroke();

  ctx.fillStyle = stuck ? "#e63946" : "#c9a8ff";
  ctx.fillRect(cam.x - CAMERA_BODY_HALF, cam.y - CAMERA_BODY_HALF, CAMERA_BODY_HALF * 2, CAMERA_BODY_HALF * 2);

  if (selected) {
    ctx.strokeStyle = "#ffffff";
    ctx.lineWidth = 3;
    ctx.strokeRect(cam.x - CAMERA_BODY_HALF - 4, cam.y - CAMERA_BODY_HALF - 4, CAMERA_BODY_HALF * 2 + 8, CAMERA_BODY_HALF * 2 + 8);
  }

  ctx.fillStyle = "#e8eaed";
  ctx.font = CANVAS_FONT;
  ctx.textAlign = "center";
  ctx.textBaseline = "middle";
  ctx.fillText(label, cam.x, labelY(cam.y, CAMERA_BODY_HALF));
}

function drawWalls(ctx, walls) {
  ctx.fillStyle = "#555c66";
  for (const wall of walls) ctx.fillRect(wall.x, wall.y, wall.w, wall.h);
}

function drawWallDraft(ctx, draft) {
  ctx.strokeStyle = "#4cc9f0";
  ctx.setLineDash([4, 4]);
  ctx.strokeRect(draft.x, draft.y, draft.w, draft.h);
  ctx.setLineDash([]);
}

function drawPatrolPath(ctx, path, blockedLegs) {
  if (path.length >= 2) {
    for (let i = 0; i < path.length; i++) {
      const a = path[i];
      const b = path[(i + 1) % path.length];
      const isBlocked = blockedLegs.includes(i);
      ctx.strokeStyle = isBlocked ? "#e63946" : "#8899aa";
      ctx.lineWidth = isBlocked ? 3 : 1.5;
      ctx.beginPath();
      ctx.moveTo(a.x, a.y);
      ctx.lineTo(b.x, b.y);
      ctx.stroke();
    }
  }

  path.forEach((p, i) => {
    ctx.fillStyle = "#c9a8ff";
    ctx.beginPath();
    ctx.arc(p.x, p.y, 5, 0, Math.PI * 2);
    ctx.fill();
    ctx.fillStyle = "#e8eaed";
    ctx.font = CANVAS_FONT;
    ctx.textAlign = "left";
    ctx.textBaseline = "alphabetic";
    ctx.fillText(String(i), p.x + 8, p.y - 8);
  });
}

function drawMarker(ctx, pos, color, label, stuck) {
  ctx.fillStyle = stuck ? "#e63946" : color;
  ctx.beginPath();
  ctx.arc(pos.x, pos.y, 12, 0, Math.PI * 2);
  ctx.fill();
  ctx.fillStyle = "#e8eaed";
  ctx.font = CANVAS_FONT;
  ctx.textAlign = "center";
  ctx.textBaseline = "middle";
  ctx.fillText(label, pos.x, labelY(pos.y, 12));
}

function drawGuard(ctx, guard, stuck) {
  ctx.fillStyle = stuck ? "#ff8fa3" : "#e63946";
  ctx.beginPath();
  ctx.arc(guard.x, guard.y, 10, 0, Math.PI * 2);
  ctx.fill();
  ctx.fillStyle = "#e8eaed";
  ctx.font = CANVAS_FONT;
  ctx.textAlign = "center";
  ctx.textBaseline = "middle";
  ctx.fillText("GUARD", guard.x, labelY(guard.y, 10));
}
