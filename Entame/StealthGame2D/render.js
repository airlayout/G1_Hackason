// Canvas描画。ゲーム状態を受け取って毎フレーム描画するだけの純粋な関数群。

import { computeVisionPolygon } from "./guard.js";
import { MINI_ROBOT_COLOR, MODE_LABEL } from "./miniRobot.js";
import { CANVAS_FONT, CANVAS_FONT_PX, labelY } from "./canvasText.js";

export function renderStage(ctx, stage, player, guard, stageLabel, securityCameras = [], miniRobot = null) {
  ctx.clearRect(0, 0, stage.width, stage.height);

  ctx.fillStyle = "#1b1f24";
  ctx.fillRect(0, 0, stage.width, stage.height);

  drawVisionCone(ctx, guard, stage.walls);
  for (const camera of securityCameras) drawVisionCone(ctx, camera, stage.walls);

  ctx.fillStyle = "#555c66";
  for (const wall of stage.walls) {
    ctx.fillRect(wall.x, wall.y, wall.w, wall.h);
  }

  drawMarker(ctx, stage.start, "#3ddc84", "START");
  if (!player.hasTarget) {
    drawMarker(ctx, stage.target, "#f4d35e", "TARGET");
  }

  drawGuard(ctx, guard);
  for (const camera of securityCameras) drawSecurityCamera(ctx, camera);
  if (miniRobot) drawMiniRobot(ctx, miniRobot);
  drawPlayer(ctx, player);

  drawHud(ctx, stageLabel, player);
}

const CAMERA_BODY_HALF = 9;

function drawSecurityCamera(ctx, camera) {
  ctx.fillStyle = "#c9a8ff";
  ctx.fillRect(camera.x - CAMERA_BODY_HALF, camera.y - CAMERA_BODY_HALF, CAMERA_BODY_HALF * 2, CAMERA_BODY_HALF * 2);

  ctx.strokeStyle = "#ffffff";
  ctx.lineWidth = 3;
  ctx.beginPath();
  ctx.moveTo(camera.x, camera.y);
  ctx.lineTo(camera.x + Math.cos(camera.facingAngle) * 20, camera.y + Math.sin(camera.facingAngle) * 20);
  ctx.stroke();

  ctx.fillStyle = "#e8eaed";
  ctx.font = CANVAS_FONT;
  ctx.textAlign = "center";
  ctx.textBaseline = "middle";
  ctx.fillText("Camera", camera.x, labelY(camera.y, CAMERA_BODY_HALF));
}

function drawGuard(ctx, guard) {
  ctx.fillStyle = "#e63946";
  ctx.beginPath();
  ctx.arc(guard.x, guard.y, 10, 0, Math.PI * 2);
  ctx.fill();

  // 向いている方向を示す小さな三角形（プレイヤーの丸との見分け＋方向転換の視認用）
  const noseLength = 16;
  const noseWidth = 6;
  const tipX = guard.x + Math.cos(guard.facingAngle) * noseLength;
  const tipY = guard.y + Math.sin(guard.facingAngle) * noseLength;
  const perpAngle = guard.facingAngle + Math.PI / 2;
  ctx.fillStyle = "#ffffff";
  ctx.beginPath();
  ctx.moveTo(tipX, tipY);
  ctx.lineTo(guard.x + Math.cos(perpAngle) * noseWidth, guard.y + Math.sin(perpAngle) * noseWidth);
  ctx.lineTo(guard.x - Math.cos(perpAngle) * noseWidth, guard.y - Math.sin(perpAngle) * noseWidth);
  ctx.closePath();
  ctx.fill();

  ctx.fillStyle = "#e8eaed";
  ctx.font = CANVAS_FONT;
  ctx.textAlign = "center";
  ctx.textBaseline = "middle";
  ctx.fillText("Guard", guard.x, labelY(guard.y, guard.radius));
}

function drawMiniRobot(ctx, robot) {
  ctx.fillStyle = MINI_ROBOT_COLOR;
  ctx.fillRect(robot.x - robot.radius, robot.y - robot.radius, robot.radius * 2, robot.radius * 2);

  ctx.fillStyle = "#e8eaed";
  ctx.font = CANVAS_FONT;
  ctx.textAlign = "center";
  ctx.textBaseline = "middle";
  // 今のモードを "Mini: follower" の形で出す（人を見失ったときは "Mini: search"）。
  ctx.fillText(`Mini: ${MODE_LABEL[robot.mode]}`, robot.x, labelY(robot.y, robot.radius));
}

function drawPlayer(ctx, player) {
  ctx.fillStyle = "#4cc9f0";
  ctx.beginPath();
  ctx.arc(player.x, player.y, player.radius, 0, Math.PI * 2);
  ctx.fill();

  ctx.fillStyle = "#e8eaed";
  ctx.font = CANVAS_FONT;
  ctx.textAlign = "center";
  ctx.textBaseline = "middle";
  ctx.fillText("Player", player.x, labelY(player.y, player.radius));
}

function drawMarker(ctx, pos, color, label) {
  ctx.fillStyle = color;
  ctx.beginPath();
  ctx.arc(pos.x, pos.y, 12, 0, Math.PI * 2);
  ctx.fill();
  ctx.fillStyle = "#e8eaed";
  ctx.font = CANVAS_FONT;
  ctx.textAlign = "center";
  ctx.textBaseline = "middle";
  ctx.fillText(label, pos.x, labelY(pos.y, 12));
}

const VISION_COLOR_DEFAULT = "rgba(61, 220, 132, 0.25)"; // 緑: 通常巡回
const VISION_COLOR_CAUGHT = "rgba(230, 57, 70, 0.45)"; // 赤: 検知（一発アウト）

function visionConeColor(guard) {
  return guard.caught ? VISION_COLOR_CAUGHT : VISION_COLOR_DEFAULT;
}

function drawVisionCone(ctx, guard, walls) {
  const points = computeVisionPolygon(guard, walls);
  ctx.fillStyle = visionConeColor(guard);
  ctx.beginPath();
  ctx.moveTo(points[0].x, points[0].y);
  for (let i = 1; i < points.length; i++) {
    ctx.lineTo(points[i].x, points[i].y);
  }
  ctx.closePath();
  ctx.fill();
}

function drawHud(ctx, stageLabel, player) {
  ctx.fillStyle = "#e8eaed";
  ctx.font = CANVAS_FONT;
  ctx.textAlign = "left";
  ctx.textBaseline = "alphabetic";
  ctx.fillText(stageLabel, 12, CANVAS_FONT_PX + 4);
  if (player.hasTarget) {
    ctx.fillText("TARGET GET! Return to START", 12, (CANVAS_FONT_PX + 4) * 2);
  }
}
