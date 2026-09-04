// エディタ画面のCanvas描画。ゲーム本体のrender.jsとは別に、
// 巡回ルートの点・順序・壁との衝突警告を描く必要があるため専用に用意する。

export function renderEditor(ctx, stage, validation, wallDraft) {
  ctx.clearRect(0, 0, stage.width, stage.height);
  ctx.fillStyle = "#1b1f24";
  ctx.fillRect(0, 0, stage.width, stage.height);

  drawWalls(ctx, stage.walls);
  if (wallDraft) drawWallDraft(ctx, wallDraft);
  drawPatrolPath(ctx, stage.guard.patrolPath, validation.blockedLegs);
  drawMarker(ctx, stage.start, "#3ddc84", "START", validation.stuckPoints.includes("START"));
  drawMarker(ctx, stage.target, "#f4d35e", "TARGET", validation.stuckPoints.includes("TARGET"));
  drawGuard(ctx, stage.guard, validation.stuckPoints.includes("GUARD"));
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
    ctx.font = "10px monospace";
    ctx.textAlign = "left";
    ctx.fillText(String(i), p.x + 6, p.y - 6);
  });
}

function drawMarker(ctx, pos, color, label, stuck) {
  ctx.fillStyle = stuck ? "#e63946" : color;
  ctx.beginPath();
  ctx.arc(pos.x, pos.y, 12, 0, Math.PI * 2);
  ctx.fill();
  ctx.fillStyle = "#e8eaed";
  ctx.font = "12px monospace";
  ctx.textAlign = "center";
  ctx.fillText(label, pos.x, pos.y - 18);
}

function drawGuard(ctx, guard, stuck) {
  ctx.fillStyle = stuck ? "#ff8fa3" : "#e63946";
  ctx.beginPath();
  ctx.arc(guard.x, guard.y, 10, 0, Math.PI * 2);
  ctx.fill();
  ctx.fillStyle = "#e8eaed";
  ctx.font = "12px monospace";
  ctx.textAlign = "center";
  ctx.fillText("GUARD", guard.x, guard.y - 18);
}
