// 骨格（MediaPipe Pose の worldLandmarks。単位 m、x=画像の右、y=下、z=奥）から、腕の角度（度）を出す純粋な計算。
// 角度の意味は skeleton.py の冒頭を参照。向きの正負はロボットとの対応に関係しない（較正の直線が符号も決める）。
// 隠れている・向きが定まらない角度は、結果に入れない（サーバーはその関節を今の姿勢のまま保つ）。
export const ARMS = {
  right: {shoulder: 12, elbow: 14, wrist: 16, pinky: 18, index: 20},   // 画像では左側に写る、人の右腕
  left: {shoulder: 11, elbow: 13, wrist: 15, pinky: 17, index: 19},
};
export const MIN_VISIBILITY = 0.5;    // 肩・肘・手首に必要な見えやすさ
export const MIN_HAND_VISIBILITY = 0.3;
export const MIN_SEGMENT = 0.03;      // これより短い部位（m）は、向きを決められない
export const MIN_HORIZONTAL = 0.35;   // 上腕がほぼ真上・真下だと、横の向き（pan）は決まらない（水平成分の割合）
export const MIN_ROLL_PERP = 0.35;    // 前腕がほぼ鉛直だと、前腕まわりの回転（roll）は決まらない
const DOWN = {x: 0, y: 1, z: 0};
const RAD = 180/Math.PI;

const sub = (a, b) => ({x: a.x-b.x, y: a.y-b.y, z: a.z-b.z});
const dot = (a, b) => a.x*b.x + a.y*b.y + a.z*b.z;
const cross = (a, b) => ({x: a.y*b.z - a.z*b.y, y: a.z*b.x - a.x*b.z, z: a.x*b.y - a.y*b.x});
const scale = (a, k) => ({x: a.x*k, y: a.y*k, z: a.z*k});
const norm = a => Math.hypot(a.x, a.y, a.z);
const unit = a => scale(a, 1/norm(a));
const mid = (a, b) => scale({x: a.x+b.x, y: a.y+b.y, z: a.z+b.z}, 0.5);
const angleBetween = (a, b) => Math.acos(Math.max(-1, Math.min(1, dot(a, b)/(norm(a)*norm(b))))) * RAD;

/** 角度の差を -180〜180 にする。 */
export const wrap = degrees => ((degrees + 180) % 360 + 360) % 360 - 180;

/** 前回の値へ、今回の値を割合 alpha だけ近づける（角度の折り返しをまたいでも正しい）。前回が無ければ今回の値。 */
export const smooth = (previous, next, alpha) => previous === undefined ? next : wrap(previous + alpha*wrap(next - previous));

/**
 * 腕の角度を返す。world は worldLandmarks[0]（33点）、visibility は同じ番号の見えやすさ（0〜1）の配列、side は "left" か "right"。
 * 肩・肘・手首が見えなければ空。手の点（人差し指・小指）が見えなければ flex と roll は入らない。
 */
export function armFeatures(world, visibility, side) {
  const ids = ARMS[side];
  const seen = (id, min) => (visibility[id] ?? 0) >= min && world[id];
  if (!seen(ids.shoulder, MIN_VISIBILITY) || !seen(ids.elbow, MIN_VISIBILITY) || !seen(ids.wrist, MIN_VISIBILITY)) return {};
  const shoulder = world[ids.shoulder], elbow = world[ids.elbow], wrist = world[ids.wrist];
  const upper = sub(elbow, shoulder), fore = sub(wrist, elbow);
  if (norm(upper) < MIN_SEGMENT || norm(fore) < MIN_SEGMENT) return {};
  const result = {lift: angleBetween(upper, DOWN), elbow: angleBetween(sub(shoulder, elbow), fore)};
  if (Math.hypot(upper.x, upper.z) >= MIN_HORIZONTAL*norm(upper)) result.pan = Math.atan2(upper.x, -upper.z) * RAD;
  if (seen(ids.index, MIN_HAND_VISIBILITY) && seen(ids.pinky, MIN_HAND_VISIBILITY)) {
    const index = sub(world[ids.index], wrist), pinky = sub(world[ids.pinky], wrist);
    const hand = sub(mid(world[ids.index], world[ids.pinky]), wrist);
    const palm = cross(index, pinky);                 // 手のひらの法線（向きは左右の腕で逆になるが、較正が吸収する）
    if (norm(hand) >= MIN_SEGMENT/3 && norm(palm) > 0) {
      const f = unit(fore), p = unit(palm);
      const across = unit(sub(world[ids.pinky], world[ids.index]));       // 手のひらを横切る向き（手首の曲げの回転軸）
      result.flex = Math.atan2(dot(across, cross(f, hand)), dot(f, hand)) * RAD;
      const gravity = sub(DOWN, scale(f, dot(DOWN, f)));         // 前腕に垂直な面へ映した「真下」
      if (norm(gravity) >= MIN_ROLL_PERP) {
        const palmPerp = sub(p, scale(f, dot(p, f)));
        result.roll = Math.atan2(dot(f, cross(gravity, palmPerp)), dot(gravity, palmPerp)) * RAD;
      }
    }
  }
  return result;
}

/**
 * 静止した姿勢の撮影用に、複数フレームの角度をまとめる。各角度は、全フレームの minShare 以上に入っていて、
 * ばらつき（最大-最小）が maxSpread 度以内のものだけ平均する。戻り値は {angles, unstable}（unstable はばらついた角度の名前）。
 */
export function steadyAngles(frames, {minShare = 0.8, maxSpread = 8} = {}) {
  const angles = {}, unstable = [];
  const keys = new Set(frames.flatMap(f => Object.keys(f)));
  for (const key of keys) {
    const values = frames.filter(f => key in f).map(f => f[key]);
    if (values.length < frames.length*minShare) continue;
    const offsets = values.map(v => wrap(v - values[0]));
    if (Math.max(...offsets) - Math.min(...offsets) > maxSpread) { unstable.push(key); continue; }
    angles[key] = wrap(values[0] + offsets.reduce((a, b) => a + b, 0)/offsets.length);
  }
  return {angles, unstable};
}
