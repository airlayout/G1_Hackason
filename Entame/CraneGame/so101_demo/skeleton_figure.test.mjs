// 実行: node --test skeleton_figure.test.mjs
// 図の腕の点が、skeleton_math.js の角度の定義と食い違わないこと（図の通りにポーズをとれば、その角度が測れる）を確かめる。
import test from "node:test";
import assert from "node:assert/strict";
import {armFeatures, wrap} from "./skeleton_math.js";
import {armPoints} from "./skeleton_figure.js";

const POSES = [
  {pan: 0, lift: 40, elbow: 90, flex: 0},
  {pan: 30, lift: 75, elbow: 125, flex: 25},
  {pan: -30, lift: 25, elbow: 55, flex: -25},
  {pan: 0, lift: 90, elbow: 90, flex: 0},          // 腕が真っ直ぐ前（曲げる向きが特別なとき）
];

function measure(angles) {
  const points = armPoints(angles), world = Array.from({length: 33}, () => ({x: 0, y: 0, z: 0}));
  world[12] = points.shoulder; world[14] = points.elbow; world[16] = points.wrist;
  return armFeatures(world, Array(33).fill(1).map((v, i) => (i === 18 || i === 20 ? 0 : v)), "right");   // 手の点は与えない
}

test("図の腕から測った lift・elbow・pan は、指定した角度と一致する", () => {
  for (const pose of POSES) {
    const measured = measure(pose);
    assert.ok(Math.abs(measured.lift - pose.lift) < 0.5, `lift ${measured.lift} ≠ ${pose.lift}`);
    assert.ok(Math.abs(measured.elbow - pose.elbow) < 0.5, `elbow ${measured.elbow} ≠ ${pose.elbow}`);
    if (Math.sin(pose.lift*Math.PI/180) >= 0.4) assert.ok(Math.abs(wrap(measured.pan - pose.pan)) < 0.5, `pan ${measured.pan} ≠ ${pose.pan}`);
  }
});

test("肘は前（カメラ側＝z がマイナス）へ曲がる", () => {
  const points = armPoints({pan: 0, lift: 0, elbow: 90, flex: 0});     // 上腕が真下
  assert.ok(points.wrist.z < points.elbow.z - 0.1);
});

test("flex を正にすると、手は肘と同じ向きへ曲がる（前腕に対する角度が flex）", () => {
  const points = armPoints({pan: 0, lift: 0, elbow: 90, flex: 30});
  const fore = [points.wrist.x - points.elbow.x, points.wrist.y - points.elbow.y, points.wrist.z - points.elbow.z];
  const hand = [points.hand.x - points.wrist.x, points.hand.y - points.wrist.y, points.hand.z - points.wrist.z];
  const cosine = fore.reduce((s, v, i) => s + v*hand[i], 0) / (Math.hypot(...fore)*Math.hypot(...hand));
  assert.ok(Math.abs(Math.acos(cosine)*180/Math.PI - 30) < 0.5);
});
