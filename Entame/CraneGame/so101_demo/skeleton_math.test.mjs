// 実行: node --test skeleton_math.test.mjs
import test from "node:test";
import assert from "node:assert/strict";
import {armFeatures, smooth, steadyAngles, wrap} from "./skeleton_math.js";

const near = (actual, expected, tolerance = 0.5) => assert.ok(Math.abs(wrap(actual - expected)) <= tolerance, `${actual} ≠ ${expected}`);

// 右腕（肩12・肘14・手首16・小指18・人差し指20）の点を作る。z は奥が＋（カメラへ向かうと－）。
function arm({shoulder = [0, 0, 0], elbow, wrist, index, pinky, visible = 1}) {
  const world = Array.from({length: 33}, () => ({x: 0, y: 0, z: 0}));
  const put = (id, [x, y, z]) => { world[id] = {x, y, z}; };
  put(12, shoulder); put(14, elbow); put(16, wrist); put(20, index); put(18, pinky);
  return {world, visibility: Array(33).fill(visible)};
}
const features = (spec, side = "right") => { const {world, visibility} = arm(spec); return armFeatures(world, visibility, side); };

// 前腕が前（カメラ側＝z がマイナス）へ水平、手のひらの法線が真下を向く。
const forward = {elbow: [0, 0.3, 0], wrist: [0, 0.3, -0.25], index: [0.03, 0.3, -0.33], pinky: [-0.03, 0.3, -0.31]};

test("wrap は半回転に収める", () => {
  near(wrap(190), -170, 1e-9); near(wrap(-190), 170, 1e-9); near(wrap(360), 0, 1e-9);
});

test("腕を下ろすと lift 0、水平だと 90、真上だと 180", () => {
  near(features({elbow: [0, 0.3, 0], wrist: [0, 0.55, 0], index: [0, 0.6, 0], pinky: [0.02, 0.6, 0]}).lift, 0);
  near(features({elbow: [0, 0, -0.3], wrist: [0, 0, -0.55], index: [0, 0, -0.6], pinky: [0.02, 0, -0.6]}).lift, 90);
  near(features({elbow: [0, -0.3, 0], wrist: [0, -0.55, 0], index: [0, -0.6, 0], pinky: [0.02, -0.6, 0]}).lift, 180);
});

test("pan は上腕の水平の向き。前が 0、画像の右が 90。真下では出ない", () => {
  near(features({elbow: [0, 0, -0.3], wrist: [0, 0, -0.55], index: [0, 0, -0.6], pinky: [0.02, 0, -0.6]}).pan, 0);
  near(features({elbow: [0.3, 0, 0], wrist: [0.55, 0, 0], index: [0.6, 0, 0], pinky: [0.6, 0, 0.02]}).pan, 90);
  assert.equal("pan" in features({elbow: [0, 0.3, 0], wrist: [0, 0.55, 0], index: [0, 0.6, 0], pinky: [0.02, 0.6, 0]}), false);
});

test("肘は伸ばすと 180、直角だと 90", () => {
  near(features({elbow: [0, 0, -0.3], wrist: [0, 0, -0.55], index: [0, 0, -0.6], pinky: [0.02, 0, -0.6]}).elbow, 180);
  near(features({elbow: [0, 0, -0.3], wrist: [0, -0.25, -0.3], index: [0, -0.3, -0.3], pinky: [0.02, -0.3, -0.3]}).elbow, 90);
});

test("手首が真っ直ぐなら flex 0、曲げる向きで符号が変わる", () => {
  near(features(forward).flex, 0, 1);
  const bend = (dy) => features({...forward, index: [0.03, 0.3 + dy, -0.33], pinky: [-0.03, 0.3 + dy, -0.31]}).flex;
  assert.ok(Math.abs(bend(0.08)) > 10 && Math.abs(bend(-0.08)) > 10);
  assert.ok(bend(0.08) * bend(-0.08) < 0, "曲げる向きで符号が変わる");
});

test("roll は前腕まわりの手のひらの回転。真下向きが 0、横向きで ±90", () => {
  near(features(forward).roll, 0, 1);
  const rolled = features({...forward, index: [0.0, 0.27, -0.33], pinky: [0.0, 0.33, -0.31]});   // 手のひらの向きが横へ
  assert.ok(Math.abs(Math.abs(rolled.roll) - 90) < 15, `roll=${rolled.roll}`);
});

test("前腕がほぼ鉛直だと roll は出ない", () => {
  const vertical = features({elbow: [0, 0, -0.3], wrist: [0, -0.25, -0.3], index: [0, -0.3, -0.3], pinky: [0.02, -0.3, -0.3]});
  assert.equal("roll" in vertical, false);
  assert.ok("flex" in vertical);
});

test("見えない肩・肘・手首なら何も返さない。手の点が見えないと flex と roll は出ない", () => {
  const hidden = arm(forward); hidden.visibility[14] = 0.1;
  assert.deepEqual(armFeatures(hidden.world, hidden.visibility, "right"), {});
  const noHand = arm(forward); noHand.visibility[20] = 0;
  const result = armFeatures(noHand.world, noHand.visibility, "right");
  assert.ok("lift" in result && "elbow" in result);
  assert.equal("flex" in result || "roll" in result, false);
});

test("左腕は左の番号を使う", () => {
  const {world, visibility} = arm(forward);
  assert.deepEqual(armFeatures(world, visibility, "left"), {});     // 11・13・15 は原点のまま（長さ 0）
});

test("smooth は折り返しをまたいで近づく", () => {
  near(smooth(170, -170, 0.5), 180, 1e-9);
  near(smooth(undefined, 12, 0.5), 12, 1e-9);
  near(smooth(0, 10, 0.4), 4, 1e-9);
});

test("steadyAngles は静止した角度だけ平均し、ばらつく角度は除く", () => {
  const frames = Array.from({length: 10}, (_, i) => ({lift: 50 + (i % 2), elbow: 100 + i*5, ...(i < 3 ? {roll: 10} : {})}));
  const {angles, unstable} = steadyAngles(frames);
  near(angles.lift, 50.5, 1e-9);
  assert.deepEqual(unstable, ["elbow"]);
  assert.equal("roll" in angles, false);                            // 写った割合が少ない
});

test("steadyAngles は折り返しをまたいだ平均も正しい", () => {
  const {angles} = steadyAngles([{pan: 178}, {pan: -178}, {pan: 180}, {pan: -180}, {pan: 179}]);
  assert.ok(Math.abs(Math.abs(angles.pan) - 180) < 3);
});
