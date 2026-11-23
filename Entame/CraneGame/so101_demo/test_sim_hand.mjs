// sim_hand.js の試験。実行: node --test test_sim_hand.mjs
import test from "node:test";
import assert from "node:assert/strict";
import {
  handFeatures, featureUnits, unitsToCommand, openingFromRatio, deadZone, OneEuro, UnitSmoother,
  FALLBACK_FINGER, EDGE_CLOSE, REACH_FAR, REACH_NEAR, retryBox, mapFromBox, RETRY_BOX_MIN_PX,
} from "./sim_hand.js";

const W = 640, H = 480;
const RANGES = { x: [0.2, 0.3], height: [0, 0.12], pan: [-45, 45], roll: [-60, 60], opening: [0, 1] };

/** 手首を (cx, cy)・手首→中指の付け根を (dx, dy)（画面の 0〜1。元の映像のまま）、人差し指〜小指の付け根の幅を widthN（画面幅の割合）にした 21 点。 */
function hand({ cx = 0.5, cy = 0.5, dx = 0, dy = -0.2, widthN = 0.125, pinchPx = 100 } = {}) {
  const pts = Array.from({ length: 21 }, () => ({ x: cx, y: cy }));
  pts[9] = { x: cx + dx, y: cy + dy };
  pts[5] = { x: cx + dx - widthN / 2, y: cy + dy };
  pts[17] = { x: cx + dx + widthN / 2, y: cy + dy };
  pts[4] = { x: cx - pinchPx / 2 / W, y: cy };
  pts[8] = { x: cx + pinchPx / 2 / W, y: cy };
  return pts;
}
/** 3 次元の手（メートル）。手首〜中指の付け根を 0.09、親指〜人差し指の先を gap にする。 */
function world(gap) {
  const pts = Array.from({ length: 21 }, () => ({ x: 0, y: 0, z: 0 }));
  pts[9] = { x: 0, y: -0.09, z: 0 };
  pts[4] = { x: -gap / 2, y: 0, z: 0 };
  pts[8] = { x: gap / 2, y: 0, z: 0 };
  return pts;
}
const units = (opts, finger, roll) => featureUnits(handFeatures(hand(opts), null, W, H), finger, roll);

test("手を元の映像の右（＝鏡の左）へ動かすと pan は＋、左へ動かすと－。中央は 0", () => {
  assert.ok(units({ cx: 0.8 }).pan > 0.9);
  assert.ok(units({ cx: 0.2 }).pan < -0.9);
  assert.equal(units({ cx: 0.5 }).pan, 0);
});

test("手首を上げると height は＋、下げると－。中央は 0", () => {
  assert.ok(units({ cy: 0.2 }).height > 0.9);
  assert.ok(units({ cy: 0.8 }).height < -0.9);
  assert.equal(units({ cy: 0.5 }).height, 0);
});

test("手のひらが大きく映れば reach は＋（前へ）、小さければ－。範囲の中ほどは 0", () => {
  assert.ok(units({ widthN: REACH_NEAR * 1.2 }).reach > 0.9);
  assert.ok(units({ widthN: REACH_FAR * 0.8 }).reach < -0.9);
  assert.equal(units({ widthN: (REACH_FAR + REACH_NEAR) / 2 }).reach, 0);
});

test("roll は既定でオフ。オンなら、手を画面の右へ傾けると＋", () => {
  const tilted = { dx: -0.1, dy: -0.2 };       // 元の映像では中指が左 ＝ 鏡では右へ傾く
  assert.equal(units(tilted).roll, 0);
  assert.ok(units(tilted, FALLBACK_FINGER, true).roll > 0.1);
  assert.ok(units({}, FALLBACK_FINGER, true).roll === 0);
});

test("指の開きは 3 次元の距離比。画面上の比と食い違っても 3 次元に従う", () => {
  const closedWorld = world(0.062), openWorld = world(0.12);      // 比 0.69（閉じた）と 1.33（開いた）
  const pts = hand({ pinchPx: 300 });                              // 画面上は大きく離れて見える
  const f = handFeatures(pts, closedWorld, W, H);
  assert.ok(f.have3d);
  assert.ok(Math.abs(f.ratio - 0.062 / 0.09) < 1e-9);
  assert.ok(f.ratio2d > 1.5);
  assert.equal(featureUnits(f).opening, 0);
  assert.equal(featureUnits(handFeatures(pts, openWorld, W, H)).opening, 1);
  assert.equal(handFeatures(hand(), null, W, H).have3d, false);    // 3 次元が無ければ画面上の比
});

test("実測した閉じ（比 0.62）で全閉、軽くつまむだけでも閉じる。開き（比 1.2）で全開。中間は直線", () => {
  const { open, closed } = FALLBACK_FINGER;
  assert.equal(openingFromRatio(closed), 0);
  assert.equal(openingFromRatio(closed + (open - closed) * EDGE_CLOSE * 0.9), 0);
  assert.equal(openingFromRatio(open), 1);
  assert.equal(openingFromRatio(open * 1.3), 1);
  const mid = openingFromRatio((open + closed) / 2);
  assert.ok(Math.abs(mid - 0.5) < 1e-9);
  assert.equal(openingFromRatio(0.5, { open: 1, closed: 0 }), 0.5);       // 渡した校正を使う
});

test("不感帯: 真ん中付近は 0。外は 0〜±1 へ伸ばし直す", () => {
  assert.equal(deadZone(0.03, 0.05), 0);
  assert.equal(deadZone(-0.05, 0.05), 0);
  assert.equal(deadZone(1, 0.05), 1);
  assert.equal(deadZone(-1, 0.05), -1);
  assert.ok(deadZone(0.5, 0.05) > 0 && deadZone(0.5, 0.05) < 0.5);
});

test("操作量は範囲の中に収まり、中央は範囲の真ん中になる", () => {
  const mid = unitsToCommand({ pan: 0, height: 0, reach: 0, roll: 0, opening: 0.5 }, RANGES);
  assert.deepEqual([mid.x, mid.height, mid.pan, mid.roll, mid.opening], [0.25, 0.06, 0, 0, 0.5]);
  const top = unitsToCommand({ pan: 1, height: 1, reach: 1, roll: 1, opening: 1 }, RANGES);
  assert.deepEqual([top.x, top.height, top.pan, top.roll, top.opening], [0.3, 0.12, 45, 60, 1]);
  const bottom = unitsToCommand({ pan: -1, height: -1, reach: -1, roll: -1, opening: 0 }, RANGES);
  assert.deepEqual([bottom.x, bottom.height, bottom.pan, bottom.roll, bottom.opening], [0.2, 0, -45, -60, 0]);
});

test("手首と中指の付け根が近すぎる（12 px 未満）ときは null", () => {
  assert.equal(handFeatures(hand({ dx: 0, dy: 0 }), null, W, H), null);
  assert.equal(handFeatures(hand({ dy: -0.02 }), null, W, H), null);
});

test("One Euro: 最初の値はそのまま。一定なら動かず、ステップには追いつく", () => {
  const f = new OneEuro();
  assert.equal(f.step(0.5, 1 / 30), 0.5);
  for (let i = 0; i < 30; i++) assert.equal(f.step(0.5, 1 / 30), 0.5);
  let v = 0;
  for (let i = 0; i < 90; i++) v = f.step(1, 1 / 30);
  assert.ok(v > 0.99);
  f.reset();
  assert.equal(f.step(-1, 1 / 30), -1);
});

test("UnitSmoother: 名前を保つ。reset で取り直す。前後は強く、開きは速くならす", () => {
  const s = new UnitSmoother();
  s.step({ pan: 1, reach: 0, opening: 0 }, 1 / 30);
  const out = s.step({ pan: 0, reach: 0, opening: 0 }, 1 / 30);
  assert.deepEqual(Object.keys(out), ["pan", "reach", "opening"]);
  assert.ok(out.pan > 0 && out.pan < 1);
  s.reset();
  assert.equal(s.step({ pan: -1, reach: 0, opening: 0 }, 1 / 30).pan, -1);
  const fresh = new UnitSmoother();
  fresh.step({ reach: 0, opening: 0 }, 1 / 30);
  const step = fresh.step({ reach: 1, opening: 1 }, 1 / 30);
  assert.ok(step.opening > step.reach, "開きのほうが速く追いつく");
});

test("再検出の切り出し: 前の手がなければ中央。あれば手の周りで、下限以上・画面の内側", () => {
  const center = retryBox(null, W, H);
  assert.equal(center.size, H * 0.75);
  assert.equal(center.x + center.size / 2, W / 2);
  const small = retryBox(hand({ cx: 0.5, cy: 0.5, dx: 0.02, dy: -0.02, widthN: 0.05, pinchPx: 10 }), W, H);
  assert.equal(small.size, RETRY_BOX_MIN_PX);                    // 手が小さくても下限まで広げる
  const corner = retryBox(hand({ cx: 0.98, cy: 0.97, dx: 0, dy: -0.05 }), W, H);
  assert.ok(corner.x >= 0 && corner.y >= 0 && corner.x + corner.size <= W && corner.y + corner.size <= H, "画面の内側に収める");
  const huge = retryBox(hand({ cx: 0.5, cy: 0.5, dx: 0, dy: -0.45, widthN: 0.9 }), W, H);
  assert.ok(huge.size <= H, "画面の短い辺を超えない");
});

test("再検出: 切り出した画像の点を元の画面の座標へ戻す（往復で元に戻る）", () => {
  const original = hand({ cx: 0.4, cy: 0.6, dx: 0.03, dy: -0.06 });
  const box = retryBox(original, W, H);
  const inBox = original.map((p) => ({ x: (p.x * W - box.x) / box.size, y: (p.y * H - box.y) / box.size, z: 0.1 }));
  const back = mapFromBox(inBox, box, W, H);
  back.forEach((p, i) => { assert.ok(Math.abs(p.x - original[i].x) < 1e-9 && Math.abs(p.y - original[i].y) < 1e-9); });
  assert.ok(Math.abs(back[0].z - 0.1 * box.size / W) < 1e-9);
});
