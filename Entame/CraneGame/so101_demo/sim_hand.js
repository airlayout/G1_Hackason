// /sim/camera の、手の認識 → シミュレーションの指令の計算。DOM に触らない純粋な関数だけ（node --test で試せる）。
// 実機の操作画面（app.js）で、この利用者のカメラで確かめた対応（2026-11-22）をそのまま使う。基準を取る手間はない。
// 画面は鏡のように左右を反転して見せる。手を画面の左へ動かせば、腕も（台座の＋＝左へ）左へ回る。
// 手首の傾き（roll）の向きだけは、この画面では未確認（既定ではオフ）。

export const PAN_SPAN = 0.30;          // 手の付け根が画面の中央から左右へこの幅（画面幅の割合）動くと、台座の向きが端まで回る
export const HEIGHT_CENTER = 0.5;      // 手首がこの高さ（画面の上から）で、高さが真ん中
export const HEIGHT_SPAN = 0.22;       // 手首が中央から上下へこの幅（画面高さの割合）動くと、高さが端まで動く
export const REACH_FAR = 0.09;         // 手のひらの幅（人差し指の付け根〜小指の付け根。画面幅の割合）。これ以下で、腕を引ききる
export const REACH_NEAR = 0.16;        // これ以上で、腕を伸ばしきる（手をカメラへ近づけると大きく映る）
export const ROLL_SPAN_DEG = 45;       // 手首の傾きがこの角度で、手首の回転が端まで回る
export const ROLL_SIGN = 1;            // 手を画面の右へ傾けたとき、roll の＋へ回す（向きが逆なら -1）
export const EDGE_OPEN = 0.05;         // 開きが、全開に対してこの割合以内なら全開にする
export const EDGE_CLOSE = 0.25;        // 開きが、この割合以下なら全閉にする（軽くつまむだけで閉じる。アシストのつかむ判定は 0.35 以下）
export const DEAD_ZONE = 0.05;         // 操作量の真ん中付近の、この幅は 0 とみなす（手の細かい揺れで腕が動かないように）
export const FALLBACK_FINGER = { open: 1.2, closed: 0.62 };      // 指の校正（finger_calibration.json）が読めないときの値（この利用者で測った値）
export const INPUT_MIN_CUTOFF = 1.0, INPUT_BETA = 2.0;           // One Euro（ik_follow.py の手の入力と同じ値）
export const SMOOTH_PARAMS = {         // 操作量ごとの One Euro。前後は手の大きさから出すのでノイズが多く、強めにならす
  reach: { minCutoff: 0.5, beta: 0.5 },
  opening: { minCutoff: 1.5, beta: 3.0 },      // つまむ・開くは遅れると「閉じない」ように感じるので、反応を速く
};

const WRIST = 0, THUMB_TIP = 4, INDEX_MCP = 5, INDEX_TIP = 8, MIDDLE_MCP = 9, PINKY_MCP = 17;

export const clamp = (v, low, high) => Math.min(high, Math.max(low, v));
const lerp = (low, high, t) => low + (high - low) * t;

/**
 * 手の 21 点（0〜1 の画面座標。左右は元の映像のまま）、手の 3 次元の 21 点（worldLandmarks。メートル。無くてもよい）、
 * 映像の大きさ（px）から、操作に使う量を取り出す。手が小さすぎる（手首〜中指の付け根が 12 px 未満）ときは null。
 */
export function handFeatures(points, world, width, height) {
  const px = (i) => [points[i].x * width, points[i].y * height];
  const dist = (a, b) => Math.hypot(a[0] - b[0], a[1] - b[1]);
  const wrist = px(WRIST), middle = px(MIDDLE_MCP);
  const palm = dist(wrist, middle);       // 横向きでも手のひら幅ほど縮まない
  if (!(palm >= 12)) return null;
  const ratio2d = dist(px(THUMB_TIP), px(INDEX_TIP)) / palm;
  const d3 = (a, b) => Math.hypot(world[a].x - world[b].x, world[a].y - world[b].y, world[a].z - world[b].z);
  // 指の開きは 3 次元の距離比で測る。画面上の比は、腕を上げて手のひらが奥を向くと palm が縮んで膨らみ、閉じても閉じて見えない
  // （app.js の実測: palm 128→16 px で比 0.17→1.8）
  const have3d = world?.length >= 21;
  const ratio = have3d ? d3(THUMB_TIP, INDEX_TIP) / (d3(WRIST, MIDDLE_MCP) || 1) : ratio2d;
  // 鏡に映した向きに直す。手首→中指の付け根が、真上から画面の右へ何度傾いたか（右が＋）
  const tiltDeg = (Math.atan2(-(middle[0] - wrist[0]), -(middle[1] - wrist[1])) * 180) / Math.PI;
  return {
    x: points[MIDDLE_MCP].x, y: points[WRIST].y,       // 元の映像の座標のまま
    width: dist(px(INDEX_MCP), px(PINKY_MCP)) / width, palm, tiltDeg, ratio, ratio2d, have3d,
  };
}

/** 親指と人差し指の距離比から、グリッパーの開き（0 閉〜1 全開）。校正の開・閉の間を直線で結び、両端は寄せる。 */
export function openingFromRatio(ratio, finger = FALLBACK_FINGER) {
  const v = (ratio - finger.closed) / (finger.open - finger.closed);
  return v <= EDGE_CLOSE ? 0 : v >= 1 - EDGE_OPEN ? 1 : v;
}

/** 真ん中付近の幅 dead（-1〜1 の値で）を 0 にし、その外を 0〜±1 へ伸ばし直す。 */
export function deadZone(v, dead = DEAD_ZONE) {
  const m = Math.abs(v);
  return m <= dead ? 0 : Math.sign(v) * Math.min(1, (m - dead) / (1 - dead));
}

/** 特徴量を、操作量 -1〜1（opening だけ 0〜1）にする。useRoll が false なら、手首は回さない（0）。 */
export function featureUnits(f, finger = FALLBACK_FINGER, useRoll = false) {
  return {
    pan: deadZone(clamp((f.x - 0.5) / PAN_SPAN, -1, 1)),                 // 元の映像の右（＝鏡の左）→ ＋（左へ）
    height: deadZone(clamp((HEIGHT_CENTER - f.y) / HEIGHT_SPAN, -1, 1)),  // 画面の上 → ＋（上へ）
    reach: deadZone(clamp(2 * ((f.width - REACH_FAR) / (REACH_NEAR - REACH_FAR)) - 1, -1, 1)),
    roll: useRoll ? deadZone(clamp(ROLL_SIGN * f.tiltDeg / ROLL_SPAN_DEG, -1, 1)) : 0,
    opening: openingFromRatio(f.ratio, finger),
  };
}

/** 操作量を、/sim/command の set の本体にする。ranges は /sim/model の ranges（{x,height,pan,roll,opening: [低, 高]}）。 */
export function unitsToCommand(u, ranges) {
  const span = (key, t) => lerp(ranges[key][0], ranges[key][1], t);
  return {
    op: "set",
    x: span("x", (u.reach + 1) / 2),
    height: span("height", (u.height + 1) / 2),
    pan: span("pan", (u.pan + 1) / 2),
    roll: span("roll", (u.roll + 1) / 2),
    opening: span("opening", u.opening),
  };
}

/** One Euro フィルタ（smoothing.py と同じ式）。dt は前回からの秒数。 */
export class OneEuro {
  constructor(minCutoff = INPUT_MIN_CUTOFF, beta = INPUT_BETA, dCutoff = 1.0) {
    Object.assign(this, { minCutoff, beta, dCutoff, x: null, dx: 0 });
  }
  static alpha(cutoff, dt) { return 1 / (1 + 1 / (2 * Math.PI * cutoff) / dt); }
  reset() { this.x = null; this.dx = 0; }
  step(value, dt) {
    if (this.x === null || dt <= 0) { this.x = value; this.dx = 0; return value; }
    this.dx += OneEuro.alpha(this.dCutoff, dt) * ((value - this.x) / dt - this.dx);
    this.x += OneEuro.alpha(this.minCutoff + this.beta * Math.abs(this.dx), dt) * (value - this.x);
    return this.x;
  }
}

/** 操作量それぞれの One Euro をまとめたもの。params は操作量の名前 → {minCutoff, beta}（無い量は既定）。 */
export class UnitSmoother {
  constructor(params = SMOOTH_PARAMS) { this.params = params; this.filters = {}; }
  reset() { for (const f of Object.values(this.filters)) f.reset(); }
  step(units, dt) {
    const make = (k) => { const p = this.params[k]; return p ? new OneEuro(p.minCutoff, p.beta) : new OneEuro(); };
    return Object.fromEntries(Object.entries(units).map(([k, v]) => [k, (this.filters[k] ??= make(k)).step(v, dt)]));
  }
}

// --- 手が見つからないときの再検出（切り出して拡大する）
// 検出器は入力を小さく縮めて手を探すので、遠くの小さな手（画面幅の 10% ほど）は数十 px にしかならず、よく外れる。
// 外れたフレームだけ、前に手があった周り（なければ画面の中央）を切り出して拡大し、もう一度探す。
export const RETRY_BOX_SCALE = 2.6;      // 切り出す正方形の一辺 = 前の手の大きさ（外接する四角の長い辺）のこの倍
export const RETRY_BOX_MIN_PX = 200;     // 切り出す一辺の下限（px。手が動いても入るように）
export const RETRY_CENTER_RATIO = 0.75;  // 前の手がないとき、画面の短い辺のこの割合の正方形を中央から切り出す
export const RETRY_FRESH_MS = 1500;      // 前に手を見てからこの時間より経っていたら、前の位置は使わず中央から切り出す

/** 切り出す正方形 {x, y, size}（px、画面の内側に収める）。points は前に見つかった手の正規化された点（なければ null）。 */
export function retryBox(points, width, height) {
  const side = Math.min(width, height);
  let cx = width / 2, cy = height / 2, size = side * RETRY_CENTER_RATIO;
  if (points?.length) {
    const xs = points.map((p) => p.x * width), ys = points.map((p) => p.y * height);
    const [x0, x1, y0, y1] = [Math.min(...xs), Math.max(...xs), Math.min(...ys), Math.max(...ys)];
    cx = (x0 + x1) / 2; cy = (y0 + y1) / 2;
    size = clamp(Math.max(x1 - x0, y1 - y0) * RETRY_BOX_SCALE, RETRY_BOX_MIN_PX, side);
  }
  size = Math.min(size, side);
  return { x: clamp(cx - size / 2, 0, width - size), y: clamp(cy - size / 2, 0, height - size), size };
}

/** 切り出した画像で見つかった点（0〜1）を、元の画面の 0〜1 へ戻す。奥行き z も、画像の幅に対する割合なので同じ比で直す。 */
export function mapFromBox(points, box, width, height) {
  return points.map((p) => ({ ...p, x: (box.x + p.x * box.size) / width, y: (box.y + p.y * box.size) / height, z: (p.z ?? 0) * box.size / width }));
}
