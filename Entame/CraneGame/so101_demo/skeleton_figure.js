// 人の腕の形（skeleton_math.js と同じ角度の定義）から、正面と横の線画を描く。較正で「この形をとって」と指示するための図。
// 座標は skeleton_math.js と同じ（x=画像の右、y=下、z=奥。カメラへ向かうと z がマイナス）。
const RAD = Math.PI/180;
export const LENGTHS = {upper: 0.30, fore: 0.26, hand: 0.09};     // m。図の比率だけに使う
const SVG_NS = "http://www.w3.org/2000/svg";
const VIEW = {width: 100, height: 120, shoulder: [50, 28], scale: 105};   // 図の大きさ（SVG の単位）と、肩の位置、1m あたりの長さ

const add = (a, b) => ({x: a.x+b.x, y: a.y+b.y, z: a.z+b.z});
const scale = (a, k) => ({x: a.x*k, y: a.y*k, z: a.z*k});
const dot = (a, b) => a.x*b.x + a.y*b.y + a.z*b.z;
const sub = (a, b) => add(a, scale(b, -1));
const unit = a => scale(a, 1/Math.hypot(a.x, a.y, a.z));

/**
 * 角度（度）から、腕の点を返す。肩が原点。pan は上腕の水平の向き（前が0、画像の右が+）、lift は上腕の真下からの角度、
 * elbow は肘の角度（伸ばすと180）、flex は手首の曲げ（肘と同じ向きへ曲げると＋）。肘は前（カメラ側）へ曲がる。
 */
export function armPoints({pan = 0, lift, elbow, flex = 0}) {
  const horizontal = {x: Math.sin(pan*RAD), y: 0, z: -Math.cos(pan*RAD)};
  const upper = unit(add(scale(horizontal, Math.sin(lift*RAD)), {x: 0, y: Math.cos(lift*RAD), z: 0}));
  const towards = direction => unit(sub(direction, scale(upper, dot(direction, upper))));
  const bendDirection = Math.hypot(...Object.values(sub({x: 0, y: 0, z: -1}, scale(upper, upper.z*-1)))) > 0.2
    ? towards({x: 0, y: 0, z: -1}) : towards({x: 0, y: -1, z: 0});      // 腕が真っ直ぐ前のときは、上へ曲げる
  const theta = (180 - elbow)*RAD;
  const fore = add(scale(upper, Math.cos(theta)), scale(bendDirection, Math.sin(theta)));
  const foreNext = add(scale(upper, -Math.sin(theta)), scale(bendDirection, Math.cos(theta)));
  const hand = add(scale(fore, Math.cos(flex*RAD)), scale(foreNext, Math.sin(flex*RAD)));
  const elbowAt = scale(upper, LENGTHS.upper);
  const wristAt = add(elbowAt, scale(fore, LENGTHS.fore));
  return {shoulder: {x: 0, y: 0, z: 0}, elbow: elbowAt, wrist: wristAt, hand: add(wristAt, scale(hand, LENGTHS.hand))};
}

function element(name, attributes, parent) {
  const node = document.createElementNS(SVG_NS, name);
  for (const [key, value] of Object.entries(attributes)) node.setAttribute(key, value);
  parent?.append(node);
  return node;
}

/** 1つの向きの図。horizontal(p) は点から図の横方向の値（m）を返す。 */
function view(points, horizontal, label, ghosts) {
  const svg = element("svg", {viewBox: `0 0 ${VIEW.width} ${VIEW.height}`, role: "img", "aria-label": label});
  const [sx, sy] = VIEW.shoulder, at = p => [sx + horizontal(p)*VIEW.scale, sy + p.y*VIEW.scale];
  element("rect", {width: VIEW.width, height: VIEW.height, rx: 8, fill: "#eaf2ef"}, svg);
  element("circle", {cx: sx, cy: sy - 17, r: 9, fill: "#c9d8d4"}, svg);                    // 頭
  element("line", {x1: sx, y1: sy, x2: sx, y2: VIEW.height - 6, stroke: "#c9d8d4", "stroke-width": 9, "stroke-linecap": "round"}, svg);   // 体
  for (const ghost of ghosts) {      // ほかの姿勢を、うすい点線で重ねる（違いが分かるように）
    const other = [ghost.shoulder, ghost.elbow, ghost.wrist, ghost.hand].map(at);
    element("polyline", {points: other.map(([x, y]) => `${x},${y}`).join(" "), fill: "none", stroke: "#9aa9a6", "stroke-width": 2.5, "stroke-dasharray": "3 3", "stroke-linecap": "round", "stroke-linejoin": "round"}, svg);
  }
  const chain = [points.shoulder, points.elbow, points.wrist, points.hand].map(at);
  const colors = ["#0f6b60", "#12877a", "#17302f"];
  for (let i = 0; i < 3; i++) {
    element("line", {x1: chain[i][0], y1: chain[i][1], x2: chain[i+1][0], y2: chain[i+1][1], stroke: colors[i], "stroke-width": 5 - i, "stroke-linecap": "round"}, svg);
  }
  for (const [x, y] of chain.slice(0, 3)) element("circle", {cx: x, cy: y, r: 3, fill: "#fff", stroke: "#17302f", "stroke-width": 1.5}, svg);
  element("text", {x: 4, y: 12, "font-size": 8, fill: "#4b5d5a"}, svg).textContent = label;
  return svg;
}

/** 正面（カメラから見た図）と横（右がカメラ側）の図を、1つの要素にして返す。others（ほかの姿勢の角度の配列）は、点線で重ねる。 */
export function armFigure(angles, others = []) {
  const points = armPoints(angles), ghosts = others.map(armPoints), box = document.createElement("div");
  box.className = "figure";
  box.append(view(points, p => p.x, "正面", ghosts), view(points, p => -p.z, "横（右がカメラ）", ghosts));
  return box;
}
