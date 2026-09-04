"""PCD を「ダブルクリックで開ける1枚のHTML」に変換する。

作業ログ（Markdown）から点群を見せるための道具。Markdown 自体は PCD を描画できず、
MD 内の <script> も多くのビューアで除去されるため、**MD からはこのHTMLへリンクする**。

## 設計の理由

- **外部CDNを使わない。** three.js をCDNから読むと、将来リンクが変わった時点で
  ログが読めなくなる。素のWebGLで書けば10年後も開ける
- **点群をHTMLに埋め込む。** 別ファイルを fetch する作りだと file:// では
  CORS で読めず、ローカルHTTPサーバが必要になる。埋め込めばダブルクリックで開く
- **uint16 に量子化して埋め込む。** float32 のままだと base64 が倍になる。
  65535段階なら60mの範囲で0.9mm刻みで、地図の用途には十分
- **複数の点群を1枚に入れてA/B切替できる。** before/after の比較がログの主目的

## 2つの出力形式

- 既定: **単体のHTMLページ**。ダブルクリックで開く
- `--fragment`: **報告書に貼り込む断片**（style + markup + script）。
  クラス名を `pcv-` で名前空間化し、色はホストページの CSS 変数
  （`--surface` / `--ink` など）を継承する。JS は IIFE に包んで衝突を避ける

埋め込み時はホイールでページスクロールを奪わないよう、**クリックで操作を有効化**する。
ドラッグ（回転・平行移動）は常に効く。

## 使い方

    python pcd_to_html.py out.html 加工前=map_raw.pcd 加工後=map_clean.pcd
    python pcd_to_html.py frag.html 加工前=a.pcd 加工後=b.pcd --fragment
    python pcd_to_html.py out.html 地図=map.pcd --voxel 0.05 --title "UiS_room_v1"

`--voxel` は表示用の間引き。既定0.08で100万点級が30万点程度になり、1点群あたり
約3MBになる。0にすると間引かない（重くなる）。
"""
from __future__ import annotations

import argparse
import base64
import json
from pathlib import Path

import numpy as np
import open3d as o3d


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="PCD を自己完結HTMLビューアに変換する")
    parser.add_argument("output", type=Path, help="出力する .html")
    parser.add_argument("clouds", nargs="+", metavar="ラベル=path.pcd",
                        help="表示する点群。`ラベル=パス` の形。複数指定でA/B切替になる")
    parser.add_argument("--voxel", type=float, default=0.08, help="表示用の間引き[m]（0で無効）")
    parser.add_argument("--title", default=None, help="ページのタイトル")
    parser.add_argument("--trim", type=float, default=1.0,
                        help="各軸で上下から捨てるパーセンタイル。外れ値で画角が潰れるのを防ぐ")
    parser.add_argument("--fragment", action="store_true",
                        help="単体ページではなく、報告書に貼り込む断片を出力する")
    return parser.parse_args()


def load_cloud(path: Path, voxel: float, trim: float) -> np.ndarray:
    points = np.asarray(o3d.io.read_point_cloud(str(path)).points)
    if points.size == 0:
        raise SystemExit(f"点が読めません: {path}")
    if trim > 0:
        keep = np.ones(len(points), bool)
        for axis in range(3):
            low, high = np.percentile(points[:, axis], [trim, 100.0 - trim])
            keep &= (points[:, axis] >= low) & (points[:, axis] <= high)
        points = points[keep]
    if voxel > 0:
        keys = np.floor(points / voxel).astype(np.int64)
        _, first = np.unique(keys, axis=0, return_index=True)
        points = points[np.sort(first)]
    return points


def quantize(points: np.ndarray) -> tuple[str, list[float], list[float]]:
    """uint16 に量子化して base64 にする。戻り値は (base64, scale, offset)。"""
    low = points.min(axis=0)
    span = np.maximum(points.max(axis=0) - low, 1e-9)
    scale = span / 65535.0
    grid = np.round((points - low) / scale).clip(0, 65535).astype(np.uint16)
    return (base64.b64encode(np.ascontiguousarray(grid).tobytes()).decode("ascii"),
            [float(v) for v in scale], [float(v) for v in low])


# ── 埋め込み用の断片 ───────────────────────────────────────────
# クラス名は pcv- で名前空間化し、色はホストの CSS 変数を継承する
# （単体ページのときは PAGE 側で既定値を定義する）。
FRAGMENT = """
<style>
.pcv { margin: 26px 0; }
.pcv-frame {
  border: 1px solid var(--line-strong, #c3cbd7);
  border-radius: var(--r, 6px);
  overflow: hidden;
  background: var(--surface, #fff);
}
.pcv-bar {
  display: flex; flex-wrap: wrap; align-items: center; gap: 8px 14px;
  padding: 11px 14px;
  border-bottom: 1px solid var(--line, #dce1e9);
  background: var(--surface-2, #eef1f6);
}
.pcv-group { display: flex; gap: 5px; }
.pcv-bar button {
  padding: 6px 12px;
  font: 500 12.5px/1.3 var(--sans, system-ui);
  color: var(--ink-2, #46505f);
  background: var(--surface, #fff);
  border: 1px solid var(--line-strong, #c3cbd7);
  border-radius: 4px; cursor: pointer;
  transition: background .12s, color .12s, border-color .12s;
}
.pcv-bar button:hover { color: var(--ink, #111); border-color: var(--ink-3, #6b7686); }
.pcv-bar button[aria-pressed="true"] {
  color: #fff; background: var(--accent, #0f6fc4); border-color: var(--accent, #0f6fc4);
}
.pcv-bar button:focus-visible {
  outline: 2px solid var(--accent, #0f6fc4); outline-offset: 2px;
}
.pcv-lbl {
  font: 500 10.5px/1.3 var(--mono, monospace); letter-spacing: .1em;
  text-transform: uppercase; color: var(--ink-3, #6b7686);
}
.pcv-stat {
  margin-left: auto; display: flex; gap: 4px 16px; flex-wrap: wrap;
  font: 400 12px/1.5 var(--mono, monospace);
  font-variant-numeric: tabular-nums; color: var(--ink-3, #6b7686);
}
.pcv-stage {
  position: relative;
  height: clamp(400px, 66vh, 720px);
  background: #0b0e13;
}
.pcv-stage canvas {
  position: absolute; inset: 0; display: block;
  width: 100%; height: 100%; cursor: grab;
}
.pcv-stage canvas:active { cursor: grabbing; }
.pcv-stage.pcv-live canvas { touch-action: none; }
.pcv-hint {
  position: absolute; left: 50%; bottom: 14px; transform: translateX(-50%);
  padding: 7px 14px; border-radius: 999px; pointer-events: none;
  background: rgba(11,14,19,.82); color: #e8eaed;
  border: 1px solid rgba(232,234,237,.18);
  font: 500 12px/1.4 var(--sans, system-ui); white-space: nowrap;
  transition: opacity .18s;
}
.pcv-stage.pcv-live .pcv-hint { opacity: 0; }
.pcv-legend {
  position: absolute; right: 14px; bottom: 14px; text-align: center;
  padding: 8px 11px; border-radius: 6px; pointer-events: none;
  background: rgba(11,14,19,.82); border: 1px solid rgba(232,234,237,.18);
  font: 400 10.5px/1.4 var(--mono, monospace); color: #b3bcc9;
}
.pcv-legend .pcv-ramp {
  width: 108px; height: 8px; border-radius: 4px; margin: 4px 0 3px;
  background: linear-gradient(90deg,#2159d9,#1abfe6,#40d95a,#f2e533,#fa8c19,#e62a2a);
}
.pcv-legend .pcv-ends {
  display: flex; justify-content: space-between; font-variant-numeric: tabular-nums;
}
.pcv-help {
  padding: 10px 14px; border-top: 1px solid var(--line, #dce1e9);
  background: var(--surface-2, #eef1f6);
  font: 400 12.5px/1.6 var(--sans, system-ui); color: var(--ink-3, #6b7686);
}
.pcv-help kbd {
  font: 500 11px/1.4 var(--mono, monospace);
  background: var(--surface, #fff); border: 1px solid var(--line-strong, #c3cbd7);
  border-bottom-width: 2px; border-radius: 4px; padding: 1px 5px;
  color: var(--ink, #111);
}
.pcv-fallback { padding: 28px 22px; color: var(--ink-2, #46505f); font-size: 14.5px; }
@media (prefers-reduced-motion: reduce) {
  .pcv-hint, .pcv-stage.pcv-live .pcv-hint { transition: none; }
}
</style>

<div class="pcv">
  <div class="pcv-frame">
    <div class="pcv-bar">
      <span class="pcv-lbl" data-pcv="swlbl">点群</span>
      <span class="pcv-group" data-pcv="tabs"></span>
      <span class="pcv-lbl">視点</span>
      <span class="pcv-group">
        <button type="button" data-pcv="view" data-view="iso">斜め</button>
        <button type="button" data-pcv="view" data-view="top">真上</button>
        <button type="button" data-pcv="view" data-view="side">真横</button>
      </span>
      <span class="pcv-stat">
        <span data-pcv="n"></span><span data-pcv="ext"></span><span data-pcv="zr"></span>
      </span>
    </div>
    <div class="pcv-stage" data-pcv="stage">
      <canvas data-pcv="canvas"></canvas>
      <div class="pcv-hint" data-pcv="hint">クリックすると操作できます</div>
      <div class="pcv-legend">
        高さ Z [m]
        <div class="pcv-ramp"></div>
        <div class="pcv-ends"><span data-pcv="zlo"></span><span data-pcv="zhi"></span></div>
      </div>
    </div>
    <div class="pcv-help">
      <kbd>左ドラッグ</kbd> 回転 &nbsp; <kbd>右ドラッグ</kbd> 平行移動 &nbsp;
      <kbd>ホイール</kbd> ズーム（クリック後） &nbsp;
      <kbd>+</kbd><kbd>-</kbd> 点の大きさ &nbsp; <kbd>Esc</kbd> 操作を解除
    </div>
  </div>
</div>

<script>
(function () {
"use strict";
const CLOUDS = __DATA__;
const root = document.currentScript.previousElementSibling;
const q = (k) => root.querySelector('[data-pcv="' + k + '"]');
const stage = q("stage"), canvas = q("canvas");

const gl = canvas.getContext("webgl", { antialias: true, alpha: false });
if (!gl) {
  root.querySelector(".pcv-frame").innerHTML =
    '<p class="pcv-fallback">このブラウザは WebGL が使えないため 3D 表示を出せません。' +
    '点群は下のリンクからダウンロードできます。</p>';
  return;
}

function decode(b64) {
  const bin = atob(b64), buf = new Uint8Array(bin.length);
  for (let i = 0; i < bin.length; i++) buf[i] = bin.charCodeAt(i);
  return new Uint16Array(buf.buffer);
}

const VERT = `
attribute vec3 aQ;
uniform mat4 uMVP;
uniform vec3 uScale, uOffset;
uniform vec2 uZRange;
uniform float uPointSize;
varying float vT;
void main() {
  vec3 p = aQ * uScale + uOffset;
  gl_Position = uMVP * vec4(p, 1.0);
  vT = clamp((p.z - uZRange.x) / max(uZRange.y - uZRange.x, 1e-6), 0.0, 1.0);
  gl_PointSize = uPointSize;
}`;
const FRAG = `
precision mediump float;
varying float vT;
vec3 ramp(float t) {
  vec3 c0 = vec3(0.13,0.35,0.85), c1 = vec3(0.10,0.75,0.90), c2 = vec3(0.25,0.85,0.35);
  vec3 c3 = vec3(0.95,0.90,0.20), c4 = vec3(0.98,0.55,0.10), c5 = vec3(0.90,0.16,0.16);
  float s = t * 5.0;
  if (s < 1.0) return mix(c0, c1, s);
  if (s < 2.0) return mix(c1, c2, s - 1.0);
  if (s < 3.0) return mix(c2, c3, s - 2.0);
  if (s < 4.0) return mix(c3, c4, s - 3.0);
  return mix(c4, c5, s - 4.0);
}
void main() { gl_FragColor = vec4(ramp(vT), 1.0); }`;

function shader(type, src) {
  const s = gl.createShader(type);
  gl.shaderSource(s, src); gl.compileShader(s);
  if (!gl.getShaderParameter(s, gl.COMPILE_STATUS)) throw new Error(gl.getShaderInfoLog(s));
  return s;
}
const prog = gl.createProgram();
gl.attachShader(prog, shader(gl.VERTEX_SHADER, VERT));
gl.attachShader(prog, shader(gl.FRAGMENT_SHADER, FRAG));
gl.linkProgram(prog);
if (!gl.getProgramParameter(prog, gl.LINK_STATUS)) throw new Error(gl.getProgramInfoLog(prog));
gl.useProgram(prog);

const loc = {
  aQ: gl.getAttribLocation(prog, "aQ"),
  mvp: gl.getUniformLocation(prog, "uMVP"),
  scale: gl.getUniformLocation(prog, "uScale"),
  offset: gl.getUniformLocation(prog, "uOffset"),
  zRange: gl.getUniformLocation(prog, "uZRange"),
  pointSize: gl.getUniformLocation(prog, "uPointSize"),
};
for (const c of CLOUDS) {
  const data = decode(c.b64);
  c.b64 = null;                       // 復号後は元の文字列を解放する
  c.count = data.length / 3;
  c.buffer = gl.createBuffer();
  gl.bindBuffer(gl.ARRAY_BUFFER, c.buffer);
  gl.bufferData(gl.ARRAY_BUFFER, data, gl.STATIC_DRAW);
}

// 行列（列優先）
function perspective(fovy, aspect, near, far) {
  const f = 1 / Math.tan(fovy / 2), nf = 1 / (near - far);
  return new Float32Array([f / aspect,0,0,0, 0,f,0,0,
                           0,0,(far + near) * nf,-1, 0,0,2 * far * near * nf,0]);
}
function lookAt(eye, center, up) {
  let z0 = eye[0]-center[0], z1 = eye[1]-center[1], z2 = eye[2]-center[2];
  let len = 1 / Math.hypot(z0, z1, z2); z0 *= len; z1 *= len; z2 *= len;
  let x0 = up[1]*z2 - up[2]*z1, x1 = up[2]*z0 - up[0]*z2, x2 = up[0]*z1 - up[1]*z0;
  len = Math.hypot(x0, x1, x2);
  if (len < 1e-9) { x0 = 1; x1 = 0; x2 = 0; } else { len = 1/len; x0 *= len; x1 *= len; x2 *= len; }
  const y0 = z1*x2 - z2*x1, y1 = z2*x0 - z0*x2, y2 = z0*x1 - z1*x0;
  return new Float32Array([x0,y0,z0,0, x1,y1,z1,0, x2,y2,z2,0,
    -(x0*eye[0] + x1*eye[1] + x2*eye[2]),
    -(y0*eye[0] + y1*eye[1] + y2*eye[2]),
    -(z0*eye[0] + z1*eye[1] + z2*eye[2]), 1]);
}
function mul(a, b) {
  const o = new Float32Array(16);
  for (let i = 0; i < 4; i++) for (let j = 0; j < 4; j++) {
    let s = 0;
    for (let k = 0; k < 4; k++) s += a[k * 4 + j] * b[i * 4 + k];
    o[i * 4 + j] = s;
  }
  return o;
}

const VIEWS = {
  iso:  { front: [0.6, -0.8, 0.55], up: [0, 0, 1], el: 1.15 },
  top:  { front: [0, 0, 1],         up: [0, 1, 0], el: Math.PI / 2 - 0.001 },
  side: { front: [1, 0, 0],         up: [0, 0, 1], el: 0.02 },
};
const FOVY = Math.PI / 4;
let active = 0, pointSize = 2.0, view = "iso";
const cam = { az: -Math.PI / 2, el: 1.15, dist: 40, target: [0, 0, 0] };

function reset(name) {
  view = name || view;
  const c = CLOUDS[active], v = VIEWS[view];
  cam.target = c.center.slice();
  // 外接球がちょうど収まる距離。どの角度へ回しても画面から出ない。
  // 縦長の枠では水平画角のほうが狭くなるので、狭いほうに合わせる。
  const radius = 0.5 * Math.hypot(c.span[0], c.span[1], c.span[2]);
  const aspect = Math.max(stage.clientWidth, 1) / Math.max(stage.clientHeight, 1);
  const halfV = FOVY / 2, halfH = Math.atan(Math.tan(halfV) * aspect);
  cam.dist = radius / Math.sin(Math.min(halfV, halfH)) * 1.06;
  cam.az = -Math.PI / 2;
  cam.el = v.el;
  root.querySelectorAll('[data-pcv="view"]').forEach(
    (b) => b.setAttribute("aria-pressed", String(b.dataset.view === view)));
  draw();
}

function draw() {
  const dpr = Math.min(window.devicePixelRatio || 1, 2);
  const w = Math.max(Math.floor(stage.clientWidth * dpr), 1);
  const h = Math.max(Math.floor(stage.clientHeight * dpr), 1);
  if (canvas.width !== w || canvas.height !== h) { canvas.width = w; canvas.height = h; }
  gl.viewport(0, 0, w, h);
  gl.clearColor(0.043, 0.055, 0.075, 1);
  gl.enable(gl.DEPTH_TEST);
  gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);

  const ce = Math.cos(cam.el), se = Math.sin(cam.el);
  const eye = [cam.target[0] + cam.dist * ce * Math.cos(cam.az),
               cam.target[1] + cam.dist * ce * Math.sin(cam.az),
               cam.target[2] + cam.dist * se];
  const proj = perspective(FOVY, w / h, cam.dist * 0.002, cam.dist * 12);
  const mvp = mul(proj, lookAt(eye, cam.target, VIEWS[view].up));

  const c = CLOUDS[active];
  gl.uniformMatrix4fv(loc.mvp, false, mvp);
  gl.uniform3fv(loc.scale, c.scale);
  gl.uniform3fv(loc.offset, c.offset);
  gl.uniform2fv(loc.zRange, [c.zlo, c.zhi]);
  gl.uniform1f(loc.pointSize, pointSize * dpr);
  gl.bindBuffer(gl.ARRAY_BUFFER, c.buffer);
  gl.enableVertexAttribArray(loc.aQ);
  gl.vertexAttribPointer(loc.aQ, 3, gl.UNSIGNED_SHORT, false, 0, 0);
  gl.drawArrays(gl.POINTS, 0, c.count);
}

// 操作。ホイールはクリックで有効化するまでページスクロールに譲る。
let live = false, drag = null;
function setLive(on) {
  if (live === on) return;
  live = on;
  stage.classList.toggle("pcv-live", on);
  q("hint").textContent = on ? "" : "クリックすると操作できます";
}
canvas.addEventListener("contextmenu", (e) => e.preventDefault());
canvas.addEventListener("pointerdown", (e) => {
  setLive(true);
  drag = { x: e.clientX, y: e.clientY, pan: e.button === 2 || e.shiftKey };
  canvas.setPointerCapture(e.pointerId);
});
canvas.addEventListener("pointerup", () => { drag = null; });
canvas.addEventListener("pointercancel", () => { drag = null; });
canvas.addEventListener("pointermove", (e) => {
  if (!drag) return;
  const dx = e.clientX - drag.x, dy = e.clientY - drag.y;
  drag.x = e.clientX; drag.y = e.clientY;
  if (drag.pan) {
    const right = [-Math.sin(cam.az), Math.cos(cam.az), 0];
    const se = Math.sin(cam.el), ce = Math.cos(cam.el);
    const upv = [-se * Math.cos(cam.az), -se * Math.sin(cam.az), ce];
    const k = cam.dist * 0.0016;
    for (let i = 0; i < 3; i++) cam.target[i] += (-dx * right[i] + dy * upv[i]) * k;
  } else {
    cam.az -= dx * 0.006;
    cam.el = Math.max(-1.5607, Math.min(1.5607, cam.el + dy * 0.006));
  }
  draw();
});
canvas.addEventListener("wheel", (e) => {
  if (!live) return;                  // 未有効ならページを普通にスクロールさせる
  e.preventDefault();
  cam.dist = Math.max(0.4, cam.dist * Math.exp(e.deltaY * 0.0012));
  draw();
}, { passive: false });
document.addEventListener("pointerdown", (e) => {
  if (!stage.contains(e.target)) setLive(false);
});
window.addEventListener("keydown", (e) => {
  if (e.key === "Escape") { setLive(false); return; }
  if (!live) return;
  if (e.key === "+" || e.key === "=") { pointSize = Math.min(8, pointSize + 0.5); draw(); }
  else if (e.key === "-") { pointSize = Math.max(0.5, pointSize - 0.5); draw(); }
});
new ResizeObserver(() => draw()).observe(stage);

// UI
const tabs = q("tabs");
CLOUDS.forEach((c, i) => {
  const b = document.createElement("button");
  b.type = "button"; b.textContent = c.name;
  b.onclick = () => select(i);
  tabs.appendChild(b);
});
if (CLOUDS.length < 2) { tabs.style.display = "none"; q("swlbl").style.display = "none"; }
root.querySelectorAll('[data-pcv="view"]').forEach(
  (b) => { b.onclick = () => reset(b.dataset.view); });

function select(i) {
  active = i;
  const c = CLOUDS[i];
  [...tabs.children].forEach((b, j) => b.setAttribute("aria-pressed", String(j === i)));
  q("n").textContent = c.count.toLocaleString() + " 点";
  q("ext").textContent = c.span.map((v) => v.toFixed(1)).join(" × ") + " m";
  q("zr").textContent = "Z " + c.zlo.toFixed(2) + " 〜 " + c.zhi.toFixed(2);
  q("zlo").textContent = c.zlo.toFixed(1);
  q("zhi").textContent = c.zhi.toFixed(1);
  reset(view);
}
select(0);
requestAnimationFrame(() => reset(view));   // レイアウト確定後に画角を合わせ直す
})();
</script>
"""

# ── 単体ページの包み ───────────────────────────────────────────
PAGE = """<!doctype html>
<html lang="ja">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>__TITLE__</title>
<style>
  :root {
    color-scheme: dark;
    --surface:#151a22; --surface-2:#1b212b; --ink:#e8eaed; --ink-2:#b3bcc9;
    --ink-3:#8b94a3; --line:#262d3a; --line-strong:#38414f; --accent:#57a8ea; --r:6px;
    --sans:-apple-system,"Hiragino Sans","Noto Sans JP",sans-serif;
    --mono:ui-monospace,"SFMono-Regular",Menlo,monospace;
  }
  * { box-sizing: border-box; }
  body { margin: 0; padding: 22px; background: #0e1116; color: var(--ink);
         font: 400 15px/1.7 var(--sans); }
  h1 { margin: 0 0 4px; font-size: 20px; font-weight: 600; letter-spacing: -.01em; }
  .sub { margin: 0; font: 400 12.5px/1.6 var(--mono); color: var(--ink-3); }
  .pcv { margin-top: 18px; }
  .pcv-stage { height: calc(100vh - 190px); }
</style>
</head>
<body>
<h1>__TITLE__</h1>
<p class="sub">高さで色分け · 外部依存なし · オフラインで開ける</p>
__FRAGMENT__
</body>
</html>
"""
def main() -> None:
    args = parse_args()
    entries = []
    for spec in args.clouds:
        if "=" not in spec:
            raise SystemExit(f"`ラベル=パス` の形で指定してください: {spec}")
        label, _, raw_path = spec.partition("=")
        path = Path(raw_path)
        points = load_cloud(path, args.voxel, args.trim)
        b64, scale, offset = quantize(points)
        low, high = points.min(axis=0), points.max(axis=0)
        entries.append({
            "name": label,
            "b64": b64,
            "scale": scale,
            "offset": offset,
            "center": [float(v) for v in (low + high) / 2],
            "span": [float(v) for v in (high - low)],
            "zlo": float(np.percentile(points[:, 2], 2)),
            "zhi": float(np.percentile(points[:, 2], 98)),
        })
        print(f"[html] {label}: {path.name} -> {len(points)} 点 "
              f"({len(b64) / 1e6:.1f} MB の base64)")

    fragment = FRAGMENT.replace("__DATA__", json.dumps(entries, ensure_ascii=False))
    if args.fragment:
        html = fragment
    else:
        title = args.title or args.output.stem
        html = (PAGE.replace("__TITLE__", title)
                    .replace("__FRAGMENT__", fragment))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(html, encoding="utf-8")
    print(f"[OK] {args.output} ({args.output.stat().st_size / 1e6:.1f} MB)")
    if args.fragment:
        print("[NEXT] 報告書HTMLの本文に、この断片をそのまま貼り込んでください")
    else:
        print("[NEXT] ダブルクリックで開けます（サーバ不要・オフライン可）")


if __name__ == "__main__":
    main()
