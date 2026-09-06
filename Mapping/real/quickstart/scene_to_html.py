#!/usr/bin/env python3
"""pcd_to_mjcf.py が作った MuJoCo シーンを、1枚のHTMLで見比べられるようにする。

## 何を見せるか

`maxRange` を変えて作った複数のシーンを**同じ視点で切り替えて**見る。数値表だけでは
「偽の障害物が減った」のか「壁ごと消えた」のかが判断できないため、**赤く塗った偽障害物**
（軌跡から近すぎて静止物ではありえない箱）と、灰色の構造を同時に出す。

- 箱の色 … 高さ（青が低く、赤茶が高い）
- **赤い箱** … 偽の障害物。軌跡から phantom_radius 以内に残った箱
- 床の面 … (C) のハイトフィールド。色は床の高さ
- 白い線 … ロボットの軌跡

## 設計の理由（pcd_to_html.py と同じ方針）

- **外部CDNを使わない。** 素のWebGLで書けば10年後も開ける
- **データをHTMLに埋め込む。** file:// で fetch すると CORS で読めない
- **箱は「中心と半径」で送り、三角形への展開はブラウザ側でやる。** 三角形を送ると
  1変種で2MB近くなるが、箱のままなら 4,500個で 110KB で済む

## 使い方

    ../../Navigation/.venv/bin/python quickstart/scene_to_html.py \\
        runs/20260904T183457_UiS_room_v2
"""
from __future__ import annotations

import argparse
import base64
import json
import sys
from pathlib import Path

import numpy as np
import open3d as o3d
from scipy.spatial import cKDTree

QUICKSTART = Path(__file__).resolve().parent
sys.path.insert(0, str(QUICKSTART))

from pcd_to_mjcf import boxes_from_grid  # noqa: E402

ELEVATION_STRIDE = 2      # 床の表示だけ間引く。当たり判定側は間引いていない
DISPLAY_TRIANGLES = 25000 # 表示用のメッシュの三角数。MJCF 側は間引いていない


def encode(array: np.ndarray, dtype) -> str:
    return base64.b64encode(np.ascontiguousarray(array, dtype=dtype).tobytes()).decode("ascii")


def pack_mesh(sim_dir: Path) -> "dict | None":
    """(B) メッシュ方式の表示データ。

    メッシュ本体に加えて、**MuJoCo が実際に当たり判定に使う凸包**をタイルの一辺ごとに
    入れる。凸包を見せないと「メッシュを入れれば済む」と読めてしまう。
    """
    path = sim_dir / "mesh.npz"
    if not path.exists():
        return None
    data = np.load(path)
    report = json.loads((sim_dir / "mesh.json").read_text())
    mesh = o3d.geometry.TriangleMesh(
        o3d.utility.Vector3dVector(data["vertices"].astype(np.float64)),
        o3d.utility.Vector3iVector(data["faces"].astype(np.int32)))
    if len(mesh.triangles) > DISPLAY_TRIANGLES:
        mesh = mesh.simplify_quadric_decimation(target_number_of_triangles=DISPLAY_TRIANGLES)
    hulls = []
    for entry in report["variants"]:
        tag = entry["tag"]
        if f"hull_{tag}_v" not in data:
            continue
        hulls.append({
            "tag": tag,
            "label": "1 枚" if entry["tile_m"] is None else f"{entry['tile_m']:.1f} m",
            "verts": encode(data[f"hull_{tag}_v"].ravel(), np.float32),
            "faces": encode(data[f"hull_{tag}_f"].ravel(), np.uint32),
            "faceCount": int(len(data[f"hull_{tag}_f"])),
        })
    return {
        "verts": encode(np.asarray(mesh.vertices).ravel(), np.float32),
        "faces": encode(np.asarray(mesh.triangles).ravel(), np.uint32),
        "faceCount": len(mesh.triangles),
        "hulls": hulls,
        "stats": report,
    }


MESH_ROWS = [
    ("破片の数（= MuJoCo の mesh geom）", "pieces", "{:,}", None),
    ("三角の合計", "triangles", "{:,}", None),
    ("<b>塞がる自由空間</b>", "blocked_m3", "{:.1f} m³", "low"),
    ("　同 割合", "blocked_pct", "{:.1f} %", "low"),
    ("<b>落としたボールの最終 z</b>", "final_z", "{:.2f} m", None),
]


def build_mesh_table(report: dict) -> str:
    variants = report["variants"]
    columns = ["メッシュ1枚" if v["tile_m"] is None else f"タイル {v['tile_m']:.1f} m"
               for v in variants]
    head = "".join(f"<th>{c}</th>" for c in columns)
    body = []
    for label, key, form, better in MESH_ROWS:
        values = [v.get(key) for v in variants]
        # 「1枚」は比較の対象外。潰れて使えないことが分かっているので最良になりようがない
        pool = [x for x in values[1:] if isinstance(x, (int, float))]
        best = (min(pool) if better == "low" else max(pool)) if pool and better else None
        cells = []
        for value in values:
            mark = ' class="best"' if better and best is not None and value == best else ""
            text = form.format(value) if value is not None else "—"
            cells.append(f"<td{mark}>{text}</td>")
        body.append(f'<tr><th scope="row">{label}</th>{"".join(cells)}</tr>')
    return (f'<table><caption>(B) メッシュ方式 — タイルの一辺を振ったときの当たり判定'
            f'（到達できる自由空間 {report["free_m3"]:,.0f} m³ に対する割合）</caption>'
            f'<thead><tr><th scope="col">指標</th>{head}</tr></thead>'
            f'<tbody>{"".join(body)}</tbody></table>')


def load_video(sim_dir: Path) -> "tuple[str, dict] | None":
    """MuJoCo の画面をそのまま録った mp4 と、そのときの照合結果。

    **HTML の再生（線と矢印）は抽象で、機体が本当に脚を運んでいるかは写らない。**
    実際に歩けていることの証拠はシミュレータが描いた絵の方にある。
    """
    for meta in sorted(sim_dir.glob("walk_r*.json")):
        report = json.loads(meta.read_text())
        video = sim_dir / report.get("video", "")
        if report.get("video") and video.exists():
            return base64.b64encode(video.read_bytes()).decode("ascii"), report
    return None


def load_walk(sim_dir: Path, variant: str) -> dict:
    """`walk_scene.py` が残した計画経路と実際に歩いた軌跡。無ければ空で返す。"""
    tag = variant.replace("octomap_", "")
    path, meta = sim_dir / f"walk_{tag}.npz", sim_dir / f"walk_{tag}.json"
    if not (path.exists() and meta.exists()):
        return {"route": "", "routeCount": 0, "walk": "", "walkCount": 0,
                "walkTime": "", "walkYaw": "", "walkStats": None}
    data = np.load(path)
    history = data["history"]
    return {
        "route": encode(data["route"].ravel(), np.float32),
        "routeCount": int(len(data["route"])),
        "walk": encode(history[:, 1:3].ravel(), np.float32),
        "walkCount": int(len(history)),
        # 再生用。時刻[s] と向き[rad] を別に持つ（線の描画は x,y だけで足りる）
        "walkTime": encode(history[:, 0], np.float32),
        "walkYaw": encode(history[:, 3], np.float32),
        "walkStats": json.loads(meta.read_text()),
    }


WALK_ROWS = [
    ("計画した経路長", "planned_m", "{:.1f} m", None),
    ("実際に歩いた距離", "walked_m", "{:.1f} m", None),
    ("目標との差", "final_gap_m", "{:.2f} m", "low"),
    ("シミュレーション時間", "sim_s", "{:.0f} s", "low"),
]


def build_walk_table(variants: "list[dict]") -> str:
    rows = [v for v in variants if v.get("walkStats")]
    if not rows:
        return ""
    head = "".join(f"<th>maxRange {v['variant'].replace('octomap_r', '')} m</th>" for v in rows)
    body = []
    verdict = "".join(
        f"<td>{'到達' if v['walkStats']['reached'] else ('転倒' if v['walkStats']['fallen'] else '時間切れ')}</td>"
        for v in rows)
    body.append(f'<tr><th scope="row"><b>結果</b></th>{verdict}</tr>')
    for label, key, form, better in WALK_ROWS:
        values = [v["walkStats"].get(key) for v in rows]
        pool = [x for x in values if isinstance(x, (int, float))]
        best = (min(pool) if better == "low" else max(pool)) if pool and better else None
        cells = []
        for value in values:
            mark = ' class="best"' if better and best is not None and value == best else ""
            text = form.format(value) if value is not None else "—"
            cells.append(f"<td{mark}>{text}</td>")
        body.append(f'<tr><th scope="row">{label}</th>{"".join(cells)}</tr>')
    first = rows[0]["walkStats"]
    return (f'<table><caption>端から端まで歩かせた結果 — 出発 ({first["start"][0]:.1f}, '
            f'{first["start"][1]:.1f}) → 目標 ({first["goal"][0]:.1f}, {first["goal"][1]:.1f})'
            f'（直線 {first["straight_m"]:.1f} m）</caption>'
            f'<thead><tr><th scope="col">指標</th>{head}</tr></thead>'
            f'<tbody>{"".join(body)}</tbody></table>')


def pack_variant(sim_dir: Path, stats: dict, phantom_radius: float) -> dict:
    """npz から箱・床・軌跡を作り、base64 に詰める。"""
    data = np.load(sim_dir / f"{stats['variant']}.npz")
    mask, level = data["occupied"], data["level"]
    origin, cell = data["origin"], float(data["cell"])
    trajectory = data["trajectory"]

    boxes = boxes_from_grid(mask, level, cell, origin)
    packed = np.array([[x, y, z, hx, cell / 2.0, hz] for x, y, z, hx, hz in boxes], dtype=np.float32)

    # 箱ごとに「偽の障害物か」を付ける。判定は箱の中心で行う（1セル幅の帯なので中心で足りる）
    if len(trajectory) and len(packed):
        distance = cKDTree(trajectory).query(packed[:, :2])[0]
        phantom = (distance <= phantom_radius).astype(np.uint8)
    else:
        phantom = np.zeros(len(packed), dtype=np.uint8)

    elevation = data["elevation"][::ELEVATION_STRIDE, ::ELEVATION_STRIDE]
    walk = load_walk(sim_dir, stats["variant"])
    return {
        **walk,
        "variant": stats["variant"],
        "boxes": encode(packed.ravel(), np.float32),
        "boxCount": len(packed),
        "phantom": encode(phantom, np.uint8),
        "elev": encode(elevation.ravel(), np.float32),
        "elevRows": int(elevation.shape[0]),
        "elevCols": int(elevation.shape[1]),
        "elevCell": cell * ELEVATION_STRIDE,
        "origin": [float(origin[0]), float(origin[1])],
        "traj": encode(trajectory.ravel(), np.float32),
        "trajCount": int(len(trajectory)),
        "stats": stats,
    }


ROWS = [
    ("入力の点数", "points", "{:,}", None),
    ("箱の数（MuJoCo の geom）", "boxes", "{:,}", None),
    ("占有セル", "cells_kept", "{:,}", None),
    ("除去した小さい塊", "dropped_clusters", "{:,}", None),
    ("<b>偽の障害物セル</b>（軌跡から0.5m以内）", "phantom_cells", "{:,}", "low"),
    ("　同 面積", "phantom_area_m2", "{:.2f} m²", "low"),
    ("<b>遠方の構造セル</b>（軌跡から4m以上）", "structure_cells", "{:,}", "high"),
    ("<b>遠方の壁高セル</b>（1.75m以上）", "tall_far_cells", "{:,}", "high"),
    ("床を実測できたセル", "floor_measured_pct", "{:.1f} %", None),
    ("床の外れセル（除去）", "floor_outliers", "{:,}", None),
    ("補正した傾き", "tilt_corrected_deg", "{:.2f}°", None),
    ("床の高低差（hfield）", "hfield_range_m", "{:.3f} m", None),
]


def build_table(variants: "list[dict]") -> str:
    head = "".join(f"<th>maxRange {v['variant'].replace('octomap_r', '')} m</th>" for v in variants)
    body = []
    for label, key, form, better in ROWS:
        values = [v["stats"].get(key) for v in variants]
        numeric = [x for x in values if isinstance(x, (int, float))]
        best = (min(numeric) if better == "low" else max(numeric)) if numeric and better else None
        cells = []
        for value in values:
            mark = " class=\"best\"" if better and value == best else ""
            cells.append(f"<td{mark}>{form.format(value) if value is not None else '—'}</td>")
        body.append(f"<tr><th scope=\"row\">{label}</th>{''.join(cells)}</tr>")
    return (f"<table><thead><tr><th scope=\"col\">指標</th>{head}</tr></thead>"
            f"<tbody>{''.join(body)}</tbody></table>")


PAGE = r"""<!doctype html>
<html lang="ja"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>__TITLE__</title>
<style>
:root {
  --surface: #fbfbfd; --surface-2: #eef1f6; --line: #dce1e9; --line-strong: #c3cbd7;
  --ink: #14181f; --ink-2: #46505f; --ink-3: #6b7686; --accent: #0f6fc4; --warn: #d6392c;
  --sans: system-ui, -apple-system, "Hiragino Sans", "Noto Sans JP", sans-serif;
  --mono: ui-monospace, SFMono-Regular, Menlo, monospace;
}
/* docs/作業ログ/index.html と同じ配色。フォントは外部CDNを使わない方針のまま */
@media (prefers-color-scheme: dark) {
  :root:not([data-theme="light"]) {
    --surface: #151a22; --surface-2: #1b212b; --line: #262d3a; --line-strong: #38414f;
    --ink: #e8eaed; --ink-2: #b3bcc9; --ink-3: #8b94a3; --accent: #57a8ea;
  }
  :root:not([data-theme="light"]) body { background: #0e1116; }
}
:root[data-theme="dark"] {
  --surface: #151a22; --surface-2: #1b212b; --line: #262d3a; --line-strong: #38414f;
  --ink: #e8eaed; --ink-2: #b3bcc9; --ink-3: #8b94a3; --accent: #57a8ea;
}
:root[data-theme="dark"] body { background: #0e1116; }
* { box-sizing: border-box; }
body { margin: 0; background: var(--surface); color: var(--ink);
       font: 400 15px/1.7 var(--sans); }
.wrap { max-width: 1180px; margin: 0 auto; padding: 34px 22px 70px; }
h1 { font-size: clamp(20px, 1.1rem + 1vw, 27px); letter-spacing: -.01em; margin: 0 0 6px; }
.sub { color: var(--ink-3); font: 400 13px/1.6 var(--mono); margin: 0 0 26px; }
.frame { border: 1px solid var(--line-strong); border-radius: 7px; overflow: hidden;
         background: var(--surface); }
.bar { display: flex; flex-wrap: wrap; align-items: center; gap: 9px 16px;
       padding: 11px 14px; border-bottom: 1px solid var(--line); background: var(--surface-2); }
.lbl { font: 500 10.5px/1.3 var(--mono); letter-spacing: .1em; text-transform: uppercase;
       color: var(--ink-3); }
.group { display: flex; gap: 5px; }
.pair { display: flex; align-items: center; gap: 9px; }
button { padding: 6px 13px; font: 500 12.5px/1.3 var(--sans); color: var(--ink-2);
         background: var(--surface); border: 1px solid var(--line-strong);
         border-radius: 4px; cursor: pointer; transition: background .12s, color .12s; }
button:hover { color: var(--ink); border-color: var(--ink-3); }
button[aria-pressed="true"] { color: #fff; background: var(--accent); border-color: var(--accent); }
button:focus-visible { outline: 2px solid var(--accent); outline-offset: 2px; }
button[disabled] { opacity: .38; cursor: default; }
button[disabled]:hover { color: var(--ink-2); border-color: var(--line-strong); }
.slider { display: flex; align-items: center; gap: 9px; margin-left: auto; }
.slider input { width: 168px; }
.slider output { font: 400 12px/1.5 var(--mono); font-variant-numeric: tabular-nums;
                 color: var(--ink-3); min-width: 54px; }
.stage { position: relative; height: clamp(430px, 68vh, 760px); background: #0b0e13; }
.stage canvas { position: absolute; inset: 0; display: block; width: 100%; height: 100%;
                cursor: grab; touch-action: none; }
.stage canvas:active { cursor: grabbing; }
.legend { position: absolute; right: 14px; bottom: 14px; padding: 9px 12px; border-radius: 6px;
          pointer-events: none; background: rgba(11,14,19,.84);
          border: 1px solid rgba(232,234,237,.18); font: 400 10.5px/1.5 var(--mono);
          color: #b3bcc9; }
.legend .ramp { width: 116px; height: 8px; border-radius: 4px; margin: 4px 0 3px;
                background: linear-gradient(90deg,#2159d9,#1abfe6,#40d95a,#f2e533,#fa8c19,#e62a2a); }
.legend .ends { display: flex; justify-content: space-between; }
.legend .swatch { display: inline-block; width: 9px; height: 9px; border-radius: 2px;
                  background: #ff2fd0; margin-right: 5px; vertical-align: -1px; }
.play {
  display: flex; align-items: center; gap: 12px; padding: 10px 14px;
  border-top: 1px solid var(--line); background: var(--surface-2);
}
.play input[type=range] { flex: 1; min-width: 120px; }
.play output {
  font: 400 12.5px/1.5 var(--mono); font-variant-numeric: tabular-nums;
  color: var(--ink-3); white-space: nowrap;
}
.play #pb-toggle { min-width: 82px; }
.play.off { opacity: .4; pointer-events: none; }
.order {
  display: flex; align-items: center; gap: 12px; padding: 11px 14px;
  border-top: 1px solid var(--line); background: var(--accent-soft, #e7f1fb);
}
.order code {
  flex: 1; overflow-x: auto; white-space: nowrap; padding: 6px 9px;
  background: var(--surface); border: 1px solid var(--line-strong); border-radius: 4px;
  font: 400 12.5px/1.6 var(--mono); color: var(--ink);
}
.stage.picking canvas { cursor: crosshair; }
figure.clip { margin: 30px 0 0; }
figure.clip video {
  width: 100%; display: block; border: 1px solid var(--line-strong);
  border-radius: var(--r, 6px); background: #0b0e13;
}
figure.clip figcaption {
  margin-top: 9px; font: 400 13px/1.7 var(--sans); color: var(--ink-3);
}
.help { padding: 11px 14px; border-top: 1px solid var(--line); background: var(--surface-2);
        font: 400 12.5px/1.7 var(--ink-3); color: var(--ink-3); }
kbd { font: 500 11px/1.4 var(--mono); background: var(--surface);
      border: 1px solid var(--line-strong); border-bottom-width: 2px; border-radius: 4px;
      padding: 1px 5px; color: var(--ink); }
table { width: 100%; border-collapse: collapse; margin-top: 30px;
        font-variant-numeric: tabular-nums; }
caption { text-align: left; font: 500 13px/1.6 var(--sans); color: var(--ink-3);
          padding-bottom: 9px; }
th, td { padding: 8px 12px; border-bottom: 1px solid var(--line); text-align: right;
         font: 400 13.5px/1.6 var(--mono); }
thead th { font: 500 12px/1.5 var(--sans); color: var(--ink-3); text-align: right;
           border-bottom: 1px solid var(--line-strong); }
tbody th[scope="row"], thead th:first-child { text-align: left;
           font: 400 13.5px/1.6 var(--sans); color: var(--ink-2); }
td.best { color: var(--accent); font-weight: 600; background: color-mix(in srgb, var(--accent) 9%, transparent); }
.note { margin-top: 26px; padding: 15px 18px; border-left: 3px solid var(--accent);
        background: var(--surface-2); border-radius: 0 6px 6px 0;
        font-size: 14px; color: var(--ink-2); }
.note p { margin: 0 0 9px; } .note p:last-child { margin: 0; }
.fallback { padding: 30px 22px; color: var(--ink-2); }
</style></head>
<body><div class="wrap">
<h1>__TITLE__</h1>
<p class="sub">__SUBTITLE__</p>

<section class="frame">
  <div class="bar">
    <span class="pair"><span class="lbl">表現</span>
      <span class="group">
        <button id="mode-box" aria-pressed="true">箱 (A)</button>
        <button id="mode-mesh" aria-pressed="false">メッシュ (B)</button>
        <button id="mode-hull" aria-pressed="false">当たり判定（凸包）</button>
      </span></span>
    <span class="pair"><span class="lbl" id="tabs-label">maxRange</span>
      <span class="group" id="tabs"></span></span>
    <span class="pair"><span class="lbl">指示</span>
      <span class="group">
        <button id="pick-toggle" aria-pressed="false">地点を指定</button>
        <button id="pick-clear">消去</button>
      </span></span>
    <span class="pair"><span class="lbl">視点</span>
      <span class="group">
        <button id="view-iso" aria-pressed="true">斜め</button>
        <button id="view-top" aria-pressed="false">真上</button>
      </span></span>
    <label class="slider"><span class="lbl">高さで切る</span>
      <input id="cut" type="range" min="0.25" max="2.0" step="0.05" value="2.0">
      <output id="cut-out">2.00 m</output></label>
  </div>
  <div class="stage" id="stage">
    <canvas id="gl"></canvas>
    <div class="legend">
      箱の高さ<div class="ramp"></div>
      <div class="ends"><span>0.25</span><span>2.0 m</span></div>
      <div style="margin-top:6px"><span class="swatch"></span>偽の障害物（箱のみ）</div>
      <div><span class="swatch" style="background:#ffc400"></span>計画した経路</div>
      <div><span class="swatch" style="background:#3ddc84"></span>歩いた軌跡</div>
      <div><span class="swatch" style="background:#ebebeb"></span>記録時の軌跡</div>
    </div>
  </div>
  <div class="play" id="play">
    <button id="pb-toggle">▶ 再生</button>
    <input id="pb-time" type="range" min="0" max="1" step="0.001" value="0" aria-label="再生位置">
    <output id="pb-clock">0.0 / 0.0 s</output>
    <span class="lbl">速さ</span>
    <span class="group">
      <button id="pb-x1" aria-pressed="false">×1</button>
      <button id="pb-x4" aria-pressed="true">×4</button>
      <button id="pb-x16" aria-pressed="false">×16</button>
    </span>
  </div>
  <div class="order" id="order" hidden>
    <span class="lbl">この指示を出す</span>
    <code id="order-cmd"></code>
    <button id="order-copy">コピー</button>
  </div>
  <p class="help">ドラッグで回転 / <kbd>Shift</kbd>+ドラッグ（または右ドラッグ）で平行移動 /
     ホイールで拡大縮小 / <kbd>1</kbd>〜<kbd>4</kbd> で maxRange 切替。
     「凸包」が <strong>MuJoCo がメッシュに対して実際に使う当たり判定の形</strong>。
     「高さで切る」を下げると天井側の箱が消えて部屋の中が見える。<b>再生</b>は歩行シミュレーションの記録の再生で、緑の矢印が機体、緑の線が通ってきた跡。<kbd>Space</kbd> でも再生／停止できる。</p>
</section>

__TABLE__

__WALK_TABLE__

__VIDEO__

<div class="note">__WALK_NOTE__</div>

<div class="note">__NOTE__</div>

__MESH_TABLE__

<div class="note">__NOTE_B__</div>
</div>
<script>
(function () {
"use strict";
const DATA = __DATA__;
const MESH = __MESH__;
const SESSION = __SESSION__;
const NAVGRID = __NAVGRID__;
const canvas = document.getElementById("gl");
const gl = canvas.getContext("webgl", { antialias: true, alpha: false });
if (!gl) {
  document.getElementById("stage").innerHTML =
    '<p class="fallback">WebGL が使えないため3D表示を省略した。下の表は読める。</p>';
  return;
}

// ── シェーダ ────────────────────────────────────────────────
const VERT = `
attribute vec3 aPos; attribute vec3 aCol; attribute float aTop;
uniform mat4 uMVP; uniform float uCut; varying vec3 vCol;
void main() {
  vCol = aCol;
  if (aTop > uCut) { gl_Position = vec4(2.0, 2.0, 2.0, 1.0); return; }
  gl_Position = uMVP * vec4(aPos, 1.0);
}`;
const FRAG = `precision mediump float; varying vec3 vCol;
void main() { gl_FragColor = vec4(vCol, 1.0); }`;

function compile(type, src) {
  const s = gl.createShader(type);
  gl.shaderSource(s, src); gl.compileShader(s);
  if (!gl.getShaderParameter(s, gl.COMPILE_STATUS)) throw new Error(gl.getShaderInfoLog(s));
  return s;
}
const prog = gl.createProgram();
gl.attachShader(prog, compile(gl.VERTEX_SHADER, VERT));
gl.attachShader(prog, compile(gl.FRAGMENT_SHADER, FRAG));
gl.linkProgram(prog); gl.useProgram(prog);
const loc = {
  pos: gl.getAttribLocation(prog, "aPos"),
  col: gl.getAttribLocation(prog, "aCol"),
  top: gl.getAttribLocation(prog, "aTop"),
  mvp: gl.getUniformLocation(prog, "uMVP"),
  cut: gl.getUniformLocation(prog, "uCut"),
};
gl.enable(gl.DEPTH_TEST);
gl.clearColor(0.043, 0.055, 0.075, 1.0);

// ── データ展開 ──────────────────────────────────────────────
function decode(b64, Type) {
  const bin = atob(b64), buf = new Uint8Array(bin.length);
  for (let i = 0; i < bin.length; i++) buf[i] = bin.charCodeAt(i);
  return new Type(buf.buffer);
}
// 高さ 0.25〜2.0m を青→赤茶に写す
function ramp(t) {
  const stops = [[33,89,217],[26,191,230],[64,217,90],[242,229,51],[250,140,25],[230,42,42]];
  t = Math.max(0, Math.min(0.9999, t)) * (stops.length - 1);
  const i = Math.floor(t), f = t - i, a = stops[i], b = stops[i + 1];
  return [a[0]+(b[0]-a[0])*f, a[1]+(b[1]-a[1])*f, a[2]+(b[2]-a[2])*f];
}
// 立方体の6面。最後の数字は面ごとの明るさ（法線計算の代わり）
const FACES = [
  [[1,-1,-1],[1,1,-1],[1,1,1],[1,-1,1], 0.84],
  [[-1,-1,-1],[-1,-1,1],[-1,1,1],[-1,1,-1], 0.68],
  [[-1,1,-1],[-1,1,1],[1,1,1],[1,1,-1], 0.92],
  [[-1,-1,-1],[1,-1,-1],[1,-1,1],[-1,-1,1], 0.62],
  [[-1,-1,1],[1,-1,1],[1,1,1],[-1,1,1], 1.00],
  [[-1,-1,-1],[-1,1,-1],[1,1,-1],[1,-1,-1], 0.50],
];
const CORNER_ORDER = [0, 1, 2, 0, 2, 3];

function buildBoxes(v) {
  const b = decode(v.boxes, Float32Array), flag = decode(v.phantom, Uint8Array);
  const n = v.boxCount, verts = n * 36;
  const pos = new Float32Array(verts * 3), col = new Uint8Array(verts * 3);
  const top = new Float32Array(verts);
  let k = 0;
  for (let i = 0; i < n; i++) {
    const cx = b[i*6], cy = b[i*6+1], cz = b[i*6+2];
    const hx = b[i*6+3], hy = b[i*6+4], hz = b[i*6+5];
    const height = cz + hz;
    // 高さランプの最上色が赤なので、偽障害物は赤ではなくマゼンタにする。
    // 最初の描画では壁（2.0m＝ランプの赤）と見分けが付かなかった。
    const base = flag[i] ? [255, 47, 208] : ramp((height - 0.25) / 1.75);
    for (const face of FACES) {
      const shade = face[4];
      for (const ci of CORNER_ORDER) {
        const c = face[ci];
        pos[k*3] = cx + c[0]*hx; pos[k*3+1] = cy + c[1]*hy; pos[k*3+2] = cz + c[2]*hz;
        col[k*3] = base[0]*shade; col[k*3+1] = base[1]*shade; col[k*3+2] = base[2]*shade;
        top[k] = height; k++;
      }
    }
  }
  return { pos, col, top, count: verts };
}

function buildFloor(v) {
  const e = decode(v.elev, Float32Array);
  const rows = v.elevRows, cols = v.elevCols, cell = v.elevCell;
  let lo = Infinity, hi = -Infinity;
  for (let i = 0; i < e.length; i++) { if (e[i] < lo) lo = e[i]; if (e[i] > hi) hi = e[i]; }
  const span = Math.max(hi - lo, 1e-3);
  const quads = (rows - 1) * (cols - 1), verts = quads * 6;
  const pos = new Float32Array(verts * 3), col = new Uint8Array(verts * 3);
  const top = new Float32Array(verts).fill(-999);
  let k = 0;
  const put = (r, c) => {
    const z = e[r*cols + c];
    pos[k*3] = v.origin[0] + c*cell; pos[k*3+1] = v.origin[1] + r*cell; pos[k*3+2] = z;
    const t = (z - lo) / span;
    col[k*3] = 52 + t*70; col[k*3+1] = 62 + t*78; col[k*3+2] = 78 + t*86;
    k++;
  };
  for (let r = 0; r < rows - 1; r++)
    for (let c = 0; c < cols - 1; c++) {
      put(r, c); put(r, c+1); put(r+1, c+1);
      put(r, c); put(r+1, c+1); put(r+1, c);
    }
  return { pos, col, top, count: verts };
}

// 面の法線で陰影を付ける。箱と同じ高さランプで色を決める
function buildTriMesh(vertsB64, facesB64, count, flat) {
  const p = decode(vertsB64, Float32Array), f = decode(facesB64, Uint32Array);
  const pos = new Float32Array(count * 9), col = new Uint8Array(count * 9);
  const top = new Float32Array(count * 3);
  const light = [0.34, 0.24, 0.91];
  for (let i = 0; i < count; i++) {
    const a = f[i*3]*3, b = f[i*3+1]*3, c = f[i*3+2]*3;
    const ux = p[b]-p[a], uy = p[b+1]-p[a+1], uz = p[b+2]-p[a+2];
    const vx = p[c]-p[a], vy = p[c+1]-p[a+1], vz = p[c+2]-p[a+2];
    let nx = uy*vz-uz*vy, ny = uz*vx-ux*vz, nz = ux*vy-uy*vx;
    const len = Math.hypot(nx, ny, nz) || 1; nx/=len; ny/=len; nz/=len;
    const shade = 0.40 + 0.60 * Math.abs(nx*light[0] + ny*light[1] + nz*light[2]);
    const zc = (p[a+2]+p[b+2]+p[c+2])/3;
    const base = flat ? [150, 158, 172] : ramp((zc - 0.25) / 1.75);
    const zmax = Math.max(p[a+2], p[b+2], p[c+2]);
    for (let k = 0; k < 3; k++) {
      const s = [a, b, c][k];
      pos[(i*3+k)*3] = p[s]; pos[(i*3+k)*3+1] = p[s+1]; pos[(i*3+k)*3+2] = p[s+2];
      col[(i*3+k)*3] = base[0]*shade; col[(i*3+k)*3+1] = base[1]*shade;
      col[(i*3+k)*3+2] = base[2]*shade;
      top[i*3+k] = zmax;
    }
  }
  return { pos, col, top, count: count * 3 };
}

function buildLine(b64, n, z, rgb) {
  if (!n) return { pos: new Float32Array(0), col: new Uint8Array(0),
                   top: new Float32Array(0), count: 0 };
  const t = decode(b64, Float32Array);
  const pos = new Float32Array(n * 3), col = new Uint8Array(n * 3);
  const top = new Float32Array(n).fill(-999);
  for (let i = 0; i < n; i++) {
    pos[i*3] = t[i*2]; pos[i*3+1] = t[i*2+1]; pos[i*3+2] = z;
    col[i*3] = rgb[0]; col[i*3+1] = rgb[1]; col[i*3+2] = rgb[2];
  }
  return { pos, col, top, count: n };
}

// 機体を表す矢印。ロボット座標系[m]で、前方が +x。2 枚の三角で輪郭を作る
const MARKER = [[0.36, 0.0], [-0.16, 0.20], [-0.04, 0.0],
                [0.36, 0.0], [-0.04, 0.0], [-0.16, -0.20]];
const MARKER_Z = 0.22;

function markerVertices(x, y, yaw) {
  const c = Math.cos(yaw), s = Math.sin(yaw);
  const out = new Float32Array(MARKER.length * 3);
  for (let i = 0; i < MARKER.length; i++) {
    const [px, py] = MARKER[i];
    out[i*3] = x + px*c - py*s; out[i*3+1] = y + px*s + py*c; out[i*3+2] = MARKER_Z;
  }
  return out;
}

function upload(mesh) {
  const make = (data, type) => {
    const buf = gl.createBuffer();
    gl.bindBuffer(gl.ARRAY_BUFFER, buf);
    gl.bufferData(gl.ARRAY_BUFFER, data, gl.STATIC_DRAW);
    return buf;
  };
  return { pos: make(mesh.pos), col: make(mesh.col), top: make(mesh.top), count: mesh.count };
}

const cache = {};
function scene(index) {
  if (!cache[index]) {
    const v = DATA[index];
    cache[index] = {
      boxes: upload(buildBoxes(v)), floor: upload(buildFloor(v)),
      traj:  upload(buildLine(v.traj,  v.trajCount,  0.06, [235, 235, 235])),
      route: upload(buildLine(v.route, v.routeCount, 0.10, [255, 196,  0])),
      walk:  upload(buildLine(v.walk,  v.walkCount,  0.14, [ 61, 220, 132])),
    };
  }
  return cache[index];
}
// 矢印は毎フレーム作り直すので、使い回す動的バッファを1つ持つ
const markerBuf = { pos: gl.createBuffer(), col: gl.createBuffer(), top: gl.createBuffer() };
(function initMarker() {
  const n = MARKER.length;
  const col = new Uint8Array(n * 3), top = new Float32Array(n).fill(-999);
  for (let i = 0; i < n; i++) { col[i*3] = 61; col[i*3+1] = 220; col[i*3+2] = 132; }
  gl.bindBuffer(gl.ARRAY_BUFFER, markerBuf.col); gl.bufferData(gl.ARRAY_BUFFER, col, gl.STATIC_DRAW);
  gl.bindBuffer(gl.ARRAY_BUFFER, markerBuf.top); gl.bufferData(gl.ARRAY_BUFFER, top, gl.STATIC_DRAW);
})();

let meshCache = null;
const hullCache = {};
function meshParts() {
  if (!meshCache && MESH)
    meshCache = upload(buildTriMesh(MESH.verts, MESH.faces, MESH.faceCount, false));
  return meshCache;
}
function hullParts(i) {
  if (!MESH || !MESH.hulls[i]) return null;
  if (!hullCache[i]) {
    const h = MESH.hulls[i];
    hullCache[i] = upload(buildTriMesh(h.verts, h.faces, h.faceCount, true));
  }
  return hullCache[i];
}

// ── 行列 ────────────────────────────────────────────────────
function perspective(fovy, aspect, near, far) {
  const f = 1 / Math.tan(fovy / 2), d = near - far;
  return [f/aspect,0,0,0, 0,f,0,0, 0,0,(far+near)/d,-1, 0,0,2*far*near/d,0];
}
function lookAt(eye, center, up) {
  const z = norm(sub(eye, center)), x = norm(cross(up, z)), y = cross(z, x);
  return [x[0],y[0],z[0],0, x[1],y[1],z[1],0, x[2],y[2],z[2],0,
          -dot(x,eye), -dot(y,eye), -dot(z,eye), 1];
}
const sub = (a,b) => [a[0]-b[0], a[1]-b[1], a[2]-b[2]];
const dot = (a,b) => a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
const cross = (a,b) => [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]];
const norm = (a) => { const l = Math.hypot(a[0],a[1],a[2]) || 1; return [a[0]/l,a[1]/l,a[2]/l]; };
function mul(a, b) {
  const out = new Float32Array(16);
  for (let i = 0; i < 4; i++) for (let j = 0; j < 4; j++) {
    let s = 0; for (let k = 0; k < 4; k++) s += a[k*4+j] * b[i*4+k];
    out[i*4+j] = s;
  }
  return out;
}

// ── カメラ ──────────────────────────────────────────────────
const bounds = DATA[0].stats.extent_m;
const cam = { az: -90, el: 42, dist: Math.max(bounds[0], bounds[1]) * 1.15,
              target: [0, 0, 0.8] };
(function centreOnMap() {
  const v = DATA[0];
  cam.target = [v.origin[0] + v.elevCols * v.elevCell / 2,
                v.origin[1] + v.elevRows * v.elevCell / 2, 0.8];
})();

let current = 0, hull = 0, cut = 2.0, mode = "box";
// 再生の状態。playT は sim 時刻[s]、speed は実時間に対する倍率
let playing = false, playT = 0, speed = 4, lastFrame = 0;

function walkTimes(i) {
  const v = DATA[i];
  if (!v.walkCount) return null;
  if (!v._t) { v._t = decode(v.walkTime, Float32Array); v._yaw = decode(v.walkYaw, Float32Array);
               v._xy = decode(v.walk, Float32Array); }
  return v;
}
function duration() {
  const v = walkTimes(current);
  return v ? v._t[v._t.length - 1] : 0;
}
// playT に対応する記録の添字。時刻は単調増加なので二分探索でよい
function playIndex() {
  const v = walkTimes(current);
  if (!v) return 0;
  let lo = 0, hi = v._t.length - 1;
  while (lo < hi) { const mid = (lo + hi) >> 1; if (v._t[mid] < playT) lo = mid + 1; else hi = mid; }
  return lo;
}

function draw() {
  const dpr = Math.min(window.devicePixelRatio || 1, 2);
  const w = Math.round(canvas.clientWidth * dpr), h = Math.round(canvas.clientHeight * dpr);
  if (canvas.width !== w || canvas.height !== h) { canvas.width = w; canvas.height = h; }
  gl.viewport(0, 0, w, h);
  gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);

  const ar = (cam.az) * Math.PI / 180, er = cam.el * Math.PI / 180;
  const eye = [cam.target[0] + cam.dist * Math.cos(er) * Math.cos(ar),
               cam.target[1] + cam.dist * Math.cos(er) * Math.sin(ar),
               cam.target[2] + cam.dist * Math.sin(er)];
  const mvp = mul(perspective(Math.PI / 4, w / h, 0.15, cam.dist * 6 + 120),
                  lookAt(eye, cam.target, [0, 0, 1]));
  gl.uniformMatrix4fv(loc.mvp, false, mvp);
  gl.uniform1f(loc.cut, cut);

  const s = scene(current);
  const bind = (part) => {
    gl.bindBuffer(gl.ARRAY_BUFFER, part.pos);
    gl.enableVertexAttribArray(loc.pos); gl.vertexAttribPointer(loc.pos, 3, gl.FLOAT, false, 0, 0);
    gl.bindBuffer(gl.ARRAY_BUFFER, part.col);
    gl.enableVertexAttribArray(loc.col); gl.vertexAttribPointer(loc.col, 3, gl.UNSIGNED_BYTE, true, 0, 0);
    gl.bindBuffer(gl.ARRAY_BUFFER, part.top);
    gl.enableVertexAttribArray(loc.top); gl.vertexAttribPointer(loc.top, 1, gl.FLOAT, false, 0, 0);
  };
  if (mode === "box") {
    bind(s.floor); gl.drawArrays(gl.TRIANGLES, 0, s.floor.count);
    bind(s.boxes); gl.drawArrays(gl.TRIANGLES, 0, s.boxes.count);
  } else {
    const part = mode === "mesh" ? meshParts() : hullParts(hull);
    if (part) { bind(part); gl.drawArrays(gl.TRIANGLES, 0, part.count); }
  }
  if (pickCount) { bind(pickBuf); gl.drawArrays(gl.TRIANGLES, 0, pickCount); }
  bind(s.traj); gl.drawArrays(gl.LINE_STRIP, 0, s.traj.count);
  if (mode === "box") {
    if (s.route.count) { bind(s.route); gl.drawArrays(gl.LINE_STRIP, 0, s.route.count); }
    const v = walkTimes(current);
    if (s.walk.count) {
      // 再生位置までの跡だけを描く。止めているときは全部
      const upto = v ? Math.max(2, playIndex() + 1) : s.walk.count;
      bind(s.walk); gl.drawArrays(gl.LINE_STRIP, 0, Math.min(upto, s.walk.count));
    }
    if (v) {
      const i = playIndex();
      const verts = markerVertices(v._xy[i*2], v._xy[i*2+1], v._yaw[i]);
      gl.bindBuffer(gl.ARRAY_BUFFER, markerBuf.pos);
      gl.bufferData(gl.ARRAY_BUFFER, verts, gl.DYNAMIC_DRAW);
      bind(markerBuf);
      gl.drawArrays(gl.TRIANGLES, 0, MARKER.length);
    }
  }
}
function schedule() { requestAnimationFrame(draw); }

// ── 操作 ────────────────────────────────────────────────────
let drag = null;
canvas.addEventListener("contextmenu", (e) => e.preventDefault());
canvas.addEventListener("pointerdown", (e) => {
  try { canvas.setPointerCapture(e.pointerId); } catch (_) { /* 合成イベントでは失敗する */ }
  drag = { x: e.clientX, y: e.clientY, pan: e.button === 2 || e.shiftKey,
           x0: e.clientX, y0: e.clientY, moved: false };
});
canvas.addEventListener("pointerup", (e) => {
  // 動かさずに離したときだけ「クリック」とみなす。回した拍子に地点が増えないように
  if (picking && drag && !drag.moved && e.button === 0) {
    const point = pickFloor(e.clientX, e.clientY);
    if (point && !navigable(point[0], point[1])) {
      blocked = [point[0], point[1]];
      setTimeout(() => { blocked = null; rebuildPicks(); schedule(); }, 1400);
      rebuildPicks(); schedule();
    } else if (point) {
      blocked = null; picked.push(point); rebuildPicks(); schedule();
    }
  }
  drag = null;
});
canvas.addEventListener("pointercancel", () => { drag = null; });
canvas.addEventListener("pointermove", (e) => {
  if (!drag) return;
  const dx = e.clientX - drag.x, dy = e.clientY - drag.y;
  drag.x = e.clientX; drag.y = e.clientY;
  if (Math.hypot(e.clientX - drag.x0, e.clientY - drag.y0) > 4) drag.moved = true;
  if (drag.pan) {
    const ar = cam.az * Math.PI / 180, k = cam.dist * 0.0016;
    cam.target[0] += (Math.sin(ar) * dx + Math.cos(ar) * dy) * k;
    cam.target[1] += (-Math.cos(ar) * dx + Math.sin(ar) * dy) * k;
  } else {
    cam.az -= dx * 0.35;
    cam.el = Math.max(-4, Math.min(89.5, cam.el + dy * 0.3));
  }
  schedule();
});
canvas.addEventListener("wheel", (e) => {
  e.preventDefault();
  cam.dist = Math.max(1.5, Math.min(400, cam.dist * Math.exp(e.deltaY * 0.0013)));
  schedule();
}, { passive: false });

// ── UI ──────────────────────────────────────────────────────
const tabs = document.getElementById("tabs");
const tabsLabel = document.getElementById("tabs-label");

function renderTabs() {
  const hullMode = mode === "hull";
  const items = hullMode ? MESH.hulls.map((h) => h.label)
                         : DATA.map((v) => v.variant.replace("octomap_r", "") + " m");
  tabsLabel.textContent = hullMode ? "タイルの一辺" : "maxRange";
  tabs.textContent = "";
  items.forEach((text, i) => {
    const b = document.createElement("button");
    b.textContent = text;
    b.setAttribute("aria-pressed", String(i === (hullMode ? hull : current)));
    b.disabled = mode === "mesh";
    b.addEventListener("click", () => select(i));
    tabs.appendChild(b);
  });
}
function select(i) {
  if (mode === "hull") hull = Math.min(i, MESH.hulls.length - 1);
  else { current = i; playT = 0; setPlaying(false); syncClock(); }
  renderTabs(); schedule();
}
const modeButtons = { box: document.getElementById("mode-box"),
                      mesh: document.getElementById("mode-mesh"),
                      hull: document.getElementById("mode-hull") };
function setMode(next) {
  if (!MESH && next !== "box") return;
  mode = next;
  for (const key in modeButtons)
    modeButtons[key].setAttribute("aria-pressed", String(key === mode));
  // メッシュは r4 からしか作っていないので、そのときだけタブを止める
  renderTabs();
  syncClock();
  schedule();
}
for (const key in modeButtons) {
  if (!MESH && key !== "box") modeButtons[key].disabled = true;
  modeButtons[key].addEventListener("click", () => setMode(key));
}
const iso = document.getElementById("view-iso"), top = document.getElementById("view-top");
iso.addEventListener("click", () => { cam.el = 42; cam.az = -90;
  iso.setAttribute("aria-pressed", "true"); top.setAttribute("aria-pressed", "false"); schedule(); });
top.addEventListener("click", () => { cam.el = 89.4; cam.az = -90;
  iso.setAttribute("aria-pressed", "false"); top.setAttribute("aria-pressed", "true"); schedule(); });
const slider = document.getElementById("cut"), out = document.getElementById("cut-out");
slider.addEventListener("input", () => {
  cut = parseFloat(slider.value); out.textContent = cut.toFixed(2) + " m"; schedule();
});
window.addEventListener("keydown", (e) => {
  const n = parseInt(e.key, 10);
  const limit = mode === "hull" ? MESH.hulls.length : DATA.length;
  if (n >= 1 && n <= limit) select(n - 1);
});
renderTabs();
// ------------------------------------------------------- 地点の指定（GUI→指示）
// クリックした画素から視線を作り、床(z=0)との交点を取る。
// **RViz2 の「2D Goal Pose」がやっているのもこれと同じで、Nav2 は関係しない。**
// 向こうは得た姿勢を PoseStamped にして /goal_pose へ publish しているだけ。
function eyeAndBasis() {
  const ar = cam.az * Math.PI / 180, er = cam.el * Math.PI / 180;
  const eye = [cam.target[0] + cam.dist * Math.cos(er) * Math.cos(ar),
               cam.target[1] + cam.dist * Math.cos(er) * Math.sin(ar),
               cam.target[2] + cam.dist * Math.sin(er)];
  const forward = norm(sub(cam.target, eye));
  const right = norm(cross(forward, [0, 0, 1]));
  const up = cross(right, forward);
  return { eye, forward, right, up };
}
function pickFloor(clientX, clientY) {
  const rect = canvas.getBoundingClientRect();
  const ndcX = ((clientX - rect.left) / rect.width) * 2 - 1;
  const ndcY = 1 - ((clientY - rect.top) / rect.height) * 2;
  const { eye, forward, right, up } = eyeAndBasis();
  const tan = Math.tan(Math.PI / 8);            // fovy=45° の半分
  const aspect = rect.width / rect.height;
  const dir = norm([
    forward[0] + right[0] * ndcX * tan * aspect + up[0] * ndcY * tan,
    forward[1] + right[1] * ndcX * tan * aspect + up[1] * ndcY * tan,
    forward[2] + right[2] * ndcX * tan * aspect + up[2] * ndcY * tan,
  ]);
  if (Math.abs(dir[2]) < 1e-6) return null;      // 床と平行
  const distance = -eye[2] / dir[2];
  if (distance <= 0) return null;                // 床は視線の後ろ
  return [eye[0] + dir[0] * distance, eye[1] + dir[1] * distance];
}

// 経路計画が使う格子（機体半径ぶん膨張済み）。ここで弾けば、あとで失敗しない
const navBits = NAVGRID ? decode(NAVGRID.bits, Uint8Array) : null;
function navigable(x, y) {
  if (!NAVGRID) return true;
  const col = Math.floor((x - NAVGRID.origin[0]) / NAVGRID.res);
  const row = Math.floor((y - NAVGRID.origin[1]) / NAVGRID.res);
  if (col < 0 || row < 0 || col >= NAVGRID.cols || row >= NAVGRID.rows) return false;
  const index = row * NAVGRID.cols + col;
  return ((navBits[index >> 3] >> (7 - (index & 7))) & 1) === 0;
}

let picking = false;
const picked = [];
const pickBuf = { pos: gl.createBuffer(), col: gl.createBuffer(), top: gl.createBuffer() };
let pickCount = 0;

// 指した地点は、動画に出てくる目印と同じ「柱＋輪」で描く
let blocked = null;   // 通れない所を指したとき、そこに×印を出すための一時保持

function rebuildPicks() {
  const marks = blocked ? picked.concat([blocked]) : picked;
  const perPoint = 6 + 24 * 3;                   // 柱2枚 + 輪24分割
  const pos = new Float32Array(marks.length * perPoint * 3);
  const col = new Uint8Array(marks.length * perPoint * 3);
  const top = new Float32Array(marks.length * perPoint).fill(-999);
  let k = 0;
  const put = (x, y, z, rgb) => {
    pos[k*3] = x; pos[k*3+1] = y; pos[k*3+2] = z;
    col[k*3] = rgb[0]; col[k*3+1] = rgb[1]; col[k*3+2] = rgb[2];
    k++;
  };
  marks.forEach(([x, y], i) => {
    const isBlocked = blocked && i === marks.length - 1;
    const last = !isBlocked && i === picked.length - 1 && picked.length > 1;
    const rgb = isBlocked ? [120, 130, 145] : (last ? [255, 76, 38] : [255, 199, 0]);
    const w = 0.09, h = 1.8;
    put(x - w, y, 0, rgb); put(x + w, y, 0, rgb); put(x + w, y, h, rgb);
    put(x - w, y, 0, rgb); put(x + w, y, h, rgb); put(x - w, y, h, rgb);
    for (let s = 0; s < 24; s++) {
      const a = s / 24 * Math.PI * 2, b = (s + 1) / 24 * Math.PI * 2;
      put(x, y, 0.03, rgb);
      put(x + Math.cos(a) * 0.4, y + Math.sin(a) * 0.4, 0.03, rgb);
      put(x + Math.cos(b) * 0.4, y + Math.sin(b) * 0.4, 0.03, rgb);
    }
  });
  pickCount = k;
  gl.bindBuffer(gl.ARRAY_BUFFER, pickBuf.pos); gl.bufferData(gl.ARRAY_BUFFER, pos, gl.DYNAMIC_DRAW);
  gl.bindBuffer(gl.ARRAY_BUFFER, pickBuf.col); gl.bufferData(gl.ARRAY_BUFFER, col, gl.DYNAMIC_DRAW);
  gl.bindBuffer(gl.ARRAY_BUFFER, pickBuf.top); gl.bufferData(gl.ARRAY_BUFFER, top, gl.DYNAMIC_DRAW);
  syncOrder();
}

const orderBar = document.getElementById("order");
const orderCmd = document.getElementById("order-cmd");
function syncOrder() {
  orderBar.hidden = picked.length < 2 && !blocked;
  if (blocked) {
    orderCmd.textContent =
      `(${blocked[0].toFixed(2)}, ${blocked[1].toFixed(2)}) は通れない — 障害物の中か、`
      + `壁から機体半径 0.40 m 以内。別の点を指す`;
  } else if (picked.length >= 2) {
    // 等号で繋ぐ。空白区切りだと負の座標が argparse にオプション扱いされて落ちる
    const args = picked.map(([x, y]) => `--waypoint=${x.toFixed(2)},${y.toFixed(2)}`).join(" ");
    orderCmd.textContent =
      `quickstart/walk_scene.py runs/${SESSION} ${args} --record`;
  }
}
document.getElementById("pick-toggle").addEventListener("click", (e) => {
  picking = !picking;
  e.currentTarget.setAttribute("aria-pressed", String(picking));
  document.getElementById("stage").classList.toggle("picking", picking);
});
document.getElementById("pick-clear").addEventListener("click", () => {
  picked.length = 0; rebuildPicks(); schedule();
});
document.getElementById("order-copy").addEventListener("click", () => {
  navigator.clipboard && navigator.clipboard.writeText(orderCmd.textContent);
});

// ---------------------------------------------------------------- 再生
const playBar = document.getElementById("play");
const toggle = document.getElementById("pb-toggle");
const timeBar = document.getElementById("pb-time");
const clock = document.getElementById("pb-clock");
const speeds = { 1: document.getElementById("pb-x1"), 4: document.getElementById("pb-x4"),
                 16: document.getElementById("pb-x16") };

function syncClock() {
  const total = duration();
  timeBar.max = String(Math.max(total, 0.001));
  timeBar.value = String(playT);
  clock.textContent = total ? `${playT.toFixed(1)} / ${total.toFixed(1)} s` : "記録なし";
  playBar.classList.toggle("off", !(mode === "box" && total > 0));
}
function setPlaying(on) {
  playing = on && duration() > 0;
  toggle.textContent = playing ? "⏸ 停止" : "▶ 再生";
  if (playing) { lastFrame = performance.now(); requestAnimationFrame(tickPlay); }
}
function tickPlay(now) {
  if (!playing) return;
  const dt = Math.min((now - lastFrame) / 1000, 0.1);
  lastFrame = now;
  playT += dt * speed;
  if (playT >= duration()) { playT = duration(); setPlaying(false); }
  syncClock(); draw();
  if (playing) requestAnimationFrame(tickPlay);
}
toggle.addEventListener("click", () => {
  if (!playing && playT >= duration()) playT = 0;   // 終端なら頭から
  setPlaying(!playing);
});
timeBar.addEventListener("input", () => {
  playT = parseFloat(timeBar.value); setPlaying(false); syncClock(); draw();
});
for (const key in speeds) {
  speeds[key].addEventListener("click", () => {
    speed = Number(key);
    for (const other in speeds)
      speeds[other].setAttribute("aria-pressed", String(Number(other) === speed));
    schedule();
  });
}
window.addEventListener("keydown", (e) => {
  if (e.code === "Space" && e.target === document.body) { e.preventDefault(); toggle.click(); }
});
window.addEventListener("resize", schedule);
syncClock();
schedule();
})();
</script></body></html>
"""

NOTE = """<p><b>読み方。</b> マゼンタの箱は「ロボットが実際に歩いた場所に残っている箱」で、
静止物ではありえないので<b>シミュレータの中で偽の障害物になる</b>。maxRange を上げると
これが減る。ただし必ず「遠方の構造セル」と対で見ること。これが減っていたら、
人ではなく机や壁を消している。</p>
<p><b>この記録での結論は maxRange = 4 m。</b> 遠方の壁高セルは 2〜5m のどの設定でも
1,327〜1,337（振れ幅 0.8%）で、<b>壁はどの設定でも削れていない</b>。遠方の構造セルも
4m までは 4,827〜4,828 と完全に横ばいのまま、偽の障害物だけが 575 → 196 と 66% 減る。
5m にすると構造セルが 4,426（−8.3%）に落ちるのに、偽の障害物は 42 セルしか減らない。
削れているのは壁ではなく机・椅子などの低い什器である。</p>
<p><b>点群のまま評価したときの最良（3m）とは1段ずれる。</b> 格子に落とす過程で、
孤立した点が連結成分フィルタと軌跡からの距離で先に落ちるため、
点群評価で 3m → 4m のときに見えていた遠方構造の目減りが、格子では出なくなる。
<b>合否は最終的な用途の表現で測るべき</b>という例になっている。</p>"""


WALK_NOTE = """<p><b>作った地図の中を実際に歩かせた（2026-09-05）。</b>
<code>unitree_rl_gym</code> の学習済み 12DoF ポリシーで、部屋の端から端まで横断させる。
出発点と目標は「<b>最短経路が最も長くなる 2 点</b>」を幅優先探索の 2 回掃引で選んだ
（直線距離で選ぶと壁を挟んだ 2 点になる）。経路は A* ＋ string pulling。
<b>4 変種すべてで到達した</b>。黄色が計画した経路、緑が実際に歩いた軌跡、白が地図を録ったときの軌跡。</p>

<p><b>ただし最初は転んだ。原因は 2 つあり、どちらも地図側ではなかった。</b></p>

<p><b>(1) 床のハイトフィールドが荒すぎた。</b> 実測できるのは床の 36 % だけで、残りは
最近傍で埋めた外挿である。そのモザイクの継ぎ目が <b>最大勾配 48.3°</b> の段差になり、
平地で学習したポリシーが 13.7 m 地点で転倒した。床を平面に差し替えた対照では
到達したので、床が原因だと確定した。0.35 m 幅で均して <b>最大勾配 5.7°</b> にしたところ、
44.7 m 歩いて到達し、平面の対照（44.6 m）とほぼ一致した。
<b>実測できていない床を「measured らしく」持たせてはいけない。</b></p>

<p><b>(2) 追従制御が発停を繰り返した。</b> 「方位のずれが 35° を超えたらその場旋回」を
単一の閾値でやると、閾値の近くで前進が 0 と全開を往復する。実測で速度が
0.53 → 0.04 → 0.20 → 0.14 → 0.48 m/s と 1.5 秒振動したあと歩容が崩れた。
接触でも地形でもなく（転倒地点は最寄りの箱から 1.36 m、勾配 0.2°）、
<b>こちらの制御の作りの問題</b>である。戻す閾値を 20° に分けて解決した。</p>

<p><b>HTML の再生とシミュレータの絵は別物。</b> 上のビューの「再生」は記録した骨盤の
x, y, 向きを線と矢印で描き直したもので、<b>脚が運べているかは写らない</b>。
実際に歩けていることの証拠は、その下の mp4（MuJoCo の描画をそのまま録ったもの）にある。
また「絵が MJCF とズレていないか」は、シミュレータが読んだモデルの側から
geom を数え直して照合してある（箱 3,847 個・体積 86.4 m³・hfield 309×243 が一致）。</p>

<p><b>速度。</b> sim 88 秒が実時間 2.1 秒（約 40 倍速、録画時は 5 倍速）。地図 1 枚に対する横断試験を
数秒で回せるので、地図の作り方を変えるたびに「歩けるか」を確かめられる。</p>"""


NOTE_B = """<p><b>(B) メッシュ方式は MuJoCo では素直に使えない（2026-09-05 実測）。</b>
「当たり判定（凸包）」で <b>タイルの一辺を「1 枚」</b>にすると、部屋の凹凸が全部潰れて
ただの塊になるのが見える。<b>これが MuJoCo がメッシュに対して実際に使う形</b>である。
到達できる自由空間 1,928 m³ のうち <b>906.6 m³（47.0 %）が塞がる</b>。
部屋の中 (z = 1.20 m) に半径 0.10 m のボールを置くと、凸包の内側にめり込んだ判定になって
<b>z = 13.52 m まで弾き飛ばされた</b>。G1 も同じ理由で部屋に入れない。</p>

<p><b>測り方に注意。</b> レイキャストでは分からない。<code>mj_ray</code> は三角形を直接見るので、
凸包で潰れていても素通りして床を返す（最初これで測って「問題なし」と誤判定した）。
<b>剛体を置いて落とす</b>のが正しい測り方である。</p>

<p><b>タイルに割れば動く。ただし只ではない。</b> メッシュを一辺 <var>L</var> のタイルに切って
geom を分けると、タイルごとに凸包が取られるので元の形に近づき、ボールは床に落ちる。
表のとおり <b>塞がる空間は 1.0 m で 4.7 %、3.0 m で 13.0 %</b> と、タイルを粗くするほど増える。
<b>1.0 m まで細かくしても 4.7 %（90.7 m³）は塞がったままで、ゼロにはならない。</b>
机と机の隙間や壁ぎわの通路がここで消える。geom 数は 419 個で、
(A) の箱 3,847 個より少ないので計算量の問題ではない。</p>

<p><b>タイル分割は代用品である。</b> 本式は <code>CoACD</code> / <code>V-HACD</code> による
凸分解で、形に沿って割るぶん同じ破片数ならもっと精度が出る。この環境には入っていないため
空間タイル分割にしてある（<code>pip install coacd</code> で差し替えられる）。</p>

<p><b>結論。</b> MuJoCo で完結させるなら <b>(A) 箱＋(C) hfield</b>。箱は凸なので凸包で潰れず、
塞がる空間が原理的に発生しない。<b>(B) を採るなら Isaac Sim</b> — PhysX は静的コライダーに
<code>approximation = none</code> の三角メッシュをそのまま使えるので、凸分解自体が要らない
（動的剛体には使えない、という制約は残る）。</p>"""


def main() -> None:
    parser = argparse.ArgumentParser(description="MuJoCo シーンの比較HTMLを書き出す")
    parser.add_argument("session", type=Path, help="runs/<session_id>")
    parser.add_argument("-o", "--output", type=Path, default=None,
                        help="出力先（既定は <session>/report/scenes.html）")
    args = parser.parse_args()

    session = args.session.resolve()
    sim_dir = session / "sim"
    report = json.loads((sim_dir / "scenes.json").read_text())
    radius = report["settings"]["phantom_radius"]
    variants = [pack_variant(sim_dir, stats, radius) for stats in report["variants"]]
    mesh = pack_mesh(sim_dir)
    nav = sim_dir / "nav_grid.npz"
    if nav.exists():
        g = np.load(nav)
        navgrid = {"bits": encode(g["blocked"], np.uint8),
                   "rows": int(g["shape"][0]), "cols": int(g["shape"][1]),
                   "origin": [float(g["origin"][0]), float(g["origin"][1])],
                   "res": float(g["resolution"])}
    else:
        navgrid = None
    clip = load_video(sim_dir)
    if clip:
        encoded, walk_report = clip
        check = walk_report.get("check", {})
        agree = (check.get("mjcf_boxes") == check.get("html_boxes")
                 and abs(check.get("mjcf_volume_m3", 0) - check.get("html_volume_m3", 1)) < 0.5)
        video_block = (
            '<figure class="clip">'
            f'<video controls loop preload="metadata" '
            f'src="data:video/mp4;base64,{encoded}"></video>'
            '<figcaption><b>MuJoCo が描いた画面そのもの</b>（maxRange 4 m / 実時間の 4 倍速 / '
            f'{walk_report["sim_s"]:.0f} 秒ぶん）。上の 3D ビューの「再生」は骨盤の位置と向きだけを'
            '線と矢印にした抽象で、脚が運べているかは写らない。'
            'こちらは追従カメラで撮ったシミュレータの描画で、'
            '<b>脚の運びと箱への寄り方がそのまま見える</b>。'
            '<b>橙の柱が指示した目標地点</b>で、経路はそこへ向けて A* が引いたもの。<br>'
            f'HTML が描いている形が MJCF と同じであることも確かめた: '
            f'箱 {check.get("mjcf_boxes", 0):,} 個 = {check.get("html_boxes", 0):,} 個 / '
            f'体積 {check.get("mjcf_volume_m3", 0):,.1f} = '
            f'{check.get("html_volume_m3", 0):,.1f} m³ / '
            f'hfield {check.get("hfield_rows", 0)}×{check.get("hfield_cols", 0)} = 格子 '
            f'{check.get("grid_rows", 0)}×{check.get("grid_cols", 0)} → '
            f'<b>{"一致" if agree else "不一致"}</b>。'
            '</figcaption></figure>')
    else:
        video_block = ""

    first = report["variants"][0]
    subtitle = (f"{report['session']} ／ 格子 {report['settings']['cell']} m ／ "
                f"障害物帯 {report['settings']['obstacle_band'][0]}–"
                f"{report['settings']['obstacle_band'][1]} m ／ "
                f"部屋 {first['extent_m'][0]}×{first['extent_m'][1]} m")
    html = (PAGE
            .replace("__TITLE__", f"MuJoCo シーン比較 — {report['name']}")
            .replace("__SUBTITLE__", subtitle)
            .replace("__TABLE__", f'<table-wrap>{build_table(variants)}</table-wrap>'
                     .replace("<table-wrap>", "").replace("</table-wrap>", ""))
            .replace("__WALK_TABLE__", build_walk_table(variants))
            .replace("__VIDEO__", video_block)
            .replace("__WALK_NOTE__", WALK_NOTE)
            .replace("__NOTE__", NOTE)
            .replace("__NOTE_B__", NOTE_B)
            .replace("__MESH_TABLE__",
                     build_mesh_table(mesh["stats"]) if mesh else "")
            .replace("__MESH__", json.dumps(mesh, ensure_ascii=False) if mesh else "null")
            .replace("__SESSION__", json.dumps(report["session"]))
            .replace("__NAVGRID__", json.dumps(navgrid, ensure_ascii=False) if navgrid else "null")
            .replace("__DATA__", json.dumps(variants, ensure_ascii=False)))

    output = args.output or (session / "report" / "scenes.html")
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(html, encoding="utf-8")
    print(f"[完了] {output}（{output.stat().st_size / 1e6:.1f} MB）", flush=True)


if __name__ == "__main__":
    main()
