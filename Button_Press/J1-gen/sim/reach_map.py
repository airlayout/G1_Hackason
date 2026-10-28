"""ボトルを押すときの、机の位置の許容範囲の地図を作る（計画だけ。MuJoCo は動かさない）。

    P=G1_HuggingFace/venv/bin/python
    $P Button_Press/J1-gen/sim/reach_map.py --workers 6                  # 既定の格子（約 700 通り）
    $P Button_Press/J1-gen/sim/reach_map.py --edges 0.24 0.30 --gaps 0.0 --ys -0.2 -0.1 --tops 0.0   # 小さく試す

机の手前の縁 x（y = 0 での）、縁からボトルの手前の面までの距離 gap、ボトルの左右 y、天板の高さ z を振って、実際の計画
（common/press_planner.py。IK・体との衝突・経由の姿勢を通る動き・押し込みの直線）が通るかを調べる。通らないときは、
どの段階で・何に当たって（机か体か）断られたかを集計する。あわせて、頭カメラ（URDF の約 48° 下向き、640×360）の
画像にボトルが全部入るかも出す。
- 机の箱は、real/make_table_boxes.py と同じ作り（左右 9 cm ごと、y −0.5〜+0.15 m の範囲（10/21 の机は −0.285〜+0.15 m）の箱。手前の縁の 3 cm 手前から奥へ 0.6 m、天板の 0.8 m 下から 3 cm 上まで）。
  縁は x = edge + slope × y（slope は縁の傾き。-0.033 が 10/21 の実機の机）
- 押す点は、ボトルの手前の面の天板から 10 cm 上（実機 10/21 の depth_anchor 0.45 に相当）。押す向きは前方
- 開始の姿勢: 10/21 の実機の立った姿勢（lowstate の 29 関節。下ろした手が股関節の当たり判定に入っている）。
  --start zero なら全関節ゼロ。別の開始の腕は --start-q（7 関節 [rad]）で与える
- IK の初期値は pipeline と同じ順（教えた姿勢 press_bottle → 今の姿勢 → 腕のゼロ姿勢）。どれかで通れば「通る」
出力: _local/button_press/reach_map/<日時>/map.csv（1 通り 1 行）と map.txt（見やすい地図）
記号: # = 計画が通り、画像にも入る / o = 計画は通るが、ボトルが画像から出る / 英字 = 断られた段階
  W 作業空間の外 / R 手前の姿勢に届かない / A 手前の姿勢で体か机に当たる / V 経由の姿勢で当たる
  1 開始→経由の途中で当たる / 2 経由→手前の途中で当たる / P 押し込みの直線で届かない / Q 押し込みの直線で当たる / J 関節が急に動く
  大文字は「当たった相手が体」、小文字は「当たった相手が机」（当たりを伴う段階だけ）
"""

from __future__ import annotations

import argparse
import csv
import re
import sys
import time
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path
from typing import Any

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from common.camera_geometry import HeadCameraTransform  # noqa: E402
from common.collision import CollisionChecker  # noqa: E402
from common.config import REPO_ROOT, load_config  # noqa: E402
from common.press_planner import PressPlanner, UnreachableError  # noqa: E402
from common.realday import load_taught_poses  # noqa: E402

# 段階の順（大きいほど先まで進んだ）
STAGES = ["W", "R", "A", "V", "1", "2", "P", "Q", "J"]
BOTTLE_R = 0.033   # 500 ml の茶の半径 [m]
BOTTLE_H = 0.21    # ボトルの高さ [m]
PRESS_H = 0.10     # 押す点の、天板からの高さ [m]
# 2026-10-21 の実機の立った姿勢の lowstate（29 関節。tests/test_kinematics.py と同じ）。下ろした手が股関節の当たり判定に入っている
REAL_STAND_1021 = np.array([
    -0.290485, -0.004960, -0.002531, 0.593250, -0.318594, 0.008501, -0.271123, 0.003017,
    -0.002223, 0.601994, -0.338352, -0.005524, -0.000576, 0.000470, -0.002349, 0.291049,
    0.129657, 0.015711, 0.981556, 0.087161, 0.076807, 0.015232, 0.286950, -0.125894,
    0.024520, 0.982035, -0.096293, 0.058938, -0.020649])
W, H, FX, FY, CX, CY = 640, 360, 454.2, 453.8, 320.0, 180.0

_G: dict[str, Any] = {}


def desk_boxes(edge0: float, top_z: float, slope: float = 0.0, margin: float = 0.03, depth: float = 0.6,
               below: float = 0.8, strip: float = 0.09, y_range: tuple[float, float] = (-0.5, 0.15)) -> list[dict[str, Any]]:
    """机の箱（real/make_table_boxes.py と同じ作り）: 左右 strip ごとの箱。手前の縁は x = edge0 + slope * y
    （edge0 は y = 0 での縁。slope が負なら、左（y が大きい）ほど縁が手前）。各箱の手前は、strip の中で縁がいちばん手前の所 − margin。"""
    out = []
    y0 = y_range[0]
    k = 0
    while y0 < y_range[1] - 1e-9:
        ys = (y0, y0 + strip)
        x_front = edge0 + min(slope * ys[0], slope * ys[1])
        lo = np.array([x_front - margin, ys[0] - margin, top_z - below])
        hi = np.array([x_front + depth + margin, ys[1] + margin, top_z + margin])
        out.append({"name": f"table_{k + 1}", "center": ((lo + hi) / 2).tolist(), "half_size": ((hi - lo) / 2).tolist()})
        y0 += strip
        k += 1
    return out


def bottle_visible(tf: HeadCameraTransform, x_face: float, y: float, top_z: float, margin_px: float = 8.0) -> bool:
    """ボトル（手前の面が x_face、天板から BOTTLE_H の高さ）の四隅が画像に入るか。"""
    r, p = tf.pelvis_from_optical(np.zeros(3))
    xc = x_face + BOTTLE_R
    for dz in (0.0, BOTTLE_H):
        for dy in (-BOTTLE_R, BOTTLE_R):
            o = r.T @ (np.array([xc, y + dy, top_z + dz]) - p)
            if o[2] < 0.2:
                return False
            u, v = CX + FX * o[0] / o[2], CY + FY * o[1] / o[2]
            if not (margin_px <= u <= W - margin_px and margin_px <= v <= H - margin_px):
                return False
    return True


def classify(msg: str) -> tuple[str, str]:
    """断りの文から、段階の記号と、当たった相手（desk / body / ""）を出す。"""
    partner = ""
    if "ぶつかる" in msg:
        partner = "desk" if "obstacle_" in msg else "body"
    if "作業空間" in msg or "workspace" in msg:
        return "W", partner
    if "手前の姿勢に届かない" in msg:
        return "R", partner
    if "手前の姿勢で腕が体にぶつかる" in msg:
        return "A", partner
    if "経由の姿勢で腕が体にぶつかる" in msg:
        return "V", partner
    if "経由の姿勢へ移動する途中" in msg:
        return "1", partner
    if "経由の姿勢から手前の姿勢へ移動する途中" in msg:
        return "2", partner
    if "押し込みの直線" in msg and "関節が 1 周期" in msg:
        return "J", partner
    if "押し込みの直線" in msg and "届かない" in msg:
        return "P", partner
    if "押し込みの直線" in msg:
        return "Q", partner
    return "R", partner


def init_worker(side: str, start: str, start_q: list[float] | None, clearance: float | None = None,
                via_deg: list[float] | None = None) -> None:
    robot_cfg = load_config("robot.yaml")
    press_cfg = load_config("press.yaml")
    if clearance is not None:
        press_cfg["collision"]["clearance_m"] = clearance
    arm_cfg = load_config("arm.yaml")
    arm_cfg["arm"] = side
    pipe = load_config("pipeline.yaml")
    planner = PressPlanner.from_config(robot_cfg, press_cfg, arm_cfg, side)
    arm = planner.kin.arm_idx
    q0 = REAL_STAND_1021.copy()
    if start_q is not None:
        q0[arm] = np.asarray(start_q, dtype=float)
    elif start == "zero":
        q0 = np.zeros(29)
    via = np.radians(np.asarray(pipe["via_pose"] if via_deg is None else via_deg, dtype=float))
    seeds: list[tuple[str, np.ndarray | None]] = []
    poses = load_taught_poses()
    if pipe.get("seed_pose") in poses:
        q = q0.copy()
        q[arm] = poses[pipe["seed_pose"]]["arm_q_rad"]
        seeds.append(("taught", q))
    seeds.append(("now", None))
    zero = q0.copy()
    zero[arm] = 0.0
    seeds.append(("zero", zero))
    _G.update(robot_cfg=robot_cfg, press_cfg=press_cfg, planner=planner, q0=q0, via=via, seeds=seeds, side=side,
              tf=HeadCameraTransform(robot_cfg), desk_key=None)


def run_cell(cell: tuple[float, float, float, float, float], via: np.ndarray | None = None) -> dict[str, Any]:
    """1 通りを計画する。via（片腕 7 関節 [rad]）を与えると、その経由の姿勢を使う（無ければ pipeline.yaml の via_pose）。"""
    edge, gap, y, top, slope = cell
    G = _G
    if G["desk_key"] != (edge, top, slope):
        planner: PressPlanner = G["planner"]
        planner.collision = CollisionChecker(G["robot_cfg"], G["side"], float(G["press_cfg"]["collision"]["clearance_m"]),
                                             desk_boxes(edge, top, slope))
        G["desk_key"] = (edge, top, slope)
    x_face = edge + slope * y + gap   # ボトルの手前の面。gap は、ボトルの左右 y での縁からの距離
    target = np.array([x_face, y, top + PRESS_H])
    push = np.array([1.0, 0.0, 0.0])
    best: tuple[int, str, str, str] | None = None
    ok_seed = ""
    for label, q_seed in G["seeds"]:
        try:
            G["planner"].plan(G["q0"], target, push, q_seed=q_seed, q_via_arm=G["via"] if via is None else via)
            ok_seed = label
            break
        except UnreachableError as e:
            st, who = classify(str(e))
            rank = STAGES.index(st)
            if best is None or rank > best[0]:
                best = (rank, st, who, str(e)[:160])
    vis = bottle_visible(G["tf"], x_face, y, top)
    row = {"edge": edge, "gap": gap, "y": y, "top": top, "slope": slope, "plan_ok": int(bool(ok_seed)), "seed": ok_seed,
           "visible": int(vis), "stage": "" if ok_seed else best[1], "partner": "" if ok_seed else best[2],
           "reason": "" if ok_seed else best[3]}
    return row


def sym(r: dict[str, Any]) -> str:
    if r["plan_ok"]:
        return "#" if r["visible"] else "o"
    s = r["stage"]
    return s.lower() if r["partner"] == "desk" else s


def render(rows: list[dict[str, Any]], edges: list[float], gaps: list[float], ys: list[float], tops: list[float],
           slopes: list[float]) -> str:
    idx = {(r["edge"], r["gap"], r["y"], r["top"], r["slope"]): r for r in rows}
    out = []
    for slope in slopes:
        for top in tops:
            for gap in gaps:
                out.append(f"\n■ 縁の傾き {slope:+.3f}（x/y。0 = 体の正面に平行）、天板 z {top:+.2f} m（床から約 {top + 0.73:.2f} m）、"
                           f"縁からボトルの面まで {gap * 100:.0f} cm   （行 = ボトルの左右 y、列 = 机の縁 x（y = 0 での）[cm]）")
                out.append("   y \\ 縁 " + " ".join(f"{e * 100:3.0f}" for e in edges))
                for y in ys:
                    out.append(f"  {y:+.2f}   " + " ".join(f"  {sym(idx[(e, gap, y, top, slope)])}" for e in edges))
    return "\n".join(out)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--edges", type=float, nargs="+", default=[round(0.16 + 0.02 * i, 2) for i in range(13)])
    ap.add_argument("--gaps", type=float, nargs="+", default=[0.0, 0.03, 0.06, 0.12])
    ap.add_argument("--ys", type=float, nargs="+", default=[round(-0.30 + 0.05 * i, 2) for i in range(7)])
    ap.add_argument("--tops", type=float, nargs="+", default=[-0.04, 0.0])
    ap.add_argument("--slopes", type=float, nargs="+", default=[0.0, -0.033],
                    help="机の縁の傾き dx/dy（-0.033 は 10/21 の実機の机。左ほど手前）")
    ap.add_argument("--side", default="right")
    ap.add_argument("--start", choices=["real", "zero"], default="real")
    ap.add_argument("--start-q", type=float, nargs=7, help="開始の腕 7 関節 [rad]（実機の lowstate）")
    ap.add_argument("--clearance", type=float, help="腕と体・机の余裕 [m]（既定は press.yaml の collision.clearance_m = 0.01）")
    ap.add_argument("--via", type=float, nargs=7, help="経由の姿勢（右腕 7 関節 [deg]）。既定は pipeline.yaml の via_pose")
    ap.add_argument("--workers", type=int, default=4)
    ap.add_argument("--out", type=Path)
    args = ap.parse_args()

    cells = [(e, g, y, t, sl) for sl in args.slopes for t in args.tops for e in args.edges for g in args.gaps for y in args.ys]
    t0 = time.time()
    print(f"[map] {len(cells)} 通りを計画する（{args.workers} プロセス）")
    with ProcessPoolExecutor(args.workers, initializer=init_worker,
                             initargs=(args.side, args.start, args.start_q, args.clearance, args.via)) as ex:
        rows = []
        for i, r in enumerate(ex.map(run_cell, cells, chunksize=8), start=1):
            rows.append(r)
            if i % 50 == 0:
                print(f"[map] {i}/{len(cells)}（{time.time() - t0:.0f} 秒）")
    out = args.out or REPO_ROOT / "_local" / "button_press" / "reach_map" / time.strftime("%Y%m%d_%H%M%S")
    out.mkdir(parents=True, exist_ok=True)
    with open(out / "map.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    text = render(rows, sorted(args.edges), sorted(args.gaps), sorted(args.ys), sorted(args.tops), sorted(args.slopes))
    ok = sum(r["plan_ok"] for r in rows)
    both = sum(r["plan_ok"] and r["visible"] for r in rows)
    summ = [f"計画が通る {ok}/{len(rows)}、画像にも入る {both}/{len(rows)}（{time.time() - t0:.0f} 秒）", "断られた段階（件数）:"]
    cnt: dict[tuple[str, str], int] = {}
    for r in rows:
        if not r["plan_ok"]:
            cnt[(r["stage"], r["partner"])] = cnt.get((r["stage"], r["partner"]), 0) + 1
    for (st, who), n in sorted(cnt.items(), key=lambda kv: -kv[1]):
        summ.append(f"  {st} {who or '-':5s} {n}")
    (out / "map.txt").write_text("\n".join(summ) + "\n" + text + "\n", encoding="utf-8")
    print("\n".join(summ))
    print(f"[map] 保存: {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
