"""フラップを押し下げている間の頭カメラの深度から、ゴミ箱の中身がどこまで溜まっているかを測る。sim / real 共通。

開口（上面の中央の穴）を通して見える点の高さを、ゴミ箱の底から測る（詳しくは measure_contents）:
- 押し下げたフラップと手は、開口の縁から flap_clear_m（既定 9 cm）より上にある（フラップの半分 10.75 cm が約 57° まで傾く範囲）。
  それより深い点だけを中身とみなす。開口のすぐ下（9 cm 以内）まで溜まっていると、フラップと区別できないので「満杯に近い」とだけ言う
- 中身の上面の高さ = 開口の中の点の高さの 90 パーセンタイル（手前の縁で切れた点や、ノイズの低い点に引っ張られないように）

⚠️ 実物のゴミ袋は黒いことが多く、黒は赤外線を吸うので深度が欠けやすい。点が少なければ「測れない」を返す（画像で人が見る）。
"""

from __future__ import annotations

from typing import Any

import numpy as np

from bin_geometry import BinPose, local_geometry
from locate_bin import _floor_level, depth_to_pelvis


def measure_contents(frame: Any, transform: Any, q_waist: np.ndarray, cfg: dict[str, Any], pose: BinPose,
                     flap_clear_m: float = 0.09, wall_margin_m: float = 0.025, min_points: int = 40,
                     level_with_floor: bool = True) -> dict[str, Any]:
    """中身の上面の高さ（ゴミ箱の底から [m]）と判定を返す。

    頭カメラは斜め上から見下ろすので、開口を通った視線はゴミ箱の奥の内壁に当たり、底までは届かない（立つ位置で、開口の手前の縁を
    通る視線でも底から約 13 cm まで。2026-11-08、MuJoCo）。そこで 2 つを見る:
    - 中身: ゴミ箱の内側（内壁から wall_margin_m 離れた所）で、開口の縁から flap_clear_m より深い点。90 パーセンタイルを上面とする
    - 奥の内壁: 中身が見えないとき、奥の内壁がどの高さまで見えているか。そこまで見えていれば「それより上には何も無い」と言える
    """
    g = local_geometry(cfg)
    t = float(cfg["bin"].get("wall_thickness", 0.004))
    ph = float(cfg["pelvis_height_m"])
    pts = depth_to_pelvis(frame, transform, q_waist)
    if level_with_floor:
        level = _floor_level(pts, transform, q_waist, -ph)
        if level is not None:
            pts = level.apply(pts)
    rel = pts[:, :2] - np.asarray(pose.center_xy, dtype=float)
    c, s = np.cos(pose.yaw), np.sin(pose.yaw)
    lx, ly = rel[:, 0] * c + rel[:, 1] * s, -rel[:, 0] * s + rel[:, 1] * c
    bottom = g["z_floor"] + 0.01  # 底の板の上面
    deepest_visible = g["z_open"] - flap_clear_m  # これより上はフラップと手と区別できない
    z = pts[:, 2]
    band = (z < deepest_visible) & (z > bottom - 0.03)
    m = wall_margin_m
    interior = (lx > g["x0"] + t + m) & (lx < g["x1"] - t - m) & (ly > g["ya"] + t + m) & (ly < g["yb"] - t - m)
    sel = pts[interior & band]
    out: dict[str, Any] = {"n_points": int(len(sel)), "max_measurable_m": round(float(deepest_visible - bottom), 3)}
    if len(sel) >= min_points:
        h = sel[:, 2] - bottom
        top = float(np.percentile(h, 90))
        if top > float(deepest_visible - bottom) - 0.05:  # 測れる上限（開口の縁から 9 cm 下）まで 5 cm 以内
            verdict = "満杯に近い"
        elif top < 0.03:
            verdict = "空に近い"
        else:
            verdict = "途中まで溜まっている"
        out.update(ok=True, verdict=verdict, top_m=round(top, 3), median_m=round(float(np.median(h)), 3),
                   fill_ratio=round(top / float(g["z_open"] - bottom), 2))
        return out
    # 中身が見えない: 奥の内壁（開口の幅の中）がどこまで低く見えているか
    back = (lx > g["x1"] - t - 0.03) & (lx < g["x1"] + 0.01) & (ly > g["oy0"]) & (ly < g["oy1"]) & band
    wall = pts[back]
    out["wall_points"] = int(len(wall))
    if len(wall) >= min_points:
        low = float(np.percentile(wall[:, 2] - bottom, 5))
        out.update(ok=True, verdict="見える範囲には無い", top_m=None, clear_above_m=round(low, 3))
        return out
    out.update(ok=False, verdict="測れない",
               reason=f"ゴミ箱の中の深い所の点が {len(sel)} 個・奥の内壁が {len(wall)} 個しかない"
                      "（黒い袋・鏡面で深度が欠けた、フラップが開いていない、など）")
    return out


def best_of(results: list[dict[str, Any]]) -> dict[str, Any] | None:
    """何枚かの結果から、測れた点が一番多いものを選ぶ（フラップが一番開いた瞬間に近い）。"""
    ok = [r for r in results if r.get("ok")]
    pool = ok or results
    return max(pool, key=lambda r: r.get("n_points", 0)) if pool else None


def summary(r: dict[str, Any] | None) -> str:
    if r is None:
        return "中身: 測っていない"
    if not r.get("ok"):
        return f"中身: {r['verdict']}（{r.get('reason', '')}）"
    if r.get("top_m") is None:
        return (f"中身: {r['verdict']}（奥の内壁が底から {r['clear_above_m'] * 100:.0f} cm まで見えている。"
                f"それより上には何も無い。空か、それより低い所にある）")
    return (f"中身: {r['verdict']}（上面が底から {r['top_m'] * 100:.0f} cm、開口までの {r['fill_ratio'] * 100:.0f}%。"
            f"点 {r['n_points']} 個。開口の縁から 9 cm 以内はフラップと区別できない）")
