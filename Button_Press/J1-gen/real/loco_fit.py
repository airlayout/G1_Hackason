"""実機で 1 回分ずつ歩かせた記録（real/loco_pulse.py の pulse.json）をまとめ、configs/approach.yaml の loco の値の案を出す。

    G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/loco_fit.py                     # _local/button_press/loco_pulse/ の全部
    G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/loco_fit.py --since 20261113    # その日から（フォルダ名の時刻で選ぶ）

動いた量は、メジャーで書いた measured を優先し、無ければ頭カメラの measured_by_camera を使う。向き（前・横・回る）と
指令の時間ごとに、動いた量の平均・ばらつきと、1 秒あたりに動いた量（rate。時間に比例するとみなして原点を通る直線を当てはめる）を出す。
動いた量が「rate × 時間」の 30% に届かない時間は「歩き出さない」とみなし、min_duration_s の案はそれより長い最短の時間にする。
前に進んだときの横ずれから進む向きのずれ（max_heading_error_deg の参考）も出す。**設定ファイルは書き換えない**（案を表示するだけ）。
"""

from __future__ import annotations

import argparse
import json
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from common.config import REPO_ROOT  # noqa: E402

# 向き → (pulse の速度の成分, 動いた量の項目, 単位の変換)
AXES = {
    "forward": ("vx", 1.0, "forward_cm", 0.01),
    "backward": ("vx", -1.0, "forward_cm", 0.01),
    "left": ("vy", 1.0, "left_cm", 0.01),
    "right": ("vy", -1.0, "left_cm", 0.01),
    "turn_left": ("wz", 1.0, "turn_deg", np.pi / 180),
    "turn_right": ("wz", -1.0, "turn_deg", np.pi / 180),
}


def axis_of(p: dict) -> str | None:
    for name, (comp, sign, _, _) in AXES.items():
        v = float(p[comp])
        if v * sign > 0 and all(float(p[c]) == 0.0 for c in ("vx", "vy", "wz") if c != comp):
            return name
    return None


def load(paths: list[Path]) -> list[dict]:
    rows = []
    for f in paths:
        d = json.loads(f.read_text(encoding="utf-8"))
        p = d["pulse"]
        axis = axis_of(p)
        if axis is None:
            continue
        comp, sign, key, unit = AXES[axis]
        tape, cam = d.get("measured") or {}, d.get("measured_by_camera") or {}
        src, val = ("メジャー", tape.get(key)) if tape.get(key) is not None else ("頭カメラ", cam.get(key))
        if val is None:
            continue
        moved = sign * float(val) * unit  # 動かしたい向きに正
        side = None
        if axis == "forward":
            s = tape.get("left_cm") if tape.get("left_cm") is not None else cam.get("left_cm")
            side = None if s is None else float(s) / 100
        rows.append({"dir": f.parent.name, "axis": axis, "speed": abs(float(p[comp])), "duration": float(p["duration_s"]),
                     "moved": moved, "side": side, "settle_s": d.get("settle_s"), "stopped": d.get("stopped"), "src": src})
    return rows


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--dir", type=Path, default=REPO_ROOT / "_local" / "button_press" / "loco_pulse")
    ap.add_argument("--since", default="", help="この時刻（フォルダ名の先頭、例 20261113）以降だけ")
    args = ap.parse_args()

    paths = sorted(p for p in args.dir.glob("*/pulse.json") if p.parent.name >= args.since)
    rows = load(paths)
    if not rows:
        print(f"[loco_fit] 動いた量が書かれた記録が無い（{args.dir}、{len(paths)} 個の pulse.json）。"
              "pulse.json の measured にメジャーの値を書くか、--measure-face で測る")
        return 1
    by_axis: dict[str, list[dict]] = defaultdict(list)
    for r in rows:
        by_axis[r["axis"]].append(r)
    print(f"[loco_fit] {len(rows)} 回分（{len(paths)} 個の pulse.json のうち、動いた量があるもの）")
    suggest = {}
    for axis, rs in by_axis.items():
        unit_name, scale = ("°", 180 / np.pi) if axis.startswith("turn") else ("cm", 100)
        print(f"\n== {axis}（速さ {sorted({r['speed'] for r in rs})}）")
        t = np.array([r["duration"] for r in rs])
        m = np.array([r["moved"] for r in rs])
        rate = float((t @ m) / (t @ t))  # 原点を通る直線
        # 歩き出さない時間の記録が混ざると rate が小さく出るので、歩き出した時間だけで当てはめ直す（2 回）
        for _ in range(2):
            walk = np.array([m[t == d].mean() >= 0.3 * rate * d for d in t])
            if walk.any():
                rate = float((t[walk] @ m[walk]) / (t[walk] @ t[walk]))
        for dur in sorted(set(t)):
            sel = m[t == dur]
            ok = sel.mean() >= 0.3 * rate * dur
            print(f"   {dur:.2f} 秒: {len(sel)} 回、動いた量 平均 {sel.mean() * scale:+.1f} {unit_name}"
                  f"（ばらつき {sel.std() * scale:.1f}）{'' if ok else '  ← 歩き出さないとみなす'}")
        walking = sorted(d for d in set(t) if m[t == d].mean() >= 0.3 * rate * d)
        walk = np.isin(t, walking)
        resid = (m - rate * t)[walk]
        # 多めに動いた割合の最大（approach.yaml の step_uncertainty の参考）
        unc = float(np.max(resid / (rate * t[walk]))) if walk.sum() > 1 else None
        print(f"   1 秒あたり（rate、歩き出した時間だけ）: {rate * scale:.1f} {unit_name}/秒、"
              f"当てはめからのずれの最大 {np.abs(resid).max() * scale if len(resid) else 0.0:.1f} {unit_name}")
        sides = [(r["moved"], r["side"]) for r in rs if r["side"] is not None and r["duration"] in walking]
        if sides:
            ang = [np.degrees(np.arctan2(s, f)) for f, s in sides if f > 0]
            print(f"   進む向きのずれ: 平均 {np.mean(ang):+.1f}°、最大 {np.max(np.abs(ang)):.1f}°（+ が左。approach.yaml の max_heading_error_deg の参考）")
        settle = [r["settle_s"] for r in rs if r["settle_s"] is not None]
        if settle:
            print(f"   止まるまで: 平均 {np.mean(settle):.2f} 秒、最大 {np.max(settle):.2f} 秒。止まらなかった回: "
                  f"{sum(1 for r in rs if r['stopped'] is False)}")
        print(f"   出どころ: {sorted({r['src'] for r in rs})}")
        suggest[axis] = {"rate": round(rate, 3), "min_duration_s": walking[0] if walking else None, "unc": unc}
    print("\n[loco_fit] approach.yaml の loco.axes の案（speed と max_duration_s はそのまま。min_duration_s が None なら、"
          "どの時間でも歩き出さなかった）:")
    for axis, sgg in suggest.items():
        print(f"    {axis}: rate {sgg['rate']}, min_duration_s {sgg['min_duration_s']}"
              + (f"（step_uncertainty の参考: {sgg['unc']:.2f}）" if sgg["unc"] is not None else ""))
    return 0


if __name__ == "__main__":
    sys.exit(main())
