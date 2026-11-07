"""近づく → 押す を、実機に近づける乱しをランダムに入れて何度も回し、成功率と失敗の原因を数える。

    G1_HuggingFace/venv/bin/python Trash_Bin/sim/stress_test.py --trials 30 --workers 4
    G1_HuggingFace/venv/bin/python Trash_Bin/sim/stress_test.py --trials 30 --max-dropout 0.8   # 深度がもっと欠ける

1 回の試行 = sim/approach_sim.py --then-push を、次の条件をランダムに選んで実行する（実機を触れない間に、壊れ方を先に見つける）:
- 最初のゴミ箱の置き方: 正面が体の前 0.5〜1.0 m、中心が左右 ±25 cm、向き ±25°
- 歩行のずれ: 歩いた量の倍率 ±25%、横すべり・回り過ぎ 15%
- 上面（鏡面）の深度の欠け: 0〜max-dropout、深度のノイズ 1 m あたり 1 cm
- 頭カメラの本当の角度: 43〜49°（位置の計算は 47.6° のまま。実機の通常は約 46°）
- 較正のずれ: 各軸 ±1 cm
- 後ろの壁: ゴミ箱の背面から 0〜max-wall-gap（実物は壁にほぼ付けて置いてある。--max-wall-gap -1 なら壁なし）
結果は _local/trash_bin/stress/<日時>/results.csv と、画面の表。
"""

from __future__ import annotations

import argparse
import csv
import json
import os
import re
import subprocess
import sys
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[2]
PY = sys.executable


def run_trial(i: int, args: argparse.Namespace) -> dict:
    rng = np.random.default_rng(10_000 + i)
    cond = {
        "trial": i,
        "front": round(float(rng.uniform(0.5, 1.0)), 3),
        "y": round(float(rng.uniform(-0.25, 0.25)), 3),
        "yaw": round(float(rng.uniform(-25, 25)), 1),
        "dropout": round(float(rng.uniform(0.0, args.max_dropout)), 2),
        "cam_true": round(float(rng.uniform(43.0, 49.0)), 1),
        "wall_gap": round(float(rng.uniform(args.min_wall_gap, args.max_wall_gap)), 3) if args.max_wall_gap >= 0 else None,
    }
    cmd = [PY, str(REPO / "Trash_Bin/sim/approach_sim.py"), "--start", str(cond["front"]), str(cond["y"]), str(cond["yaw"]),
           "--seed", str(i), "--walk-error", "0.25", "--slip", "0.15", "--depth-dropout", str(cond["dropout"]),
           "--depth-noise", "0.01", "--cam-pitch-true", str(cond["cam_true"]), "--calib-error-m", "0.01", "--then-push"]
    if cond["wall_gap"] is not None:
        cmd += ["--back-wall-gap", str(cond["wall_gap"])]
    t0 = time.monotonic()
    p = subprocess.run(cmd, capture_output=True, text=True, timeout=900, env={**os.environ, "MUJOCO_GL": "glfw"})
    out = p.stdout + p.stderr
    steps = len(re.findall(r"\[approach\] \d+ 回目", out))
    arrived = "[approach] 着いた" in out
    res = {**cond, "seconds": round(time.monotonic() - t0), "steps": steps, "arrived": arrived}
    m = re.search(r"最後の立つ位置のずれ（正解）: 前 ([-+0-9.]+) cm・左 ([-+0-9.]+) cm・向き ([-+0-9.]+)°", out)
    if m:
        res.update(stand_err_x_cm=float(m.group(1)), stand_err_y_cm=float(m.group(2)), stand_err_yaw_deg=float(m.group(3)))
    m = re.search(r"\[push_bin_sim\] 結果: (\{.*\})", out)
    if m:
        r = json.loads(m.group(1))
        px = r.get("visible_px_at_end") or {}
        res.update(push_code=r["code"], flap_deg=r.get("flap_deg_at_end"), red_px=px.get("red"),
                   locate_err_cm=(r.get("located") or {}).get("center_err_cm"), push_result=r["result"][:120])
    if not arrived:
        why = re.findall(r"\[approach\] (❌.*|\d+ 回目: 見つからない.*)", out)
        res["fail_reason"] = (why[-1] if why else out.strip().splitlines()[-1] if out.strip() else "?")[:120]
    elif res.get("push_code", 1) != 0:
        res["fail_reason"] = res.get("push_result", "押す流れの結果が無い")
    return res


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--trials", type=int, default=30)
    p.add_argument("--workers", type=int, default=4, help="同時に走らせる数")
    p.add_argument("--max-dropout", type=float, default=0.6, help="上面の深度を欠けさせる割合の上限")
    p.add_argument("--max-wall-gap", type=float, default=0.05, help="後ろの壁までの距離の上限 [m]（-1 で壁なし）")
    p.add_argument("--min-wall-gap", type=float, default=0.0, help="後ろの壁までの距離の下限 [m]")
    args = p.parse_args()

    out_dir = REPO / "_local/trash_bin/stress" / time.strftime("%Y%m%d_%H%M%S")
    out_dir.mkdir(parents=True, exist_ok=True)
    rows = []
    with ThreadPoolExecutor(max_workers=args.workers) as ex:
        for res in ex.map(lambda i: run_trial(i, args), range(args.trials)):
            ok = res["arrived"] and res.get("push_code") == 0
            print(f"[stress] {res['trial']:2d}: {'成功' if ok else '失敗'}  開始(前 {res['front']} m, 左 {res['y']} m, {res['yaw']}°)"
                  f" 欠け {res['dropout']:.0%} カメラ {res['cam_true']}° 壁 {res['wall_gap']} m | {res['steps']} 回で"
                  f"{'着いた' if res['arrived'] else '着かず'}"
                  + (f"、フラップ {res.get('flap_deg')}°、赤 {res.get('red_px')} 画素" if ok else f" | {res.get('fail_reason', '')}"),
                  flush=True)
            rows.append(res)
    keys = sorted({k for r in rows for k in r})
    with open(out_dir / "results.csv", "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        w.writerows(rows)
    n_ok = sum(1 for r in rows if r["arrived"] and r.get("push_code") == 0)
    n_arr = sum(1 for r in rows if r["arrived"])
    print(f"[stress] 着いた {n_arr}/{len(rows)}、押すまで成功 {n_ok}/{len(rows)}。結果: {out_dir / 'results.csv'}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
